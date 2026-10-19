"""C13 の車速のフィルタと遅れの予測器（speed_predictor）と C13 の候補・config のテスト。
2026-10-19 段2u。"""

from __future__ import annotations

import math
import random
from pathlib import Path

import pytest

from tests.research import config as cfgmod
from tests.research.dob import PedalDisturbanceObserver
from tests.research.ff_candidate import (
    CoastAssistFeedforward,
    PredictorFeedforward,
    make_candidate,
)
from tests.research.speed_predictor import (
    PredictorStats,
    SpeedFilterPredictor,
    SpeedKalman,
    accel_error_var_from_meta,
    noise_var_from_second_difference,
)

DT = 0.05


class _Table:
    """a = −1 + 0.3u（u≥0）/ 0.6u（u<0）。速度に依らない合成の表（test_research_dob と同じ）。"""

    meta: dict = {}

    def accel(self, v: float, u: float, *, launch: bool = False, stop: bool = False,
              release: float | None = None) -> float:
        return -1.0 + (0.3 * u if u >= 0 else 0.6 * u)


# ─── 車速のフィルタ ──────────────────────────────────────────────────────


def test_kalman_tracks_constant_acceleration_better_than_raw_can() -> None:
    """一定加速（2 km/h/s）で、予測の加速度が合っていれば推定は真値に寄り、CAN の乱れより小さい。"""
    rng = random.Random(1)
    sigma = 0.05
    kf = SpeedKalman(accel_var=0.04, r_var=sigma**2)
    err_est, err_raw = [], []
    for k in range(400):
        truth = 10.0 + 2.0 * k * DT
        y = truth + rng.gauss(0.0, sigma)
        kf.predict(2.0, DT)
        x = kf.update(y)
        if k > 40:
            err_est.append(x - truth)
            err_raw.append(y - truth)
    rms = lambda e: math.sqrt(sum(v * v for v in e) / len(e))  # noqa: E731
    assert rms(err_est) < 0.6 * rms(err_raw)
    assert abs(sum(err_est) / len(err_est)) < 0.01  # 偏りなし


def test_kalman_larger_r_is_smoother_but_follows_slower() -> None:
    rng = random.Random(2)
    ys = [20.0 + rng.gauss(0.0, 0.1) for _ in range(300)]

    def roughness(r: float) -> float:
        kf = SpeedKalman(accel_var=0.04, r_var=r)
        xs = []
        for y in ys:
            kf.predict(0.0, DT)
            xs.append(kf.update(y))
        d2 = [xs[k] - 2 * xs[k - 1] + xs[k - 2] for k in range(60, len(xs))]
        return math.sqrt(sum(v * v for v in d2) / len(d2))

    assert roughness(1.0e-1) < 0.5 * roughness(1.0e-3)
    # 追従: 大きい R の推定は、速度の階段に遅れる
    lo, hi = SpeedKalman(accel_var=0.04, r_var=1e-3), SpeedKalman(accel_var=0.04, r_var=1.0)
    for kf in (lo, hi):
        kf.update(0.0)
        for _ in range(10):
            kf.predict(0.0, DT)
            kf.update(10.0)
    assert lo.x > hi.x


def test_noise_var_from_second_difference_recovers_sigma_and_ignores_trend() -> None:
    rng = random.Random(3)
    sigma = 0.02
    # 直線の傾きは 2 階差分で消える
    ys = [0.5 * k * DT + rng.gauss(0.0, sigma) for k in range(2000)]
    assert noise_var_from_second_difference(ys) == pytest.approx(sigma**2, rel=0.15)
    assert noise_var_from_second_difference([0.0] * 5) is None  # 点が少ない
    assert noise_var_from_second_difference([0.0] * 50) == 0.0  # 張り付き


def test_accel_error_var_from_meta_uses_seed_mae_else_fallback() -> None:
    var, note = accel_error_var_from_meta({"seed_mae": {0: 0.1, 1: 0.2}}, 0.5)
    assert var == pytest.approx((math.sqrt(math.pi / 2) * 0.15) ** 2) and "seed_mae" in note
    var2, note2 = accel_error_var_from_meta({}, 0.5)
    assert var2 == pytest.approx(0.25) and "仮値" in note2
    assert accel_error_var_from_meta({"seed_mae": {0: float("nan")}}, 0.3)[0] == pytest.approx(0.09)


# ─── 予測器 ──────────────────────────────────────────────────────────────


def _make(lag_a: float = 0.2, lag_b: float = 0.2, **kw: object) -> tuple[
        PedalDisturbanceObserver, SpeedFilterPredictor]:
    dob = PedalDisturbanceObserver(
        _Table(), tq_accel_s=1.0, tq_brake_s=1.0, lag_accel_s=lag_a, lag_brake_s=lag_b
    )
    args: dict = {
        "period_s": DT, "accel_var": 0.04, "r_override": 1e-4, "r_floor": 1e-4,
        "lag_accel_s": lag_a, "lag_brake_s": lag_b, "coast_lag_s": lag_a,
    }
    args.update(kw)
    return dob, SpeedFilterPredictor(dob, **args)  # type: ignore[arg-type]


def _drive(dob: PedalDisturbanceObserver, pred: SpeedFilterPredictor, n: int, *, u: float,
           v0: float = 30.0) -> tuple[float, float]:
    """表どおりの車（遅れなしで表の加速度）を、指令 u 一定で n 周期走らせる。(時刻, 車速)。"""
    t, v = 0.0, v0
    for _ in range(n):
        pred.step(t=t, can_kmh=v, launch=False, stop=False, held=False, idle=False)
        dob.update(t=t, speed_kmh=pred.kf.x or v, u_now=u, governed=False, at_limit_now=False,
                   launch=False, stop=False)
        t += DT
        v += _Table().accel(v, u) * DT
    return t, v


def test_predictor_with_constant_command_is_table_accel_times_lag() -> None:
    """指令が一定なら、L 秒先の予測 ＝ 推定車速 ＋ 表の加速度（−1 ＋ 0.3 × 10 ＝ 2）× L。"""
    dob, pred = _make(lag_a=0.2)
    t, v = _drive(dob, pred, 200, u=10.0)  # 10 秒走らせて履歴を満たす
    x = pred.kf.x
    assert x == pytest.approx(v - 2.0 * DT, abs=0.05)  # 最後の CAN 車速は今の周期の前
    hat = pred.predict_ahead(t=t, lag_s=0.2, launch=False, stop=False, held=False)
    assert hat == pytest.approx(x + 2.0 * 0.2, abs=0.01)
    # 惰行（u=0）に切り替えた直後は、遅れの間は前の指令（u=10）の加速度が続く
    t2, v2 = _drive_more(dob, pred, t, v, 2, u=0.0)
    hat2 = pred.predict_ahead(t=t2, lag_s=0.2, launch=False, stop=False, held=False)
    assert hat2 is not None and pred.kf.x is not None
    gain = hat2 - pred.kf.x
    assert 0.2 * -1.0 < gain <= 0.2 * 2.0 + 1e-6  # 前の指令の分（+2）と惰行（−1）が混ざる


def _drive_more(dob: PedalDisturbanceObserver, pred: SpeedFilterPredictor, t: float, v: float,
                n: int, *, u: float) -> tuple[float, float]:
    for _ in range(n):
        pred.step(t=t, can_kmh=v, launch=False, stop=False, held=False, idle=False)
        dob.update(t=t, speed_kmh=pred.kf.x or v, u_now=u, governed=False, at_limit_now=False,
                   launch=False, stop=False)
        t += DT
        v += _Table().accel(v, u) * DT
    return t, v


def test_predictor_adds_dob_disturbance_times_lag() -> None:
    """DOB の推定外乱 d は、押しているペダルの分だけ L 秒ぶん足される。"""
    dob, pred = _make()
    t, v = _drive(dob, pred, 200, u=10.0)
    base = pred.predict_ahead(t=t, lag_s=0.2, launch=False, stop=False, held=False)
    dob.d_accel = -0.4
    lower = pred.predict_ahead(t=t, lag_s=0.2, launch=False, stop=False, held=False)
    assert base is not None and lower is not None
    d0 = 0.0  # 表どおりに走らせたので、もとの推定はほぼ 0
    assert lower == pytest.approx(base - (0.4 + d0) * 0.2, abs=1e-4)
    # 低速の行（held）は d を足さない
    held = pred.predict_ahead(t=t, lag_s=0.2, launch=False, stop=False, held=True)
    assert held == pytest.approx(base, abs=1e-4)


def test_predictor_without_history_is_no_prediction() -> None:
    dob, pred = _make()
    assert pred.predict_ahead(t=0.0, lag_s=0.2, launch=False, stop=False, held=False) is None
    pred.step(t=0.0, can_kmh=20.0, launch=False, stop=False, held=False, idle=False)
    # 履歴が無い（遅れた u が引けない）ので加速度 0 ＝ 今の推定のまま
    assert pred.predict_ahead(
        t=0.0, lag_s=0.2, launch=False, stop=False, held=False
    ) == pytest.approx(20.0)


def test_lag_for_follows_the_pedal_in_use() -> None:
    _, pred = _make(lag_a=0.23, lag_b=0.2, coast_lag_s=0.25)
    assert pred.lag_for(5.0) == 0.23 and pred.lag_for(-5.0) == 0.2 and pred.lag_for(0.0) == 0.25
    _, none = _make(lag_b=None, coast_lag_s=None)  # type: ignore[arg-type]
    assert none.lag_for(-1.0) is None and none.lag_for(0.0) is None


def test_r_is_measured_in_first_hold_with_floor() -> None:
    """最初の停車保持の CAN 車速の乱れから R を測る。張り付いて 0 なら下限。"""
    rng = random.Random(4)
    dob, pred = _make(r_override=None, r_floor=1e-5)
    for k in range(100):
        pred.step(t=k * DT, can_kmh=abs(rng.gauss(0.0, 0.03)), launch=False, stop=True,
                  held=True, idle=True)
    pred.step(t=5.0, can_kmh=0.5, launch=False, stop=False, held=True, idle=False)
    # 速度の絶対値を取っているので、白色雑音より少し小さくなる（半正規分布の分散 σ²(1 − 2/π)）
    assert pred.r_measured == pytest.approx(0.03**2 * (1 - 2 / math.pi), rel=0.5)
    assert pred.kf.r_var >= 1e-5 and "停車保持" in pred.r_source
    # 張り付き（定数）は下限
    dob2, pred2 = _make(r_override=None, r_floor=1e-4)
    for k in range(100):
        pred2.step(t=k * DT, can_kmh=0.0, launch=False, stop=True, held=True, idle=True)
    pred2.step(t=5.0, can_kmh=0.0, launch=False, stop=False, held=True, idle=False)
    assert pred2.kf.r_var == pytest.approx(1e-4) and "下限" in pred2.r_source
    # yaml の上書きは測らない
    _, pred3 = _make(r_override=3e-4)
    assert pred3.kf.r_var == pytest.approx(3e-4) and "上書き" in pred3.r_source


def test_filter_accel_est_is_smoother_than_can_difference() -> None:
    rng = random.Random(5)
    dob, pred = _make(r_override=1e-3)
    can_d, est_d = [], []
    prev_can = None
    v = 20.0
    for k in range(400):
        y = v + rng.gauss(0.0, 0.03)
        pred.step(t=k * DT, can_kmh=y, launch=False, stop=False, held=False, idle=False)
        dob.update(t=k * DT, speed_kmh=pred.kf.x or y, u_now=3.33, governed=False,
                   at_limit_now=False, launch=False, stop=False)
        if prev_can is not None and k > 80:
            can_d.append((y - prev_can) / DT)
            est_d.append(pred.accel_est)
        prev_can = y
        v += 0.0 * DT  # 表の加速度 0 ≒ −1 ＋ 0.3 × 3.33 ＝ 0
    sd = lambda e: math.sqrt(sum(a * a for a in e) / len(e))  # noqa: E731
    assert sd(est_d) < 0.7 * sd(can_d)


# ─── 集計 ────────────────────────────────────────────────────────────────


def test_predictor_stats_scores_prediction_against_naive() -> None:
    st = PredictorStats(period_s=DT)
    lag = 0.2
    for k in range(200):
        t = k * DT
        v = 10.0 + 4.0 * t  # 一定加速 4 km/h/s
        st.add_cycle(t=t, can_kmh=v, est_kmh=v + 0.01)
        st.add_prediction(t=t, lag_s=lag, pred_kmh=v + 4.0 * lag, can_kmh=v)
    assert st.pred_n > 150 and st.pred_rms == pytest.approx(0.0, abs=1e-9)
    assert st.naive_rms == pytest.approx(4.0 * lag, abs=1e-9)
    assert st.est_rms == pytest.approx(0.01)
    text = st.summary()
    assert "予測なし" in text and "RMS" in text and "比 0.00" in text


# ─── C13 の候補・config ──────────────────────────────────────────────────


def test_make_candidate_c13_is_a_c12_with_predictor_settings() -> None:
    ff = make_candidate("C13")
    assert isinstance(ff, PredictorFeedforward) and isinstance(ff, CoastAssistFeedforward)
    assert ff.candidate == "C13" and ff.expected_kind == "model"
    assert ff.predictor_enabled is True and ff.filter_q is None and ff.filter_r is None
    ff.set_predictor(enabled=False, q=0.05, r=2e-4, r_floor=1e-5, accel_sigma=0.3)
    assert (ff.predictor_enabled, ff.filter_q, ff.filter_r) == (False, 0.05, 2e-4)
    assert (ff.filter_r_floor, ff.filter_accel_sigma) == (1e-5, 0.3)


def test_config_c13_defaults_yaml_and_validation(tmp_path: Path) -> None:
    fm = cfgmod.ForwardMapSection()
    assert fm.predictor_enabled is True and fm.speed_filter_q is None and fm.speed_filter_r is None
    assert fm.speed_filter_r_floor == pytest.approx(1e-4) and fm.speed_filter_accel_sigma == 0.2
    assert "C13" in cfgmod.CANDIDATE_NAMES
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.feedforward.candidate = "C13"
    assert not any("candidate" in p for p in cfgmod.validate_config(cfg))
    # yaml の数値で上書き（null は自動）
    text = cfgmod.DEFAULT_CONFIG_PATH.read_text(encoding="utf-8")
    path = tmp_path / "c13.yaml"
    path.write_text(
        text.replace("forward_map:\n",
                     "forward_map:\n  speed_filter_q: 0.09\n  speed_filter_r: 0.0002\n"
                     "  predictor_enabled: false\n", 1)
        .replace("  speed_filter_q: null\n", "").replace("  speed_filter_r: null\n", "")
        .replace("  predictor_enabled: true\n", ""),
        encoding="utf-8",
    )
    loaded = cfgmod.load_config(path)
    assert loaded.forward_map.speed_filter_q == 0.09 and loaded.forward_map.speed_filter_r == 0.0002
    assert loaded.forward_map.predictor_enabled is False
    for key, bad in (("speed_filter_q", 0.0), ("speed_filter_r", -1.0),
                     ("speed_filter_r_floor", 0.0), ("speed_filter_accel_sigma", 0.0)):
        c = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
        setattr(c.forward_map, key, bad)
        assert any(f"forward_map.{key}" in p for p in cfgmod.validate_config(c))
