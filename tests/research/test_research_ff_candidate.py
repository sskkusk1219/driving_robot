"""改善案 C1（ff_candidate.py）のユニットテスト。

A1（学習行の選別）はそのペダルが効いている行だけを使うこと、B1〜B3（レジーム合成）は
停車保持・クリープ任せを保ったまま惰行カーブでペダルを選び、不感帯以上へ切り上げること、
そして `kaizen.decide_openings`（レポート 3 章の C1 と同じ判定）と一致することを確かめる。
"""

from __future__ import annotations

import pickle
from datetime import UTC, datetime, timedelta
from pathlib import Path

import numpy as np
import pytest

from src.domain.control.feedforward import FeedforwardController
from src.domain.learning_drive import LearningDataError
from src.domain.model_training import DEFAULT_FEATURE_SPEC, STOP_SPEED_KMH
from src.models.drive_log import DriveLog
from src.models.profile import (
    FeedforwardParams,
    PIDGains,
    StopConfig,
    VehicleProfile,
    coast_decel_at,
)
from tests.research.cruise_curve import CruiseCurve
from tests.research.ff_candidate import (
    CANDIDATE_CLASSES,
    TRAINING_ROWS_EFFECTIVE,
    CandidateC2,
    CandidateC3,
    CandidateC4,
    CandidateC5,
    CandidateC6,
    CandidateFeedforward,
    cruise_skeleton,
    make_candidate,
    train_inverse_model_effective,
)

SPEEDS = (5.0, 15.0, 25.0, 35.0, 45.0, 55.0, 65.0, 75.0, 85.0, 95.0, 105.0, 115.0, 125.0, 135.0)
COAST = (1.6, 1.73, 2.58, 3.195, 3.575, 3.57, 3.11, 2.54, 1.83, 1.6, 1.6, 1.6, 1.6, 1.6)
PARAMS = FeedforwardParams(
    creep_speed_kmh=4.944,
    creep_rate_kmhs=0.155,
    stop_brake_opening_pct=30.53,
    coast_decel_speeds_kmh=SPEEDS,
    coast_decel_kmhs=COAST,
    accel_deadband_pct=10.0,
    brake_deadband_pct=13.68,
)
SPEC = DEFAULT_FEATURE_SPEC


class _Const:
    """与えた定数を返すだけの推定器（predict_effort の分岐だけを見るため）。"""

    def __init__(self, value: float) -> None:
        self.value = value

    def predict(self, x: np.ndarray) -> np.ndarray:
        return np.full(len(x), self.value, dtype=float)


def _ff(accel_pred: float, brake_pred: float, clip: float | None = None) -> CandidateFeedforward:
    ff = CandidateFeedforward()
    ff.set_params(PARAMS)
    ff._accel_model = _Const(accel_pred)  # noqa: SLF001 - テスト用の差し込み
    ff._brake_model = _Const(brake_pred)  # noqa: SLF001
    ff._speed_clip_max = clip  # noqa: SLF001
    return ff


def _points(v0: float, a: float) -> tuple[list[float], list[float]]:
    """一定加速度 a [km/h/s] の軌跡（先読み・過去）。"""
    return (
        [v0 + a * h for h in SPEC.lookahead_horizons_s],
        [v0 - a * h for h in SPEC.past_horizons_s],
    )


# ── B1〜B3: レジーム合成 ────────────────────────────────────────────


def test_unloaded_model_raises() -> None:
    ff = CandidateFeedforward()
    ff.set_params(PARAMS)
    with pytest.raises(RuntimeError):
        ff.predict_effort(50.0, *_points(50.0, 0.0))


def test_stop_regime_holds_brake() -> None:
    """停車判定は現行のまま（停車保持ブレーキ開度をそのまま出す）。"""
    ff = _ff(20.0, 20.0)
    future, past = _points(0.0, 0.0)
    assert ff.predict_effort(0.0, future, past) == pytest.approx(-PARAMS.stop_brake_opening_pct)


def test_creep_regime_releases_both_pedals() -> None:
    """クリープ能力内の加速要求はペダル不要（現行のまま。B3 の切り上げも効かせない）。"""
    ff = _ff(20.0, 20.0)
    v0 = PARAMS.creep_speed_kmh - 0.5
    future, past = _points(v0, PARAMS.creep_rate_kmhs * 0.5)
    assert v0 > STOP_SPEED_KMH
    assert ff.predict_effort(v0, future, past) == 0.0


def test_b1_mild_decel_above_coast_selects_accel() -> None:
    """B1: 惰行より緩い減速はアクセル側（現行は dv_1.0 が負なのでブレーキ側に落ちていた）。"""
    ff = _ff(16.0, 20.0)
    v0 = 60.0
    coast = -coast_decel_at(PARAMS, v0)  # 約 -3.3 km/h/s
    future, past = _points(v0, coast / 2.0)  # 惰行の半分の減速 = 惰行より緩い
    assert ff.predict_effort(v0, future, past) == pytest.approx(16.0)


def test_b1_decel_stronger_than_coast_selects_brake() -> None:
    ff = _ff(16.0, 20.0)
    v0 = 60.0
    coast = -coast_decel_at(PARAMS, v0)
    future, past = _points(v0, coast * 1.5)  # 惰行より強い減速
    assert ff.predict_effort(v0, future, past) == pytest.approx(-20.0)


def test_b2_no_coast_taper() -> None:
    """B2: 惰行テーパを廃止したので、緩減速でもアクセル予測が絞られない。"""
    v0, accel_pred = 60.0, 16.0
    coast = -coast_decel_at(PARAMS, v0)
    future, past = _points(v0, coast * 0.9)  # 現行なら accel_pred × 0.1 まで絞られる領域
    assert _ff(accel_pred, 20.0).predict_effort(v0, future, past) == pytest.approx(accel_pred)
    # 現行（FeedforwardController）はここでテーパがかかることを対照として確かめる
    current = FeedforwardController()
    current.set_params(PARAMS)
    current._accel_model = _Const(accel_pred)  # noqa: SLF001
    current._brake_model = _Const(20.0)  # noqa: SLF001
    assert current.predict_effort(v0, future, past) == pytest.approx(accel_pred * 0.1, abs=0.2)


@pytest.mark.parametrize(
    ("accel_pred", "brake_pred", "a", "expected"),
    [
        (2.0, 0.0, 1.0, PARAMS.accel_deadband_pct),  # 不感帯未満のアクセル予測 → 不感帯へ
        (0.0, 2.0, -8.0, -PARAMS.brake_deadband_pct),  # 不感帯未満のブレーキ予測 → 不感帯へ
        (24.0, 0.0, 1.0, 24.0),  # 不感帯以上はそのまま
        (0.0, 30.0, -8.0, -30.0),
    ],
)
def test_b3_rounds_up_to_deadband(
    accel_pred: float, brake_pred: float, a: float, expected: float
) -> None:
    ff = _ff(accel_pred, brake_pred)
    assert ff.predict_effort(60.0, *_points(60.0, a)) == pytest.approx(expected)


def test_negative_prediction_is_clamped_then_rounded_up() -> None:
    """モデルが負を返しても 0 クランプ後に不感帯へ切り上げる（効かない指令を出さない）。"""
    assert _ff(-5.0, 0.0).predict_effort(60.0, *_points(60.0, 1.0)) == pytest.approx(
        PARAMS.accel_deadband_pct
    )


def test_speed_clip_shifts_trajectory() -> None:
    """学習域クリップは現行のまま（v0 を上限へ置き、先読み/過去を平行移動して dv を保つ）。"""
    ff = _ff(30.0, 40.0, clip=100.0)
    future, past = _points(130.0, 2.0)
    assert ff.predict_effort(130.0, future, past) == pytest.approx(30.0)  # 加速要求は保たれる


# ── kaizen.decide_openings（レポート 3 章の C1）との一致 ──────────────


def test_matches_kaizen_decide_openings() -> None:
    """レポート 3 章で比べた C1 の判定と、走行に使う predict_effort が一致すること。"""
    from tests.research.config import load_config  # noqa: PLC0415 - 設定は重いので中で読む
    from tests.research.kaizen import decide_openings  # noqa: PLC0415

    cfg = load_config(Path("tests/research/config_testVehicle.yaml"))
    accel_pred, brake_pred = 16.0, 20.0
    ff = _ff(accel_pred, brake_pred)

    v0s = np.array([0.0, 2.0, 4.0, 6.0, 20.0, 60.0, 100.0, 130.0])
    accels = np.array([-8.0, -3.0, -1.0, -0.1, 0.0, 0.1, 1.0, 3.0])
    rows = [(float(v), float(a)) for v in v0s for a in accels]

    v0_arr = np.array([v for v, _ in rows])
    a_arr = np.array([a for _, a in rows])
    near = np.array([v + a * SPEC.lookahead_horizons_s[0] for v, a in rows])
    accel_ref, brake_ref = decide_openings(
        PARAMS, cfg, v0_arr, near, v0_arr, a_arr,
        np.full(len(rows), accel_pred), np.full(len(rows), brake_pred),
    )
    for i, (v, a) in enumerate(rows):
        effort = ff.predict_effort(v, *_points(v, a))
        got_accel = max(0.0, effort)
        got_brake = max(0.0, -effort)
        assert got_accel == pytest.approx(accel_ref[i]), f"v0={v} a={a}"
        assert got_brake == pytest.approx(brake_ref[i]), f"v0={v} a={a}"


# ── A1: 学習行の選別 ───────────────────────────────────────────────


def _logs(rows: list[tuple[float, float, float]]) -> list[DriveLog]:
    """(車速, アクセル開度, ブレーキ開度) の列から 0.1s 刻みのログを作る。"""
    t0 = datetime(2026, 9, 12, tzinfo=UTC)
    return [
        DriveLog(
            id=i, session_id="s1", timestamp=t0 + timedelta(seconds=0.1 * i),
            ref_speed_kmh=None, actual_speed_kmh=v,
            accel_opening=a, brake_opening=b,
            accel_pos=0, brake_pos=0, accel_current=0.0, brake_current=0.0,
        )
        for i, (v, a, b) in enumerate(rows)
    ]


def _profile() -> VehicleProfile:
    now = datetime(2026, 9, 12, tzinfo=UTC)
    return VehicleProfile(
        id="unit_test", name="unit_test", max_speed=140.0, max_decel_g=0.4,
        max_accel_opening=80.0, max_brake_opening=80.0,
        pid_gains=PIDGains(kp=0.0, ki=0.0, kd=0.0),
        stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=9999.0),
        calibration=None, model_path=None,
        created_at=now, updated_at=now, feedforward_params=PARAMS,
    )


def _mixed_rows(n: int = 300) -> list[tuple[float, float, float]]:
    """加速（アクセル 20%）・惰行（両方 0）・減速（ブレーキ 20%）を混ぜた合成走行。"""
    rows: list[tuple[float, float, float]] = []
    v = 5.0
    for i in range(n):
        if i % 3 == 0:
            v += 0.5
            rows.append((v, 20.0, 0.0))
        elif i % 3 == 1:
            v = max(1.0, v - 0.2)
            rows.append((v, 0.0, 0.0))  # 惰行（どちらのモデルにも入らない）
        else:
            v = max(1.0, v - 0.5)
            rows.append((v, 0.0, 20.0))
    return rows


def test_a1_uses_only_effective_rows(tmp_path: Path) -> None:
    logs = _logs(_mixed_rows())
    path, metrics = train_inverse_model_effective(logs, _profile(), output_dir=str(tmp_path))
    assert Path(path).exists()
    # 惰行の行（開度 0）はどちらのモデルにも入らない = 全行の 1/3 ずつが上限
    assert 0 < metrics["accel"]["n"] <= len(logs) / 3 + 1
    assert 0 < metrics["brake"]["n"] <= len(logs) / 3 + 1
    assert "below_deadband" in metrics["accel"]
    assert "below_deadband" in metrics["brake"]


def test_a1_model_loads_into_feedforward(tmp_path: Path) -> None:
    """pkl の形式は本番と同じ（FeedforwardController.load_model がそのまま読める）。"""
    path, _ = train_inverse_model_effective(_logs(_mixed_rows()), _profile(), str(tmp_path))
    ff = CandidateFeedforward()
    ff.set_params(PARAMS)
    ff.load_model(path)
    assert ff.has_model
    with Path(path).open("rb") as f:
        payload = pickle.load(f)  # noqa: S301 - テストで作った自前のファイル
    assert payload["training_rows"] == TRAINING_ROWS_EFFECTIVE
    assert payload["deadbands_pct"] == {"accel": 10.0, "brake": 13.68}


def test_a1_raises_when_a_pedal_has_no_effective_rows(tmp_path: Path) -> None:
    """ブレーキが不感帯を超えない走行では、ブレーキモデルが作れないことを明示する。"""
    rows = [(5.0 + 0.1 * i, 20.0, 5.0) for i in range(300)]  # ブレーキは常に不感帯未満
    with pytest.raises(LearningDataError, match="ブレーキ"):
        train_inverse_model_effective(_logs(rows), _profile(), output_dir=str(tmp_path))


# ── V1: C2〜C5（KAIZEN 報告書 3 章 表 3-1） ─────────────────────────


GAIN_SPEEDS = (5.0, 60.0, 130.0)
ACCEL_GAIN = (0.5, 0.5, 0.5)
BRAKE_GAIN = (0.4, 0.4, 0.4)
PARAMS_WITH_GAIN = FeedforwardParams(
    creep_speed_kmh=PARAMS.creep_speed_kmh,
    creep_rate_kmhs=PARAMS.creep_rate_kmhs,
    stop_brake_opening_pct=PARAMS.stop_brake_opening_pct,
    coast_decel_speeds_kmh=SPEEDS,
    coast_decel_kmhs=COAST,
    accel_deadband_pct=10.0,
    brake_deadband_pct=13.68,
    pedal_gain_speeds_kmh=GAIN_SPEEDS,
    accel_gain_kmhs_per_pct=ACCEL_GAIN,
    brake_gain_kmhs_per_pct=BRAKE_GAIN,
)


def test_candidate_classes_registered_with_matching_names() -> None:
    for name, cls in CANDIDATE_CLASSES.items():
        assert cls().candidate == name
    assert set(CANDIDATE_CLASSES) == {"C1", "C2", "C3", "C4", "C5", "C6"}


def test_make_candidate_unknown_name_raises() -> None:
    with pytest.raises(ValueError, match="C9"):
        make_candidate("C9")


def test_c3_is_logic_identical_to_c1_except_name() -> None:
    """C3 はロジックが C1 と同一（ずれはモデルの horizons で表現するので候補名だけ違う）。"""
    v0 = 60.0
    coast = -coast_decel_at(PARAMS, v0)
    future, past = _points(v0, coast * 1.5)  # 惰行より強い減速 → ブレーキ側

    c1 = _ff(16.0, 20.0)
    c3 = CandidateC3()
    c3.set_params(PARAMS)
    c3._accel_model = c1._accel_model  # noqa: SLF001
    c3._brake_model = c1._brake_model  # noqa: SLF001

    assert c3.candidate == "C3"
    assert not c3.uses_actual_speed
    assert c3.predict_effort(v0, future, past) == pytest.approx(
        c1.predict_effort(v0, future, past)
    )


def test_c4_and_c5_flag_actual_speed_use() -> None:
    assert CandidateC4().uses_actual_speed
    assert CandidateC5().uses_actual_speed
    assert not CandidateFeedforward().uses_actual_speed
    assert not CandidateC2().uses_actual_speed
    assert not CandidateC3().uses_actual_speed


def _c2(accel_pred: float, brake_pred: float, params: FeedforwardParams) -> CandidateC2:
    ff = CandidateC2()
    ff.set_params(params)
    ff._accel_model = _Const(accel_pred)  # noqa: SLF001
    ff._brake_model = _Const(brake_pred)  # noqa: SLF001
    return ff


def test_c2_brake_uses_analytic_formula_when_gain_identified() -> None:
    v0 = 60.0
    coast = -coast_decel_at(PARAMS_WITH_GAIN, v0)  # 惰行加速度（負）
    a_req = coast * 1.5  # 惰行より強い減速 → ブレーキ側
    future, past = _points(v0, a_req)
    ff = _c2(16.0, 999.0, PARAMS_WITH_GAIN)  # brake_pred はモデル予測（使われないはず）

    effort = ff.predict_effort(v0, future, past)

    delta_a = a_req - coast
    gain = 0.4  # BRAKE_GAIN の一定値
    expected = -(-delta_a / gain + PARAMS_WITH_GAIN.brake_deadband_pct)
    assert effort == pytest.approx(expected)
    assert effort != pytest.approx(-999.0)  # モデル予測（C1 の分岐）ではない


def test_c2_falls_back_to_model_when_gain_not_identified() -> None:
    """ペダルゲイン未同定（PARAMS はゲイン曲線を持たない）は C1 と同じ挙動。"""
    v0 = 60.0
    coast = -coast_decel_at(PARAMS, v0)
    future, past = _points(v0, coast * 1.5)
    c1 = _ff(16.0, 20.0)
    c2 = _c2(16.0, 20.0, PARAMS)
    assert c2.predict_effort(v0, future, past) == pytest.approx(
        c1.predict_effort(v0, future, past)
    )


# ── C6: 骨格（定速階段の実測テーブル）+ 残差 ML ─────────────────────


def _c6_curve() -> CruiseCurve:
    return CruiseCurve(
        speeds_kmh=(30.0, 60.0, 90.0), openings_pct=(12.65, 16.96, 16.82), n_rows=(6, 6, 6)
    )


def test_cruise_skeleton_adds_gain_term_when_identified() -> None:
    """骨格 = opening_at(v0) + a_req ÷ k(v0)（ゲイン同定済み）。"""
    curve = _c6_curve()
    skeleton = cruise_skeleton(curve, PARAMS_WITH_GAIN, 60.0, 1.0)
    assert skeleton == pytest.approx(16.96 + 1.0 / 0.5)  # ACCEL_GAIN の一定値 0.5


def test_cruise_skeleton_a_req_term_is_zero_when_gain_not_identified() -> None:
    """ゲイン未同定（None）・0 以下なら a_req 項は 0（実測テーブルの値だけを返す）。"""
    curve = _c6_curve()
    assert cruise_skeleton(curve, PARAMS, 60.0, 1.0) == pytest.approx(16.96)
    assert cruise_skeleton(curve, PARAMS, 60.0, -3.0) == pytest.approx(16.96)


def _sample_curve_for_training() -> CruiseCurve:
    """学習ログの車速レンジ（_mixed_rows は概ね 1〜60 km/h）を覆う小さな実測テーブル。"""
    return CruiseCurve(speeds_kmh=(10.0, 50.0), openings_pct=(12.0, 16.0), n_rows=(5, 5))


def test_train_inverse_model_effective_with_curve_writes_cruise_curve_key(tmp_path: Path) -> None:
    curve = _sample_curve_for_training()
    path, metrics = train_inverse_model_effective(
        _logs(_mixed_rows()), _profile(), output_dir=str(tmp_path), cruise_curve=curve
    )
    with Path(path).open("rb") as f:
        payload = pickle.load(f)  # noqa: S301 - テストで作った自前のファイル
    assert payload["cruise_curve"] == curve.to_dict()
    assert "below_deadband" in metrics["accel"]
    assert metrics["accel"]["n"] > 0


def test_train_inverse_model_effective_without_curve_has_no_cruise_curve_key(
    tmp_path: Path,
) -> None:
    """C1（cruise_curve 省略）の pkl には研究用キーが増えないこと。"""
    path, _ = train_inverse_model_effective(
        _logs(_mixed_rows()), _profile(), output_dir=str(tmp_path)
    )
    with Path(path).open("rb") as f:
        payload = pickle.load(f)  # noqa: S301
    assert "cruise_curve" not in payload


def test_candidate_c6_load_model_rejects_c1_pkl(tmp_path: Path) -> None:
    """C1 の pkl（cruise_curve キーなし）を C6 に読ませたら ValueError（黙って動かさない）。"""
    path, _ = train_inverse_model_effective(
        _logs(_mixed_rows()), _profile(), output_dir=str(tmp_path)
    )
    ff = CandidateC6()
    ff.set_params(PARAMS)
    with pytest.raises(ValueError, match="cruise_curve"):
        ff.load_model(path)


def test_candidate_c6_load_model_reads_cruise_curve(tmp_path: Path) -> None:
    curve = _sample_curve_for_training()
    path, _ = train_inverse_model_effective(
        _logs(_mixed_rows()), _profile(), output_dir=str(tmp_path), cruise_curve=curve
    )
    ff = CandidateC6()
    ff.set_params(PARAMS)
    ff.load_model(path)
    assert ff.has_model
    assert ff._cruise_curve == curve  # noqa: SLF001 - テスト用の確認


def _c6(
    residual: float, brake_pred: float, curve: CruiseCurve, clip: float | None = None
) -> CandidateC6:
    ff = CandidateC6()
    ff.set_params(PARAMS)
    ff._accel_model = _Const(residual)  # noqa: SLF001
    ff._brake_model = _Const(brake_pred)  # noqa: SLF001
    ff._speed_clip_max = clip  # noqa: SLF001
    ff._cruise_curve = curve  # noqa: SLF001
    return ff


def test_c6_accel_prediction_is_skeleton_plus_residual() -> None:
    """C6 のアクセル予測 = max(0, 骨格 + 残差)。B3 の不感帯切り上げは C1 と同じ。"""
    curve = _c6_curve()
    residual = 2.0
    ff = _c6(residual, brake_pred=0.0, curve=curve)
    v0 = 60.0  # 表の点そのもの（16.96%）
    future, past = _points(v0, 1.0)  # 加速要求 → アクセル側（B1）

    effort = ff.predict_effort(v0, future, past)

    expected_skeleton = 16.96  # ゲイン未同定の PARAMS なので a_req 項は 0
    assert effort == pytest.approx(max(PARAMS.accel_deadband_pct, expected_skeleton + residual))


def test_c6_accel_opening_uses_raw_v0_for_skeleton_above_speed_clip() -> None:
    """骨格の v0 は学習域クリップ前の実測値（表の外は opening_at の直線延長で外挿する）。"""
    curve = _c6_curve()
    ff = _c6(residual=0.0, brake_pred=0.0, curve=curve, clip=50.0)
    v0 = 100.0  # 表の最高速 90 を超え、かつ学習域クリップ (50) も超える
    future, past = _points(v0, 1.0)

    effort = ff.predict_effort(v0, future, past)

    high_slope = (16.82 - 16.96) / (90.0 - 60.0)
    expected_skeleton = 16.82 + high_slope * (v0 - 90.0)  # v0=100（クリップ後の 50 ではない）
    assert effort == pytest.approx(max(PARAMS.accel_deadband_pct, expected_skeleton))


def test_make_candidate_c6() -> None:
    ff = make_candidate("C6")
    assert isinstance(ff, CandidateC6)
    assert ff.candidate == "C6"
    assert not ff.uses_actual_speed
