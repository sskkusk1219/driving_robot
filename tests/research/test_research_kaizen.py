"""改善案の比較解析（kaizen.py）のユニットテスト。

DB・実走行ログを使わない部分（再生の総当たり・最小の組の選び方・グループ別の偏差・閉ループ模擬の
指令の受け渡し）を合成データで確かめる。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from tests.research import kaizen
from tests.research import vehicle_sim as vs
from tests.research.research_types import FeedforwardParams

PARAMS = FeedforwardParams(
    creep_speed_kmh=4.944,
    creep_rate_kmhs=0.155,
    coast_decel_speeds_kmh=(5.0, 135.0),
    coast_decel_kmhs=(2.0, 2.0),
    accel_deadband_pct=10.0,
    brake_deadband_pct=13.68,
)


def _model(name: str, gain: float) -> vs.VehicleModel:
    resp = vs.proportional_response(name, (0.0, 140.0), (gain, gain))
    return vs.VehicleModel(name, PARAMS, resp, resp)


def _log(name: str) -> vs.LogSeries:
    """アクセル 14% を 10s 保持 → 離して 10s（60 km/h から）。"""
    accel = np.concatenate([np.full(100, 14.0), np.zeros(100)])
    brake = np.zeros(200)
    model = _model("真値", 0.5)
    speed = np.empty(200)
    v = 60.0
    for i in range(200):
        speed[i] = v
        a = float(model.coast_accel(v)) + float(model.pedal_accel(v, accel[i], brake[i]))
        v = max(0.0, v + a * vs.LOG_DT_S)
    return vs.LogSeries(name, np.arange(200) * vs.LOG_DT_S, speed, accel, brake)


def test_replay_grid_covers_every_combination() -> None:
    models = [_model("A", 0.5), _model("B", 0.3)]
    logs = [_log("log1"), _log("log2")]
    rows = kaizen.replay_grid(models, logs, delays=(0.0, 0.2), lags=(0.0,))
    assert len(rows) == 2 * 2 * 1 * 2
    assert {r.model for r in rows} == {"A", "B"}
    assert {r.log for r in rows} == {"log1", "log2"}


def test_best_row_picks_lowest_rmse() -> None:
    """ゲイン 0.5 の合成ログには、同じゲインのモデルが当たる。"""
    models = [_model("合う", 0.5), _model("合わない", 0.2)]
    rows = kaizen.replay_grid(models, [_log("log1")], delays=(0.0,), lags=(0.0,))
    assert kaizen.best_row(rows, "合う", "log1").rmse < 1e-9
    assert kaizen.best_row(rows, "合わない", "log1").rmse > 1.0
    with pytest.raises(ValueError):
        kaizen.best_row(rows, "無い", "log1")


def test_deviation_by_group_skips_times_after_simulation_stopped() -> None:
    keys = np.array(["Low", "Low", "Mid", "Mid"])
    real = np.array([1.0, 3.0, -2.0, -4.0])
    sim = np.array([2.0, 4.0, -1.0, np.nan])  # 模擬は最後の行の前に打ち切られた
    groups = kaizen.deviation_by_group(keys, ("Low", "Mid", "High"), real, [sim], dt=0.1)
    assert [g.name for g in groups] == ["Low", "Mid"]  # 行の無いグループは出さない
    low, mid = groups
    assert low.duration_s == pytest.approx(0.2)
    assert low.real_mean == pytest.approx(2.0)
    assert low.sim_means[0] == pytest.approx(3.0)
    assert mid.real_mean == pytest.approx(-3.0)
    assert mid.sim_means[0] == pytest.approx(-1.0)  # NaN の行は数えない


def test_deviation_by_group_reports_nan_when_no_overlap() -> None:
    keys = np.array(["Low"])
    groups = kaizen.deviation_by_group(
        keys, ("Low",), np.array([1.0]), [np.array([np.nan])], dt=0.1
    )
    assert np.isnan(groups[0].sim_means[0])


def test_run_closed_loop_uses_commands_per_cycle() -> None:
    cmd = kaizen.FFCommands(
        t=np.arange(40) * vs.SIM_DT_S,
        ref=np.full(40, 60.0),
        accel=np.concatenate([np.full(20, 20.0), np.zeros(20)]),
        brake=np.zeros(40),
    )
    run = kaizen.run_closed_loop(_model("A", 0.5), cmd, stop_above_kmh=200.0)
    assert run.stopped_at_s is None
    assert list(run.accel[:3]) == [20.0, 20.0, 20.0]
    assert run.speed[20] > run.speed[0]  # 踏んでいる間は加速
    assert run.speed[-1] < run.speed[20]  # 離したら惰行で減速


def _cfg() -> object:
    """decide_openings が見る設定だけを持つ最小の入れ物。"""

    class Vehicle:
        max_accel_opening_pct = 80.0
        max_brake_opening_pct = 80.0

    class Cfg:
        vehicle = Vehicle()

    return Cfg()


def _decide(v0: float, a_req: float, accel_pred: float, brake_pred: float,
            v0_raw: float | None = None, near: float | None = None) -> tuple[float, float]:
    accel, brake = kaizen.decide_openings(
        PARAMS, _cfg(),
        np.array([v0 if v0_raw is None else v0_raw]),
        np.array([v0 if near is None else near]),
        np.array([v0]), np.array([a_req]), np.array([accel_pred]), np.array([brake_pred]),
    )
    return float(accel[0]), float(brake[0])


def test_coast_accel_array_matches_production_rule() -> None:
    v = np.array([0.0, 3.0, 60.0])
    got = kaizen.coast_accel_array(PARAMS, v)
    assert float(got[0]) == pytest.approx(PARAMS.creep_rate_kmhs)  # クリープ域は押す側
    assert float(got[1]) == pytest.approx(PARAMS.creep_rate_kmhs)
    assert float(got[2]) == pytest.approx(-2.0)  # 惰行減速カーブ（この合成では一定 2.0）


def test_decide_openings_holds_at_stop() -> None:
    assert _decide(0.0, 0.0, 5.0, 0.0, v0_raw=0.0, near=0.0) == (0.0, PARAMS.stop_brake_opening_pct)


def test_decide_openings_leaves_creep_alone() -> None:
    """クリープ車速未満で、クリープ加速率に収まる加速要求はペダルを踏まない。"""
    assert _decide(3.0, 0.1, 12.0, 0.0, v0_raw=3.0, near=3.5) == (0.0, 0.0)


def test_decide_openings_chooses_accel_for_gentle_deceleration() -> None:
    """惰行（−2.0）より緩い減速はアクセル側。予測が浅くても不感帯以上に切り上げる。"""
    accel, brake = _decide(60.0, -1.0, 3.0, 40.0)
    assert accel == pytest.approx(PARAMS.accel_deadband_pct)
    assert brake == 0.0


def test_decide_openings_chooses_brake_below_coast() -> None:
    accel, brake = _decide(60.0, -4.0, 20.0, 5.0)
    assert accel == 0.0
    assert brake == pytest.approx(PARAMS.brake_deadband_pct)  # 切り上げ


def test_decide_openings_clamps_to_max_opening() -> None:
    accel, _ = _decide(60.0, 3.0, 120.0, 0.0)
    assert accel == pytest.approx(80.0)


def test_ref_frames_shift_moves_the_lookahead_window() -> None:
    from datetime import UTC, datetime

    from tests.research.research_types import DrivingMode, SpeedPoint

    mode = DrivingMode(
        id="x", name="ramp", description="", total_duration=20.0, max_speed=20.0,
        created_at=datetime.now(tz=UTC), is_system=False,
        reference_speed=[SpeedPoint(time_s=0.0, speed_kmh=0.0),
                         SpeedPoint(time_s=20.0, speed_kmh=20.0)],  # 1 km/h/s のランプ
    )
    ref = kaizen.ReferenceSpeed(mode)
    t = np.array([5.0, 6.0])
    plain = kaizen.ref_frames(ref, t, kaizen.DEFAULT_FEATURE_SPEC)
    shifted = kaizen.ref_frames(ref, t, kaizen.DEFAULT_FEATURE_SPEC, 0.5)
    assert float(plain.v0_model[0]) == pytest.approx(5.0)
    assert float(shifted.v0_model[0]) == pytest.approx(5.5)  # ref(t + 0.5)
    assert float(plain.v0_raw[0]) == pytest.approx(5.0)  # 停車判定は t のまま
    assert float(shifted.v0_raw[0]) == pytest.approx(5.0)
    assert float(plain.a_req[0]) == pytest.approx(1.0)  # ランプなのでどちらも 1 km/h/s
    assert float(shifted.a_req[0]) == pytest.approx(1.0)


def test_array_command_returns_precomputed_openings() -> None:
    command = kaizen.array_command(np.array([1.0, 2.0]), np.array([0.0, 3.0]))
    assert command(0, 50.0) == (1.0, 0.0)
    assert command(1, 99.0) == (2.0, 3.0)


def test_run_closed_loop_stops_above_limit() -> None:
    cmd = kaizen.FFCommands(
        t=np.arange(200) * vs.SIM_DT_S,
        ref=np.full(200, 130.0),
        accel=np.full(200, 60.0),
        brake=np.zeros(200),
    )
    run = kaizen.run_closed_loop(_model("A", 0.5), cmd, stop_above_kmh=140.0)
    assert run.stopped_at_s is not None
    assert run.end_s == pytest.approx(run.stopped_at_s)


class _Recorder:
    """渡された特徴行列を覚えて、指定列に比例した値を返すだけのモデル。"""

    def __init__(self, col: int, gain: float) -> None:
        self.col, self.gain = col, gain
        self.seen: np.ndarray | None = None

    def predict(self, x: np.ndarray) -> np.ndarray:
        self.seen = x
        return x[:, self.col] * self.gain


def _stub_models() -> tuple[kaizen.TrainedModels, _Recorder]:
    """アクセル予測 = 5 × dv_1.0 のモデル（モデルに渡された特徴量を見るため）。"""
    from tests.research.ff_explain import FFModel

    spec = kaizen.DEFAULT_FEATURE_SPEC
    rec = _Recorder(spec.regime_col(), 5.0)
    ff = FFModel(path="stub", accel_model=rec, brake_model=_Recorder(spec.regime_col(), 0.0),
                 spec=spec, speed_clip_max=None)
    models = kaizen.TrainedModels(label="stub", ff=ff, shift_s=0.0, rows=(1, 1),
                                  mae=(0.0, 0.0), below_db=(0.0, 0.0))
    return models, rec


def _ramp_frames(n: int = 40) -> kaizen.RefFrames:
    """40 km/h から 1 km/h/s で上がる基準（停車・クリープの分岐に入らない領域）。"""
    from datetime import UTC, datetime

    from tests.research.research_types import DrivingMode, SpeedPoint

    mode = DrivingMode(
        id="x", name="ramp", description="", total_duration=60.0, max_speed=100.0,
        created_at=datetime.now(tz=UTC), is_system=False,
        reference_speed=[SpeedPoint(time_s=0.0, speed_kmh=40.0),
                         SpeedPoint(time_s=60.0, speed_kmh=100.0)],
    )
    return kaizen.ref_frames(
        kaizen.ReferenceSpeed(mode), np.arange(n) * vs.SIM_DT_S, kaizen.DEFAULT_FEATURE_SPEC
    )


def test_replan_command_uses_absolute_future_and_measured_past() -> None:
    """C5: 未来は ref(t+h) の絶対値、過去は実車速の履歴から作る。"""
    models, rec = _stub_models()
    frames = _ramp_frames()
    cmd = kaizen.ReplanCommand(models, PARAMS, _cfg(), frames, [])
    for i in range(31):
        cmd(i, 50.0)  # 実車速は 50 km/h で一定（基準は 40 → 41.5 km/h へ上がる）
    seen = rec.seen
    assert seen is not None
    want = float(frames.v0_model[30] + frames.dv_future[30][1]) - 50.0
    assert float(seen[0, kaizen.DEFAULT_FEATURE_SPEC.regime_col()]) == pytest.approx(want)
    # 過去は実測（一定なので 0）。基準から作っていれば 0 にはならない
    assert float(seen[0, 7]) == pytest.approx(0.0)
    assert float(seen[0, 8]) == pytest.approx(0.0)
    assert float(frames.dv_past[30][0]) > 0.1


def test_replan_command_falls_back_to_reference_past_at_start() -> None:
    """履歴が足りない開始直後は、過去の変化量を基準から代用する。"""
    models, rec = _stub_models()
    frames = _ramp_frames()
    cmd = kaizen.ReplanCommand(models, PARAMS, _cfg(), frames, [])
    cmd(5, 50.0)
    seen = rec.seen
    assert seen is not None
    assert float(seen[0, 7]) == pytest.approx(float(frames.dv_past[5][0]))
    assert float(seen[0, 8]) == pytest.approx(float(frames.dv_past[5][1]))


def test_replan_command_opens_more_than_c4_when_behind() -> None:
    """遅れているとき C5 は要求 Δv が増えて開度が増える（C4 は増分だけなので増えない）。"""
    frames = _ramp_frames()
    c5 = kaizen.ReplanCommand(_stub_models()[0], PARAMS, _cfg(), frames, [])
    c4 = kaizen.ActualSpeedCommand(_stub_models()[0], PARAMS, _cfg(), frames)
    accel5 = 0.0
    for i in range(31):
        accel5, _ = c5(i, float(frames.v0_model[i]) - 5.0)
    accel4, _ = c4(30, float(frames.v0_model[30]) - 5.0)
    assert accel5 > accel4


# ─── 学習データ（パターン走行 CSV）を使う分析 ───
# （--part coverage は 2026-09-25 段4 で削除した。網羅マップは test_research_wltp_grid.py）


def _write_train_csv(path: Path) -> Path:
    """パターン走行の最小 CSV（3 系統・0.1s 刻み）。

    1:ACCEL_SWEEP   100 行 アクセル 24%（DRIVE_ACCEL・9.9s）
    2:BRAKE_HOLD    201 行 ブレーキ 20%（BRAKE_HOLD・20.0s ＝打ち切りに張り付く）
    3:CRUISE_TRIM   100 行 アクセル 13%（CRUISE_TRIM・9.9s）
    """
    plan = [("1:ACCEL_SWEEP", "DRIVE_ACCEL", 100, 24.0, 0.0),
            ("2:BRAKE_HOLD", "BRAKE_HOLD", 201, 0.0, 20.0),
            ("3:CRUISE_TRIM", "CRUISE_TRIM", 100, 13.0, 0.0)]
    header = ("elapsed_s", "ref_speed_kmh", "actual_speed_kmh", "accel_opening", "brake_opening",
              "accel_pos", "brake_pos", "accel_current", "brake_current",
              "section", "pattern", "phase")
    lines = [",".join(header)]
    i = 0
    speed = 20.0
    for pattern, phase, count, accel, brake in plan:
        for _ in range(count):
            speed = max(5.0, speed + (0.4 if accel > 0.0 else -0.2))
            lines.append(
                f"{i * 0.1:.1f},,{speed:.2f},{accel:.2f},{brake:.2f},0,0,0.0,0.0,"
                f"PATTERN_DRIVE,{pattern},{phase}"
            )
            i += 1
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def test_train_models_exclude_kinds_drops_only_that_family(tmp_path: Path) -> None:
    csv_path = _write_train_csv(tmp_path / "train.csv")
    base = kaizen.train_models(csv_path, PARAMS)
    without = kaizen.train_models(csv_path, PARAMS, exclude_kinds=("CRUISE_TRIM",))
    assert 0 < without.rows[0] < base.rows[0]  # アクセルは減るが無くならない
    assert without.rows[1] == base.rows[1]  # ブレーキ側は CRUISE_TRIM を使っていない
