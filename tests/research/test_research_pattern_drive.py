"""研究開発用ハーネス 手順 2-1/2-2（パターン走行 → FF モデル作成）のユニットテスト。

スタブ HW（StubVehicle 付き）で研究用の状態機械 PatternLoop（本番 LearningLoop のアルゴリズムを
移植した自前実装）を短いパターン列で回し、走行後の停車保持・非常停止時のペダル解放・CSV・
モデル作成と YAML 書き戻しを確認する。
2-0 のペダル探索は test_research_pedal_search.py で確認し、ここでは結果を与えて始める。
"""

from __future__ import annotations

import pickle
from dataclasses import replace
from datetime import UTC, datetime, timedelta
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from tests.research import config as cfgmod
from tests.research import drive_log as dlmod
from tests.research import hardware as hwmod
from tests.research import main as mainmod
from tests.research import pattern_drive as pdmod
from tests.research.ff_model import MODEL_TYPE
from tests.research.learning_patterns import COAST_DOWN_COUNT, LearningDataError
from tests.research.live_plot import PlotSample, save_drive_figure
from tests.research.pattern_loop import (
    CreepLaunchPattern,
    GridLaunchPattern,
    GridStationPattern,
    PatternLoopConfig,
)
from tests.research.pedal_search import PedalSearchResult
from tests.research.research_types import (
    VEHICLE_STOP_SPEED_KMH,
    DriveLogData,
    DrivingMode,
    LearningPattern,
    PatternKind,
    SpeedPoint,
)
from tests.research.vehicle import (
    STROKE_LIMIT_PULSE,
    build_vehicle_profile,
    feedforward_params,
    opening_to_pulse,
)

# 2-0 のペダル探索が終わった想定の結果（スタブの真の遊び 6%/8% に近い値）
PEDAL = PedalSearchResult(
    creep_speed_kmh=5.0,
    accel_deadband_pct=6.3,
    brake_deadband_pct=8.4,
    stop_confirm_pct=12.0,
    stop_brake_opening_pct=22.0,
)

# 本番の既定（数分）では単体テストにならないので、状態機械はそのままに時間だけ縮める
FAST_LOOP = PatternLoopConfig(
    accel_ramp_time_s=0.2,
    brake_ramp_time_s=0.2,
    creep_settle_min_s=0.3,
    creep_settle_stable_duration_s=0.2,
    creep_settle_timeout_s=1.0,
    accel_full_range_timeout_s=1.5,
    brake_stop_timeout_s=4.0,
    coast_timeout_s=1.0,
    coast_accel_rate_gain=1000.0,  # 短い試験でも開度が上がるように踏む速さを上げる
)
SHORT_PATTERNS = [
    LearningPattern(PatternKind.CREEP, accel_opening=0.0, brake_opening=22.0, hold_duration_s=0.3),
    LearningPattern(
        PatternKind.CREEP_SETTLE, accel_opening=0.0, brake_opening=0.0, hold_duration_s=0.3
    ),
    LearningPattern(
        PatternKind.COAST_DOWN, accel_opening=40.0, brake_opening=0.0, hold_duration_s=0.3
    ),
]


class _Stats:
    """`wltp_grid.wltp_cell_stats` の結果と同じ形（速度 14 行 × 加速度 7 列）。"""

    def __init__(self) -> None:
        n_speed, n_accel = 14, 7
        self.seconds = np.full((n_speed, n_accel), 9.0)
        self.seconds[13] = 0.0  # 130〜140 は WLTP が無い
        self.mean_accel = np.tile(np.array([-4.0, -2.0, -1.0, 0.0, 1.0, 2.0, 4.0]), (n_speed, 1))
        self.max_accel_by_speed = np.full(n_speed, 5.0)
        self.min_accel_by_speed = np.full(n_speed, -5.0)


def _tmp_cfg(tmp_path: Path) -> cfgmod.ResearchConfig:
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    cfg.save(
        {
            "output.results_dir": str(tmp_path / "results"),
            "feedforward.model_path": str(tmp_path / "results" / "models" / "ff.pkl"),
            "output.plot": False,
            # 走行後の緩減速を短くする（判定ロジックは既定と同じ）
            "decel_stop.step_mm": 1.0,
            "decel_stop.dwell_s": 0.2,
            "decel_stop.slope_window_s": 0.2,
            # 実質Kp 合否条件（ProblemReport_20260921 手順6 段2。案a）は無効化する。この
            # ヘルパーが作る合成ログ（短時間・少パターン）は実車の物理を表していないため、
            # deviation_gain の符号が偶然どちらに転んでもおかしくない。この安全網自体は
            # test_research_horizon_search.py で確認する
            "features.search_gain_check_speeds_kmh": [],
        }
    )
    return cfgmod.load_config(path)


async def _held_stub(cfg: cfgmod.ResearchConfig) -> hwmod.ResearchHardware:
    """初期化済みで、2-0 の終わりと同じく停車保持開度で止まっているスタブ。"""
    hw = hwmod.build_hardware(cfg, hwmod.HW_STUB)
    await hwmod.run_initialize(hw)
    await hw.brake.move_to_position(opening_to_pulse(PEDAL.stop_brake_opening_pct))
    return hw


def _synthetic_samples(cfg: cfgmod.ResearchConfig) -> list[dlmod.DriveSample]:
    """スタブ車両モデルを時間指定で回した、加速・制動・惰行を含む 0.1s 刻みのログ。"""
    accel = hwmod.StubActuator("accel", connected=True)
    brake = hwmod.StubActuator("brake", connected=True)
    # スタブ車両（hardware.StubVehicle: アクセル 0.25・ブレーキ 0.5 km/h/s/% 固定）と釣り合う
    # 惰行にする。実機 config の 9.2〜10.8 km/h/s だとアクセル 25% でも 14 km/h で釣り合い、
    # ブレーキ区間がクリープ以下に沈んでブレーキゲイン推定のサンプルが採れない
    # （config_testVehicle.yaml はユーザーが実機に合わせて書き換える値なので決め打ちしない）
    params = replace(
        feedforward_params(cfg),
        coast_decel_speeds_kmh=(),
        coast_decel_kmhs=(),
        engine_brake_decel_kmhs=1.6,
        creep_speed_kmh=5.0,
        creep_rate_kmhs=0.5,
    )
    vehicle = hwmod.StubVehicle(accel=accel, brake=brake, params=params)
    wall0 = datetime(2026, 9, 10, tzinfo=UTC)
    samples: list[dlmod.DriveSample] = []
    speed = vehicle.advance(0.0, now=0.0)
    t = 0.0
    for accel_pct, brake_pct, seconds in (
        (25.0, 0.0, 8.0), (0.0, 18.0, 6.0), (45.0, 0.0, 5.0), (0.0, 0.0, 10.0),
        (0.0, 38.0, 4.0), (15.0, 0.0, 10.0), (0.0, 12.0, 10.0), (65.0, 0.0, 4.0), (0.0, 0.0, 15.0),
    ):
        accel.position = opening_to_pulse(accel_pct)
        brake.position = opening_to_pulse(brake_pct)
        for _ in range(round(seconds / 0.1)):
            t += 0.1
            speed = vehicle.advance(speed, now=t)
            samples.append(
                dlmod.DriveSample(
                    elapsed_s=t,
                    timestamp=wall0 + timedelta(seconds=t),
                    data=DriveLogData(
                        ref_speed_kmh=None,
                        actual_speed_kmh=speed,
                        accel_opening=accel_pct,
                        brake_opening=brake_pct,
                        accel_pos=accel.position,
                        brake_pos=brake.position,
                        accel_current=0.0,
                        brake_current=0.0,
                    ),
                    section=dlmod.SECTION_PATTERN_DRIVE,
                    phase="TEST",
                    pattern="1:TEST",
                )
            )
    return samples


# ── 開度の定義・プロファイル・パターン ────────────────────────────────


def test_opening_is_home_to_stroke_limit() -> None:
    assert opening_to_pulse(0.0) == 0
    assert opening_to_pulse(100.0) == STROKE_LIMIT_PULSE == 9500
    profile = build_vehicle_profile(cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH))
    assert profile.calibration is not None
    assert profile.calibration.accel_zero_pos == 0
    assert profile.calibration.brake_full_pos == STROKE_LIMIT_PULSE


def test_build_vehicle_profile_maps_yaml() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    profile = build_vehicle_profile(cfg)
    assert profile.max_speed == cfg.vehicle.max_speed_kmh
    assert profile.max_decel_g == cfg.vehicle.max_decel_g
    ffp = profile.feedforward_params
    assert ffp.stop_brake_opening_pct == cfg.feedforward.stop_brake_opening_pct
    assert ffp.brake_deadband_pct == cfg.feedforward.brake_deadband_pct


def _profile(cfg: cfgmod.ResearchConfig) -> Any:
    return PEDAL.apply_to_profile(build_vehicle_profile(cfg))


def test_build_patterns_layout_is_coast_grid_creep_launch_creep_hold() -> None:
    """段4・段6a: コーストダウン → G 校正 → 格子ステップ走行（ステーション → 発進・停車）
    → クリープ発進 → クリープ域ブレーキ保持。旧パターン（ACCEL_SWEEP など）は無い。"""
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    patterns = pdmod.build_patterns(cfg, _profile(cfg), _Stats())
    lr = cfg.learning

    coast = [p for p in patterns if p.kind is PatternKind.COAST_DOWN]
    stations = [p for p in patterns if isinstance(p, GridStationPattern)]
    launches = [p for p in patterns if isinstance(p, GridLaunchPattern)]
    creeps = [p for p in patterns if isinstance(p, CreepLaunchPattern)]
    assert len(coast) == COAST_DOWN_COUNT
    assert len(stations) == 12 and len(launches) == 1
    assert len(creeps) == lr.creep_launch_count + len(lr.creep_brake_hold_fracs)
    calib = [p for p in patterns if p.kind is PatternKind.G_CALIB]
    sweeps = [p for p in patterns if p.kind is PatternKind.GRID_SWEEP]
    assert len(calib) == 1 and len(sweeps) == 1
    assert len(patterns) == len(coast) + 1 + len(stations) + len(launches) + 1 + len(creeps)

    kinds = [p.kind for p in patterns]
    assert kinds[:COAST_DOWN_COUNT] == [PatternKind.COAST_DOWN] * COAST_DOWN_COUNT
    assert kinds[COAST_DOWN_COUNT] is PatternKind.G_CALIB  # コーストダウンの直後・格子の前
    grid_end = COAST_DOWN_COUNT + 1 + len(stations) + len(launches)
    assert isinstance(patterns[grid_end - 1], GridLaunchPattern)  # 発進・停車は格子の最後
    assert patterns[grid_end].kind is PatternKind.GRID_SWEEP  # 通し掃引はその後・クリープの前
    grid_end += 1
    assert all(isinstance(p, CreepLaunchPattern) for p in patterns[grid_end:])
    assert not any(p.kind.name in {"ACCEL_SWEEP", "BRAKE_HOLD", "CRUISE_TRIM"} for p in patterns)


def test_build_patterns_coast_down_is_vehicle_independent_and_clamped() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    profile = _profile(cfg)
    patterns = pdmod.build_patterns(cfg, profile, _Stats())
    for p in patterns[:COAST_DOWN_COUNT]:
        assert p.accel_opening == min(70.0, profile.max_accel_opening)
        assert p.brake_opening == 0.0


def test_build_patterns_creep_launches_then_brake_holds_at_end() -> None:
    """クリープ発進（停車復帰）→ ブレーキ保持（開度 = 不感帯 + frac×(停車保持−不感帯)）。"""
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    profile = _profile(cfg)
    patterns = pdmod.build_patterns(cfg, profile, _Stats())
    lr = cfg.learning
    n_new = lr.creep_launch_count + len(lr.creep_brake_hold_fracs)
    tail = patterns[-n_new:]
    launches, holds = tail[:lr.creep_launch_count], tail[lr.creep_launch_count:]

    for p in launches:
        assert isinstance(p, CreepLaunchPattern) and p.kind is PatternKind.CREEP_SETTLE
        assert not p.hold_after
        assert (p.accel_opening, p.brake_opening) == (0.0, 0.0)
        assert p.target_kmh == pytest.approx(lr.creep_launch_target_kmh)
        assert p.timeout_s == pytest.approx(lr.creep_launch_timeout_s)
        assert "クリープ発進" in pdmod._describe(p) and "停車復帰" in pdmod._describe(p)

    ff = profile.feedforward_params
    span = ff.stop_brake_opening_pct - ff.brake_deadband_pct
    expected = [
        min(round(ff.brake_deadband_pct + frac * span, 2), profile.max_brake_opening)
        for frac in lr.creep_brake_hold_fracs
    ]
    assert len(holds) == len(expected)
    for p, opening in zip(holds, expected, strict=True):
        assert isinstance(p, CreepLaunchPattern) and p.hold_after
        assert p.accel_opening == 0.0
        assert p.brake_opening == pytest.approx(opening)
        assert "ブレーキ保持" in pdmod._describe(p)


def test_build_patterns_brake_hold_follows_measured_pedal_values() -> None:
    """別の車（2-0 の実測が違う）でも、ブレーキ保持は同じ frac のまま実測に追従する。"""
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    profile = build_vehicle_profile(cfg)
    other = replace(
        PEDAL, brake_deadband_pct=4.0, stop_brake_opening_pct=10.0
    ).apply_to_profile(profile)
    holds = [
        p for p in pdmod.build_patterns(cfg, other, _Stats())
        if isinstance(p, CreepLaunchPattern) and p.hold_after
    ]
    fracs = cfg.learning.creep_brake_hold_fracs
    assert [p.brake_opening for p in holds] == pytest.approx([4.0 + f * 6.0 for f in fracs])


def test_build_patterns_without_creep_settings_has_no_creep_launch() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.learning.creep_launch_count = 0
    cfg.learning.creep_brake_hold_fracs = []
    patterns = pdmod.build_patterns(cfg, _profile(cfg), _Stats())
    assert not any(isinstance(p, CreepLaunchPattern) for p in patterns)


async def test_run_pattern_drive_requires_patterns_or_wltp_stats(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = await _held_stub(cfg)
    with pytest.raises(cfgmod.ConfigError, match="wltp_stats"):
        await pdmod.run_pattern_drive(hw, cfg, pedal=PEDAL)
    await hwmod.shutdown(hw)


# ── スタブ車両 ───────────────────────────────────────────────────────


def test_stub_vehicle_accelerates_and_brake_hold_stops() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    accel = hwmod.StubActuator("accel", connected=True)
    brake = hwmod.StubActuator("brake", connected=True)
    vehicle = hwmod.StubVehicle(accel=accel, brake=brake, params=feedforward_params(cfg))
    accel.position = opening_to_pulse(40.0)
    speed = vehicle.advance(0.0, now=0.0)
    for i in range(1, 21):
        speed = vehicle.advance(speed, now=i * 0.1)
    assert speed > 10.0

    accel.position = 0
    brake.position = opening_to_pulse(30.0)
    for i in range(21, 61):
        speed = vehicle.advance(speed, now=i * 0.1)
    assert speed == 0.0


def test_stub_vehicle_ignores_pedal_inside_play() -> None:
    """遊びの中の踏み込みは車速に効かない（ペダル探索が測る対象）。"""
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    accel = hwmod.StubActuator("accel", connected=True)
    brake = hwmod.StubActuator("brake", connected=True)
    free = hwmod.StubVehicle(accel=accel, brake=brake, params=feedforward_params(cfg))
    pressed = hwmod.StubVehicle(
        accel=hwmod.StubActuator("accel", position=opening_to_pulse(5.0), connected=True),
        brake=brake,
        params=feedforward_params(cfg),
    )
    free.advance(3.0, now=0.0)
    pressed.advance(3.0, now=0.0)
    assert free.advance(3.0, now=0.1) == pressed.advance(3.0, now=0.1)


# ── 2-1. パターン走行 ────────────────────────────────────────────────


async def test_pattern_drive_runs_pattern_loop_and_holds_stop(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = await _held_stub(cfg)

    result = await pdmod.run_pattern_drive(
        hw, cfg, pedal=PEDAL, patterns=SHORT_PATTERNS, loop_config=FAST_LOOP
    )

    lines = result.csv_path.read_text(encoding="utf-8").splitlines()
    assert lines[0].split(",") == list(dlmod.CSV_COLUMNS)
    column = dlmod.CSV_COLUMNS.index("section")
    sections = [line.split(",")[column] for line in lines[1:]]
    assert sections.count(dlmod.SECTION_PATTERN_DRIVE) == len(result.samples) >= 20
    assert max(s.data.actual_speed_kmh for s in result.samples) > 5.0
    kinds = {s.pattern.split(":")[1] for s in result.samples}
    assert kinds >= {"CREEP", "CREEP_SETTLE", "COAST_DOWN"}
    # 走行後の緩減速〜停車保持も同じ CSV の後ろに残る
    assert sections[-1] == dlmod.SECTION_DECEL_TO_STOP
    assert result.stop is not None
    assert not result.csv_path.with_suffix(".png").exists()  # output.plot=false
    # 走行後: アクセル 0%・停車保持開度（2-0 の値）・停車
    assert hw.accel.position == 0
    assert hw.brake.position == opening_to_pulse(PEDAL.stop_brake_opening_pct)
    assert await hw.can.read_speed() < VEHICLE_STOP_SPEED_KMH
    await hwmod.shutdown(hw)


async def test_pattern_drive_emergency_releases_pedals(tmp_path: Path) -> None:
    """CAN が途中で読めなくなったら PatternLoop が非常停止し、ペダルを離して止まる。"""
    cfg = _tmp_cfg(tmp_path)
    hw = await _held_stub(cfg)
    original = hw.can.read_speed
    calls = 0

    async def flaky_read() -> float:
        nonlocal calls
        calls += 1
        if calls > 15:
            raise TimeoutError("CAN 車速が 0.2s 更新されていません")
        return await original()

    hw.can.read_speed = flaky_read  # type: ignore[method-assign]

    with pytest.raises(pdmod.DriveError, match="非常停止"):
        await pdmod.run_pattern_drive(
            hw, cfg, pedal=PEDAL, patterns=SHORT_PATTERNS, loop_config=FAST_LOOP
        )
    assert hw.accel.position == 0 and hw.brake.position == 0  # 原点復帰済み
    # 途中までのログも残す（原因調査用）
    assert list((tmp_path / "results").glob("drive_log_stub_*.csv"))
    await hwmod.shutdown(hw)


# ── CSV・図 ──────────────────────────────────────────────────────────


def test_csv_roundtrip_to_drive_logs(tmp_path: Path) -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    samples = _synthetic_samples(cfg)[:30]
    path = tmp_path / "drive.csv"
    dlmod.write_csv(samples, path)

    logs = dlmod.read_drive_logs(path)
    assert len(logs) == 30
    assert logs[1].timestamp - logs[0].timestamp == timedelta(seconds=0.1)
    assert logs[5].accel_opening == pytest.approx(samples[5].data.accel_opening)
    assert logs[5].actual_speed_kmh == pytest.approx(samples[5].data.actual_speed_kmh, abs=1e-3)
    assert logs[0].ref_speed_kmh is None


def test_save_drive_figure_writes_png(tmp_path: Path) -> None:
    samples = [PlotSample(i * 0.1, None, i * 0.5, 20.0, 0.0) for i in range(50)]
    path = save_drive_figure(
        samples, tmp_path / "fig.png", title="テスト", max_speed_kmh=140.0, has_ref=False
    )
    assert path.stat().st_size > 1000


# ── 2-2. モデル作成 ──────────────────────────────────────────────────


def test_build_ff_model_real_saves_model_but_not_measured_values(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg), csv_path)

    result = pdmod.build_ff_model(cfg, csv_path, hw_mode=hwmod.HW_REAL, pedal=PEDAL)

    saved = cfgmod.load_config(cfg.source_path)
    assert saved.feedforward.model_path == result.model_path
    assert saved.feedforward.is_model_trained
    with Path(result.model_path).open("rb") as f:
        assert pickle.load(f)["model_type"] == MODEL_TYPE
    assert set(result.metrics) == {"accel", "brake"}
    assert saved.feedforward.engine_brake_decel_kmhs == pytest.approx(
        result.params.engine_brake_decel_kmhs, rel=1e-5
    )
    # 不感帯・停車保持開度は 2-0 の実測が正。推定値では上書きしない
    for key in pdmod.MEASURED_KEYS:
        assert getattr(saved.feedforward, key) == getattr(cfg.feedforward, key)
        assert not any(line.startswith(f"feedforward.{key}:") for line in result.changed)
    # ペダルゲインは研究側の推定（不感帯 + 0.5% 以上）で両側とも同定し、グリッドと同じ点数で書き戻す
    ff = saved.feedforward
    assert len(ff.pedal_gain_speeds_kmh) >= 2
    assert len(ff.accel_gain_kmhs_per_pct) == len(ff.pedal_gain_speeds_kmh)
    assert len(ff.brake_gain_kmhs_per_pct) == len(ff.pedal_gain_speeds_kmh)
    assert not [p for p in cfgmod.validate_config(saved) if "pedal_gain" in p]
    # ユーザーが編集するファイルなのでコメントは消さない
    assert "# 2次多項式 Ridge 逆モデル" in cfg.source_path.read_text(encoding="utf-8")
    # 参照用: 選ばれたホライズン・特徴量・係数 yaml の場所が config に残る（走行時は読まない）
    with Path(result.model_path).open("rb") as f:
        payload = pickle.load(f)
    assert ff.model_accel_horizons_s == pytest.approx(
        list(payload["accel_feature_spec"]["lookahead_horizons_s"])
    )
    assert ff.model_brake_horizons_s == pytest.approx(
        list(payload["brake_feature_spec"]["lookahead_horizons_s"])
    )
    assert ff.model_accel_features and ff.model_brake_features
    assert ff.model_coef_path == str(Path(result.model_path).with_suffix(".yaml"))
    assert Path(ff.model_coef_path).exists()


def test_build_ff_model_order_and_reference_basis_for_coast_creep_gain(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """段2.5（ProblemReport_20260916）: 推定順は 惰行 → クリープ → ペダルゲイン。

    クリープ加速カーブ・ペダルゲインの基準（reference）は、estimate_dynamics_params が返す
    生の `after`（不感帯が表示専用の粗い推定値。実測 8.53/12.00% に対し推定 3.0/3.0% 相当）
    ではなく、**before の実測不感帯・停車保持・creep_speed_kmh を保ったまま惰行カーブだけ
    差し替えたもの**であること。これを取り違えると「不感帯以下＝ペダルオフ」判定とゲインの
    分母（開度−不感帯）が壊れ、惰行サンプルが 995→361 点まで減る不具合が実機ログの relearn
    dry-run で確認された回帰。
    """
    cfg = _tmp_cfg(tmp_path)
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg), csv_path)

    calls: list[str] = []
    captured: dict[str, Any] = {}
    original_dynamics = pdmod.estimate_dynamics_params
    original_coast = pdmod._estimate_research_coast_curve
    original_creep = pdmod._estimate_research_creep_curve
    original_gain = pdmod._estimate_research_pedal_gains

    def spy_dynamics(logs, current):  # type: ignore[no-untyped-def]
        captured["before"] = current
        return original_dynamics(logs, current)

    def spy_coast(cfg_, logs, reference, target):  # type: ignore[no-untyped-def]
        calls.append("coast")
        captured["coast_reference"] = reference
        result = original_coast(cfg_, logs, reference, target)
        captured["coast_result"] = result
        return result

    def spy_creep(cfg_, logs, params, before):  # type: ignore[no-untyped-def]
        calls.append("creep")
        captured["creep_params"] = params
        return original_creep(cfg_, logs, params, before)

    def spy_gain(cfg_, logs, before, after, research):  # type: ignore[no-untyped-def]
        calls.append("gain")
        captured["gain_before"] = before
        captured["gain_after"] = after
        return original_gain(cfg_, logs, before, after, research)

    monkeypatch.setattr(pdmod, "estimate_dynamics_params", spy_dynamics)
    monkeypatch.setattr(pdmod, "_estimate_research_coast_curve", spy_coast)
    monkeypatch.setattr(pdmod, "_estimate_research_creep_curve", spy_creep)
    monkeypatch.setattr(pdmod, "_estimate_research_pedal_gains", spy_gain)

    pdmod.build_ff_model(cfg, csv_path, hw_mode=hwmod.HW_REAL, pedal=PEDAL)

    assert calls == ["coast", "creep", "gain"]

    before = captured["before"]
    coast_result = captured["coast_result"]

    # 惰行カーブ同定そのもののサンプル抽出条件は before（2-0 実測）を渡す
    assert captured["coast_reference"] is before

    # クリープ・ゲインの基準は before の実測値そのもの（estimate_dynamics_params の粗い
    # 推定値ではない）を保っている
    for ref in (captured["creep_params"], captured["gain_before"]):
        assert ref.accel_deadband_pct == pytest.approx(PEDAL.accel_deadband_pct)
        assert ref.brake_deadband_pct == pytest.approx(PEDAL.brake_deadband_pct)
        assert ref.accel_deadband_pct == pytest.approx(before.accel_deadband_pct)
        assert ref.brake_deadband_pct == pytest.approx(before.brake_deadband_pct)
        assert ref.creep_speed_kmh == pytest.approx(before.creep_speed_kmh)
        assert ref.stop_brake_opening_pct == pytest.approx(before.stop_brake_opening_pct)
        # 惰行カーブだけは今回同定した結果と一致する
        assert ref.coast_decel_speeds_kmh == coast_result.coast_decel_speeds_kmh
        assert ref.coast_decel_kmhs == coast_result.coast_decel_kmhs

    assert captured["gain_after"] is coast_result


def test_build_ff_model_stop_brake_floor_uses_reference_basis_and_saves(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """段4改訂（ProblemReport_20260916 クリープ域ブレーキの下限）: 下限も段2.5 と同じ形で
    reference（before 基準）で同定し、FF_PARAM_KEYS 経由で保存される。

    `test_build_ff_model_order_and_reference_basis_for_coast_creep_gain` と同じ形で、基準に
    渡された params の不感帯・停車保持開度が before（2-0 実測）の値であることをアサートする。
    """
    cfg = _tmp_cfg(tmp_path)
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg), csv_path)

    captured: dict[str, Any] = {}
    original_stop_brake = pdmod._estimate_research_stop_brake_floor

    def spy_stop_brake(cfg_, logs, params, before):  # type: ignore[no-untyped-def]
        captured["params"] = params
        result = original_stop_brake(cfg_, logs, params, before)
        captured["result"] = result
        return result

    monkeypatch.setattr(pdmod, "_estimate_research_stop_brake_floor", spy_stop_brake)

    result = pdmod.build_ff_model(cfg, csv_path, hw_mode=hwmod.HW_REAL, pedal=PEDAL)

    # サンプル抽出条件の基準は before（2-0 実測）そのもの。estimate_dynamics_params の粗い
    # 推定値ではない
    params = captured["params"]
    assert params.accel_deadband_pct == pytest.approx(PEDAL.accel_deadband_pct)
    assert params.brake_deadband_pct == pytest.approx(PEDAL.brake_deadband_pct)
    assert params.stop_brake_opening_pct == pytest.approx(PEDAL.stop_brake_opening_pct)

    # 同定結果がそのまま research_params に乗り、FF_PARAM_KEYS 経由で YAML に保存される
    floor_result = captured["result"]
    assert (
        result.research_params.stop_brake_floor_offset_pct
        == floor_result.stop_brake_floor_offset_pct
    )

    saved = cfgmod.load_config(cfg.source_path)
    assert saved.feedforward.stop_brake_floor_offset_pct == pytest.approx(
        floor_result.stop_brake_floor_offset_pct
    )
    # brake_trim_max_kmh・brake_trim_ref_kmh は人が決める値なので自動保存の対象外
    # （期待値は走行前の設定値。_tmp_cfg は config_testVehicle.yaml のコピーなので
    # dataclass 既定値 0.0 とは限らない）
    assert saved.feedforward.brake_trim_max_kmh == pytest.approx(
        cfg.feedforward.brake_trim_max_kmh
    )
    assert saved.feedforward.brake_trim_ref_kmh == pytest.approx(cfg.feedforward.brake_trim_ref_kmh)


def test_build_ff_model_stub_does_not_touch_yaml(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg), csv_path)
    before = cfg.source_path.read_text(encoding="utf-8")

    result = pdmod.build_ff_model(cfg, csv_path, hw_mode=hwmod.HW_STUB)

    assert cfg.source_path.read_text(encoding="utf-8") == before
    assert "_stub_" in Path(result.model_path).name
    assert result.changed == []


def test_build_ff_model_raises_on_too_few_samples(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg)[:10], csv_path)
    with pytest.raises(LearningDataError):
        pdmod.build_ff_model(cfg, csv_path, hw_mode=hwmod.HW_REAL)


# ── 段2（ProblemReport_20260925）: 学習サンプルの WLTP 重み付け ──────────────


def _tiny_wltp_mode() -> DrivingMode:
    return DrivingMode(
        id="wltp", name="wltp", description="", total_duration=20.0, max_speed=40.0,
        created_at=datetime(2026, 9, 25, tzinfo=UTC), is_system=False,
        reference_speed=[
            SpeedPoint(time_s=0.0, speed_kmh=0.0), SpeedPoint(time_s=20.0, speed_kmh=40.0),
        ],
    )


def test_build_ff_model_raises_config_error_when_enabled_without_wltp_mode(tmp_path: Path) -> None:
    """sample_weight_enabled=true なのに wltp_mode が無いと ConfigError（重みには WLTP が要る）。"""
    cfg = _tmp_cfg(tmp_path)
    cfg.learning.sample_weight_enabled = True
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg), csv_path)
    with pytest.raises(cfgmod.ConfigError):
        pdmod.build_ff_model(cfg, csv_path, hw_mode=hwmod.HW_STUB)


def test_build_ff_model_with_wltp_mode_adds_mae_wltp_and_does_not_require_enabled(
    tmp_path: Path,
) -> None:
    """wltp_mode を渡せば enabled=False でも mae_wltp が付く（重みなし/ありを比較できるように）。"""
    cfg = _tmp_cfg(tmp_path)
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg), csv_path)

    result = pdmod.build_ff_model(
        cfg, csv_path, hw_mode=hwmod.HW_STUB, wltp_mode=_tiny_wltp_mode(),
    )
    assert "mae_wltp" in result.metrics["accel"]
    assert "mae_wltp" in result.metrics["brake"]


def test_build_ff_model_sample_weight_enabled_override_ignores_config(tmp_path: Path) -> None:
    """`sample_weight_enabled` 引数は config の値を一時的に上書きできる（config は変更しない）。"""
    cfg = _tmp_cfg(tmp_path)
    assert cfg.learning.sample_weight_enabled is False
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg), csv_path)

    # enabled=False のまま（config どおり）だと wltp_mode が無くてもエラーにならない
    result_off = pdmod.build_ff_model(
        cfg, csv_path, hw_mode=hwmod.HW_STUB, sample_weight_enabled=False
    )
    assert "mae_wltp" not in result_off.metrics["accel"]

    # 明示的に True を渡すと、config が False のままでも wltp_mode が要る
    with pytest.raises(cfgmod.ConfigError):
        pdmod.build_ff_model(cfg, csv_path, hw_mode=hwmod.HW_STUB, sample_weight_enabled=True)
    assert cfg.learning.sample_weight_enabled is False  # config 自体は変わらない


# ── CLI 経由 ─────────────────────────────────────────────────────────


async def _fake_search(hw: object, cfg: object, **kwargs: object) -> PedalSearchResult:
    return PEDAL


async def _fake_pre_check(hw: object, cfg: object, **kwargs: object) -> int:
    """走行前チェックは test_research_pre_drive_check.py で確認する。"""
    return 0


async def _fake_load_mode(cfg: object, name: str) -> DrivingMode:
    """手順2 は走る前に WLTP の基準車速を DB から読む。テストでは DB を使わない。"""
    return _tiny_wltp_mode()


def test_step2_requires_step1(capsys: pytest.CaptureFixture[str]) -> None:
    assert mainmod.main(["--only", "2"]) == 4
    assert "手順 1 を先に実行" in capsys.readouterr().out


def test_drive_error_exit_code_is_5(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    async def failing_search(hw: object, cfg: object, **kwargs: object) -> None:
        raise pdmod.DriveError("テスト用の探索失敗")

    cfg = _tmp_cfg(tmp_path)  # 走行ログを tmp に書く
    monkeypatch.setattr(mainmod, "run_pre_drive_check", _fake_pre_check)
    monkeypatch.setattr(mainmod, "load_mode", _fake_load_mode)
    monkeypatch.setattr(mainmod, "run_pedal_search", failing_search)
    assert mainmod.main(["--steps", "1,2", "--config", str(cfg.source_path)]) == 5
    out = capsys.readouterr().out
    assert "走行エラー: テスト用の探索失敗" in out
    assert "終了処理: 完了" in out  # 失敗しても原点復帰・サーボOFF まで行く


def test_step2_loads_every_coverage_mode_before_the_pedal_search(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """手順2 は modes.coverage_mode_names の全モードを走る前に読む（重複して読まない）。"""
    loaded: list[str] = []
    events: list[str] = []

    async def recording_load_mode(cfg: object, name: str) -> DrivingMode:
        loaded.append(name)
        return _tiny_wltp_mode()

    async def failing_search(hw: object, cfg: object, **kwargs: object) -> None:
        events.append("pedal_search")
        raise pdmod.DriveError("テスト用の探索失敗")

    cfg = _tmp_cfg(tmp_path)
    wltp = cfg.modes.wltp_mode_name
    cfg.save({"modes.coverage_mode_names": [wltp, "09_US06"]})
    monkeypatch.setattr(mainmod, "run_pre_drive_check", _fake_pre_check)
    monkeypatch.setattr(mainmod, "load_mode", recording_load_mode)
    monkeypatch.setattr(mainmod, "run_pedal_search", failing_search)
    assert mainmod.main(["--steps", "1,2", "--config", str(cfg.source_path)]) == 5
    assert loaded == [wltp, "09_US06"]  # WLTP は 1 回だけ（MAE 用と共用）
    assert events == ["pedal_search"]
    out = capsys.readouterr().out
    assert "格子外" in out  # モードごとの格子の外の秒数を出す


def test_step2_stops_before_driving_when_a_coverage_mode_is_missing(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    async def missing_mode(cfg: object, name: str) -> DrivingMode:
        if name == "09_US06":
            raise cfgmod.ConfigError("走行モード '09_US06' が driving_modes にありません")
        return _tiny_wltp_mode()

    async def never_search(hw: object, cfg: object, **kwargs: object) -> None:
        raise AssertionError("ペダル探索まで進んではいけない")

    cfg = _tmp_cfg(tmp_path)
    cfg.save({"modes.coverage_mode_names": [cfg.modes.wltp_mode_name, "09_US06"]})
    monkeypatch.setattr(mainmod, "run_pre_drive_check", _fake_pre_check)
    monkeypatch.setattr(mainmod, "load_mode", missing_mode)
    monkeypatch.setattr(mainmod, "run_pedal_search", never_search)
    assert mainmod.main(["--steps", "1,2", "--config", str(cfg.source_path)]) != 0
    assert "09_US06" in capsys.readouterr().out


def test_model_error_exit_code_is_6(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    async def fake_drive(hw: object, cfg: object, **kwargs: object) -> pdmod.PatternDriveResult:
        return pdmod.PatternDriveResult(csv_path=tmp_path / "x.csv", samples=[], duration_s=0.0)

    def failing_build(cfg: object, csv_path: object, **kwargs: object) -> None:
        raise LearningDataError("学習サンプルが不足しています (3 点)")

    monkeypatch.setattr(mainmod, "run_pre_drive_check", _fake_pre_check)
    monkeypatch.setattr(mainmod, "load_mode", _fake_load_mode)
    monkeypatch.setattr(mainmod, "run_pedal_search", _fake_search)
    monkeypatch.setattr(mainmod, "run_pattern_drive", fake_drive)
    monkeypatch.setattr(mainmod, "build_ff_model", failing_build)
    cfg = _tmp_cfg(tmp_path)  # 走行ログを tmp に書く
    assert mainmod.main(["--steps", "1,2", "--config", str(cfg.source_path)]) == 6
    assert "モデル作成エラー" in capsys.readouterr().out
