"""研究開発用ハーネス 手順 3（FF のみでモード走行 → CSV/レポート）のユニットテスト。

スタブ HW（StubVehicle 付き）で数秒の合成モードを走らせ、ループ・安全停止・CSV・レポートを確認する。
FF は手順 2 のモデルに依存しないよう、同じインターフェースの簡単な偽物を使う。
KPI の数え方は本番 KPIMonitor と突き合わせる。
"""

from __future__ import annotations

import csv
import random
from datetime import UTC, datetime, timedelta
from pathlib import Path

import numpy as np
import pytest

from src.domain.control.conversions import G_TO_KMHS
from src.domain.control.kpi_monitor import KPIMonitor
from src.models.drive_log import DriveLogData
from src.models.driving_mode import DrivingMode, SpeedPoint
from src.models.profile import FeedforwardParams
from tests.research import config as cfgmod
from tests.research import drive_log as dlmod
from tests.research import ff_candidate
from tests.research import hardware as hwmod
from tests.research import kpi as kpimod
from tests.research import main as mainmod
from tests.research import mode_drive as mdmod
from tests.research import mode_report as mrmod
from tests.research.axis_safety import ALARM_CHECK_INTERVAL_S
from tests.research.vehicle import feedforward_params, opening_to_pulse, pulse_to_opening

HOLD_PCT = 22.0


def _tmp_cfg(tmp_path: Path) -> cfgmod.ResearchConfig:
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    cfg.save(
        {
            "output.results_dir": str(tmp_path / "results"),
            "output.plot": False,
            "feedforward.stop_brake_opening_pct": HOLD_PCT,
            # 走行後の緩減速を短くする（判定ロジックは既定と同じ）
            "decel_stop.step_mm": 1.0,
            "decel_stop.dwell_s": 0.2,
            "decel_stop.slope_window_s": 0.2,
        }
    )
    return cfgmod.load_config(path)


def _mode(points: list[tuple[float, float]]) -> DrivingMode:
    return DrivingMode(
        id="test",
        name="test_mode",
        description="",
        reference_speed=[SpeedPoint(t, v) for t, v in points],
        total_duration=points[-1][0],
        max_speed=max(v for _, v in points),
        created_at=datetime(2026, 9, 11, tzinfo=UTC),
    )


class FakeFF:
    """FeedforwardController と同じ呼び方の偽物: 停車は保持ブレーキ、それ以外は向きで ±。"""

    horizons = (0.5, 1.0, 2.0, 3.0)
    past_horizons = (0.5, 1.0)
    candidate = "C1"
    uses_actual_speed = False

    def __init__(self) -> None:
        self.calls = 0

    def predict_effort(self, v0: float, future: list[float], past: list[float]) -> float:
        self.calls += 1
        if v0 <= 0.02 and future[0] <= 0.02:
            return -HOLD_PCT
        return 30.0 if future[1] >= v0 else -12.0


async def _held_stub(cfg: cfgmod.ResearchConfig) -> hwmod.ResearchHardware:
    """初期化済みで、走行前チェックの終わりと同じく停車してブレーキを踏んでいるスタブ。"""
    hw = hwmod.build_hardware(cfg, hwmod.HW_STUB)
    await hwmod.run_initialize(hw)
    assert isinstance(hw.can, hwmod.StubCANReader)
    hw.can.speed_kmh = 0.0
    await hw.brake.move_to_position(opening_to_pulse(HOLD_PCT))
    return hw


# ── 部品 ──────────────────────────────────────────────────────────────


def test_reference_speed_interpolates_and_clamps() -> None:
    ref = mdmod.ReferenceSpeed(_mode([(0.0, 0.0), (1.0, 10.0), (3.0, 10.0), (4.0, 0.0)]))
    assert ref.at(-1.0) == 0.0
    assert ref.at(0.5) == pytest.approx(5.0)
    assert ref.at(2.0) == pytest.approx(10.0)
    assert ref.at(3.25) == pytest.approx(7.5)
    assert ref.at(99.0) == 0.0


def test_split_effort_is_exclusive_and_clamped() -> None:
    assert mdmod.split_effort(12.5, 80.0, 80.0) == (12.5, 0.0)
    assert mdmod.split_effort(-30.0, 80.0, 25.0) == (0.0, 25.0)
    assert mdmod.split_effort(95.0, 80.0, 80.0) == (80.0, 0.0)
    assert mdmod.split_effort(0.0, 80.0, 80.0) == (0.0, 0.0)


def test_governor_caps_brake_while_decel_exceeds_limit_and_releases() -> None:
    gov = mdmod.DecelGovernor(max_decel_g=0.4, reduce_step_pct=2.0)
    hard = gov.limit_kmhs * 1.5  # 上限を超える減速 [km/h/s]
    speed, t = 100.0, 0.0
    brake, active = gov.apply(t, speed, 20.0)
    assert (brake, active) == (20.0, False)
    for expected in (20.0, 18.0, 16.0):  # 超えた周期は前周期の値で頭打ち → 以降 2% ずつ下げる
        t += 0.05
        speed -= hard * 0.05
        brake, active = gov.apply(t, speed, 35.0)
        assert active
        assert brake == pytest.approx(expected)
    t += 0.45  # 減速が収まって（傾きの窓 0.4s を過ぎて）も頭打ちは保つ（FF が浅くするまで）
    brake, active = gov.apply(t, speed, 35.0)
    assert gov.decel_kmhs == 0.0
    assert (brake, active) == (pytest.approx(16.0), True)
    t += 0.05
    brake, active = gov.apply(t, speed, 10.0)  # FF 自身が頭打ちより浅くした → 解除
    assert (brake, active) == (10.0, False)


def test_governor_disabled_passes_through() -> None:
    gov = mdmod.DecelGovernor(max_decel_g=0.4, reduce_step_pct=2.0, enabled=False)
    for i in range(10):
        brake, active = gov.apply(i * 0.05, 100.0 - i * 3.0, 40.0)
        assert (brake, active) == (40.0, False)
    assert gov.decel_kmhs == pytest.approx(3.0 / 0.05)
    assert gov.limit_kmhs == pytest.approx(0.4 * G_TO_KMHS * 0.98)


def test_arbiter_options_are_rejected(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    cfg.arbiter.enable_rate_limit = True
    with pytest.raises(cfgmod.ConfigError, match="enable_rate_limit"):
        mdmod.require_simple_arbiter(cfg)


def test_segment_at_uses_bounds() -> None:
    modes = cfgmod.ModesSection()
    assert [modes.segment_at(t) for t in (0.0, 588.9, 589.0, 1500.0, 1800.0)] == [
        "Low", "Low", "Mid", "ExHi", "ExHi"
    ]


def test_segment_config_is_validated() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.modes.segment_bounds_s = [589.0, 1022.0]  # 区間名 4 つに境界 2 つ
    assert any("segment_bounds_s" in p for p in cfgmod.validate_config(cfg))


def test_load_feedforward_requires_trained_model(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    cfg.feedforward.model_path = str(tmp_path / "none.pkl")
    with pytest.raises(cfgmod.ConfigError, match="手順 2"):
        mdmod.load_feedforward(cfg)


def test_load_feedforward_passes_coast_band_to_candidate(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """段2: `load_feedforward` が config の `coast_band_kmhs` を候補へ渡すこと。"""
    cfg = _tmp_cfg(tmp_path)
    cfg.feedforward.coast_band_kmhs = 0.5
    cfg.feedforward.model_path = str(tmp_path / "dummy.pkl")
    (tmp_path / "dummy.pkl").write_bytes(b"")  # is_model_trained は存在チェックのみ

    calls: dict[str, object] = {}

    class _FakeFF(ff_candidate.CandidateFeedforward):
        def set_research_params(self, research: object) -> None:
            calls["research"] = research
            super().set_research_params(research)  # type: ignore[arg-type]

        def load_model(self, model_path: str) -> None:
            calls["model_path"] = model_path  # 実ファイルは読まない（このテストの関心外）

    monkeypatch.setattr(mdmod, "make_candidate", lambda name: _FakeFF())  # noqa: ARG005

    ff = mdmod.load_feedforward(cfg)

    assert calls["model_path"] == cfg.feedforward.model_path
    assert calls["research"].coast_band_kmhs == pytest.approx(0.5)  # type: ignore[attr-defined]
    assert ff._research.coast_band_kmhs == pytest.approx(0.5)  # noqa: SLF001


def test_load_feedforward_rejects_reach_horizon_not_in_model(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """段3: `reach_horizons_s` にモデルの先読みホライズンに無い値があれば ConfigError。

    `_FakeFF.load_model` は実ファイルを読まないため `ff.horizons` は既定の
    `DEFAULT_FEATURE_SPEC.lookahead_horizons_s`（0.5, 1.0, 2.0, 3.0）のまま。5.0 はそこに
    無いので弾かれる。
    """
    cfg = _tmp_cfg(tmp_path)
    cfg.feedforward.reach_horizons_s = [0.5, 5.0]
    cfg.feedforward.model_path = str(tmp_path / "dummy.pkl")
    (tmp_path / "dummy.pkl").write_bytes(b"")

    class _FakeFF(ff_candidate.CandidateFeedforward):
        def load_model(self, model_path: str) -> None:
            pass  # 実ファイルは読まない（このテストの関心外）

    monkeypatch.setattr(mdmod, "make_candidate", lambda name: _FakeFF())  # noqa: ARG005

    with pytest.raises(cfgmod.ConfigError, match="reach_horizons_s"):
        mdmod.load_feedforward(cfg)


# ── KPI ───────────────────────────────────────────────────────────────


def test_kpi_matches_production_monitor() -> None:
    """最大逸脱・符号反転は本番 KPIMonitor と一致、p95 はビン幅 0.01 以内。"""
    rng = random.Random(3)
    t, dev = [], []
    value = 0.0
    for i in range(3000):
        value = 0.9 * value + rng.gauss(0.0, 0.35)
        t.append(i * 0.1)
        dev.append(value)
    monitor = KPIMonitor()
    for ti, d in zip(t, dev, strict=True):
        monitor.update(ref_kmh=50.0, actual_kmh=50.0 + d, now_s=ti)
    prod = monitor.summary()

    result = kpimod.compute_kpi(t, dev, cfgmod.KpiSection())
    assert result.max_abs_kmh == pytest.approx(prod["max_abs_deviation_kmh"], abs=1e-9)
    assert result.reversal_max_per_window == prod["reversal_max_per_5s"]
    assert result.reversal_max_per_window > 1  # 反転が数えられるデータになっていること
    assert 0.0 <= prod["p95_kmh"] - result.p95_kmh <= 0.0101


def test_kpi_episodes_and_verdict() -> None:
    t = [i * 0.1 for i in range(100)]
    dev = [0.0] * 100
    dev[10:15] = [1.2, 1.5, -0.2, 1.1, 1.05]  # 10-11 と 13-14 の 2 区間
    dev[50] = -2.0
    result = kpimod.compute_kpi(t, dev, cfgmod.KpiSection())
    assert [(round(e.start_s, 1), round(e.end_s, 1)) for e in result.episodes] == [
        (1.0, 1.1), (1.3, 1.4), (5.0, 5.0)
    ]
    assert result.episodes[0].peak_kmh == 1.5
    assert result.max_abs_kmh == 2.0 and result.max_abs_t_s == pytest.approx(5.0)
    assert result.time_over_limit_s == pytest.approx(0.5)
    assert not result.max_ok and result.p95_ok
    assert result.passed_count == 2 and not result.passed


# ── 走行 ──────────────────────────────────────────────────────────────


async def test_run_mode_drive_records_rows_and_ends_in_stop_hold(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    cfg.mode_drive.pedal_standby = False  # 待機位置なし（使っていないペダルは 0%）
    hw = await _held_stub(cfg)
    mode = _mode([(0.0, 0.0), (0.6, 0.0), (2.0, 6.0), (3.0, 6.0), (3.6, 0.0), (4.0, 0.0)])
    ff = FakeFF()
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")

    result = await mdmod.run_mode_drive(hw, cfg, mdmod.ModeDriveSetup(mode, ff), log=log)  # type: ignore[arg-type]
    await log.close()

    assert result.completed and result.abort_reason == ""
    assert result.run_duration_s == pytest.approx(4.0, abs=0.1)
    assert result.cycles >= 70  # 50ms 周期で 4s
    assert ff.calls == result.cycles
    rows = mrmod.rows_from_samples(result.samples)
    assert 75 <= len(rows) <= 85  # 50ms ごと（段1b: csv_interval_s 0.1→0.05）
    assert rows[0].t_s == pytest.approx(0.0, abs=0.06)
    assert rows[0].brake_pct == HOLD_PCT and rows[0].accel_pct == 0.0  # 停車中は保持ブレーキ
    assert any(r.accel_pct == 30.0 for r in rows)  # 加速中はアクセル
    assert all(r.accel_pct == 0.0 or r.brake_pct == 0.0 for r in rows)
    assert {r.segment for r in rows} == {"Low"}
    # 終わりは停車保持（止まっていなければ緩減速で止めてから）
    assert pulse_to_opening(await hw.brake.read_position()) == pytest.approx(HOLD_PCT, abs=0.02)
    assert await hw.accel.read_position() == 0

    # CSV にモード経過秒・偏差・effort が入り、CSV から同じ行を読み直せる
    from_csv = mrmod.rows_from_csv(log.csv_path)
    assert len(from_csv) == len(rows)
    assert from_csv[5].deviation_kmh == pytest.approx(rows[5].deviation_kmh, abs=2e-3)
    assert from_csv[5].ff_effort_pct == pytest.approx(rows[5].ff_effort_pct, abs=1e-3)

    # A7: FF のペダル別・指令と実開度（スタブは実開度 = 指令）・ステータス・周期の処理時間
    with log.csv_path.open(newline="", encoding="utf-8") as f:
        mode_rows = [r for r in csv.DictReader(f) if r["section"] == dlmod.SECTION_MODE_DRIVE]
    assert len(mode_rows) == len(rows)
    assert any(float(r["accel_ff_pct"]) > 0.0 for r in mode_rows)
    assert all(r["accel_ff_pct"] and r["brake_ff_pct"] and r["cycle_ms"] for r in mode_rows)
    assert all(r["accel_actual_pct"] == r["accel_cmd_pct"] for r in mode_rows)
    assert all(r["brake_actual_mm"] == r["brake_cmd_mm"] for r in mode_rows)
    assert {r["brake_servo_on"] for r in mode_rows} == {"1"}
    assert len(run_cycle_ms := [float(r["cycle_ms"]) for r in mode_rows]) == len(rows)
    assert min(run_cycle_ms) >= 0.0


def test_cycle_time_text_counts_cycles_over_period() -> None:
    text = mdmod.cycle_time_text([10.0] * 18 + [40.0, 60.0], 0.05)
    assert text == "平均 14.0 / p95 40.0 / 最大 60.0 ms（周期 50ms を超えた 1 周期）"


async def test_run_mode_drive_limit_stops_early(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = await _held_stub(cfg)
    mode = _mode([(0.0, 0.0), (0.2, 0.0), (5.0, 8.0), (30.0, 8.0), (31.0, 0.0)])
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    result = await mdmod.run_mode_drive(
        hw, cfg, mdmod.ModeDriveSetup(mode, FakeFF()), log=log, limit_s=1.5  # type: ignore[arg-type]
    )
    await log.close()
    assert result.completed
    assert result.mode_duration_s == 1.5
    assert result.run_duration_s == pytest.approx(1.5, abs=0.1)
    # 走行中に打ち切ったので緩減速で止まっている
    assert await hw.can.read_speed() < 0.05
    assert any(s.section == dlmod.SECTION_DECEL_TO_STOP for s in log.samples)


async def test_run_mode_drive_releases_pedals_on_overcurrent(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = await _held_stub(cfg)
    assert isinstance(hw.brake, hwmod.StubActuator)
    calls = 0

    async def spiking_current() -> float:
        nonlocal calls
        calls += 1
        return 1e6 if calls > 10 else 0.0

    hw.brake.read_current = spiking_current  # type: ignore[method-assign]
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    mode = _mode([(0.0, 0.0), (5.0, 0.0)])
    result = await mdmod.run_mode_drive(hw, cfg, mdmod.ModeDriveSetup(mode, FakeFF()), log=log)  # type: ignore[arg-type]
    await log.close()
    assert not result.completed
    assert "過電流" in result.abort_reason
    assert hw.brake.position == 0 and hw.accel.position == 0  # ペダルを離している


async def test_run_mode_drive_stops_on_alarm(tmp_path: Path) -> None:
    """アクチュエータのアラームは 1.0s ごとの確認で検知し、DriveError で中断してペダルを離す。"""
    cfg = _tmp_cfg(tmp_path)
    hw = await _held_stub(cfg)
    assert isinstance(hw.accel, hwmod.StubActuator)
    calls = 0

    async def alarming() -> bool:
        nonlocal calls
        calls += 1
        return calls > 2  # 1・2 回目（t=0, 1.0s）は正常、3 回目（t=2.0s）でアラーム

    hw.accel.is_alarm_active = alarming  # type: ignore[method-assign]
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    mode = _mode([(0.0, 0.0), (5.0, 8.0), (30.0, 8.0)])
    result = await mdmod.run_mode_drive(hw, cfg, mdmod.ModeDriveSetup(mode, FakeFF()), log=log)  # type: ignore[arg-type]
    await log.close()
    assert not result.completed
    assert "アラーム" in result.abort_reason and "アクセル" in result.abort_reason
    assert result.run_duration_s == pytest.approx(2.0, abs=0.1)
    assert hw.brake.position == 0 and hw.accel.position == 0  # ペダルを離している


async def test_alarm_is_polled_once_per_second_not_every_cycle(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = await _held_stub(cfg)
    assert isinstance(hw.accel, hwmod.StubActuator)
    calls = 0

    async def counting_alarm() -> bool:
        nonlocal calls
        calls += 1
        return False

    hw.accel.is_alarm_active = counting_alarm  # type: ignore[method-assign]
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    mode = _mode([(0.0, 0.0), (3.0, 0.0)])
    result = await mdmod.run_mode_drive(hw, cfg, mdmod.ModeDriveSetup(mode, FakeFF()), log=log)  # type: ignore[arg-type]
    await log.close()
    assert result.completed
    every = round(ALARM_CHECK_INTERVAL_S / cfg.control.loop_interval_s)
    assert every == 20
    assert calls == len(range(0, result.cycles, every))
    assert 1 < calls < result.cycles  # 毎周期は呼んでいない


async def test_run_mode_drive_stops_on_zero_current_after_having_moved(tmp_path: Path) -> None:
    """指令位置>0 なのに、一度は流れていた電流が 1.0s 0mA のままなら中断する（脱落の安全網）。"""
    cfg = _tmp_cfg(tmp_path)
    hw = await _held_stub(cfg)
    assert isinstance(hw.accel, hwmod.StubActuator)
    calls = 0

    async def dropping_current() -> float:
        nonlocal calls
        calls += 1
        return 300.0 if calls <= 3 else 0.0

    hw.accel.read_current = dropping_current  # type: ignore[method-assign]
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    mode = _mode([(0.0, 0.0), (5.0, 8.0), (30.0, 8.0)])
    result = await mdmod.run_mode_drive(hw, cfg, mdmod.ModeDriveSetup(mode, FakeFF()), log=log)  # type: ignore[arg-type]
    await log.close()
    assert not result.completed
    assert "0mA" in result.abort_reason and "アクセル" in result.abort_reason
    assert hw.brake.position == 0 and hw.accel.position == 0  # ペダルを離している


async def test_zero_current_never_nonzero_does_not_abort(tmp_path: Path) -> None:
    """一度も電流>0 を返さない軸（スタブ相当。既定の StubActuator は常に 0mA）では中断しない。"""
    cfg = _tmp_cfg(tmp_path)
    hw = await _held_stub(cfg)
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    mode = _mode([(0.0, 0.0), (5.0, 8.0), (6.0, 8.0)])
    result = await mdmod.run_mode_drive(hw, cfg, mdmod.ModeDriveSetup(mode, FakeFF()), log=log)  # type: ignore[arg-type]
    await log.close()
    assert result.completed  # StubActuator.read_current() は常に 0.0 だが誤検知しない
    rows = mrmod.rows_from_samples(result.samples)
    assert any(r.accel_pct > 0.0 for r in rows)  # アクセルは実際に開いていた


async def test_unused_pedal_waits_at_standby_below_deadband(tmp_path: Path) -> None:
    """使っていないペダルは 0% ではなく「不感帯 − 余裕」で待つ（phase は FF の選択のまま）。"""
    cfg = _tmp_cfg(tmp_path)
    accel_sb, brake_sb = mdmod.standby_openings(cfg)
    assert accel_sb == pytest.approx(cfg.feedforward.accel_deadband_pct - 2.0)
    assert brake_sb == pytest.approx(cfg.feedforward.brake_deadband_pct - 2.0)
    assert 0.0 < accel_sb and 0.0 < brake_sb
    hw = await _held_stub(cfg)
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    mode = _mode([(0.0, 0.0), (0.6, 0.0), (2.0, 6.0), (3.0, 6.0), (3.6, 0.0), (4.0, 0.0)])
    result = await mdmod.run_mode_drive(hw, cfg, mdmod.ModeDriveSetup(mode, FakeFF()), log=log)  # type: ignore[arg-type]
    await log.close()
    assert result.completed
    samples = result.samples
    accel_rows = [s for s in samples if s.phase == mdmod.PHASE_ACCEL]
    brake_rows = [s for s in samples if s.phase == mdmod.PHASE_BRAKE]
    assert accel_rows and brake_rows
    for s in accel_rows:  # アクセル中のブレーキは待機位置
        assert s.data.accel_opening == 30.0
        assert s.data.brake_opening == pytest.approx(brake_sb)
        assert s.data.brake_pos == opening_to_pulse(brake_sb)
    for s in brake_rows:  # ブレーキ中（停車保持を含む）のアクセルは待機位置
        assert s.data.accel_opening == pytest.approx(accel_sb)
        assert s.data.accel_pos == opening_to_pulse(accel_sb)
    # FF が待機位置より浅いブレーキ（12%）を出したら、待機位置ではなく FF の値
    assert brake_sb < 12.0 and any(s.data.brake_opening == 12.0 for s in brake_rows)
    # 走行後は従来通り: アクセル 0・ブレーキ停車保持
    assert await hw.accel.read_position() == 0
    assert pulse_to_opening(await hw.brake.read_position()) == pytest.approx(HOLD_PCT, abs=0.02)


async def test_governor_cap_to_zero_stops_at_brake_standby(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """ガバナーが頭打ちを 0% まで下げても、ブレーキは原点まで戻さず待機位置で止める。"""
    cfg = _tmp_cfg(tmp_path)
    _, brake_sb = mdmod.standby_openings(cfg)

    def cap_to_zero(
        self: mdmod.DecelGovernor, now: float, speed: float, brake: float
    ) -> tuple[float, bool]:
        return (0.0, True) if brake > 0.0 else (brake, False)

    monkeypatch.setattr(mdmod.DecelGovernor, "apply", cap_to_zero)
    hw = await _held_stub(cfg)
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    mode = _mode([(0.0, 0.0), (1.0, 0.0)])  # 停車保持のブレーキ指令 → ガバナーが 0% に頭打ち
    result = await mdmod.run_mode_drive(
        hw, cfg, mdmod.ModeDriveSetup(mode, FakeFF()), log=log, limit_s=0.5  # type: ignore[arg-type]
    )
    await log.close()
    governed = [s for s in result.samples if s.phase == mdmod.PHASE_BRAKE_GOVERNED]
    assert governed and all(s.governor_active for s in governed)
    assert all(s.data.brake_opening == pytest.approx(brake_sb) for s in governed)
    assert all(s.data.brake_pos == opening_to_pulse(brake_sb) for s in governed)


async def test_abort_releases_pedals_to_zero_even_with_standby(tmp_path: Path) -> None:
    """中断時は待機位置ではなく両軸 0 に戻す（既存の安全動作を変えない）。"""
    cfg = _tmp_cfg(tmp_path)
    assert cfg.mode_drive.pedal_standby
    hw = await _held_stub(cfg)
    assert isinstance(hw.accel, hwmod.StubActuator)
    calls = 0

    async def alarming() -> bool:
        nonlocal calls
        calls += 1
        return calls > 1

    hw.accel.is_alarm_active = alarming  # type: ignore[method-assign]
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    mode = _mode([(0.0, 0.0), (5.0, 8.0), (30.0, 8.0)])
    result = await mdmod.run_mode_drive(hw, cfg, mdmod.ModeDriveSetup(mode, FakeFF()), log=log)  # type: ignore[arg-type]
    await log.close()
    assert not result.completed
    assert hw.brake.position == 0 and hw.accel.position == 0  # type: ignore[attr-defined]


def test_standby_label_and_disabled() -> None:
    cfg = cfgmod.load_config(Path("tests/research/config_testVehicle.yaml"))
    cfg.feedforward.accel_deadband_pct, cfg.feedforward.brake_deadband_pct = 10.0, 13.16
    assert mdmod.standby_label(cfg) == "8.00% / 11.16%（不感帯 − 2.00%）"
    cfg.feedforward.accel_deadband_pct = 1.0  # 余裕より浅い不感帯は 0% で止める
    assert mdmod.standby_openings(cfg)[0] == 0.0
    cfg.mode_drive.pedal_standby = False
    assert mdmod.standby_openings(cfg) == (0.0, 0.0)
    assert mdmod.standby_label(cfg) == "なし（0%）"


def test_pedal_stats_use_phase_not_opening_with_standby() -> None:
    """待機位置で開度 > 0 のまま待つ行は「そのペダルを使った」に数えない。"""
    def row(t: float, phase: str, accel: float, brake: float) -> mrmod.ModeRow:
        return mrmod.ModeRow(
            t_s=t, ref_kmh=20.0, actual_kmh=20.0, accel_pct=accel, brake_pct=brake,
            ff_effort_pct=0.0, pid_effort_pct=0.0, effort_pct=0.0, segment="Low", phase=phase,
        )

    rows = [
        row(0.0, "ACCEL", 15.0, 11.16),
        row(0.1, "ACCEL", 15.0, 11.16),
        row(0.2, "BRAKE", 8.0, 16.0),
        row(0.3, "BRAKE_GOV", 8.0, 11.16),  # ガバナーで待機位置まで頭打ち → 不感帯より浅い指令
        row(0.4, "COAST", 8.0, 11.16),
    ]
    cfg = cfgmod.load_config(Path("tests/research/config_testVehicle.yaml"))
    st = mrmod.pedal_stats(rows, 10.0, 13.16, feedforward_params(cfg))
    assert st.accel_active_s == pytest.approx(0.2)
    assert st.accel_in_deadband_s == 0.0
    assert st.brake_active_moving_s == pytest.approx(0.2)
    assert st.brake_in_deadband_moving_s == pytest.approx(0.1)
    assert st.accel_max_pct == 15.0 and st.brake_max_moving_pct == 16.0
    assert st.switches == 1
    assert st.governor_s == pytest.approx(0.1)


# ── レポート ──────────────────────────────────────────────────────────


def _synthetic_rows(n: int = 1200) -> list[mrmod.ModeRow]:
    rows = []
    for i in range(n):
        t = i * 0.1
        ref = max(0.0, min(40.0, 4.0 * (t - 10.0))) if t < 80 else max(0.0, 40.0 - 3.0 * (t - 80))
        dev = -0.8 if 10 < t < 20 else 1.4 if 81 < t < 83 else 0.1
        effort = 15.0 if 10 < t < 20 else -14.0 if t > 80 else 5.0
        rows.append(mrmod.ModeRow(
            t_s=t, ref_kmh=ref, actual_kmh=ref + dev,
            accel_pct=max(0.0, effort), brake_pct=max(0.0, -effort),
            ff_effort_pct=effort, pid_effort_pct=0.0, effort_pct=effort,
            segment="Low" if t < 60 else "Mid", phase="ACCEL" if effort > 0 else "BRAKE",
        ))
    return rows


def test_write_mode_report_creates_markdown_and_figures(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    info = mrmod.RunInfo(
        label="FF", title="手順 3: FF のみでモード走行", controller="FF のみ",
        csv_path=tmp_path / "x.csv", hw_mode="stub", mode_name="test_mode",
        started_at=datetime(2026, 9, 11, 10, 0), completed=True, cycles=2400, overruns=0,
    )
    rows = _synthetic_rows()
    path = mrmod.write_mode_report(rows, cfg, info, tmp_path)
    assert path.name == "report20260911_RunFF.md"
    text = path.read_text(encoding="utf-8")
    assert "## 1. 結論（プライマリー KPI）" in text
    assert "最大逸脱 1.40 km/h" in text
    assert "| Mid |" in text and "| 加速 |" in text
    for name in ("overview", "deviation", "worst_zoom", "distribution"):
        fig = tmp_path / "report20260911_RunFF" / f"{name}.png"
        assert fig.stat().st_size > 1000
        assert f"report20260911_RunFF/{name}.png" in text
    # 同じ日の 2 本目は上書きしない
    assert mrmod.write_mode_report(rows, cfg, info, tmp_path).name == "report20260911_RunFF_2.md"


def test_driving_states_classify_by_reference_slope() -> None:
    rows = _synthetic_rows()
    states = mrmod.driving_states(rows)
    by_t = {round(r.t_s, 1): s for r, s in zip(rows, states, strict=True)}
    assert by_t[5.0] == mrmod.STATE_STOP
    assert by_t[15.0] == mrmod.STATE_ACCEL
    assert by_t[50.0] == mrmod.STATE_CRUISE
    assert by_t[85.0] == mrmod.STATE_DECEL


def test_report_can_be_rebuilt_from_csv(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    wall0 = datetime(2026, 9, 11, tzinfo=UTC)
    samples = [
        dlmod.DriveSample(
            elapsed_s=30.0 + r.t_s,
            timestamp=wall0 + timedelta(seconds=r.t_s),
            data=DriveLogData(
                ref_speed_kmh=r.ref_kmh, actual_speed_kmh=r.actual_kmh,
                accel_opening=r.accel_pct, brake_opening=r.brake_pct,
                accel_pos=opening_to_pulse(r.accel_pct), brake_pos=opening_to_pulse(r.brake_pct),
                accel_current=0.0, brake_current=0.0,
                plan_effort_pct=r.ff_effort_pct,
                trim_effort_pct=0.0,
                applied_effort_pct=r.effort_pct,
            ),
            section=dlmod.SECTION_MODE_DRIVE, phase=r.phase, pattern=r.segment, mode_time_s=r.t_s,
        )
        for r in _synthetic_rows(600)
    ]
    csv_path = tmp_path / "drive_log_stub_20260911_120000.csv"
    dlmod.write_csv(samples, csv_path)
    assert mrmod.main([str(csv_path), "--label", "FF", "--config", str(cfg.source_path)]) == 0
    assert (tmp_path / "report20260911_RunFF.md").exists()


# ── CLI 経由 ─────────────────────────────────────────────────────────


def test_step3_requires_step1_before_loading_mode(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    async def must_not_load(cfg: object, name: object) -> None:
        raise AssertionError("HW 未初期化なら DB・モデルを読みに行かない")

    monkeypatch.setattr(mainmod, "prepare_mode_drive", must_not_load)
    assert mainmod.main(["--only", "3"]) == 4
    assert "手順 1 を先に実行" in capsys.readouterr().out


def test_step3_prepare_error_stops_before_pre_drive_check(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    async def failing_prepare(cfg: object, name: object) -> None:
        raise cfgmod.ConfigError("走行モード 'x' が driving_modes にありません")

    async def must_not_check(hw: object, cfg: object, **kwargs: object) -> int:
        raise AssertionError("準備に失敗したら走行前チェックに進まない")

    cfg = _tmp_cfg(tmp_path)
    monkeypatch.setattr(mainmod, "prepare_mode_drive", failing_prepare)
    monkeypatch.setattr(mainmod, "run_pre_drive_check", must_not_check)
    assert mainmod.main(["--steps", "1,3", "--config", str(cfg.source_path)]) == 2
    out = capsys.readouterr().out
    assert "driving_modes にありません" in out
    assert "終了処理: 完了" in out


def test_step3_end_to_end_with_stub(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    mode = _mode([(0.0, 0.0), (0.5, 0.0), (2.0, 5.0), (3.0, 0.0), (3.5, 0.0)])

    async def fake_prepare(cfg: object, name: object) -> mdmod.ModeDriveSetup:
        return mdmod.ModeDriveSetup(mode, FakeFF())  # type: ignore[arg-type]

    async def fake_pre_check(hw: hwmod.ResearchHardware, cfg: object, **kwargs: object) -> int:
        """走行前チェックは test_research_pre_drive_check.py で確認する。停車させて返す。"""
        # 手順 3 の前なので stop_brake_opening_pct へ一気に踏む分岐（next_step != 2）で呼ばれる
        assert kwargs.get("next_step") == 3
        assert isinstance(hw.can, hwmod.StubCANReader)
        hw.can.speed_kmh = 0.0
        return 0

    cfg = _tmp_cfg(tmp_path)
    monkeypatch.setattr(mainmod, "prepare_mode_drive", fake_prepare)
    monkeypatch.setattr(mainmod, "run_pre_drive_check", fake_pre_check)
    assert mainmod.main(["--steps", "1,3", "--config", str(cfg.source_path)]) == 0
    out = capsys.readouterr().out
    assert "手順 3 完了" in out
    results = tmp_path / "results"
    reports = list(results.glob("report*_RunFF.md"))
    assert len(reports) == 1
    csvs = list(results.glob("drive_log_stub_*.csv"))
    assert len(csvs) == 1
    assert len(mrmod.rows_from_csv(csvs[0])) >= 30


def test_step3_abort_writes_report_and_returns_5(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    mode = _mode([(0.0, 0.0), (5.0, 0.0)])

    async def fake_prepare(cfg: object, name: object) -> mdmod.ModeDriveSetup:
        return mdmod.ModeDriveSetup(mode, FakeFF())  # type: ignore[arg-type]

    async def fake_pre_check(hw: hwmod.ResearchHardware, cfg: object, **kwargs: object) -> int:
        assert isinstance(hw.can, hwmod.StubCANReader)
        hw.can.speed_kmh = 0.0
        calls = 0

        async def failing_read() -> float:
            nonlocal calls
            calls += 1
            if calls > 15:
                raise OSError("CAN バスが止まった（テスト）")
            return 0.0

        hw.can.read_speed = failing_read  # type: ignore[method-assign]
        return 0

    cfg = _tmp_cfg(tmp_path)
    monkeypatch.setattr(mainmod, "prepare_mode_drive", fake_prepare)
    monkeypatch.setattr(mainmod, "run_pre_drive_check", fake_pre_check)
    assert mainmod.main(["--steps", "1,3", "--config", str(cfg.source_path)]) == 5
    out = capsys.readouterr().out
    assert "CAN 車速を読めません" in out
    assert "終了処理: 完了" in out
    report = next((tmp_path / "results").glob("report*_RunFF.md"))
    assert "中断: CAN 車速を読めません" in report.read_text(encoding="utf-8")


# ── V2: 実測車速の配線（ActualSpeedHistory・_ff_inputs） ────────────────


def test_actual_speed_history_interpolates_and_falls_back() -> None:
    hist = mdmod.ActualSpeedHistory(max_span_s=2.0)
    assert hist.at(5.0) == 0.0  # 空なら 0
    hist.record(0.0, 10.0)
    hist.record(1.0, 20.0)
    assert hist.at(-1.0) == 10.0  # 最初より前は最初の値
    assert hist.at(0.5) == pytest.approx(15.0)  # 線形補間
    assert hist.at(5.0) == 20.0  # 最後より後は最後の値


def test_actual_speed_history_drops_points_older_than_span() -> None:
    hist = mdmod.ActualSpeedHistory(max_span_s=1.0)
    for i in range(5):
        hist.record(float(i), float(i))
    assert hist.at(0.0) != 0.0 or len(hist._points) < 5  # noqa: SLF001 - 古い点は捨てている


class _FakeCandidate:
    candidate = "C1"
    uses_actual_speed = False
    horizons = (0.5, 1.0)
    past_horizons = (0.5,)


class _FakeC4:
    candidate = "C4"
    uses_actual_speed = True
    horizons = (0.5, 1.0)
    past_horizons = (0.5,)


class _FakeC5:
    candidate = "C5"
    uses_actual_speed = True
    horizons = (0.5, 1.0)
    past_horizons = (0.5,)


class _ConstModel:
    """与えた定数を返すだけの推定器（predict_effort の停車レジーム分岐だけを見るため）。

    tests/research/test_research_ff_candidate.py の `_Const` と同じ役割。
    """

    def __init__(self, value: float) -> None:
        self.value = value

    def predict(self, x: np.ndarray) -> np.ndarray:
        return np.full(len(x), self.value, dtype=float)


def _bare_mode_run(ff: object) -> mdmod._ModeRun:  # noqa: SLF001 - 単体テストで直接組み立てる
    mode = _mode([(0.0, 0.0), (10.0, 100.0)])
    run = object.__new__(mdmod._ModeRun)
    run.ff = ff  # type: ignore[attr-defined]
    run.ref = mdmod.ReferenceSpeed(mode)
    run.actual_history = mdmod.ActualSpeedHistory(max(ff.past_horizons, default=1.0) + 0.5)  # type: ignore[attr-defined]
    return run


def test_ff_inputs_uses_reference_speed_for_c1() -> None:
    run = _bare_mode_run(_FakeCandidate())
    t, ref = 4.0, 40.0
    v0, future, past = run._ff_inputs(t, ref, speed=55.0)  # noqa: SLF001
    assert v0 == ref
    assert future == [run.ref.at(t + h) for h in _FakeCandidate.horizons]
    assert past == [run.ref.at(t - h) for h in _FakeCandidate.past_horizons]


def test_ff_inputs_c4_rebases_onto_actual_speed_keeping_reference_delta() -> None:
    run = _bare_mode_run(_FakeC4())
    t, ref, speed = 4.0, 40.0, 55.0
    v0, future, past = run._ff_inputs(t, ref, speed)  # noqa: SLF001
    assert v0 == speed
    for h, f in zip(_FakeC4.horizons, future, strict=True):
        assert f == pytest.approx(speed + (run.ref.at(t + h) - ref))
    for h, p in zip(_FakeC4.past_horizons, past, strict=True):
        assert p == pytest.approx(speed + (run.ref.at(t - h) - ref))


def test_ff_inputs_c5_uses_absolute_reference_future_and_actual_past() -> None:
    run = _bare_mode_run(_FakeC5())
    run.actual_history.record(3.5, 33.0)
    run.actual_history.record(4.0, 55.0)
    t, ref, speed = 4.0, 40.0, 55.0
    v0, future, past = run._ff_inputs(t, ref, speed)  # noqa: SLF001
    assert v0 == speed
    assert future == [run.ref.at(t + h) for h in _FakeC5.horizons]  # 基準の絶対値
    assert past == [run.actual_history.at(t - h) for h in _FakeC5.past_horizons]  # 実測履歴
    assert past[0] == pytest.approx(33.0)


def _bare_mode_run_for(ff: object, mode: DrivingMode) -> mdmod._ModeRun:  # noqa: SLF001
    """`_bare_mode_run` の mode 差し替え版（停車区間を持つモードで確認したいテスト用）。"""
    run = object.__new__(mdmod._ModeRun)
    run.ff = ff  # type: ignore[attr-defined]
    run.ref = mdmod.ReferenceSpeed(mode)
    run.actual_history = mdmod.ActualSpeedHistory(max(ff.past_horizons, default=1.0) + 0.5)  # type: ignore[attr-defined]
    return run


def test_ff_inputs_stop_regime_forces_reference_speed_for_c4_and_c5() -> None:
    """基準が停車レジームなら C4・C5 も基準車速ベースの入力になる（A8 報告 5 章 #2）。

    停車指示中（t=0〜10s は基準 0 km/h）でも実車速がクリープ等で動いていることがある
    （ここでは speed=3.0）。v0 を実車速にする C4・C5 は「実車速が 0.02 km/h を下回る」
    条件が現実には成立せず停車保持に入れないため、基準車速が停車レジームのときは
    uses_actual_speed に関係なく C1 と同じ入力（基準車速そのもの）を返す。
    """
    stopped_mode = _mode([(0.0, 0.0), (10.0, 0.0), (20.0, 100.0)])
    t, speed = 5.0, 3.0
    ref = mdmod.ReferenceSpeed(stopped_mode).at(t)
    assert ref == 0.0  # 前提: t=5 は停車区間の途中

    for ff_cls in (_FakeCandidate, _FakeC4, _FakeC5):
        run = _bare_mode_run_for(ff_cls(), stopped_mode)
        v0, future, past = run._ff_inputs(t, ref, speed)  # noqa: SLF001
        assert v0 == 0.0, ff_cls.candidate
        assert future == [run.ref.at(t + h) for h in ff_cls.horizons], ff_cls.candidate
        assert future[0] == 0.0, ff_cls.candidate
        assert past == [run.ref.at(t - h) for h in ff_cls.past_horizons], ff_cls.candidate


def test_ff_inputs_stop_regime_feeds_into_stop_brake_hold() -> None:
    """停車レジームの入力を実際に predict_effort へ渡すと停車保持ブレーキが出る（結合確認）。

    _ff_inputs だけでなく CandidateFeedforward.predict_effort まで通して、A8 報告 5 章 #2 の
    症状（停車指示中に実車速が動いていて停車保持に入らない）が解消したことを確認する。
    """
    stopped_mode = _mode([(0.0, 0.0), (10.0, 0.0), (20.0, 100.0)])
    ff = ff_candidate.CandidateFeedforward()
    ff.set_params(FeedforwardParams(stop_brake_opening_pct=HOLD_PCT))
    ff._accel_model = _ConstModel(20.0)  # noqa: SLF001 - テスト用の差し込み
    ff._brake_model = _ConstModel(20.0)  # noqa: SLF001

    run = _bare_mode_run_for(ff, stopped_mode)
    t, speed = 5.0, 3.0  # 停車指示中だが実車速はクリープ等で動いている
    ref = run.ref.at(t)
    v0, future, past = run._ff_inputs(t, ref, speed)  # noqa: SLF001

    assert ff.predict_effort(v0, future, past) == pytest.approx(-HOLD_PCT)
