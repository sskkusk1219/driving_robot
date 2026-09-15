"""研究開発用ハーネス 手順 2-0（ペダル探索）のユニットテスト。

スタブ車両は設定の不感帯とは独立した「真の遊び」（アクセル 6%・ブレーキ 8%）を持つ。
探索がそれを当て、停止確認開度 + マージンで停車保持することを確かめる。
時間を縮めるため、刻みを 1mm・待ちを 0.3s にしている（判定ロジックは既定と同じ）。
"""

from __future__ import annotations

from pathlib import Path

import pytest

from src.domain.control.conversions import VEHICLE_STOP_SPEED_KMH
from tests.research import config as cfgmod
from tests.research import hardware as hwmod
from tests.research import pedal_search as psmod
from tests.research.vehicle import build_vehicle_profile, opening_to_pulse, pulse_to_opening

WINDOW_S = 0.6
FAST = {
    "pedal_search.step_mm": 1.0,
    "pedal_search.dwell_s": 0.3,
    "pedal_search.onset_margin_kmh": 0.05,  # スタブは車速ノイズが無い
    "pedal_search.creep_stable_kmh": 0.05,
    "pedal_search.creep_timeout_s": 10.0,
    "pedal_search.stop_hold_margin_pct": 3.0,
    "feedforward.creep_rate_kmhs": 3.0,  # クリープ速度まで早く上げる
}


def _tmp_cfg(tmp_path: Path, **extra: float) -> cfgmod.ResearchConfig:
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    cfg.save(
        {
            "output.results_dir": str(tmp_path / "results"),
            "feedforward.model_path": str(tmp_path / "results" / "models" / "ff.pkl"),
            "output.plot": False,
            **FAST,
            **extra,
        }
    )
    return cfgmod.load_config(path)


async def _ready_stub(cfg: cfgmod.ResearchConfig) -> hwmod.ResearchHardware:
    hw = hwmod.build_hardware(cfg, hwmod.HW_STUB)
    await hwmod.run_initialize(hw)
    return hw


async def test_search_finds_stub_play_and_holds_stop(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = await _ready_stub(cfg)
    # 停車までを短くする（既定ゲインだとクリープの押しとほぼ釣り合い、止まるまで十数秒かかる）
    hw.can.vehicle.brake_gain_kmhs_per_pct = 1.5  # type: ignore[attr-defined]
    yaml_before = cfg.source_path.read_text(encoding="utf-8")

    result = await psmod.run_pedal_search(hw, cfg, window_s=WINDOW_S)

    step_pct = pulse_to_opening(psmod.search_step_pulse(cfg))
    # 検出は応答を見てからなので真値より浅くは出ない。深い側は立ち上がり遅れぶん（数刻み）まで許す
    assert hwmod.STUB_ACCEL_PLAY_PCT <= result.accel_deadband_pct
    assert result.accel_deadband_pct <= hwmod.STUB_ACCEL_PLAY_PCT + 3 * step_pct
    assert hwmod.STUB_BRAKE_PLAY_PCT <= result.brake_deadband_pct
    assert result.brake_deadband_pct <= hwmod.STUB_BRAKE_PLAY_PCT + 3 * step_pct
    assert result.stop_confirm_pct >= result.brake_deadband_pct
    # 停車保持 = 停止確認 + マージン。その位置まで踏んで停車している
    assert result.stop_brake_opening_pct == pytest.approx(result.stop_confirm_pct + 3.0, abs=0.01)
    assert hw.brake.position == opening_to_pulse(result.stop_brake_opening_pct)
    assert hw.accel.position == 0
    assert await hw.can.read_speed() < VEHICLE_STOP_SPEED_KMH
    # スタブは YAML を書き換えない
    assert cfg.source_path.read_text(encoding="utf-8") == yaml_before
    await hwmod.shutdown(hw)


async def test_search_fails_without_creep(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path, **{"feedforward.creep_rate_kmhs": 0.0,
                                "pedal_search.creep_timeout_s": 1.0})
    hw = await _ready_stub(cfg)
    hw.can.speed_kmh = 0.0  # type: ignore[attr-defined]  # 停車から始める（クリープしない車）
    with pytest.raises(hwmod.DriveError, match="クリープ"):
        await psmod.run_pedal_search(hw, cfg, window_s=WINDOW_S)
    await hwmod.shutdown(hw)


async def test_search_fails_when_accel_has_no_effect(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path, **{"pedal_search.accel_max_pct": 3.0})
    hw = await _ready_stub(cfg)
    with pytest.raises(hwmod.DriveError, match="アクセルを 3% まで踏んでも"):
        await psmod.run_pedal_search(hw, cfg, window_s=WINDOW_S)
    await hwmod.shutdown(hw)


async def test_step_to_position_moves_in_steps_both_ways() -> None:
    axis = hwmod.StubActuator("brake", connected=True)
    visited: list[int] = []
    original = axis.move_to_position

    async def record(pos: int, *, smooth_over_s: float | None = None) -> None:
        visited.append(pos)
        await original(pos, smooth_over_s=smooth_over_s)

    axis.move_to_position = record  # type: ignore[method-assign]
    assert await psmod.step_to_position(axis, 0, 250, step_pulse=100, dwell_s=0.0) == 250
    assert await psmod.step_to_position(axis, 250, 30, step_pulse=100, dwell_s=0.0) == 30
    assert visited == [100, 200, 250, 150, 50, 30]


def test_save_to_config_writes_measured_values(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    result = psmod.PedalSearchResult(
        creep_speed_kmh=4.9,
        accel_deadband_pct=6.32,
        brake_deadband_pct=8.42,
        stop_confirm_pct=14.74,
        stop_brake_opening_pct=24.74,
    )
    psmod.save_to_config(cfg, result)
    saved = cfgmod.load_config(cfg.source_path).feedforward
    assert saved.accel_deadband_pct == pytest.approx(6.32)
    assert saved.brake_deadband_pct == pytest.approx(8.42)
    assert saved.stop_brake_opening_pct == pytest.approx(24.74)


def test_apply_to_profile_overrides_measured_values() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    result = psmod.PedalSearchResult(5.0, 6.0, 8.0, 14.0, 24.0)
    ffp = result.apply_to_profile(build_vehicle_profile(cfg)).feedforward_params
    assert (ffp.accel_deadband_pct, ffp.brake_deadband_pct, ffp.stop_brake_opening_pct) == (
        6.0, 8.0, 24.0,
    )


def test_probe_list_must_be_ascending() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.learning.accel_deadband_probe_offsets_pct = [5.0, 1.0]
    assert any("accel_deadband_probe_offsets_pct" in p for p in cfgmod.validate_config(cfg))
