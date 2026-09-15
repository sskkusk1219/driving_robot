"""研究開発用ハーネス 走行後の緩減速（stop_decel.py）のユニットテスト。

スタブ車両で、不感帯の手前まで一発で動かしてから 1 刻みずつ踏むこと、上限Gを超えたときだけ
戻すこと、停車保持開度を超えて踏まないこと、停車しなければエラーになることを確かめる。
スタブはブレーキ→減速の遅れが無いので、時間を縮めるため刻み 1mm・待ち 0.2s にしている
（判定ロジックは既定と同じ）。惰行減速は小さい定数にして、ブレーキの効きだけで判定が動くようにする。
"""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from src.domain.control.conversions import VEHICLE_STOP_SPEED_KMH
from tests.research import config as cfgmod
from tests.research import hardware as hwmod
from tests.research import stop_decel as sdmod
from tests.research.pedal_search import PedalSearchResult
from tests.research.vehicle import build_vehicle_profile, opening_to_pulse

FAST = {
    "decel_stop.step_mm": 1.0,
    "decel_stop.dwell_s": 0.2,
    "decel_stop.slope_window_s": 0.2,
}
PEDAL = PedalSearchResult(
    creep_speed_kmh=5.0,
    accel_deadband_pct=6.3,
    brake_deadband_pct=8.4,
    stop_confirm_pct=12.0,
    stop_brake_opening_pct=22.0,
)
STEP = 100  # 1mm [pulse]
APPROACH = opening_to_pulse(PEDAL.brake_deadband_pct - 1.0)  # 不感帯 − approach_margin_pct
CEILING = opening_to_pulse(PEDAL.stop_brake_opening_pct)


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
    cfg = cfgmod.load_config(path)
    assert not [p for p in cfgmod.validate_config(cfg) if p.startswith("decel_stop")]
    return cfg


async def _moving_stub(
    cfg: cfgmod.ResearchConfig, *, speed_kmh: float, brake_gain: float
) -> hwmod.ResearchHardware:
    """speed_kmh で走っている（ペダルは両方原点）スタブ。惰行減速は 0.3 km/h/s の定数。"""
    hw = hwmod.build_hardware(cfg, hwmod.HW_STUB)
    await hwmod.run_initialize(hw)
    vehicle = hw.can.vehicle  # type: ignore[attr-defined]
    vehicle.params = replace(
        vehicle.params, coast_decel_speeds_kmh=(), coast_decel_kmhs=(), engine_brake_decel_kmhs=0.3
    )
    vehicle.brake_gain_kmhs_per_pct = brake_gain
    hw.can.speed_kmh = speed_kmh  # type: ignore[attr-defined]
    return hw


def _profile(cfg: cfgmod.ResearchConfig):  # noqa: ANN202
    return PEDAL.apply_to_profile(build_vehicle_profile(cfg))


def _record_moves(axis: hwmod.StubActuator) -> list[tuple[int, float | None]]:
    moves: list[tuple[int, float | None]] = []
    original = axis.move_to_position

    async def record(pos: int, *, smooth_over_s: float | None = None) -> None:
        moves.append((pos, smooth_over_s))
        await original(pos, smooth_over_s=smooth_over_s)

    axis.move_to_position = record  # type: ignore[method-assign]
    return moves


def test_decel_from_slope_is_least_squares() -> None:
    assert sdmod.decel_from_slope([(0.0, 10.0), (0.5, 9.4), (1.0, 9.0)]) == pytest.approx(1.0)
    assert sdmod.decel_from_slope([(0.0, 10.0)]) == 0.0


async def test_approaches_at_once_then_presses_one_step_at_a_time(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = await _moving_stub(cfg, speed_kmh=20.0, brake_gain=1.5)
    moves = _record_moves(hw.brake)  # type: ignore[arg-type]

    result = await sdmod.decelerate_to_stop(hw, cfg, _profile(cfg))

    # 不感帯の手前までは速度指定なし（最高速度）で一発
    assert moves[0] == (APPROACH, None)
    # その後は 1 刻みずつ踏み増すだけ（上限Gを超えないので戻さない）
    positions = [pos for pos, _ in moves]
    assert all(0 < b - a <= STEP for a, b in zip(positions, positions[1:], strict=False))
    assert all(smooth is not None for _, smooth in moves[1:])
    assert result.releases == 0
    assert result.presses >= 1
    assert result.max_decel_g < cfg.decel_stop.release_above_g
    # 停車保持開度で止まっている
    assert max(positions) == CEILING == hw.brake.position
    assert hw.accel.position == 0
    assert await hw.can.read_speed() < VEHICLE_STOP_SPEED_KMH
    await hwmod.shutdown(hw)


async def test_releases_one_step_only_when_decel_exceeds_limit(tmp_path: Path) -> None:
    # 1 刻みで約 0.18G 変わる強いブレーキ。0.25G を超えたら戻す
    cfg = _tmp_cfg(tmp_path, **{"decel_stop.release_above_g": 0.25})
    hw = await _moving_stub(cfg, speed_kmh=20.0, brake_gain=6.0)
    moves = _record_moves(hw.brake)  # type: ignore[arg-type]

    result = await sdmod.decelerate_to_stop(hw, cfg, _profile(cfg))

    positions = [pos for pos, _ in moves]
    deltas = [b - a for a, b in zip(positions, positions[1:], strict=False)]
    assert result.releases >= 1
    assert -STEP in deltas  # 戻すのは 1 刻みだけ
    assert all(-STEP <= d <= STEP for d in deltas)
    assert min(positions) == APPROACH  # 接近位置より浅くは戻さない
    assert hw.brake.position == CEILING
    assert await hw.can.read_speed() < VEHICLE_STOP_SPEED_KMH
    await hwmod.shutdown(hw)


async def test_waits_at_hold_opening_when_brake_is_weak(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = await _moving_stub(cfg, speed_kmh=8.0, brake_gain=0.3)  # 停車保持開度でも 0.13G 程度
    moves = _record_moves(hw.brake)  # type: ignore[arg-type]

    await sdmod.decelerate_to_stop(hw, cfg, _profile(cfg))

    assert max(pos for pos, _ in moves) == CEILING  # 停車保持開度を超えて踏まない
    assert "上限で待機" in capsys.readouterr().out
    assert await hw.can.read_speed() < VEHICLE_STOP_SPEED_KMH
    await hwmod.shutdown(hw)


async def test_raises_when_not_stopped_within_timeout(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path, **{"decel_stop.timeout_s": 0.5})
    hw = await _moving_stub(cfg, speed_kmh=20.0, brake_gain=0.0)  # ブレーキが効かない
    with pytest.raises(hwmod.DriveError, match="停車しません"):
        await sdmod.decelerate_to_stop(hw, cfg, _profile(cfg))
    await hwmod.shutdown(hw)


async def test_raises_when_can_fails(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = await _moving_stub(cfg, speed_kmh=20.0, brake_gain=1.5)

    async def broken() -> float:
        raise TimeoutError("CAN 車速が 0.2s 更新されていません")

    hw.can.read_speed = broken  # type: ignore[method-assign]
    with pytest.raises(hwmod.DriveError, match="CAN 車速を読めません"):
        await sdmod.decelerate_to_stop(hw, cfg, _profile(cfg))
    await hwmod.shutdown(hw)
