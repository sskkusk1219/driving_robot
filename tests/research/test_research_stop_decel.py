"""研究開発用ハーネス 走行後の緩減速（stop_decel.py。0.2G の PI）のユニットテスト。

スタブ車両で、効き始めへの接近・0.2G に近づくこと・0.3G 超えで戻すこと・最大開度で頭打ち・
表が無いときの既定・途中の開度から始めること・停車保持開度へ移すこと・タイムアウトを確かめる。
スタブはブレーキ→減速の遅れが無いので、時間を縮めるため減速度の窓を 0.3s にしている。
惰行減速は小さい定数にして、ブレーキの効きだけで判定が動くようにする。
スタブ車の G の値は合否に使わない（流れの確認だけ。範囲は広く取る）。
"""

from __future__ import annotations

import asyncio
from dataclasses import replace
from pathlib import Path

import pytest

from tests.research import config as cfgmod
from tests.research import hardware as hwmod
from tests.research import stop_decel as sdmod
from tests.research.pedal_search import PedalSearchResult
from tests.research.research_types import G_TO_KMHS, VEHICLE_STOP_SPEED_KMH
from tests.research.vehicle import build_vehicle_profile, opening_to_pulse

FAST = {
    "decel_stop.step_mm": 1.0,
    "decel_stop.pi_slope_window_s": 0.3,
}
# 2026-10-08 段2l-6: 触れた位置 min は yaml の vehicle.min_*_opening_pct（_tmp_cfg で書く）
MIN_ACCEL_PCT = 6.3
MIN_BRAKE_PCT = 8.4
PEDAL = PedalSearchResult(
    creep_speed_kmh=5.0,
    stop_confirm_pct=12.0,
    stop_brake_opening_pct=22.0,
)
STEP = 100  # 1mm [pulse]
APPROACH = opening_to_pulse(MIN_BRAKE_PCT)  # 表なし ＝ 効き始めは min


def _tmp_cfg(tmp_path: Path, **extra: float) -> cfgmod.ResearchConfig:
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    cfg.save(
        {
            "output.results_dir": str(tmp_path / "results"),
            "feedforward.model_path": str(tmp_path / "results" / "models" / "ff.pkl"),
            "output.plot": False,
            "vehicle.min_accel_opening_pct": MIN_ACCEL_PCT,
            "vehicle.min_brake_opening_pct": MIN_BRAKE_PCT,
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


class _FakeTable:
    """meta と accel だけ持つ表。accel(v, u) = slope × u（u はブレーキが負 ＝ 減速が負）。"""

    def __init__(self, meta: dict, slope_kmhs_per_pct: float = 2.0) -> None:
        self.meta = meta
        self.slope = slope_kmhs_per_pct

    def accel(self, v: float, u: float, **_: object) -> float:
        return self.slope * u


def test_local_gain_k_defaults_without_table_and_reads_table_slope() -> None:
    assert sdmod.local_gain_k_g_per_pct(
        None, 50.0, 10.0, min_pct=8.4, default_k=0.15) == 0.15
    table = _FakeTable({"brake_onset_u_pct": 1.0})
    k = sdmod.local_gain_k_g_per_pct(table, 50.0, 12.0, min_pct=8.4, default_k=0.15)
    assert k == pytest.approx(2.0 / G_TO_KMHS)
    # 表の傾きが 0 以下なら既定
    flat = _FakeTable({}, slope_kmhs_per_pct=0.0)
    assert sdmod.local_gain_k_g_per_pct(flat, 50.0, 12.0, min_pct=8.4, default_k=0.15) == 0.15


def test_local_gain_k_multiplies_meta_local_gain_ratio() -> None:
    entry = {
        "speed_rep_kmh": [50.0], "u_from_onset_pct": [0.0, 5.0],
        "ratio": [[2.0, 2.0]], "n": [[1, 1]],
    }
    table = _FakeTable({"brake_onset_u_pct": 1.0, "local_gain_brake": entry})
    k = sdmod.local_gain_k_g_per_pct(table, 50.0, 12.0, min_pct=8.4, default_k=0.15)
    assert k == pytest.approx(2.0 * 2.0 / G_TO_KMHS)


async def test_without_table_approaches_min_and_stops_with_weakest_default(
    tmp_path: Path,
) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = await _moving_stub(cfg, speed_kmh=10.0, brake_gain=1.5)
    moves = _record_moves(hw.brake)  # type: ignore[arg-type]

    result = await sdmod.decelerate_to_stop(hw, cfg, _profile(cfg))

    # 表なし: 効き始めは min。接近は速度指定なし（最高速度）で一発
    assert moves[0] == (APPROACH, None)
    ceiling = opening_to_pulse(build_vehicle_profile(cfg).max_brake_opening)
    assert all(APPROACH <= pos <= ceiling for pos, _ in moves)
    # 既定は「強めの効き」を仮定して弱く踏む（スタブより強い仮定）ので 0.3G は超えない
    assert result.max_decel_g < cfg.decel_stop.release_above_g
    assert hw.brake.position == opening_to_pulse(PEDAL.stop_brake_opening_pct)
    assert hw.accel.position == 0
    assert await hw.can.read_speed() < VEHICLE_STOP_SPEED_KMH
    await hwmod.shutdown(hw)


async def test_pi_approaches_target_g_with_table(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = await _moving_stub(cfg, speed_kmh=25.0, brake_gain=1.5)
    moves = _record_moves(hw.brake)  # type: ignore[arg-type]
    # スタブの効き（1.5 km/h/s per %）と同じ傾きの表・遅れ 0.25 s（Ti = 1 s）
    table = _FakeTable({"dither_brake_lag_s": 0.25}, slope_kmhs_per_pct=1.5)

    result = await sdmod.decelerate_to_stop(hw, cfg, _profile(cfg), table=table)

    pi_moves = moves[1:]
    assert pi_moves
    assert all(smooth is not None for _, smooth in pi_moves[:-1])  # 周期ごとの滑らかな指令
    # 目標 0.2G に近づく（スタブの値は目安。0.3G は大きく超えない）
    assert 0.12 < result.max_decel_g < 0.35
    assert hw.brake.position == opening_to_pulse(PEDAL.stop_brake_opening_pct)
    assert await hw.can.read_speed() < VEHICLE_STOP_SPEED_KMH
    await hwmod.shutdown(hw)


async def test_approach_goes_to_onset_from_table_meta(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = await _moving_stub(cfg, speed_kmh=12.0, brake_gain=1.5)
    moves = _record_moves(hw.brake)  # type: ignore[arg-type]
    table = _FakeTable({"brake_onset_u_pct": 2.0})

    await sdmod.decelerate_to_stop(hw, cfg, _profile(cfg), table=table)

    assert moves[0] == (opening_to_pulse(MIN_BRAKE_PCT + 2.0), None)
    # PI は効き始めより浅くは戻さない（枠の下限）
    assert all(pos >= moves[0][0] for pos, _ in moves[:-1] if pos > 0)
    await hwmod.shutdown(hw)


async def test_starts_from_given_opening_without_reading_position(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = await _moving_stub(cfg, speed_kmh=12.0, brake_gain=1.5)
    moves = _record_moves(hw.brake)  # type: ignore[arg-type]

    async def must_not_read() -> int:
        raise AssertionError("start_pulse があるのでブレーキ位置は読まない")

    hw.brake.read_position = must_not_read  # type: ignore[method-assign]
    start = opening_to_pulse(15.0)

    await sdmod.decelerate_to_stop(hw, cfg, _profile(cfg), start_pulse=start)

    assert moves[0] == (start, None)  # 接近は飛ばして今の開度から
    await hwmod.shutdown(hw)


async def test_releases_when_decel_exceeds_limit(tmp_path: Path) -> None:
    # 強く踏んだ位置（約 0.9G 相当）から始める。0.3G 超えなので PI は戻す側へ動く
    cfg = _tmp_cfg(tmp_path)
    hw = await _moving_stub(cfg, speed_kmh=60.0, brake_gain=1.5)
    moves = _record_moves(hw.brake)  # type: ignore[arg-type]
    start = opening_to_pulse(30.0)

    result = await sdmod.decelerate_to_stop(hw, cfg, _profile(cfg), start_pulse=start)

    hold = opening_to_pulse(PEDAL.stop_brake_opening_pct)
    pi_positions = [pos for pos, _ in moves[1:] if pos != hold]
    assert min(pi_positions) < start  # 戻している
    assert max(pi_positions) <= start  # 0.3G 超えの間は踏み増さない
    assert result.max_decel_g > cfg.decel_stop.release_above_g
    await hwmod.shutdown(hw)


async def test_clamps_at_max_opening_and_does_not_wind_up(tmp_path: Path) -> None:
    # ブレーキが効かない車。最大開度の近くから始めると、最大開度で頭打ちになり超えない
    cfg = _tmp_cfg(tmp_path, **{"decel_stop.timeout_s": 2.0})
    hw = await _moving_stub(cfg, speed_kmh=20.0, brake_gain=0.0)
    moves = _record_moves(hw.brake)  # type: ignore[arg-type]
    ceiling = opening_to_pulse(build_vehicle_profile(cfg).max_brake_opening)

    with pytest.raises(hwmod.DriveError, match="停車しません"):
        await sdmod.decelerate_to_stop(
            hw, cfg, _profile(cfg), start_pulse=ceiling - 50
        )

    positions = [pos for pos, _ in moves]
    assert max(positions) == ceiling == hw.brake.position
    await hwmod.shutdown(hw)


async def test_holds_opening_until_slope_window_is_filled(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path, **{"decel_stop.pi_slope_window_s": 0.6})
    hw = await _moving_stub(cfg, speed_kmh=20.0, brake_gain=1.5)
    moves = _record_moves(hw.brake)  # type: ignore[arg-type]
    stamps: list[float] = []
    original = hw.brake.move_to_position

    async def stamped(pos: int, *, smooth_over_s: float | None = None) -> None:
        stamps.append(asyncio.get_running_loop().time())
        await original(pos, smooth_over_s=smooth_over_s)

    hw.brake.move_to_position = stamped  # type: ignore[method-assign]
    await sdmod.decelerate_to_stop(hw, cfg, _profile(cfg))

    # 接近の指令（1 本目）から PI の最初の指令（2 本目）まで、窓 0.6s は開度を保つ
    assert len(moves) >= 2
    assert stamps[1] - stamps[0] >= 0.55
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


def test_load_brake_table_is_none_without_model(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)  # model_path は存在しない pkl
    assert sdmod.load_brake_table(cfg) is None
