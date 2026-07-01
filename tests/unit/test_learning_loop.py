"""LearningLoop のユニットテスト。

DriveLoop のテストに倣い、`asyncio.get_running_loop`（時刻）と `asyncio.ensure_future`
（ログ書き込みの起動）をパッチして 1 サイクルを決定的に駆動する。
"""

from __future__ import annotations

import asyncio
from collections.abc import Awaitable, Callable
from datetime import datetime
from unittest.mock import AsyncMock, MagicMock, patch

import pytest

from src.domain.control.learning_loop import LearningLoop, LearningLoopConfig, _Phase
from src.models.calibration import CalibrationData
from src.models.learning_drive import LearningPattern, PatternKind
from src.models.profile import PIDGains, StopConfig, VehicleProfile

# ---------------------------------------------------------------------------
# ヘルパー
# ---------------------------------------------------------------------------


def _make_calibration() -> CalibrationData:
    return CalibrationData(
        accel_zero_pos=100,
        accel_full_pos=600,
        accel_stroke=500,
        brake_zero_pos=200,
        brake_full_pos=700,
        brake_stroke=500,
        calibrated_at=datetime(2026, 1, 1),
        is_valid=True,
    )


def _make_profile(
    max_speed: float = 100.0,
    max_accel: float = 80.0,
    max_brake: float = 80.0,
    max_decel_g: float = 0.5,
    calibration: CalibrationData | None = None,
) -> VehicleProfile:
    return VehicleProfile(
        id="profile-1",
        name="Test",
        max_accel_opening=max_accel,
        max_brake_opening=max_brake,
        max_speed=max_speed,
        max_decel_g=max_decel_g,
        pid_gains=PIDGains(kp=1.0, ki=0.0, kd=0.0),
        stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
        calibration=calibration if calibration is not None else _make_calibration(),
        model_path=None,
        created_at=datetime(2026, 1, 1),
        updated_at=datetime(2026, 1, 1),
    )


def _make_driver(current: float = 300.0) -> MagicMock:
    d = MagicMock()
    d.move_to_position = AsyncMock()
    d.move_to_position_timed = AsyncMock()
    d.read_current = AsyncMock(return_value=current)
    return d


def _make_can(speed: float = 0.0) -> MagicMock:
    r = MagicMock()
    r.read_speed = AsyncMock(return_value=speed)
    return r


def _make_safety(overcurrent: bool = False) -> MagicMock:
    sc = MagicMock()
    sc.check_overcurrent = MagicMock(return_value=overcurrent)
    return sc


def _accel_sweep_pattern(
    accel: float = 50.0, brake: float = 30.0, hold: float = 0.0
) -> LearningPattern:
    return LearningPattern(
        kind=PatternKind.ACCEL_SWEEP, accel_opening=accel, brake_opening=brake, hold_duration_s=hold
    )


def _brake_hold_pattern(
    accel: float = 70.0, brake: float = 30.0, hold: float = 0.0
) -> LearningPattern:
    return LearningPattern(
        kind=PatternKind.BRAKE_HOLD, accel_opening=accel, brake_opening=brake, hold_duration_s=hold
    )


def _coast_down_pattern(accel: float = 50.0, hold: float = 0.0) -> LearningPattern:
    return LearningPattern(
        kind=PatternKind.COAST_DOWN, accel_opening=accel, brake_opening=0.0, hold_duration_s=hold
    )


def _make_loop(
    *,
    accel_driver: MagicMock | None = None,
    brake_driver: MagicMock | None = None,
    can_reader: MagicMock | None = None,
    profile: VehicleProfile | None = None,
    patterns: list[LearningPattern] | None = None,
    safety_check: MagicMock | None = None,
    on_complete: Callable[[], Awaitable[None]] | None = None,
    on_emergency: Callable[[], Awaitable[None]] | None = None,
    log_writer: MagicMock | None = None,
    session_id: str | None = None,
    config: LearningLoopConfig | None = None,
) -> LearningLoop:
    return LearningLoop(
        accel_driver=accel_driver or _make_driver(),
        brake_driver=brake_driver or _make_driver(),
        can_reader=can_reader or _make_can(),
        profile=profile or _make_profile(),
        patterns=patterns if patterns is not None else [_accel_sweep_pattern()],
        safety_check=safety_check or _make_safety(),
        on_complete=on_complete or AsyncMock(),
        on_emergency=on_emergency or AsyncMock(),
        log_writer=log_writer,
        session_id=session_id,
        config=config,
    )


def _patch_loop_time(value: float = 0.0):  # type: ignore[no-untyped-def]
    loop_obj = MagicMock()
    loop_obj.time.return_value = value
    return loop_obj


# ---------------------------------------------------------------------------
# ログ記録
# ---------------------------------------------------------------------------


class TestLogWriting:
    @pytest.mark.asyncio
    async def test_log_enqueued_each_cycle(self) -> None:
        log_writer = MagicMock()
        log_writer.write_log = AsyncMock()
        loop = _make_loop(log_writer=log_writer, session_id="s1")

        with (
            patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)),
            patch.object(asyncio, "ensure_future") as mock_ensure,
        ):
            loop._running = True
            await loop._execute_one_cycle()
            assert mock_ensure.call_count == 1

    @pytest.mark.asyncio
    async def test_no_log_without_writer(self) -> None:
        loop = _make_loop(log_writer=None, session_id="s1")
        with (
            patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)),
            patch.object(asyncio, "ensure_future") as mock_ensure,
        ):
            loop._running = True
            await loop._execute_one_cycle()
            assert mock_ensure.call_count == 0


# ---------------------------------------------------------------------------
# 非常停止型安全（過電流・CAN 断）
# ---------------------------------------------------------------------------


class TestEmergencySafety:
    @pytest.mark.asyncio
    async def test_emergency_on_accel_overcurrent(self) -> None:
        on_emergency = AsyncMock()
        loop = _make_loop(safety_check=_make_safety(overcurrent=True), on_emergency=on_emergency)
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            await loop._execute_one_cycle()
        on_emergency.assert_awaited_once()
        assert loop._running is False

    @pytest.mark.asyncio
    async def test_emergency_on_brake_overcurrent(self) -> None:
        on_emergency = AsyncMock()
        safety = MagicMock()
        safety.check_overcurrent = MagicMock(side_effect=lambda current, axis: axis == "brake")
        loop = _make_loop(safety_check=safety, on_emergency=on_emergency)
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            await loop._execute_one_cycle()
        on_emergency.assert_awaited_once()

    @pytest.mark.asyncio
    async def test_emergency_on_can_read_failure(self) -> None:
        on_emergency = AsyncMock()
        can = MagicMock()
        can.read_speed = AsyncMock(side_effect=RuntimeError("CAN 断"))
        loop = _make_loop(can_reader=can, on_emergency=on_emergency)
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            await loop._execute_one_cycle()
        on_emergency.assert_awaited_once()


# ---------------------------------------------------------------------------
# スキップ型安全（過速度・過G）
# ---------------------------------------------------------------------------


class TestSkipSafety:
    @pytest.mark.asyncio
    async def test_overspeed_in_accel_transitions_to_brake_without_emergency(self) -> None:
        on_emergency = AsyncMock()
        # 過速度（>max_speed）は能動的に減速して復帰するため DRIVE_BRAKE へ（非常停止しない）
        loop = _make_loop(
            profile=_make_profile(max_speed=100.0),
            can_reader=_make_can(speed=150.0),
            patterns=[_accel_sweep_pattern(hold=100.0)],
            on_emergency=on_emergency,
            config=LearningLoopConfig(
                skip_consecutive_required=1, accel_full_range_timeout_s=100.0
            ),
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            loop._phase = _Phase.DRIVE_ACCEL
            loop._phase_started_at = 0.0
            await loop._execute_one_cycle()
        on_emergency.assert_not_awaited()
        assert loop._phase is _Phase.DRIVE_BRAKE  # 惰行では戻らないので能動的に制動

    @pytest.mark.asyncio
    async def test_over_g_in_accel_does_not_exit(self) -> None:
        """加速方向の過G（踏み始めの強い加速）は離脱要因にしない（cap 到達まで保持する）。"""
        loop = _make_loop(
            profile=_make_profile(max_speed=100.0, max_decel_g=0.5),
            can_reader=_make_can(speed=30.0),  # cap(90)・max_speed 未満
            patterns=[_accel_sweep_pattern(hold=100.0)],
            config=LearningLoopConfig(
                skip_consecutive_required=1, accel_full_range_timeout_s=100.0
            ),
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(1.0)):
            loop._running = True
            loop._phase = _Phase.DRIVE_ACCEL
            loop._phase_started_at = 1.0
            loop._prev_speed = 0.0
            loop._prev_time = 0.0  # 0→30km/h を 1s → 30km/h/s で加速側の過G
            await loop._execute_one_cycle()
        assert loop._phase is _Phase.DRIVE_ACCEL  # 加速継続（過Gでは抜けない）

    @pytest.mark.asyncio
    async def test_single_noisy_overspeed_sample_does_not_transition(self) -> None:
        """デバウンス（2サイクル）: 単発の過速度ノイズでは制動へ移らない。"""
        loop = _make_loop(
            profile=_make_profile(max_speed=100.0),
            can_reader=_make_can(speed=150.0),
            patterns=[_accel_sweep_pattern(hold=100.0)],
            config=LearningLoopConfig(
                skip_consecutive_required=2,
                accel_full_range_timeout_s=100.0,
                accel_speed_cap_frac=2.0,
            ),
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            loop._phase = _Phase.DRIVE_ACCEL
            loop._phase_started_at = 0.0
            await loop._execute_one_cycle()  # 1回目: skip_count=1 < 2 → 継続
            assert loop._phase is _Phase.DRIVE_ACCEL
            await loop._execute_one_cycle()  # 2回目: 連続成立 → 制動へ
            assert loop._phase is _Phase.DRIVE_BRAKE

    @pytest.mark.asyncio
    async def test_accel_sweep_reaches_cap_transitions_to_brake(self) -> None:
        """ACCEL_SWEEP は cap 到達でリセットブレーキ（DRIVE_BRAKE）へ移る（停車復帰）。"""
        loop = _make_loop(
            profile=_make_profile(max_speed=100.0),
            can_reader=_make_can(speed=92.0),  # キャップ 90 を超えた（まだ max_speed 未満）
            patterns=[_accel_sweep_pattern(hold=100.0)],
            config=LearningLoopConfig(
                accel_speed_cap_frac=0.9, accel_full_range_timeout_s=100.0
            ),
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            loop._phase = _Phase.DRIVE_ACCEL
            loop._phase_started_at = 0.0
            await loop._execute_one_cycle()
        assert loop._phase is _Phase.DRIVE_BRAKE  # cap 到達 → 停車復帰

    @pytest.mark.asyncio
    async def test_accel_sweep_timeout_transitions_to_brake(self) -> None:
        """cap に届かなくても accel_full_range_timeout で DRIVE_BRAKE へ打ち切る。"""
        loop = _make_loop(
            profile=_make_profile(max_speed=1000.0),  # 過速度・cap にならない
            can_reader=_make_can(speed=50.0),
            patterns=[_accel_sweep_pattern(hold=100.0)],
            config=LearningLoopConfig(accel_ramp_time_s=0.0, accel_full_range_timeout_s=2.0),
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(2.5)):
            loop._running = True
            loop._phase = _Phase.DRIVE_ACCEL
            loop._phase_started_at = 0.0  # elapsed=2.5 >= timeout 2.0
            await loop._execute_one_cycle()
        assert loop._phase is _Phase.DRIVE_BRAKE

    @pytest.mark.asyncio
    async def test_accel_does_not_exit_below_cap(self) -> None:
        """cap 未満かつ timeout 前はプラトーでも離脱しない（cap まで加速を伸ばす）。"""
        loop = _make_loop(
            profile=_make_profile(max_speed=100.0),
            can_reader=_make_can(speed=40.0),  # 速度一定（旧実装ではプラトー離脱した）
            patterns=[_accel_sweep_pattern(hold=100.0)],
            config=LearningLoopConfig(accel_ramp_time_s=0.0, accel_full_range_timeout_s=100.0),
        )
        loop._running = True
        loop._phase = _Phase.DRIVE_ACCEL
        loop._phase_started_at = 0.0
        loop._prev_speed = 40.0
        for t in (0.1, 0.3, 0.5):
            loop._prev_time = t - 0.1
            with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(t)):
                await loop._execute_one_cycle()
            assert loop._phase is _Phase.DRIVE_ACCEL  # プラトーでも cap/timeout まで継続

    @pytest.mark.asyncio
    async def test_coast_down_reaches_cap_transitions_to_coast(self) -> None:
        """COAST_DOWN は cap 到達で COAST（惰行）へ移る。"""
        loop = _make_loop(
            profile=_make_profile(max_speed=100.0),
            can_reader=_make_can(speed=92.0),  # キャップ 90 超
            patterns=[_coast_down_pattern(hold=100.0)],
            config=LearningLoopConfig(
                accel_speed_cap_frac=0.9, accel_full_range_timeout_s=100.0
            ),
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            loop._phase = _Phase.DRIVE_ACCEL
            loop._phase_started_at = 0.0
            await loop._execute_one_cycle()
        assert loop._phase is _Phase.COAST  # 加速→惰行

    @pytest.mark.asyncio
    async def test_brake_hold_reaches_cap_transitions_to_brake_hold(self) -> None:
        """BRAKE_HOLD は cap まで加速したら BRAKE_HOLD（定常ブレーキ保持）へ移る。"""
        loop = _make_loop(
            profile=_make_profile(max_speed=100.0),
            can_reader=_make_can(speed=92.0),
            patterns=[_brake_hold_pattern(hold=100.0)],
            config=LearningLoopConfig(
                accel_speed_cap_frac=0.9, accel_full_range_timeout_s=100.0
            ),
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            loop._phase = _Phase.DRIVE_ACCEL
            loop._phase_started_at = 0.0
            await loop._execute_one_cycle()
        assert loop._phase is _Phase.BRAKE_HOLD

    @pytest.mark.asyncio
    async def test_brake_hold_commands_fixed_brake_after_ramp(self) -> None:
        """BRAKE_HOLD は accel=0・ブレーキを目標開度へランプ後一定保持する。"""
        loop = _make_loop(
            can_reader=_make_can(speed=60.0),
            patterns=[_brake_hold_pattern(brake=30.0, hold=100.0)],
            config=LearningLoopConfig(brake_ramp_time_s=0.0),  # 即時に目標開度へ
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            loop._phase = _Phase.BRAKE_HOLD
            loop._phase_started_at = 0.0
            await loop._execute_one_cycle()
        assert loop.current_accel_opening == 0.0
        assert loop.current_brake_opening == pytest.approx(30.0)

    @pytest.mark.asyncio
    async def test_brake_hold_advances_pattern_on_stop(self) -> None:
        """BRAKE_HOLD は停車（speed≤STOP）で次パターンへ進む。"""
        loop = _make_loop(
            can_reader=_make_can(speed=0.0),
            patterns=[_brake_hold_pattern(hold=100.0), _accel_sweep_pattern()],
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            loop._phase = _Phase.BRAKE_HOLD
            loop._phase_started_at = 0.0
            loop._pattern_idx = 0
            await loop._execute_one_cycle()
        assert loop._pattern_idx == 1
        assert loop._phase is _Phase.DRIVE_ACCEL  # 次の ACCEL_SWEEP は加速から

    @pytest.mark.asyncio
    async def test_brake_hold_timeout_advances_pattern(self) -> None:
        """停車しなくても brake_hold_timeout で次パターンへ打ち切る。"""
        loop = _make_loop(
            can_reader=_make_can(speed=30.0),  # 停車しない
            patterns=[_brake_hold_pattern(hold=100.0), _accel_sweep_pattern()],
            config=LearningLoopConfig(brake_hold_timeout_s=2.0),
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(2.5)):
            loop._running = True
            loop._phase = _Phase.BRAKE_HOLD
            loop._phase_started_at = 0.0  # elapsed=2.5 >= 2.0
            loop._pattern_idx = 0
            await loop._execute_one_cycle()
        assert loop._pattern_idx == 1

    @pytest.mark.asyncio
    async def test_brake_hold_governor_caps_opening_on_over_g(self) -> None:
        """BRAKE_HOLD でも減速G が上限を超えたらガバナがブレーキ開度を頭打ちにする。"""
        loop = _make_loop(
            profile=_make_profile(max_decel_g=0.5),  # 上限 0.5G×0.9 ≈ 15.9km/h/s
            can_reader=_make_can(speed=30.0),  # まだ走行中
            patterns=[_brake_hold_pattern(brake=40.0, hold=100.0)],
            config=LearningLoopConfig(brake_ramp_time_s=4.0, g_limit_frac=0.9),
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(2.0)):
            loop._running = True
            loop._phase = _Phase.BRAKE_HOLD
            loop._phase_started_at = 0.0  # elapsed=2s → ramp 0.5 → brake 20%
            loop._current_brake_opening = 20.0  # 直前サイクルの指令開度
            loop._prev_speed = 60.0
            loop._prev_time = 1.0  # 60→30 を 1s → -30km/h/s で減速G超過
            await loop._execute_one_cycle()
        assert loop._brake_gov_cap == pytest.approx(20.0)
        assert loop.current_brake_opening <= 20.0 + 1e-6

    @pytest.mark.asyncio
    async def test_coast_commands_both_pedals_zero(self) -> None:
        """COAST は accel=0/brake=0（エンジンブレーキ計測のための惰行）。"""
        loop = _make_loop(
            can_reader=_make_can(speed=80.0),
            patterns=[_coast_down_pattern(accel=50.0, hold=100.0)],
            config=LearningLoopConfig(coast_timeout_s=100.0),
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            loop._phase = _Phase.COAST
            loop._phase_started_at = 0.0
            await loop._execute_one_cycle()
        assert loop.current_accel_opening == 0.0
        assert loop.current_brake_opening == 0.0

    @pytest.mark.asyncio
    async def test_coast_down_advances_to_next_pattern_without_brake(self) -> None:
        """COAST_DOWN は惰行後ブレーキを挟まず次パターンへ進む（低速まで惰行）。"""
        config = LearningLoopConfig(coast_down_stop_speed_kmh=5.0, coast_timeout_s=100.0)
        loop = _make_loop(
            can_reader=_make_can(speed=3.0),  # 低速まで惰行済み
            patterns=[_coast_down_pattern(hold=100.0), _accel_sweep_pattern()],
            config=config,
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            loop._phase = _Phase.COAST
            loop._phase_started_at = 0.0
            loop._pattern_idx = 0
            await loop._execute_one_cycle()
        assert loop._pattern_idx == 1  # ブレーキを挟まず次へ
        assert loop._phase is _Phase.DRIVE_ACCEL

    @pytest.mark.asyncio
    async def test_coast_down_timeout_advances(self) -> None:
        """惰行が低速まで落ちなくても coast_timeout で次パターンへ前進する（ダイナモ対策）。"""
        loop = _make_loop(
            profile=_make_profile(max_speed=100.0),
            can_reader=_make_can(speed=80.0),  # coast_down_stop(5) まで落ちない
            patterns=[_coast_down_pattern(hold=100.0), _accel_sweep_pattern()],
            config=LearningLoopConfig(coast_timeout_s=2.0),
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(2.5)):
            loop._running = True
            loop._phase = _Phase.COAST
            loop._phase_started_at = 0.0  # elapsed=2.5 >= 2.0
            loop._pattern_idx = 0
            await loop._execute_one_cycle()
        assert loop._pattern_idx == 1

    @pytest.mark.asyncio
    async def test_brake_advances_pattern_on_stop(self) -> None:
        """減速区間は停車（speed≤STOP）で次パターンへ進む。"""
        loop = _make_loop(
            can_reader=_make_can(speed=0.0),
            patterns=[_accel_sweep_pattern(hold=100.0), _accel_sweep_pattern()],
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            loop._phase = _Phase.DRIVE_BRAKE
            loop._phase_started_at = 0.0
            loop._pattern_idx = 0
            await loop._execute_one_cycle()
        assert loop._pattern_idx == 1
        assert loop._phase is _Phase.DRIVE_ACCEL  # 次の ACCEL_SWEEP は加速から

    @pytest.mark.asyncio
    async def test_brake_governor_caps_opening_on_over_g(self) -> None:
        """減速G が上限を超えたら包絡線ガバナがブレーキ開度を頭打ちにする（停車まで継続）。"""
        loop = _make_loop(
            profile=_make_profile(max_decel_g=0.5),  # 上限 0.5G×0.9 ≈ 15.9km/h/s
            can_reader=_make_can(speed=30.0),  # まだ走行中
            patterns=[_accel_sweep_pattern(brake=40.0, hold=100.0)],
            config=LearningLoopConfig(brake_ramp_time_s=4.0, g_limit_frac=0.9),
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(2.0)):
            loop._running = True
            loop._phase = _Phase.DRIVE_BRAKE
            loop._phase_started_at = 0.0  # elapsed=2s → ramp 0.5 → brake 20%
            loop._current_brake_opening = 20.0  # 直前サイクルの指令開度
            loop._prev_speed = 60.0
            loop._prev_time = 1.0  # 60→30 を 1s → -30km/h/s で減速G超過
            await loop._execute_one_cycle()
        # 過G成立 → 現在のブレーキ開度(20%)で頭打ち、停車前なので次へ進まない
        assert loop._brake_gov_cap == pytest.approx(20.0)
        assert loop.current_brake_opening <= 20.0 + 1e-6
        assert loop._pattern_idx == 0

    @pytest.mark.asyncio
    async def test_creep_settle_waits_until_speed_stable(self) -> None:
        """CREEP_SETTLE は |加速度|<tol が継続し min_s 経過するまで次へ進まない。"""
        settle = LearningPattern(
            kind=PatternKind.CREEP_SETTLE, accel_opening=0.0, brake_opening=0.0, hold_duration_s=0.0
        )
        # min_s=0.3s, stable_duration=0.2s, interval=0.1s → 安定2サイクルで確定
        config = LearningLoopConfig(
            creep_settle_min_s=0.3,
            creep_settle_stable_tol_kmhs=0.3,
            creep_settle_stable_duration_s=0.2,
            creep_settle_timeout_s=10.0,
        )
        loop = _make_loop(
            patterns=[settle, _accel_sweep_pattern()],
            can_reader=_make_can(speed=7.0),  # クリープ車速で安定
            config=config,
        )
        loop._running = True
        loop._phase = _Phase.MEASURE
        loop._phase_started_at = 0.0
        loop._pattern_idx = 0
        # 加速中（|加速度|>tol）は安定とみなされず進まない
        loop._prev_speed = 0.0
        loop._prev_time = 0.0
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.1)):
            await loop._execute_one_cycle()  # accel=70km/h/s → 不安定
        assert loop._pattern_idx == 0
        # 安定（速度一定）になり min_s 経過 → 次パターンへ
        loop._prev_speed = 7.0
        for t in (0.4, 0.5):
            loop._prev_time = t - 0.1
            with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(t)):
                await loop._execute_one_cycle()
        assert loop._pattern_idx == 1  # クリープ安定 → 次の開度振りへ
        assert loop._phase is _Phase.DRIVE_ACCEL  # 次の ACCEL_SWEEP は加速から

    @pytest.mark.asyncio
    async def test_creep_pattern_skips_to_next_release(self) -> None:
        """クリープは連続リリースのため hold 経過で直接次パターンへ進む（再制動しない）。"""
        creep = LearningPattern(
            kind=PatternKind.CREEP, accel_opening=0.0, brake_opening=10.0, hold_duration_s=0.0
        )
        creep2 = LearningPattern(
            kind=PatternKind.CREEP, accel_opening=0.0, brake_opening=5.0, hold_duration_s=0.0
        )
        loop = _make_loop(patterns=[creep, creep2])
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            loop._phase = _Phase.MEASURE
            loop._phase_started_at = 0.0
            await loop._execute_one_cycle()  # hold=0 → 次のクリープ解放へ直行
        assert loop._pattern_idx == 1
        assert loop._phase is _Phase.MEASURE  # 次の CREEP も MEASURE

    @pytest.mark.asyncio
    async def test_within_limits_does_not_transition(self) -> None:
        loop = _make_loop(
            profile=_make_profile(max_speed=100.0),
            can_reader=_make_can(speed=40.0),  # 上限内
            patterns=[_accel_sweep_pattern(hold=100.0)],
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            loop._phase = _Phase.DRIVE_ACCEL
            loop._phase_started_at = 0.0
            await loop._execute_one_cycle()
        assert loop._phase is _Phase.DRIVE_ACCEL  # 上限内・プラトー未確定なので加速継続


# ---------------------------------------------------------------------------
# 完了・開度公開・フェーズ
# ---------------------------------------------------------------------------


class TestStopAndJoin:
    @pytest.mark.asyncio
    async def test_stop_and_join_from_within_cycle_returns_immediately(self) -> None:
        """サイクルタスク内から呼ばれた stop_and_join は自タスクを join せず即 return する。

        回帰: これがないと on_complete→stop_learning_drive→stop_and_join が自タスクを
        2秒待って cancel し、後続 home_return が CancelledError で中断される。
        """
        loop = _make_loop()
        loop._cycle_task = asyncio.current_task()  # 自タスク＝サイクルタスクを模す
        loop._running = True
        # 2秒のハングがあれば 0.5s タイムアウトで TimeoutError になる
        await asyncio.wait_for(loop.stop_and_join(), timeout=0.5)
        assert loop._running is False
        current = asyncio.current_task()
        assert current is not None and not current.cancelled()


class TestCompletionAndState:
    @pytest.mark.asyncio
    async def test_on_complete_after_all_patterns(self) -> None:
        on_complete = AsyncMock()
        loop = _make_loop(patterns=[_accel_sweep_pattern()], on_complete=on_complete)
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            loop._pattern_idx = 1  # 末尾を超えた状態
            await loop._execute_one_cycle()
        on_complete.assert_awaited_once()
        assert loop._running is False

    @pytest.mark.asyncio
    async def test_current_openings_reflect_accel_target_after_ramp(self) -> None:
        # ramp_time=0 で即時に目標開度へ。加速区間は brake=0。
        loop = _make_loop(
            patterns=[_accel_sweep_pattern(accel=55.0, hold=100.0)],
            config=LearningLoopConfig(accel_ramp_time_s=0.0),
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            loop._phase = _Phase.DRIVE_ACCEL
            loop._phase_started_at = 0.0
            await loop._execute_one_cycle()
        assert loop.current_accel_opening == 55.0
        assert loop.current_brake_opening == 0.0
        assert loop.current_ref_speed is None

    @pytest.mark.asyncio
    async def test_accel_sweep_pattern_starts_in_drive_accel(self) -> None:
        loop = _make_loop(patterns=[_accel_sweep_pattern(hold=100.0)])
        assert loop._initial_phase(0) is _Phase.DRIVE_ACCEL


class TestRamp:
    @pytest.mark.asyncio
    async def test_accel_ramps_to_target_over_ramp_time(self) -> None:
        """加速区間はアクセルを 0→目標へ ramp_time_s 秒かけて線形に踏み込む。"""
        loop = _make_loop(
            patterns=[_accel_sweep_pattern(accel=50.0, hold=100.0)],
            can_reader=_make_can(speed=10.0),  # 上限内で加速継続
            config=LearningLoopConfig(accel_ramp_time_s=2.0, accel_full_range_timeout_s=100.0),
        )
        loop._running = True
        loop._phase = _Phase.DRIVE_ACCEL
        for t, expected in ((0.0, 0.0), (1.0, 25.0), (2.0, 50.0), (3.0, 50.0)):
            loop._phase_started_at = 0.0
            with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(t)):
                await loop._execute_one_cycle()
            assert loop.current_accel_opening == pytest.approx(expected)
            assert loop.current_brake_opening == 0.0

    @pytest.mark.asyncio
    async def test_brake_ramps_to_target_over_ramp_time(self) -> None:
        """減速区間はブレーキを 0→目標へ ramp_time_s 秒かけて線形に踏み込む（accel解放）。"""
        loop = _make_loop(
            patterns=[_accel_sweep_pattern(brake=40.0, hold=100.0)],
            can_reader=_make_can(speed=30.0),  # まだ走行中（減速継続）
            config=LearningLoopConfig(brake_ramp_time_s=2.0),
        )
        loop._running = True
        loop._phase = _Phase.DRIVE_BRAKE
        for t, expected in ((0.0, 0.0), (1.0, 20.0), (2.0, 40.0), (3.0, 40.0)):
            loop._phase_started_at = 0.0
            with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(t)):
                await loop._execute_one_cycle()
            assert loop.current_brake_opening == pytest.approx(expected)
            assert loop.current_accel_opening == 0.0


class TestTimedMove:
    @pytest.mark.asyncio
    async def test_move_issued_on_position_change_only(self) -> None:
        """指令位置が変化した時だけ時間指定移動を発行し、変化のないサイクルは電流のみ読む。"""
        accel_driver = _make_driver()
        loop = _make_loop(
            accel_driver=accel_driver,
            can_reader=_make_can(speed=10.0),
            patterns=[_accel_sweep_pattern(accel=50.0, hold=100.0)],
            # ramp 即時で1サイクル目に 50% へ。以降は同じ開度＝位置不変
            config=LearningLoopConfig(accel_ramp_time_s=0.0, accel_full_range_timeout_s=100.0),
        )
        loop._running = True
        loop._phase = _Phase.DRIVE_ACCEL
        loop._phase_started_at = 0.0
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            await loop._execute_one_cycle()  # 0→50% に変化 → 移動発行
        assert accel_driver.move_to_position_timed.await_count == 1
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.1)):
            await loop._execute_one_cycle()  # 50%のまま → 移動なし・電流のみ
            await loop._execute_one_cycle()
        assert accel_driver.move_to_position_timed.await_count == 1  # 増えない
        assert accel_driver.read_current.await_count >= 3

    @pytest.mark.asyncio
    async def test_move_duration_uses_step_time_in_drive(self) -> None:
        """加速/減速の踏み込み中の時間指定移動は pedal_step_time_s を duration に渡す。"""
        accel_driver = _make_driver()
        loop = _make_loop(
            accel_driver=accel_driver,
            can_reader=_make_can(speed=10.0),
            patterns=[_accel_sweep_pattern(accel=50.0, hold=100.0)],
            config=LearningLoopConfig(
                accel_ramp_time_s=0.0, accel_full_range_timeout_s=100.0, pedal_step_time_s=0.2
            ),
        )
        loop._running = True
        loop._phase = _Phase.DRIVE_ACCEL
        loop._phase_started_at = 0.0
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            await loop._execute_one_cycle()
        # move_to_position_timed(target_pos, current_pos, duration_s)
        args = accel_driver.move_to_position_timed.await_args.args
        assert args[2] == pytest.approx(0.2)  # duration = pedal_step_time_s

    @pytest.mark.asyncio
    async def test_accel_governor_caps_opening_on_over_g(self) -> None:
        """加速G が上限を超えたら包絡線ガバナが現在開度で踏み増しを止める。"""
        loop = _make_loop(
            profile=_make_profile(max_speed=1000.0, max_decel_g=0.5),  # 上限 ≈15.9km/h/s
            can_reader=_make_can(speed=30.0),
            patterns=[_accel_sweep_pattern(accel=80.0, hold=100.0)],
            config=LearningLoopConfig(accel_ramp_time_s=0.0, g_limit_frac=0.9),
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(1.0)):
            loop._running = True
            loop._phase = _Phase.DRIVE_ACCEL
            loop._phase_started_at = 1.0
            loop._current_accel_opening = 50.0  # 直前サイクルの指令開度
            loop._prev_speed = 0.0
            loop._prev_time = 0.0  # 0→30km/h を 1s → 30km/h/s で加速G超過
            await loop._execute_one_cycle()
        # 目標80%だが現在開度50%で頭打ち（それ以上踏み増さない）
        assert loop._accel_gov_cap == pytest.approx(50.0)
        assert loop.current_accel_opening <= 50.0 + 1e-6

    def test_governor_reduces_cap_when_still_over(self) -> None:
        """頭打ち後も超過が続けば gov_reduce_step_pct ずつ開度上限を下げる。"""
        loop = _make_loop(
            profile=_make_profile(max_decel_g=0.5),
            config=LearningLoopConfig(g_limit_frac=0.9, gov_reduce_step_pct=2.0),
        )
        loop._phase = _Phase.DRIVE_BRAKE
        loop._current_brake_opening = 30.0
        over = -30.0  # 減速G 30km/h/s > 上限
        loop._update_governor(over)  # 初回: 現在開度で頭打ち
        assert loop._brake_gov_cap == pytest.approx(30.0)
        loop._update_governor(over)  # まだ超過 → 2%下げる
        assert loop._brake_gov_cap == pytest.approx(28.0)
