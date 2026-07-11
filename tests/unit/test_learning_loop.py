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


def _cruise_trim_pattern(
    accel: float = 70.0, trim: float = 2.0, hold: float = 8.0
) -> LearningPattern:
    return LearningPattern(
        kind=PatternKind.CRUISE_TRIM,
        accel_opening=accel,
        brake_opening=0.0,
        hold_duration_s=hold,
        trim_opening=trim,
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
    async def test_overspeed_in_coast_enters_drive_brake_with_recovery_flag(self) -> None:
        """COAST_DOWN の惰行中に過速度が成立した場合も DRIVE_BRAKE へ遷移し、
        オーバースピード回復フラグが立つ（D1: ブレーキ0%のまま再加速するのを防ぐ前提）。"""
        loop = _make_loop(
            profile=_make_profile(max_speed=100.0),
            can_reader=_make_can(speed=150.0),
            patterns=[_coast_down_pattern(hold=100.0)],
            config=LearningLoopConfig(coast_timeout_s=100.0),
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            loop._phase = _Phase.COAST
            loop._phase_started_at = 0.0
            await loop._execute_one_cycle()
        assert loop._phase is _Phase.DRIVE_BRAKE
        assert loop._overspeed_recovery is True

    @pytest.mark.asyncio
    async def test_overspeed_recovery_commands_nonzero_brake_for_coast_down(self) -> None:
        """D1 回帰テスト: COAST_DOWN パターン（brake_opening=0.0）でも、オーバースピード
        回復中の DRIVE_BRAKE はブレーキ0%のままにならず回復用の下限開度を踏む。"""
        loop = _make_loop(
            can_reader=_make_can(speed=95.0),  # まだ走行中（停車判定に掛からない）
            patterns=[_coast_down_pattern(hold=100.0)],
            config=LearningLoopConfig(brake_ramp_time_s=0.0),  # 即時に目標開度へ
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            loop._phase = _Phase.DRIVE_BRAKE
            loop._phase_started_at = 0.0
            loop._overspeed_recovery = True
            await loop._execute_one_cycle()
        assert loop.current_accel_opening == 0.0
        assert loop.current_brake_opening == pytest.approx(
            loop._config.overspeed_recovery_brake_pct
        )
        assert loop.current_brake_opening > 0.0

    @pytest.mark.asyncio
    async def test_normal_drive_brake_for_coast_down_stays_zero_without_recovery_flag(
        self,
    ) -> None:
        """回復フラグが立っていない通常の DRIVE_BRAKE では COAST_DOWN のブレーキ0%のまま
        （回復用下限は overspeed_recovery のときのみ適用される）。"""
        loop = _make_loop(
            can_reader=_make_can(speed=95.0),
            patterns=[_coast_down_pattern(hold=100.0)],
            config=LearningLoopConfig(brake_ramp_time_s=0.0),
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            loop._phase = _Phase.DRIVE_BRAKE
            loop._phase_started_at = 0.0
            loop._overspeed_recovery = False
            await loop._execute_one_cycle()
        assert loop.current_brake_opening == 0.0

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
            config=LearningLoopConfig(accel_speed_cap_frac=0.9, accel_full_range_timeout_s=100.0),
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
            config=LearningLoopConfig(accel_speed_cap_frac=0.9, accel_full_range_timeout_s=100.0),
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
            config=LearningLoopConfig(accel_speed_cap_frac=0.9, accel_full_range_timeout_s=100.0),
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
# 予測的 cap 離脱（B-7-5: 最高車速オーバー是正）
# ---------------------------------------------------------------------------


class TestPredictiveCapExit:
    """加速度に応じて cap の手前で離脱し、応答遅れ中の惰性オーバーシュートを防ぐ。"""

    def _accel_loop(self, lead_s: float) -> LearningLoop:
        # cap = 100 × 0.9 = 90。lead_s と加速度から離脱しきい値 = 90 − accel×lead_s。
        loop = _make_loop(
            profile=_make_profile(max_speed=100.0),
            patterns=[_accel_sweep_pattern(hold=100.0)],
            config=LearningLoopConfig(
                accel_speed_cap_frac=0.9,
                accel_full_range_timeout_s=100.0,
                overspeed_lead_s=lead_s,
            ),
        )
        loop._phase = _Phase.DRIVE_ACCEL
        loop._phase_started_at = 0.0
        return loop

    def test_exits_below_cap_when_accelerating(self) -> None:
        """加速中は cap−accel×lead で離脱する（10km/h/s・lead1.2 → しきい値78）。"""
        loop = self._accel_loop(lead_s=1.2)
        # 78 未満は継続、78 以上で離脱
        loop._advance_drive_accel(
            loop._patterns[0], speed=77.0, accel_kmhs=10.0, elapsed=0.0, now=0.0
        )
        assert loop._phase is _Phase.DRIVE_ACCEL
        loop._advance_drive_accel(
            loop._patterns[0], speed=79.0, accel_kmhs=10.0, elapsed=0.0, now=0.0
        )
        assert loop._phase is _Phase.DRIVE_BRAKE

    def test_lead_zero_exits_only_at_cap(self) -> None:
        """lead=0 は従来どおり cap 到達（90）でのみ離脱する。"""
        loop = self._accel_loop(lead_s=0.0)
        loop._advance_drive_accel(
            loop._patterns[0], speed=89.0, accel_kmhs=10.0, elapsed=0.0, now=0.0
        )
        assert loop._phase is _Phase.DRIVE_ACCEL  # 加速度が高くてもしきい値は cap のまま
        loop._advance_drive_accel(
            loop._patterns[0], speed=90.0, accel_kmhs=10.0, elapsed=0.0, now=0.0
        )
        assert loop._phase is _Phase.DRIVE_BRAKE

    def test_plateau_threshold_equals_cap(self) -> None:
        """加速≈0（プラトー）ではしきい値が cap と一致し従来挙動を保つ。"""
        loop = self._accel_loop(lead_s=1.2)
        loop._advance_drive_accel(
            loop._patterns[0], speed=89.5, accel_kmhs=0.0, elapsed=0.0, now=0.0
        )
        assert loop._phase is _Phase.DRIVE_ACCEL

    @pytest.mark.asyncio
    async def test_accel_peak_stays_within_max_speed(self) -> None:
        """一次遅れプラント上で加速→惰行させ、ピーク車速が max_speed を超えないことを検証。

        アクセル解放後も応答遅れ（tau）の間は惰性で車速が伸びる。予測的 cap 離脱が無い
        （lead=0）場合はピークが max_speed を超えるが、既定 lead では超えないことを対比で示す。
        """
        max_peak_default = await self._simulate_peak(lead_s=1.2)
        max_peak_no_lead = await self._simulate_peak(lead_s=0.0)
        assert max_peak_default <= 140.0, f"予測離脱ありでも超過: {max_peak_default:.1f}"
        assert max_peak_no_lead > 140.0, f"lead=0 で超過を再現できていない: {max_peak_no_lead:.1f}"

    async def _simulate_peak(self, lead_s: float) -> float:
        """一次遅れ加速プラントを COAST_DOWN パターンで走らせ最大車速を返す。"""
        dt = 0.1
        tau = 1.0  # 加速応答の時定数 [s]（実機 fopdt_tau≈1.0 相当）
        accel_gain = 0.2  # km/h/s per %（70% ≈ 14km/h/s ≈ ガバナ上限 0.98×0.4G）
        state = {"v": 0.0, "a": 0.0}

        async def read_speed() -> float:
            return state["v"]

        can = MagicMock()
        can.read_speed = AsyncMock(side_effect=read_speed)
        loop = _make_loop(
            profile=_make_profile(max_speed=140.0, max_decel_g=0.4),
            can_reader=can,
            patterns=[_coast_down_pattern(accel=70.0, hold=100.0)],
            config=LearningLoopConfig(
                accel_ramp_time_s=1.0,
                accel_full_range_timeout_s=100.0,
                coast_timeout_s=100.0,
                overspeed_lead_s=lead_s,
            ),
        )
        loop._running = True
        loop._reset_for_start()
        loop._phase = _Phase.DRIVE_ACCEL
        loop._phase_started_at = 0.0

        peak = 0.0
        t = 0.0
        for _ in range(400):  # 40s 上限
            with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(t)):
                await loop._execute_one_cycle()
            # プラント更新: 指令開度から目標加速度を作り一次遅れで追従
            target_a = accel_gain * loop.current_accel_opening
            state["a"] += (dt / (tau + dt)) * (target_a - state["a"])
            state["v"] = max(0.0, state["v"] + state["a"] * dt)
            peak = max(peak, state["v"])
            t += dt
            if loop._phase is _Phase.COAST and state["v"] < 5.0:
                break
        return peak


# ---------------------------------------------------------------------------
# 高速巡航トリム（B-7-2: cap→微小開度保持で高速域×微小開度を採取）
# ---------------------------------------------------------------------------


class TestCruiseTrim:
    @pytest.mark.asyncio
    async def test_reaches_cap_transitions_to_cruise_trim(self) -> None:
        """CRUISE_TRIM は cap 到達で CRUISE_TRIM フェーズ（微小開度保持）へ移る。"""
        loop = _make_loop(
            profile=_make_profile(max_speed=100.0),
            can_reader=_make_can(speed=92.0),  # cap 90 超
            patterns=[_cruise_trim_pattern(hold=100.0)],
            config=LearningLoopConfig(
                accel_speed_cap_frac=0.9, accel_full_range_timeout_s=100.0, overspeed_lead_s=0.0
            ),
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            loop._phase = _Phase.DRIVE_ACCEL
            loop._phase_started_at = 0.0
            await loop._execute_one_cycle()
        assert loop._phase is _Phase.CRUISE_TRIM

    @pytest.mark.asyncio
    async def test_holds_trim_opening(self) -> None:
        """CRUISE_TRIM フェーズは微小アクセル開度を保持しブレーキ 0。"""
        loop = _make_loop(
            can_reader=_make_can(speed=95.0),
            patterns=[_cruise_trim_pattern(trim=2.5, hold=100.0)],
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            loop._phase = _Phase.CRUISE_TRIM
            loop._phase_started_at = 0.0
            await loop._execute_one_cycle()
        assert loop.current_accel_opening == pytest.approx(2.5)
        assert loop.current_brake_opening == 0.0

    @pytest.mark.asyncio
    async def test_advances_after_hold_duration(self) -> None:
        """hold_duration 経過で次パターンへ進む。"""
        loop = _make_loop(
            can_reader=_make_can(speed=95.0),
            patterns=[_cruise_trim_pattern(hold=2.0), _accel_sweep_pattern()],
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(2.5)):
            loop._running = True
            loop._phase = _Phase.CRUISE_TRIM
            loop._phase_started_at = 0.0  # elapsed=2.5 >= 2.0
            loop._pattern_idx = 0
            await loop._execute_one_cycle()
        assert loop._pattern_idx == 1

    @pytest.mark.asyncio
    async def test_exits_before_overspeed_when_climbing_past_cap(self) -> None:
        """trim が加速側に働き cap 以上へ戻ったら過速度前に離脱する（安全）。"""
        loop = _make_loop(
            profile=_make_profile(max_speed=100.0),
            can_reader=_make_can(speed=91.0),  # cap 90 以上へ戻った（trim が加速側）
            patterns=[_cruise_trim_pattern(hold=100.0), _accel_sweep_pattern()],
            config=LearningLoopConfig(accel_speed_cap_frac=0.9),
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            loop._phase = _Phase.CRUISE_TRIM
            loop._phase_started_at = 0.0
            loop._pattern_idx = 0
            await loop._execute_one_cycle()
        assert loop._pattern_idx == 1  # cap 前に離脱

    @pytest.mark.asyncio
    async def test_max_speed_exceed_enters_recovery_brake(self) -> None:
        """max_speed 超は安全網として能動減速（DRIVE_BRAKE・回復ブレーキ）へ回す。"""
        loop = _make_loop(
            profile=_make_profile(max_speed=100.0),
            can_reader=_make_can(speed=101.0),  # max_speed 超
            patterns=[_cruise_trim_pattern(hold=100.0)],
        )
        with patch.object(asyncio, "get_running_loop", return_value=_patch_loop_time(0.0)):
            loop._running = True
            loop._phase = _Phase.CRUISE_TRIM
            loop._phase_started_at = 0.0
            await loop._execute_one_cycle()
        assert loop._phase is _Phase.DRIVE_BRAKE
        assert loop._overspeed_recovery is True


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


class TestStallTracking:
    """S1 回帰テスト: CycleLoopBase 統合により LearningLoop でもストール計測が有効になる
    （旧実装は DriveLoop のみで計測していた）。"""

    @pytest.mark.asyncio
    async def test_stall_summary_accumulates_on_resolved_skip(self) -> None:
        loop = _make_loop()
        loop._running = True
        loop._consecutive_skips = 3

        loop._schedule_next_cycle()  # 前タスクなし → 正常起動＝直前のストールが解消
        try:
            summary = loop.stall_summary
            assert summary["stall_count"] == 1.0
            assert summary["stall_total_s"] == pytest.approx(3 * loop._interval_s)
            assert summary["stall_max_s"] == pytest.approx(3 * loop._interval_s)
        finally:
            loop.stop()
            assert loop._cycle_task is not None
            await loop._cycle_task

    def test_stall_summary_is_zero_before_any_stall(self) -> None:
        loop = _make_loop()
        summary = loop.stall_summary
        assert summary == {"stall_count": 0.0, "stall_total_s": 0.0, "stall_max_s": 0.0}


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
