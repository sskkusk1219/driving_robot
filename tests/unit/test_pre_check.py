"""PreCheckRunner ユニットテスト。"""

from datetime import UTC, datetime
from unittest.mock import AsyncMock

import pytest

from src.domain.pre_check import PreCheckRunner
from src.models.calibration import CalibrationData
from src.models.pre_check import PreCheckResult
from src.models.profile import PIDGains, StopConfig, VehicleProfile


def make_calibration(is_valid: bool = True) -> CalibrationData:
    return CalibrationData(
        accel_zero_pos=0,
        accel_full_pos=1000,
        accel_stroke=1000,
        brake_zero_pos=0,
        brake_full_pos=800,
        brake_stroke=800,
        calibrated_at=datetime.now(tz=UTC),
        is_valid=is_valid,
    )


def make_profile(calibration: CalibrationData | None = None) -> VehicleProfile:
    return VehicleProfile(
        id="test-id",
        name="TestVehicle",
        max_accel_opening=80.0,
        max_brake_opening=80.0,
        max_speed=120.0,
        max_decel_g=0.3,
        pid_gains=PIDGains(kp=1.0, ki=0.0, kd=0.0),
        stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
        calibration=calibration if calibration is not None else make_calibration(),
        model_path=None,
        created_at=datetime.now(tz=UTC),
        updated_at=datetime.now(tz=UTC),
    )


def make_accel_driver(position: int = 0, alarm: bool = False) -> AsyncMock:
    driver = AsyncMock()
    driver.read_position = AsyncMock(return_value=position)
    driver.is_alarm_active = AsyncMock(return_value=alarm)
    return driver


def make_brake_driver(position: int = 0, alarm: bool = False) -> AsyncMock:
    return make_accel_driver(position=position, alarm=alarm)


def make_can_reader(speed: float = 0.0) -> AsyncMock:
    reader = AsyncMock()
    reader.read_speed = AsyncMock(return_value=speed)
    return reader


def make_ups(level: float = 100.0) -> AsyncMock:
    ups = AsyncMock()
    ups.get_battery_level_pct = AsyncMock(return_value=level)
    return ups


def make_runner(
    accel_pos: int = 0,
    brake_pos: int = 0,
    accel_alarm: bool = False,
    brake_alarm: bool = False,
    can_speed: float = 0.0,
    ups_level: float = 100.0,
    profile: VehicleProfile | None = None,
) -> PreCheckRunner:
    return PreCheckRunner(
        accel_driver=make_accel_driver(position=accel_pos, alarm=accel_alarm),
        brake_driver=make_brake_driver(position=brake_pos, alarm=brake_alarm),
        can_reader=make_can_reader(speed=can_speed),
        ups_monitor=make_ups(level=ups_level),
        profile=profile if profile is not None else make_profile(),
    )


class TestPreCheckRunnerAllPass:
    @pytest.mark.asyncio
    async def test_all_pass_returns_passed_true(self) -> None:
        runner = make_runner()
        result = await runner.run()
        assert isinstance(result, PreCheckResult)
        assert result.passed is True

    @pytest.mark.asyncio
    async def test_all_pass_has_seven_items(self) -> None:
        runner = make_runner()
        result = await runner.run()
        assert len(result.items) == 7

    @pytest.mark.asyncio
    async def test_all_pass_no_failed_items(self) -> None:
        runner = make_runner()
        result = await runner.run()
        assert result.failed_items == []

    @pytest.mark.asyncio
    async def test_all_pass_all_items_passed(self) -> None:
        runner = make_runner()
        result = await runner.run()
        assert all(i.passed for i in result.items)


class TestCheckCommunication:
    @pytest.mark.asyncio
    async def test_communication_fail_when_accel_read_position_raises(self) -> None:
        runner = make_runner()
        runner._accel.read_position = AsyncMock(side_effect=OSError("Modbus タイムアウト"))
        result = await runner.run()
        assert result.passed is False
        comm_item = next(i for i in result.items if i.item_name == "通信確認")
        assert comm_item.passed is False
        assert comm_item.error_message is not None
        assert "Modbus タイムアウト" in comm_item.error_message

    @pytest.mark.asyncio
    async def test_communication_fail_when_can_read_speed_raises(self) -> None:
        runner = make_runner()
        runner._can.read_speed = AsyncMock(side_effect=RuntimeError("CAN 接続なし"))
        result = await runner.run()
        assert result.passed is False
        comm_item = next(i for i in result.items if i.item_name == "通信確認")
        assert comm_item.passed is False

    @pytest.mark.asyncio
    async def test_communication_pass_when_all_succeed(self) -> None:
        runner = make_runner()
        result = await runner.run()
        comm_item = next(i for i in result.items if i.item_name == "通信確認")
        assert comm_item.passed is True
        assert comm_item.error_message is None


class TestCheckServoState:
    @pytest.mark.asyncio
    async def test_servo_fail_when_accel_alarm_active(self) -> None:
        runner = make_runner(accel_alarm=True)
        result = await runner.run()
        assert result.passed is False
        servo_item = next(i for i in result.items if i.item_name == "サーボ状態")
        assert servo_item.passed is False
        assert "アクセル軸" in (servo_item.error_message or "")

    @pytest.mark.asyncio
    async def test_servo_fail_when_brake_alarm_active(self) -> None:
        runner = make_runner(brake_alarm=True)
        result = await runner.run()
        assert result.passed is False
        servo_item = next(i for i in result.items if i.item_name == "サーボ状態")
        assert servo_item.passed is False
        assert "ブレーキ軸" in (servo_item.error_message or "")

    @pytest.mark.asyncio
    async def test_servo_fail_both_alarms_in_message(self) -> None:
        runner = make_runner(accel_alarm=True, brake_alarm=True)
        result = await runner.run()
        servo_item = next(i for i in result.items if i.item_name == "サーボ状態")
        assert servo_item.passed is False
        assert "アクセル軸" in (servo_item.error_message or "")
        assert "ブレーキ軸" in (servo_item.error_message or "")

    @pytest.mark.asyncio
    async def test_servo_pass_when_no_alarm(self) -> None:
        runner = make_runner()
        result = await runner.run()
        servo_item = next(i for i in result.items if i.item_name == "サーボ状態")
        assert servo_item.passed is True

    @pytest.mark.asyncio
    async def test_servo_fail_when_is_alarm_active_raises(self) -> None:
        runner = make_runner()
        runner._accel.is_alarm_active = AsyncMock(side_effect=OSError("Modbus タイムアウト"))
        result = await runner.run()
        servo_item = next(i for i in result.items if i.item_name == "サーボ状態")
        assert servo_item.passed is False
        assert "サーボ状態取得エラー" in (servo_item.error_message or "")


class TestCheckProfile:
    @pytest.mark.asyncio
    async def test_profile_fail_when_none(self) -> None:
        runner = make_runner(profile=None)
        runner._profile = None
        result = await runner.run()
        assert result.passed is False
        profile_item = next(i for i in result.items if i.item_name == "プロファイル")
        assert profile_item.passed is False

    @pytest.mark.asyncio
    async def test_profile_pass_when_set(self) -> None:
        runner = make_runner()
        result = await runner.run()
        profile_item = next(i for i in result.items if i.item_name == "プロファイル")
        assert profile_item.passed is True


class TestCheckCalibration:
    @pytest.mark.asyncio
    async def test_calibration_fail_when_calibration_is_none(self) -> None:
        profile = make_profile()
        profile.calibration = None
        runner = PreCheckRunner(
            accel_driver=make_accel_driver(),
            brake_driver=make_brake_driver(),
            can_reader=make_can_reader(),
            ups_monitor=make_ups(),
            profile=profile,
        )
        result = await runner.run()
        assert result.passed is False
        cal_item = next(i for i in result.items if i.item_name == "キャリブレーション")
        assert cal_item.passed is False
        assert "キャリブレーションデータがありません" in (cal_item.error_message or "")

    @pytest.mark.asyncio
    async def test_calibration_fail_when_is_valid_false(self) -> None:
        profile = make_profile(calibration=make_calibration(is_valid=False))
        runner = PreCheckRunner(
            accel_driver=make_accel_driver(),
            brake_driver=make_brake_driver(),
            can_reader=make_can_reader(),
            ups_monitor=make_ups(),
            profile=profile,
        )
        result = await runner.run()
        assert result.passed is False
        cal_item = next(i for i in result.items if i.item_name == "キャリブレーション")
        assert cal_item.passed is False
        assert "無効" in (cal_item.error_message or "")

    @pytest.mark.asyncio
    async def test_calibration_pass_when_valid(self) -> None:
        runner = make_runner()
        result = await runner.run()
        cal_item = next(i for i in result.items if i.item_name == "キャリブレーション")
        assert cal_item.passed is True


class TestCheckUpsBattery:
    @pytest.mark.asyncio
    async def test_ups_fail_when_level_below_threshold(self) -> None:
        runner = make_runner(ups_level=19.9)
        result = await runner.run()
        assert result.passed is False
        ups_item = next(i for i in result.items if i.item_name == "UPS残量")
        assert ups_item.passed is False
        assert "19.9" in (ups_item.error_message or "")

    @pytest.mark.asyncio
    async def test_ups_pass_when_level_at_threshold(self) -> None:
        runner = make_runner(ups_level=20.0)
        result = await runner.run()
        ups_item = next(i for i in result.items if i.item_name == "UPS残量")
        assert ups_item.passed is True

    @pytest.mark.asyncio
    async def test_ups_fail_when_get_raises(self) -> None:
        runner = make_runner()
        runner._ups.get_battery_level_pct = AsyncMock(side_effect=RuntimeError("UPS 接続エラー"))
        result = await runner.run()
        ups_item = next(i for i in result.items if i.item_name == "UPS残量")
        assert ups_item.passed is False
        assert ups_item.error_message is not None


class TestCheckActuatorPosition:
    @pytest.mark.asyncio
    async def test_position_fail_when_accel_out_of_range(self) -> None:
        runner = make_runner(accel_pos=11)
        result = await runner.run()
        assert result.passed is False
        pos_item = next(i for i in result.items if i.item_name == "アクチュエータ位置")
        assert pos_item.passed is False
        assert "アクセル軸" in (pos_item.error_message or "")

    @pytest.mark.asyncio
    async def test_position_fail_when_brake_out_of_range(self) -> None:
        runner = make_runner(brake_pos=-11)
        result = await runner.run()
        pos_item = next(i for i in result.items if i.item_name == "アクチュエータ位置")
        assert pos_item.passed is False
        assert "ブレーキ軸" in (pos_item.error_message or "")

    @pytest.mark.asyncio
    async def test_position_pass_at_tolerance_boundary(self) -> None:
        runner = make_runner(accel_pos=10, brake_pos=-10)
        result = await runner.run()
        pos_item = next(i for i in result.items if i.item_name == "アクチュエータ位置")
        assert pos_item.passed is True

    @pytest.mark.asyncio
    async def test_position_pass_at_origin(self) -> None:
        runner = make_runner()
        result = await runner.run()
        pos_item = next(i for i in result.items if i.item_name == "アクチュエータ位置")
        assert pos_item.passed is True


class TestRunExclude:
    @pytest.mark.asyncio
    async def test_exclude_vehicle_stopped_skips_speed_check(self) -> None:
        from src.domain.pre_check import ITEM_VEHICLE_STOPPED  # noqa: PLC0415

        # 車速が出ていても、車速確認を除外すれば passed=True になる
        runner = make_runner(can_speed=5.0)
        result = await runner.run(exclude=frozenset({ITEM_VEHICLE_STOPPED}))
        assert result.passed is True
        assert all(i.item_name != "車速確認" for i in result.items)
        assert len(result.items) == 6

    @pytest.mark.asyncio
    async def test_exclude_actuator_position_skips_position_check(self) -> None:
        from src.domain.pre_check import ITEM_ACTUATOR_POSITION  # noqa: PLC0415

        # ブレーキを踏んで位置が原点から離れていても、位置を除外すれば passed=True
        runner = make_runner(brake_pos=1575)
        result = await runner.run(exclude=frozenset({ITEM_ACTUATOR_POSITION}))
        assert result.passed is True
        assert all(i.item_name != "アクチュエータ位置" for i in result.items)
        assert len(result.items) == 6


class TestCheckVehicleStopped:
    @pytest.mark.asyncio
    async def test_speed_check_fail_when_moving(self) -> None:
        runner = make_runner(can_speed=5.0)
        result = await runner.run()
        assert result.passed is False
        speed_item = next(i for i in result.items if i.item_name == "車速確認")
        assert speed_item.passed is False
        assert "5.0" in (speed_item.error_message or "")

    @pytest.mark.asyncio
    async def test_speed_check_pass_when_stopped(self) -> None:
        runner = make_runner(can_speed=0.0)
        result = await runner.run()
        speed_item = next(i for i in result.items if i.item_name == "車速確認")
        assert speed_item.passed is True

    @pytest.mark.asyncio
    async def test_speed_check_pass_just_below_threshold(self) -> None:
        runner = make_runner(can_speed=0.4)
        result = await runner.run()
        speed_item = next(i for i in result.items if i.item_name == "車速確認")
        assert speed_item.passed is True

    @pytest.mark.asyncio
    async def test_speed_check_fail_when_read_raises(self) -> None:
        runner = make_runner()
        runner._can.read_speed = AsyncMock(side_effect=RuntimeError("CAN 読取失敗"))
        result = await runner.run()
        speed_item = next(i for i in result.items if i.item_name == "車速確認")
        assert speed_item.passed is False


class TestMultipleFailures:
    @pytest.mark.asyncio
    async def test_multiple_failures_all_reflected_in_result(self) -> None:
        runner = make_runner(accel_alarm=True, ups_level=5.0)
        result = await runner.run()
        assert result.passed is False
        failed_names = {i.item_name for i in result.failed_items}
        assert "サーボ状態" in failed_names
        assert "UPS残量" in failed_names

    @pytest.mark.asyncio
    async def test_one_failure_makes_result_fail(self) -> None:
        runner = make_runner(ups_level=0.0)
        result = await runner.run()
        assert result.passed is False
        assert len(result.failed_items) == 1
        assert result.failed_items[0].item_name == "UPS残量"


def _make_button_servo(*, ok: bool = True, raises: bool = False) -> AsyncMock:
    servo = AsyncMock()
    if raises:
        servo.check_connection = AsyncMock(side_effect=RuntimeError("I2C error"))
    else:
        servo.check_connection = AsyncMock(return_value=ok)
    return servo


class TestCheckButtonServo:
    @pytest.mark.asyncio
    async def test_not_included_by_default(self) -> None:
        """include_button_servo=False（既定）では 7 項目のまま。"""
        runner = PreCheckRunner(
            accel_driver=make_accel_driver(),
            brake_driver=make_brake_driver(),
            can_reader=make_can_reader(),
            ups_monitor=make_ups(),
            profile=make_profile(),
            button_servo=_make_button_servo(),
        )
        result = await runner.run()
        assert len(result.items) == 7
        assert all(i.item_name != "ボタンサーボ確認" for i in result.items)

    @pytest.mark.asyncio
    async def test_included_adds_eighth_item_and_passes(self) -> None:
        runner = PreCheckRunner(
            accel_driver=make_accel_driver(),
            brake_driver=make_brake_driver(),
            can_reader=make_can_reader(),
            ups_monitor=make_ups(),
            profile=make_profile(),
            button_servo=_make_button_servo(ok=True),
        )
        result = await runner.run(include_button_servo=True)
        assert len(result.items) == 8
        item = next(i for i in result.items if i.item_name == "ボタンサーボ確認")
        assert item.passed is True
        assert result.passed is True

    @pytest.mark.asyncio
    async def test_missing_button_servo_fails(self) -> None:
        runner = PreCheckRunner(
            accel_driver=make_accel_driver(),
            brake_driver=make_brake_driver(),
            can_reader=make_can_reader(),
            ups_monitor=make_ups(),
            profile=make_profile(),
            button_servo=None,
        )
        result = await runner.run(include_button_servo=True)
        item = next(i for i in result.items if i.item_name == "ボタンサーボ確認")
        assert item.passed is False
        assert result.passed is False

    @pytest.mark.asyncio
    async def test_i2c_unreachable_fails(self) -> None:
        runner = PreCheckRunner(
            accel_driver=make_accel_driver(),
            brake_driver=make_brake_driver(),
            can_reader=make_can_reader(),
            ups_monitor=make_ups(),
            profile=make_profile(),
            button_servo=_make_button_servo(ok=False),
        )
        result = await runner.run(include_button_servo=True)
        item = next(i for i in result.items if i.item_name == "ボタンサーボ確認")
        assert item.passed is False
        assert "PCA9685" in (item.error_message or "")

    @pytest.mark.asyncio
    async def test_check_connection_exception_fails(self) -> None:
        runner = PreCheckRunner(
            accel_driver=make_accel_driver(),
            brake_driver=make_brake_driver(),
            can_reader=make_can_reader(),
            ups_monitor=make_ups(),
            profile=make_profile(),
            button_servo=_make_button_servo(raises=True),
        )
        result = await runner.run(include_button_servo=True)
        item = next(i for i in result.items if i.item_name == "ボタンサーボ確認")
        assert item.passed is False
        assert result.passed is False
