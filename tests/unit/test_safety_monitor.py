import pytest

from src.domain.safety_monitor import DEFAULT_OVERCURRENT_LIMIT_MA, SafetyMonitor
from src.models.profile import StopConfig


def make_monitor(
    threshold_kmh: float = 2.0,
    duration_s: float = 4.0,
    overcurrent_ma: float = DEFAULT_OVERCURRENT_LIMIT_MA,
) -> SafetyMonitor:
    return SafetyMonitor(
        stop_config=StopConfig(
            deviation_threshold_kmh=threshold_kmh,
            deviation_duration_s=duration_s,
        ),
        overcurrent_limit_ma=overcurrent_ma,
    )


class TestSafetyMonitorStartMonitoring:
    @pytest.mark.asyncio
    async def test_start_monitoring_sets_flag(self) -> None:
        monitor = make_monitor()
        assert monitor.is_monitoring is False
        await monitor.start_monitoring()
        assert monitor.is_monitoring is True

    @pytest.mark.asyncio
    async def test_start_monitoring_idempotent(self) -> None:
        monitor = make_monitor()
        await monitor.start_monitoring()
        await monitor.start_monitoring()
        assert monitor.is_monitoring is True


class TestSafetyMonitorCheckOvercurrent:
    def test_below_limit_returns_false(self) -> None:
        monitor = make_monitor(overcurrent_ma=3000.0)
        assert monitor.check_overcurrent(current_ma=2999.9, axis="accel") is False

    def test_at_limit_returns_false(self) -> None:
        # 閾値ちょうどは超過ではない（strictly greater than）
        monitor = make_monitor(overcurrent_ma=3000.0)
        assert monitor.check_overcurrent(current_ma=3000.0, axis="accel") is False

    def test_above_limit_returns_true(self) -> None:
        monitor = make_monitor(overcurrent_ma=3000.0)
        assert monitor.check_overcurrent(current_ma=3000.1, axis="accel") is True

    def test_both_axes(self) -> None:
        monitor = make_monitor(overcurrent_ma=3000.0)
        assert monitor.check_overcurrent(current_ma=5000.0, axis="accel") is True
        assert monitor.check_overcurrent(current_ma=5000.0, axis="brake") is True

    def test_custom_limit(self) -> None:
        monitor = make_monitor(overcurrent_ma=1500.0)
        assert monitor.check_overcurrent(current_ma=1500.1, axis="accel") is True
        assert monitor.check_overcurrent(current_ma=1499.9, axis="accel") is False


class TestSafetyMonitorCheckDeviation:
    def test_no_deviation_returns_false(self) -> None:
        monitor = make_monitor(threshold_kmh=2.0, duration_s=4.0)
        assert monitor.check_deviation(ref=60.0, actual=60.0, duration=10.0) is False

    def test_deviation_below_threshold_returns_false(self) -> None:
        monitor = make_monitor(threshold_kmh=2.0, duration_s=4.0)
        # deviation=1.9 < threshold=2.0
        assert monitor.check_deviation(ref=60.0, actual=58.1, duration=10.0) is False

    def test_deviation_at_threshold_returns_false(self) -> None:
        # 閾値ちょうどは超過ではない（strictly greater than）
        monitor = make_monitor(threshold_kmh=2.0, duration_s=4.0)
        assert monitor.check_deviation(ref=60.0, actual=58.0, duration=10.0) is False

    def test_deviation_above_threshold_but_short_duration_returns_false(self) -> None:
        monitor = make_monitor(threshold_kmh=2.0, duration_s=4.0)
        # deviation=3.0 > threshold=2.0, but duration=3.9 < 4.0
        assert monitor.check_deviation(ref=60.0, actual=57.0, duration=3.9) is False

    def test_deviation_above_threshold_and_sufficient_duration_returns_true(self) -> None:
        monitor = make_monitor(threshold_kmh=2.0, duration_s=4.0)
        # deviation=3.0 > 2.0, duration=4.0 >= 4.0
        assert monitor.check_deviation(ref=60.0, actual=57.0, duration=4.0) is True

    def test_negative_deviation_uses_absolute_value(self) -> None:
        monitor = make_monitor(threshold_kmh=2.0, duration_s=4.0)
        # 実車速が基準より高い場合
        assert monitor.check_deviation(ref=57.0, actual=60.0, duration=5.0) is True

    def test_duration_exceeds_threshold(self) -> None:
        monitor = make_monitor(threshold_kmh=2.0, duration_s=4.0)
        assert monitor.check_deviation(ref=60.0, actual=55.0, duration=100.0) is True


class TestSafetyMonitorCallbacks:
    @pytest.mark.asyncio
    async def test_no_callbacks_trigger_emergency_does_nothing(self) -> None:
        monitor = make_monitor()
        # 例外なく完了すること
        await monitor.trigger_emergency()

    @pytest.mark.asyncio
    async def test_registered_callback_is_called(self) -> None:
        monitor = make_monitor()
        called = []

        async def cb() -> None:
            called.append(True)

        monitor.register_emergency_callback(cb)
        await monitor.trigger_emergency()
        assert called == [True]

    @pytest.mark.asyncio
    async def test_multiple_callbacks_all_called_in_order(self) -> None:
        monitor = make_monitor()
        order = []

        async def cb1() -> None:
            order.append(1)

        async def cb2() -> None:
            order.append(2)

        monitor.register_emergency_callback(cb1)
        monitor.register_emergency_callback(cb2)
        await monitor.trigger_emergency()
        assert order == [1, 2]

    @pytest.mark.asyncio
    async def test_register_same_callback_twice(self) -> None:
        monitor = make_monitor()
        called_count = 0

        async def cb() -> None:
            nonlocal called_count
            called_count += 1

        monitor.register_emergency_callback(cb)
        monitor.register_emergency_callback(cb)
        await monitor.trigger_emergency()
        assert called_count == 2

    @pytest.mark.asyncio
    async def test_failing_callback_does_not_stop_subsequent_callbacks(self) -> None:
        """1件目のコールバックが例外を送出しても2件目以降が呼ばれること（フェイルセーフ）。"""
        monitor = make_monitor()
        called = []

        async def failing_cb() -> None:
            raise RuntimeError("cb1 failed")

        async def ok_cb() -> None:
            called.append(True)

        monitor.register_emergency_callback(failing_cb)
        monitor.register_emergency_callback(ok_cb)

        with pytest.raises(ExceptionGroup):
            await monitor.trigger_emergency()
        assert called == [True]  # 2件目は実行された

    @pytest.mark.asyncio
    async def test_all_failing_callbacks_collected_in_exception_group(self) -> None:
        """複数コールバックが失敗した場合、全て ExceptionGroup にまとめられること。"""
        monitor = make_monitor()

        async def fail1() -> None:
            raise ValueError("fail1")

        async def fail2() -> None:
            raise RuntimeError("fail2")

        monitor.register_emergency_callback(fail1)
        monitor.register_emergency_callback(fail2)

        with pytest.raises(ExceptionGroup) as exc_info:
            await monitor.trigger_emergency()
        assert len(exc_info.value.exceptions) == 2


class TestSafetyMonitorHandleAcPowerLoss:
    @pytest.mark.asyncio
    async def test_handle_ac_power_loss_triggers_emergency(self) -> None:
        monitor = make_monitor()
        triggered = []

        async def cb() -> None:
            triggered.append(True)

        monitor.register_emergency_callback(cb)
        await monitor.handle_ac_power_loss()
        assert triggered == [True]

    @pytest.mark.asyncio
    async def test_handle_ac_power_loss_with_no_callbacks(self) -> None:
        monitor = make_monitor()
        # 例外なく完了すること
        await monitor.handle_ac_power_loss()
