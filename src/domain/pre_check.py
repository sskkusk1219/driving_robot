"""走行前チェック: 走行開始前に実施する6項目のシステム状態確認。"""

from typing import Protocol

from src.models.pre_check import PreCheckItemResult, PreCheckResult
from src.models.profile import VehicleProfile

HOME_POSITION_TOLERANCE_PULSE: int = 10
UPS_MIN_BATTERY_PCT: float = 20.0


class ActuatorPreCheckProtocol(Protocol):
    async def read_position(self) -> int: ...

    async def is_alarm_active(self) -> bool: ...


class CANPreCheckProtocol(Protocol):
    async def read_speed(self) -> float: ...


class UPSPreCheckProtocol(Protocol):
    async def get_battery_level_pct(self) -> float: ...


class PreCheckRunner:
    """6項目の走行前チェックを実行しドメイン結果を返す。

    ハードウェア依存は Protocol で分離しているためユニットテスト可能。
    UPS 取得方法は機種確定後に UPSPreCheckProtocol の実装を提供する。
    """

    def __init__(
        self,
        accel_driver: ActuatorPreCheckProtocol,
        brake_driver: ActuatorPreCheckProtocol,
        can_reader: CANPreCheckProtocol,
        ups_monitor: UPSPreCheckProtocol,
        profile: VehicleProfile | None,
        home_position_tolerance_pulse: int = HOME_POSITION_TOLERANCE_PULSE,
        ups_min_battery_pct: float = UPS_MIN_BATTERY_PCT,
    ) -> None:
        self._accel = accel_driver
        self._brake = brake_driver
        self._can = can_reader
        self._ups = ups_monitor
        self._profile = profile
        self._tolerance = home_position_tolerance_pulse
        self._ups_min = ups_min_battery_pct

    async def run(self) -> PreCheckResult:
        """全6項目をチェックし結果を返す。いずれか1項目でも NG なら passed=False。"""
        items = [
            await self._check_communication(),
            await self._check_servo_state(),
            await self._check_calibration(),
            await self._check_profile(),
            await self._check_ups_battery(),
            await self._check_actuator_position(),
        ]
        return PreCheckResult(passed=all(i.passed for i in items), items=items)

    async def _check_communication(self) -> PreCheckItemResult:
        """通信確認: ttyUSB0・ttyUSB1・CAN が応答するか。"""
        try:
            await self._accel.read_position()
            await self._brake.read_position()
            await self._can.read_speed()
            return PreCheckItemResult(item_name="通信確認", passed=True)
        except Exception as e:
            return PreCheckItemResult(
                item_name="通信確認", passed=False, error_message=f"通信エラー: {e}"
            )

    async def _check_servo_state(self) -> PreCheckItemResult:
        """サーボ状態: 両軸のアラームがないか。

        サーボON確認は行わない。initialize() でサーボONを完了済みであることを前提とする。
        アラームが発生している場合はサーボが自動OFFになるため、アラームなし = サーボON済みと等価。
        """
        try:
            accel_alarm = await self._accel.is_alarm_active()
            brake_alarm = await self._brake.is_alarm_active()
            if accel_alarm or brake_alarm:
                axes = []
                if accel_alarm:
                    axes.append("アクセル軸")
                if brake_alarm:
                    axes.append("ブレーキ軸")
                return PreCheckItemResult(
                    item_name="サーボ状態",
                    passed=False,
                    error_message=f"アラームあり: {', '.join(axes)}",
                )
            return PreCheckItemResult(item_name="サーボ状態", passed=True)
        except Exception as e:
            return PreCheckItemResult(
                item_name="サーボ状態", passed=False, error_message=f"サーボ状態取得エラー: {e}"
            )

    async def _check_profile(self) -> PreCheckItemResult:
        """プロファイル: 車両プロファイルが選択されているか。"""
        if self._profile is None:
            return PreCheckItemResult(
                item_name="プロファイル",
                passed=False,
                error_message="車両プロファイルが選択されていません",
            )
        return PreCheckItemResult(item_name="プロファイル", passed=True)

    async def _check_calibration(self) -> PreCheckItemResult:
        """キャリブレーション: 有効なキャリブレーションデータが存在するか。"""
        if self._profile is None:
            return PreCheckItemResult(
                item_name="キャリブレーション",
                passed=False,
                error_message="プロファイル未選択のためキャリブレーション確認不可",
            )
        if self._profile.calibration is None:
            return PreCheckItemResult(
                item_name="キャリブレーション",
                passed=False,
                error_message="キャリブレーションデータがありません",
            )
        if not self._profile.calibration.is_valid:
            return PreCheckItemResult(
                item_name="キャリブレーション",
                passed=False,
                error_message="キャリブレーションデータが無効です",
            )
        return PreCheckItemResult(item_name="キャリブレーション", passed=True)

    async def _check_ups_battery(self) -> PreCheckItemResult:
        """UPS残量: バッテリー残量が閾値以上か。"""
        try:
            level = await self._ups.get_battery_level_pct()
            if level < self._ups_min:
                return PreCheckItemResult(
                    item_name="UPS残量",
                    passed=False,
                    error_message=f"UPS残量不足: {level:.1f}%（{self._ups_min:.0f}%以上必要）",
                )
            return PreCheckItemResult(item_name="UPS残量", passed=True)
        except Exception as e:
            return PreCheckItemResult(
                item_name="UPS残量", passed=False, error_message=f"UPS残量取得エラー: {e}"
            )

    async def _check_actuator_position(self) -> PreCheckItemResult:
        """アクチュエータ位置: 両軸が原点付近にあるか。"""
        try:
            accel_pos = await self._accel.read_position()
            brake_pos = await self._brake.read_position()
            off_home = []
            if abs(accel_pos) > self._tolerance:
                off_home.append(f"アクセル軸={accel_pos}pulse")
            if abs(brake_pos) > self._tolerance:
                off_home.append(f"ブレーキ軸={brake_pos}pulse")
            if off_home:
                return PreCheckItemResult(
                    item_name="アクチュエータ位置",
                    passed=False,
                    error_message=f"原点から離れています: {', '.join(off_home)}",
                )
            return PreCheckItemResult(item_name="アクチュエータ位置", passed=True)
        except Exception as e:
            return PreCheckItemResult(
                item_name="アクチュエータ位置",
                passed=False,
                error_message=f"位置取得エラー: {e}",
            )
