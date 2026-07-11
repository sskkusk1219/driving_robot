"""走行前チェック: 走行開始前に実施するシステム状態確認。

項目1〜7は全走行モード共通。項目8（ボタンサーボ確認）はタイムスケジュール実行時のみ
`run(include_button_servo=True)` で追加される。
"""

from typing import Protocol

from src.domain.control.conversions import VEHICLE_STOP_SPEED_KMH
from src.models.pre_check import PreCheckItemResult, PreCheckResult
from src.models.profile import VehicleProfile

HOME_POSITION_TOLERANCE_PULSE: int = 10
UPS_MIN_BATTERY_PCT: float = 20.0
# これ未満を「停車中」とみなす。src.domain.control.conversions の共通定数を使う（A2 レビュー指摘）。
STOPPED_SPEED_THRESHOLD_KMH: float = VEHICLE_STOP_SPEED_KMH

# チェック項目名（exclude 指定や結果参照に使う）
ITEM_COMMUNICATION = "通信確認"
ITEM_SERVO = "サーボ状態"
ITEM_CALIBRATION = "キャリブレーション"
ITEM_PROFILE = "プロファイル"
ITEM_UPS = "UPS残量"
ITEM_ACTUATOR_POSITION = "アクチュエータ位置"
ITEM_VEHICLE_STOPPED = "車速確認"
ITEM_BUTTON_SERVO = "ボタンサーボ確認"  # 項目8: タイムスケジュール実行時のみ


class ActuatorPreCheckProtocol(Protocol):
    async def read_position(self) -> int: ...

    async def is_alarm_active(self) -> bool: ...


class ButtonServoPreCheckProtocol(Protocol):
    async def check_connection(self) -> bool: ...


class CANPreCheckProtocol(Protocol):
    async def read_speed(self) -> float: ...


class UPSPreCheckProtocol(Protocol):
    async def get_battery_level_pct(self) -> float: ...


class PreCheckRunner:
    """7項目の走行前チェックを実行しドメイン結果を返す。

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
        button_servo: ButtonServoPreCheckProtocol | None = None,
        home_position_tolerance_pulse: int = HOME_POSITION_TOLERANCE_PULSE,
        ups_min_battery_pct: float = UPS_MIN_BATTERY_PCT,
        stopped_speed_threshold_kmh: float = STOPPED_SPEED_THRESHOLD_KMH,
    ) -> None:
        self._accel = accel_driver
        self._brake = brake_driver
        self._can = can_reader
        self._ups = ups_monitor
        self._profile = profile
        self._button_servo = button_servo
        self._tolerance = home_position_tolerance_pulse
        self._ups_min = ups_min_battery_pct
        self._stopped_speed = stopped_speed_threshold_kmh

    def set_profile(self, profile: VehicleProfile | None) -> None:
        """アクティブプロファイルを更新する。RobotController.select_profile() から呼ばれる。"""
        self._profile = profile

    async def run(
        self,
        exclude: frozenset[str] = frozenset(),
        *,
        include_button_servo: bool = False,
    ) -> PreCheckResult:
        """走行前チェックを実行し結果を返す。いずれか1項目でも NG なら passed=False。

        `exclude` に項目名（ITEM_* 定数）を渡すとその項目をスキップする。学習運転の arm では
        「ブレーキ踏込前は車速確認を除外（停止前なので）」「踏込後はアクチュエータ位置を除外
        （保持ブレーキで意図的に原点から離れているため）」のように2段で使い分ける。

        `include_button_servo=True` でボタンサーボ確認（項目8）を追加する
        （タイムスケジュール実行時のみ。ボタンイベントを含むスケジュールで指定する）。
        """
        checks = [
            (ITEM_COMMUNICATION, self._check_communication),
            (ITEM_SERVO, self._check_servo_state),
            (ITEM_CALIBRATION, self._check_calibration),
            (ITEM_PROFILE, self._check_profile),
            (ITEM_UPS, self._check_ups_battery),
            (ITEM_ACTUATOR_POSITION, self._check_actuator_position),
            (ITEM_VEHICLE_STOPPED, self._check_vehicle_stopped),
        ]
        if include_button_servo:
            checks.append((ITEM_BUTTON_SERVO, self._check_button_servo))
        items = [await fn() for name, fn in checks if name not in exclude]
        return PreCheckResult(passed=all(i.passed for i in items), items=items)

    async def _check_button_servo(self) -> PreCheckItemResult:
        """ボタンサーボ確認: PCA9685 の I2C 疎通を確認する（タイムスケジュール時のみ）。"""
        if self._button_servo is None:
            return PreCheckItemResult(
                item_name=ITEM_BUTTON_SERVO,
                passed=False,
                error_message="ボタンサーボが構成されていません",
            )
        try:
            if not await self._button_servo.check_connection():
                return PreCheckItemResult(
                    item_name=ITEM_BUTTON_SERVO,
                    passed=False,
                    error_message="PCA9685 と通信できません（I2C 配線・電源・アドレスを確認）",
                )
            return PreCheckItemResult(item_name=ITEM_BUTTON_SERVO, passed=True)
        except Exception as e:
            return PreCheckItemResult(
                item_name=ITEM_BUTTON_SERVO,
                passed=False,
                error_message=f"ボタンサーボ確認エラー: {e}",
            )

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

    async def _check_vehicle_stopped(self) -> PreCheckItemResult:
        """車速確認: 車速が停車しきい値未満か。走行中の学習・走行開始を防止する。"""
        try:
            speed = await self._can.read_speed()
            if abs(speed) >= self._stopped_speed:
                return PreCheckItemResult(
                    item_name="車速確認",
                    passed=False,
                    error_message=(
                        f"車速が 0 ではありません: {speed:.1f}km/h"
                        f"（{self._stopped_speed:.1f}km/h 未満で停車）"
                    ),
                )
            return PreCheckItemResult(item_name="車速確認", passed=True)
        except Exception as e:
            return PreCheckItemResult(
                item_name="車速確認", passed=False, error_message=f"車速取得エラー: {e}"
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
