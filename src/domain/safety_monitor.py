from collections.abc import Awaitable, Callable

from src.models.profile import StopConfig

# 走行中の過電流保護デフォルト閾値 (P-CON-CB 仕様から設定)
DEFAULT_OVERCURRENT_LIMIT_MA = 3000.0


class SafetyMonitor:
    """安全監視ドメインクラス。GPIO には触れない（割り込みは infra の GPIOMonitor が担当）。"""

    _stop_config: StopConfig
    _overcurrent_limit_ma: float
    _emergency_callbacks: list[Callable[[], Awaitable[None]]]
    _is_monitoring: bool

    def __init__(
        self,
        stop_config: StopConfig,
        overcurrent_limit_ma: float = DEFAULT_OVERCURRENT_LIMIT_MA,
    ) -> None:
        self._stop_config = stop_config
        self._overcurrent_limit_ma = overcurrent_limit_ma
        self._emergency_callbacks = []
        self._is_monitoring = False

    @property
    def is_monitoring(self) -> bool:
        return self._is_monitoring

    async def start_monitoring(self) -> None:
        """監視状態に移行する。実機 GPIO 割り込み設定は GPIOMonitor (infra) が担当。"""
        self._is_monitoring = True

    async def stop_monitoring(self) -> None:
        """監視状態を終了する。"""
        self._is_monitoring = False

    def register_emergency_callback(self, cb: Callable[[], Awaitable[None]]) -> None:
        """非常停止コールバックを登録する。複数登録可能。登録順に呼ばれる。"""
        self._emergency_callbacks.append(cb)

    def check_overcurrent(self, current_ma: float, axis: str) -> bool:  # noqa: ARG002
        """電流値が過電流閾値を超えているか確認する。True なら緊急停止が必要。

        axis は将来的に軸ごとの閾値設定のために予約済み（現在は全軸共通の閾値を使用）。
        """
        return current_ma > self._overcurrent_limit_ma

    def check_deviation(self, ref: float, actual: float, duration: float) -> bool:
        """基準車速からの逸脱が閾値・継続時間の両方を超えているか確認する。

        Args:
            ref: 基準車速 [km/h]
            actual: 実車速 [km/h]
            duration: 逸脱が継続している時間 [s]（呼び出し元が計測・渡す）

        Note:
            逸脱量は threshold を「超える」(>) ことが条件。
            継続時間は threshold「以上」(>=) が条件（ちょうどの瞬間から停止を開始する設計）。
        """
        deviation = abs(ref - actual)
        return (
            deviation > self._stop_config.deviation_threshold_kmh
            and duration >= self._stop_config.deviation_duration_s
        )

    async def trigger_emergency(self) -> None:
        """緊急停止コールバックを全て呼ぶ。1件失敗しても後続を継続する（フェイルセーフ）。

        GPIOMonitor (infra)・過電流検知・逸脱超過から呼ばれる。
        複数コールバックが失敗した場合は ExceptionGroup にまとめて送出する。
        """
        errors: list[Exception] = []
        for cb in self._emergency_callbacks:
            try:
                await cb()
            except Exception as e:  # noqa: BLE001
                errors.append(e)
        if errors:
            raise ExceptionGroup("緊急停止コールバックの一部が失敗", errors)

    async def handle_ac_power_loss(self) -> None:
        """AC電源断検知時に緊急停止シーケンスを起動する。GPIOMonitor から呼ばれる。"""
        await self.trigger_emergency()
