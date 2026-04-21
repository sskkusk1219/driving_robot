# 設計: SafetyMonitor 実装

## クラス設計

```python
class SafetyMonitor:
    _stop_config: StopConfig
    _overcurrent_limit_ma: float
    _emergency_callbacks: list[Callable[[], Awaitable[None]]]
    _is_monitoring: bool

    def __init__(
        self,
        stop_config: StopConfig,
        overcurrent_limit_ma: float = 3000.0,
    ) -> None: ...

    async def start_monitoring(self) -> None:
        """監視フラグを立てる。実機 GPIO 割り込み設定は GPIOMonitor (infra) が担当。"""
        self._is_monitoring = True

    def register_emergency_callback(self, cb: Callable[[], Awaitable[None]]) -> None:
        self._emergency_callbacks.append(cb)

    def check_overcurrent(self, current_ma: float, axis: str) -> bool:
        return current_ma > self._overcurrent_limit_ma

    def check_deviation(self, ref: float, actual: float, duration: float) -> bool:
        deviation = abs(ref - actual)
        return (
            deviation > self._stop_config.deviation_threshold_kmh
            and duration >= self._stop_config.deviation_duration_s
        )

    async def trigger_emergency(self) -> None:
        """緊急停止コールバックを全て呼ぶ。GPIO割り込み・過電流・逸脱検知から呼ばれる。"""
        for cb in self._emergency_callbacks:
            await cb()

    async def handle_ac_power_loss(self) -> None:
        """AC電源断を検知したときに緊急停止シーケンスを起動する。"""
        await self.trigger_emergency()
```

## レイヤー境界

```
infra: GPIOMonitor
  ↓ (GPIO割り込みでコールバック呼び出し)
domain: SafetyMonitor.trigger_emergency()
  ↓ (登録済みコールバック)
app: RobotController.emergency_stop()
```

## 型注釈

```python
from collections.abc import Awaitable, Callable
```

`Callable[[], Awaitable[None]]` で asyncio コールバックを型安全に扱う。

## テスト方針

- `check_overcurrent`: 閾値境界（以下・超過）
- `check_deviation`: 逸脱量・継続時間の AND 条件（4ケース）
- `register_emergency_callback` + `trigger_emergency`: コールバック実際に呼ばれることを確認
- `handle_ac_power_loss`: trigger_emergency への委譲を確認
- `start_monitoring`: is_monitoring フラグ
