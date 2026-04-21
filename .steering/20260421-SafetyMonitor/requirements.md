# 要求仕様: SafetyMonitor 実装

## 対象機能

`src/domain/safety_monitor.py` — 安全監視ドメインクラス。ハードウェア不要で実装・テスト可能。

## 参照仕様 (docs/functional-design.md)

```python
class SafetyMonitor:
    async def start_monitoring() -> None
    def register_emergency_callback(cb: Callable) -> None
    def check_overcurrent(current_ma: float, axis: str) -> bool
    def check_deviation(ref: float, actual: float, duration: float) -> bool
    async def handle_ac_power_loss() -> None
```

### 電流異常検出（走行中）
- `check_overcurrent(current_ma, axis)` → `current_ma > OVERCURRENT_LIMIT_MA`
- `OVERCURRENT_LIMIT_MA` はコンストラクタで設定（デフォルト 3000.0 mA）

### 逸脱判定
- `check_deviation(ref, actual, duration)` → `|ref - actual| > deviation_threshold_kmh AND duration >= deviation_duration_s`
- `StopConfig` をコンストラクタで受け取る

### AC電源断シーケンス
- `handle_ac_power_loss()` → 緊急コールバックを全呼び出し
- 実機では GPIOMonitor (infra) がこのメソッドを呼ぶ

### 非常停止コールバック管理
- `register_emergency_callback(cb)` で複数登録可能
- `trigger_emergency()` で登録済みコールバックを全て async で呼ぶ（内部メソッドとして追加）

## 設計方針

- ドメイン層のため GPIO に直接触れない（GPIO は infra の GPIOMonitor が担当）
- `start_monitoring()` はドメイン側では監視フラグを立てるのみ（実機 GPIO 設定は infra 側）
- asyncio の Callable を型安全に扱う

## 実装ファイル

| ファイル | 内容 |
|---------|------|
| `src/domain/safety_monitor.py` | SafetyMonitor クラス |
| `tests/unit/test_safety_monitor.py` | ユニットテスト |
