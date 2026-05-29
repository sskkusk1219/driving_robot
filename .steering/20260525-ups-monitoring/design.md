# 設計: UPS監視・制御機能

## アーキテクチャ概要

```
NUT daemon (upsd)
    ↑ apcsmart driver
    APC Smart-UPS 750 (serial → /dev/ttyUSBx)

NutUPSMonitor (src/infra/ups_monitor.py)
    ← TCP socket → NUT daemon (localhost:3493)
    - 5秒ポーリングでキャッシュ更新
    - UPSPreCheckProtocol 実装（factory.py で PreCheckRunner に注入）
    - AC断コールバック → SafetyMonitor.handle_ac_power_loss()

factory.py
    - NutUPSMonitor を生成・起動
    - PreCheckRunner (accel_driver, brake_driver, can_reader, ups_monitor) を生成
    - RobotController の pre_check_runner に注入
    - GPIO27 AC断コールバック登録を削除（NUT ポーリングに変更）
```

## 新規ファイル

### `src/infra/ups_monitor.py`

**クラス**: `NutUPSMonitor`

**責務**: NUT サーバーからUPS状態を取得し、AC断を検知してコールバックを呼ぶ

**NUT socket プロトコル**:
- TCP 3493 に接続
- `GET VAR <ups_name> battery.charge` → `VAR <ups_name> battery.charge "95"`
- `GET VAR <ups_name> ups.status` → `VAR <ups_name> ups.status "OL CHRG"`
- 認証なし（デフォルト NUT 設定でローカル読み取りは認証不要）

**キャッシュ戦略**:
- `_cached_battery_pct: float` ← 最後に取得したバッテリー残量
- `_cached_on_battery: bool` ← AC断フラグ
- `_nut_available: bool` ← NUT に接続できるかどうか
- ポーリング失敗時は前回キャッシュを保持（NUT 一時停止を許容）

**AC断検知**:
- 前回 `OL`（On Line）→ 今回 `OB`（On Battery）の遷移でコールバック発火
- 立ち上がりエッジのみ（チャタリング抑制）

**メソッド**:
```python
class NutUPSMonitor:
    async def get_battery_level_pct(self) -> float  # UPSPreCheckProtocol
    async def get_status(self) -> UPSStatus          # 詳細状態
    def register_ac_loss_callback(cb: AsyncCallback) -> None
    async def start_polling(loop: asyncio.AbstractEventLoop) -> None
    async def stop_polling() -> None
    @property
    def is_available(self) -> bool   # NUT に接続できているか
    @property
    def is_on_battery(self) -> bool  # キャッシュ済みAC断状態
```

**モデル**: `UPSStatus` dataclass
```python
@dataclass
class UPSStatus:
    battery_charge_pct: float
    on_battery: bool          # OB フラグ
    low_battery: bool         # LB フラグ
    status_flags: str         # 生の status 文字列 ("OL CHRG" など)
    is_available: bool        # NUT 接続状態
```

### `scripts/setup_nut.sh`

NUT のインストール・設定手順を自動化するスクリプト:
1. `sudo apt install nut nut-client`
2. `/etc/nut/nut.conf` に `MODE=standalone` を設定
3. `/etc/nut/ups.conf` に apcsmart ドライバを設定
4. `/etc/nut/upsd.conf` にローカルリスンを設定
5. `/etc/nut/upsmon.conf` にモニター設定を追加
6. NUT サービスの有効化・起動

## 変更ファイル

### `src/infra/settings.py`

`UpsSettings` dataclass を追加:
```python
@dataclass
class UpsSettings:
    nut_host: str = "localhost"
    nut_port: int = 3493
    ups_name: str = "apcups"
    poll_interval_s: float = 5.0
```
`AppSettings` に `ups: UpsSettings` フィールドを追加。

### `src/app/factory.py`

1. `NutUPSMonitor` をインポート・生成
2. `PreCheckRunner` を生成（ups_monitor, accel_driver, brake_driver, can_reader, profile=None）
3. `RobotController` に `pre_check_runner` を注入
4. `NutUPSMonitor.register_ac_loss_callback(safety_monitor.handle_ac_power_loss)` を追加
5. `gpio_monitor.register_ac_loss_callback(...)` を **削除**（GPIO27 AC断登録を外す）
6. `NutUPSMonitor.start_polling()` を起動

### `src/app/stubs.py`

`_StubUPSMonitor` クラスを追加:
```python
class _StubUPSMonitor:
    async def get_battery_level_pct(self) -> float:
        return 100.0  # 常に満充電
    async def get_status(self) -> UPSStatus:
        ...
    @property
    def is_on_battery(self) -> bool:
        return False
    @property
    def is_available(self) -> bool:
        return True
```

### `src/web/schemas.py`

`RealtimeData` に UPS フィールドを追加:
```python
class RealtimeData(BaseModel):
    ...
    ups_battery_pct: float | None = None
    ups_on_battery: bool = False
```

`UPSStatusResponse` を追加:
```python
class UPSStatusResponse(BaseModel):
    battery_charge_pct: float
    on_battery: bool
    low_battery: bool
    status_flags: str
    is_available: bool
```

### `src/web/deps.py`

`get_ups_monitor` 依存関係を追加:
```python
def get_ups_monitor(request: Request) -> NutUPSMonitor:
    return request.app.state.ups_monitor
```
→ 型は Protocol で抽象化（実機/スタブ両対応）

### `src/web/app.py`

1. `app.state.ups_monitor` に NutUPSMonitor（またはスタブ）を設定
2. lifespan で `ups_monitor.start_polling()` / `stop_polling()`
3. `ups.router` を include

### `src/web/routers/ups.py` (新規)

```
GET /api/v1/ups/status
Response: UPSStatusResponse
```

### `src/web/ws.py`

`broadcast_loop` で `ups_monitor.get_status()` を呼び WebSocket データに追加

### `config/settings.toml.example`

```toml
[ups]
# NUT サーバーホスト（通常は localhost）
nut_host = "localhost"
# NUT サーバーポート
nut_port = 3493
# NUT での UPS 名（ups.conf の設定名と一致させること）
ups_name = "apcups"
# バッテリー状態ポーリング間隔 [s]
poll_interval_s = 5.0
```

`[gpio]` セクションの `ac_detect_pin` にコメントで「NUT 使用時は不要」を追記。

## ドキュメント更新

### `docs/functional-design.md`

- SafetyMonitor の「AC UPS に関する設計前提（機種未確定）」セクションを確定情報に更新
- 走行前チェック仕様の「UPS残量（TBD）」を NUT 経由に更新

### `docs/glossary.md`

- `AC UPS` エントリの機種 TBD 記述を更新

### `docs/architecture.md`

- GPIO27 の [要確認] 記述を更新

## テスト戦略

### ユニットテスト (`tests/unit/infra/test_ups_monitor.py`)

- `_parse_nut_response` のパース正確性
- AC断コールバックのエッジ検知（OL→OB のみ発火）
- NUT 接続失敗時のフォールバック動作

### ハードウェアテスト (`tests/hardware/test_ups_monitor.py`)

- 実機 NUT サーバーへの接続確認
- バッテリー残量の実値取得
- AC電源断シミュレーション（コンセント抜き）
