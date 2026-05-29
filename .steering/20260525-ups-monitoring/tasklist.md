# タスクリスト: UPS監視・制御機能

## フェーズ1: インフラ層の実装

- [x] ステアリングファイル作成（requirements.md / design.md / tasklist.md）
- [x] `src/infra/ups_monitor.py` 実装
  - [x] `UPSStatus` dataclass 定義
  - [x] `NutUPSMonitor` クラス実装
    - [x] NUT socket プロトコルによる `battery.charge` 取得
    - [x] NUT socket プロトコルによる `ups.status` 取得
    - [x] 5秒ポーリングループ（asyncio.Task）
    - [x] AC断（OL→OB遷移）コールバック機能
    - [x] NUT 接続失敗時のフォールバック（キャッシュ保持）
    - [x] `get_battery_level_pct()` → UPSPreCheckProtocol 実装
    - [x] `is_on_battery` プロパティ
    - [x] `is_available` プロパティ
- [x] `src/infra/settings.py` 更新（`UpsSettings` 追加）

## フェーズ2: アプリケーション層の更新

- [x] `src/app/factory.py` 更新
  - [x] `NutUPSMonitor` のインポート・生成
  - [x] `PreCheckRunner` の生成と `RobotController` への注入
  - [x] `NutUPSMonitor` に AC断コールバック登録
  - [x] `GPIOMonitor` から AC断コールバック登録を削除
  - [x] `NutUPSMonitor.start_polling()` を起動（app.py lifespan で呼ぶ）
- [x] `src/app/stubs.py` 更新（`_StubUPSMonitor` 追加）

## フェーズ3: Web層の更新

- [x] `src/web/schemas.py` 更新
  - [x] `RealtimeData` に `ups_battery_pct` / `ups_on_battery` 追加
  - [x] `UPSStatusResponse` 追加
- [x] `src/web/deps.py` 更新（`get_ups_monitor` 追加）
- [x] `src/web/routers/ups.py` 新規作成（`GET /api/v1/ups/status`）
- [x] `src/web/app.py` 更新
  - [x] `ups_monitor` を `app.state` に設定
  - [x] lifespan で start/stop
  - [x] ups router を include
- [x] `src/web/ws.py` 更新（WebSocket データに UPS 情報追加）

## フェーズ4: 設定・スクリプト

- [x] `config/settings.toml.example` 更新（`[ups]` セクション追加）
- [x] `scripts/setup_nut.sh` 新規作成

## フェーズ5: ドキュメント更新

- [x] `docs/functional-design.md` 更新（UPS TBD セクションを確定情報に）
- [x] `docs/glossary.md` 更新（AC UPS エントリ）
- [x] `docs/architecture.md` 更新（GPIO27 要確認 → 確定）

## フェーズ6: テスト

- [x] `tests/unit/infra/test_ups_monitor.py` 作成
  - [x] NUT レスポンスパースのテスト
  - [x] AC断コールバックのエッジ検知テスト（OL→OB のみ発火）
  - [x] NUT 接続失敗時フォールバックのテスト
- [x] `tests/unit/test_factory.py` 修正（`build_real_controller` タプル返却対応）
  - [x] 5テストのアンパック対応（`ctrl, _ = await build_real_controller(settings)`）
  - [x] `test_gpio_ac_loss_callback_registered_to_safety_monitor` → `test_nut_ac_loss_callback_registered_to_safety_monitor` に改名・NutUPSMonitor モック対応

## 実装後の振り返り

**実装完了日**: 2026-05-25

**計画との差分**:
- `build_real_controller` の返値を `tuple[RobotController, NutUPSMonitor]` に変更したため、既存の factory テスト 5 件が失敗した。計画段階では factory テストへの影響を見落としていた。
- `PreCheckRunner.set_profile()` の追加は計画外だったが、`select_profile()` 時のプロファイル同期に必要だったため追加した。
- GPIO27 による AC断検知は当初 TBD だったが、SUA750JB に NC/NO 接点がないことが判明し、NUT ポーリング方式に確定した。

**学んだこと**:
- APC Smart-UPS 750（SUA750JB）はシリアル接続のみで USB は補助的。`apcsmart` ドライバの設定が必要。
- NUT socket プロトコルは TCP 3493 番でシンプルなテキストプロトコル。`asyncio.open_connection` で直接実装すると subprocess 不要でクリーンになる。
- OL→OB の立ち上がりエッジ検知（`_prev_on_battery` フラグ）でバッテリー運転継続中の重複コールバックを防ぐパターンは有効。

**次回への改善提案**:
- ファクトリの返値型を変更する際は、関連する全テストをスコープに含めること。
- NUT 接続設定スクリプト（`scripts/setup_nut.sh`）の動作確認は実機でシリアルポートを確認してから実施すること（`/dev/ttyUSB*` のデバイスパスは環境依存）。
