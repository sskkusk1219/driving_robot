# タスクリスト: 初期化ページのハードウェア連動

- [x] T1: `system_state.py` に `InitStepStatus` と `InitStep` を追加
- [x] T2: `robot_controller.py` に init 進捗管理（ビルド/更新/プロパティ）を追加
- [x] T3: `robot_controller.initialize()` を進捗追従型に書き換え
- [x] T4: `schemas.py` に `InitStepSchema` 追加・`RealtimeData` 拡張
- [x] T5: `ws.py` の `broadcast_loop` で init_steps を配信
- [x] T6: `app.js` の `INIT_REALTIME` に `init_steps` 追加
- [x] T7: `init.js` を WebSocket 進捗ベースに書き換え（疑似タイマー廃止）
- [x] T8: テスト・lint・typecheck を実行して通す
- [x] T9: 振り返りを tasklist.md に記載

---

## 振り返り

**実装完了日**: 2026-06-10

### 計画と実績の差分
- ほぼ計画どおり。追加で `TestInitProgress`（4ケース）を
  `tests/unit/test_robot_controller.py` に追加し、進捗連動・スキップ・エラー時
  挙動を回帰防止としてロックした。

### 実装内容
- 課題: フロント初期化画面はバックエンドの実ハード操作と無関係に、経過時間
  ベースの疑似タイマー（`doneAt = i * 1.5`）でチェックを進めていた。
- 解決: `RobotController.initialize()` が各ステップ
  （通信確認ブレーキ/アクセル/CAN・アラームリセット・サーボON・原点復帰）の
  状態を `_init_steps` に逐次反映。既存の WebSocket 配信（100ms 周期）に
  `init_steps` を相乗りさせ、フロントは配信値をそのまま描画する。
- 通信確認は実ハード応答（enable_modbus_control / read_speed）の完了で DONE 化。
  各操作の呼び出し回数は従来どおり（既存テスト不変）。

### 学んだこと
- 既存の `broadcast_loop` に相乗りすることで、新規ポーリングなしにリアルタイム
  連動を実現できた。`MagicMock.__iter__` がデフォルトで空イテレータを返すため、
  `controller.init_progress` をモックする既存 WS テストは無改修で通る。

### 検証結果
- 関連ユニット（test_robot_controller / test_ws_broadcast / test_web_drive）133件 PASS。
- 新規 TestInitProgress 4件 PASS。integration/test_web_api 8件 PASS。
- 変更ファイルの ruff / mypy はクリーン。
- 注記: フルスイートには本変更と無関係な低速/ハング気味のテストが存在するため、
  関連スコープを対象に検証した。
- 注記: `/add-feature` 手順の implementation-validator サブエージェント起動は、
  コスト面の方針に従い inline 検証（テスト/lint/typecheck/パターン整合）で代替。

### 次回への改善提案
- 原点復帰など長時間ステップの「進捗率（%）」表示があるとさらに分かりやすい。
- フルテストスイートのハング原因（integration/e2e 系の特定テスト）を別途調査推奨。
