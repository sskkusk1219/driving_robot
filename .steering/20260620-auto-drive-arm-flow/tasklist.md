# タスクリスト

- [x] `RobotController._arm_drive` / `_cancel_armed_drive` を抽出
- [x] `arm_learning_drive` / `cancel_learning_drive` を共通処理のラッパに変更
- [x] `arm_auto_drive` / `cancel_auto_drive` を追加
- [x] `start_auto_drive` を二経路（PRE_CHECK / READY）対応に変更
- [x] `/api/v1/drive/arm` / `/api/v1/drive/cancel` エンドポイント追加
- [x] `auto-drive.js` に arm フロー props を追加
- [x] コントローラ・Web のテスト追加
- [x] テスト実行（pytest）— 644 passed / ruff OK
