# タスクリスト: GUI実装

## フェーズ1: 基盤構築

- [x] ステアリングファイル作成 (requirements.md / design.md / tasklist.md)
- [x] index.html 作成 (React + Babel + Google Fonts ロード)
- [x] js/sketch.js 作成 (UI プリミティブ: Frame, Box, Btn, Sidebar, TopBar 等)
- [x] js/app.js 作成 (アプリルーター・グローバル状態・WebSocket・API クライアント)

## フェーズ2: コア画面

- [x] js/screens/init.js 作成 (初期化・BOOTING・ERROR・STANDBY・EMERGENCY リセット)
- [x] js/calib-components.js 作成 (JogKey, PosRuler)
- [x] js/screens/calibration.js 作成 (CalibrationA: ジョグ操作 + 保存)
- [x] js/screens/auto-drive.js 作成 (AutoDriveD: 3 軸グラフ + 一時停止/走行終了)
- [x] js/screens/manual.js 作成 (ManualJog: ジョグ操作 + 運転終了)
- [x] js/screens/learning.js 作成 (LearningNew: AutoDriveD と同じ UI)

## フェーズ3: 管理画面

- [x] js/screens/profiles.js 作成 (ProfilesA 一覧 + ProfileCreate + ProfileEdit)
- [x] js/screens/modes.js 作成 (ModesA 一覧 + ModeCreate CSV プレビュー付き)
- [x] js/screens/logs.js 作成 (LogsA 一覧 + 詳細グラフ)

## フェーズ4: システム状態画面

- [x] js/screens/system-states.js 作成
  - AcPowerLossScreen (AC 断安全停止進捗)
  - AutoStopDeviationScreen (逸脱超過自動停止)
  - AutoStopOvercurrentScreen (過電流検知)
  - AutoStopCanTimeoutScreen (CAN タイムアウト)
  - PreCheckNGScreen (走行前チェック NG 詳細)

## フェーズ5: Post-MVP 画面

- [x] js/screens/schedule-sequence.js 作成（Post-MVP プレースホルダー）

## フェーズ7: 車両プロファイル画面ワイヤーフレーム修正

- [x] profiles.js 一覧をテーブル形式(Row)に変更 (ProfilesA準拠)
- [x] profiles.js 作成/編集を2カラムグリッド形式に変更 (ProfileCreate/ProfileEdit準拠)

## フェーズ6: 動作確認・修正

- [x] 初期化画面をワイヤーフレームInitAに合わせて修正 (技術ステップリスト・アニメーション)
- [x] EmergencyResetScreenをワイヤーフレームInitBに合わせて修正 (傾きバナー・停止ログ)
- [x] calibration.js をワイヤーフレーム CalibrationA に合わせて全面書き直し (ステップバー・2軸ジョグ・記録テーブル)
- [x] manual.js をワイヤーフレーム ManualJog に合わせて全面書き直し (2軸ジョグ・開度バー・ConfirmStopPopup)
- [x] auto-drive.js をワイヤーフレーム AutoDriveD に合わせて修正 (外側パディング削除・一時停止ボタン・ConfirmStopPopup・BigSpeed レイアウト)
- [x] calib-components.js の PosRuler を幅 480px / width="100%" で responsive に変更
- [ ] 開発サーバー起動 (`python -m uvicorn src.web.app:app --reload`)
- [ ] ブラウザでナビゲーション・画面遷移動作確認
- [ ] WebSocket リアルタイムデータ受信確認
- [ ] API 連携 (プロファイル CRUD / 走行開始停止) 確認
- [ ] 既存 CSS (`style.css`) の削除またはリセット

## 実装後の振り返り

（完了後に記入）
