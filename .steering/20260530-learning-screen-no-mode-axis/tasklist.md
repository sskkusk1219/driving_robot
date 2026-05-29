# タスクリスト: 学習運転画面から走行モード(3軸目)を非表示

## 要求
学習運転は走行モードを使用しないため、DriveMonitorScreen の3軸目（走行モードプロファイル概観グラフ）を学習運転画面では表示しない。

## タスク

### フェーズ1: DriveMonitorScreen に showModeAxis プロップ追加

- [x] `DriveMonitorScreen` の引数に `showModeAxis = true` を追加
- [x] Axis 3 ブロック（Box全体）を `showModeAxis` で条件レンダリング
- [x] 下部セッション情報の「走行モード」「全体時間」行も `showModeAxis` で条件レンダリング

### フェーズ2: LearningScreen から showModeAxis={false} を渡す

- [x] `LearningScreen` の `DriveMonitorScreen` 呼び出しに `showModeAxis={false}` を追加

## 実装後の振り返り

- 実装完了日: 2026-05-30
- `showModeAxis` プロップを追加することで後方互換を保ちつつ学習運転のみ非表示化
- 「走行モード」行と「全体時間」行もセットで非表示にした（走行モードがないと全体時間も意味がないため）
