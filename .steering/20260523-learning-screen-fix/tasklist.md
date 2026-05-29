# タスクリスト: 学習運転ページ修正

## タスク

- [x] 走行開始/終了ボタン切替ロジックを実装
  - [x] DriveMonitorScreenにrobotStateとdriveStartPathプロップを追加
  - [x] RUNNING以外 → 緑色「▶ 走行開始」ボタン表示（driveStartPath呼び出し）
  - [x] RUNNING中 → 赤「■ 走行終了」ボタン表示（既存stop処理）
  - [x] learning.jsにdriveStartPath="/api/v1/learning/start"を渡す
  - [x] AutoDriveScreenにdriveStartPath="/api/v1/drive/start"を渡す
- [x] グラフ3つを均等サイズに拡大
  - [x] GraphSvgのSVG heightを"100%"に変更
  - [x] 各グラフBoxにflex:1 + minHeight:0を付与
  - [x] SVGをflex:1ラッパーdivで包む
  - [x] 3軸目（プロファイル概観）の独自SVGも同様に対応

## 実装後の振り返り

- 実装完了日: 2026-05-23
- 変更ファイル: `auto-drive.js`, `learning.js`
- ボタン: `robotState === 'RUNNING'`で走行終了（赤）/走行開始（緑）を切替
- グラフ: 各Boxにflex:1+minHeight:0を付与、SVG wrapperも同様→画面縦幅を3等分して均等拡大
- `GraphSvg`のSVG heightを"100%"に変更（viewBoxの座標系は維持）
- 自動走行画面も同じDriveMonitorScreenを使用するため同時に修正完了
- 自動運転の走行開始APIは mode_id ボディが必要なため driveStartBody prop を追加
- キャリブレーション保存ボタンの `<br />` 改行を除去して1行表示に修正
