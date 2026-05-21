# 要求: フロントエンド走行モードページ修正

## 背景

`docs/ideas/driving_robot_front_end-handoff.zip` のワイヤーフレームのレイアウトと、
現在の `src/web/static/js/screens/modes.js` の実装が乖離している。

## 要求内容

ModesA / ModeCreate レイアウトに合わせて走行モードページを修正する。

### ModesScreen (一覧) - ModesA 準拠

- テーブル形式: 名前 / 説明 / 長さ / 最高車速 / プレビュー / 操作
- 各行に SpeedGraph のミニプレビューを列として表示
- 選択中は強調表示 (太枠・背景色)
- 操作ボタン: 「選択」(primary)・「削除」

### ModeCreate (新規作成) - ModeCreate 準拠

- 2カラムレイアウト: 左 340px = フォーム、右 = プレビュー
- 左: 基本情報 Box + CSV drop zone Box (バリデーション結果) + Note
- 右: プレビュー Box (SpeedGraph + 統計サマリ + CSVサンプル)

## 制約

- React.createElement ベースの既存パターン維持
- API インターフェース変更なし
