# タスクリスト: サイドバー折り返し修正

## タスク

- [x] Frame の gridTemplateColumns を '180px 1fr' → '200px 1fr' に変更
- [x] nav item div に whiteSpace: 'nowrap' を追加

## 実装後の振り返り

- **実装完了日**: 2026-05-22
- **変更ファイル**: `src/web/static/js/sketch.js`

### 計画と実績の差分

計画通り。サイドバー幅の拡張と `whiteSpace: 'nowrap'` の追加のみで完結した。

### 学んだこと

- CJK文字はフルwidth(1em)で描画されるため、文字数×フォントサイズで幅を正確に見積もれる
- `whiteSpace: 'nowrap'` をサイドバー項目に付けておくと、ラベル追加時の折り返し問題を予防できる
- サイドバー幅の変更は `Frame` の `gridTemplateColumns` 1箇所を変えるだけで全体に反映される

### 次回への改善提案

- 長いラベルを追加する際は事前に「文字数×フォントサイズ < 有効テキスト幅」を確認する
