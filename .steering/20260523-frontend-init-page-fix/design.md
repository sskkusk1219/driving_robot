# 設計: フロントエンド初期化ページ修正

## 変更箇所

### init.js

1. **STEPS配列** を3要素に置き換え（subなし）
2. **キャンセルボタン** のJSX要素を削除
3. subが存在しない場合の表示行（`<div style={{ fontSize: 12 }}>{sub}</div>`）は、subが空なら非表示にする or 削除

## 影響範囲
- `src/web/static/js/screens/init.js` のみ
- バックエンドAPIへの変更なし
