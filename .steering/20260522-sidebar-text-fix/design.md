# 設計: サイドバー折り返し修正

## 原因分析

- Frame の `gridTemplateColumns: '180px 1fr'` でサイドバー幅 180px
- nav item の `padding: '7px 18px'` で有効テキスト幅 = 180 - 18×2 = 144px
- `fontSize: 17` で CJK 1文字 ≒ 17px
- 「タイムスケジュール」9文字 × 17px = 153px > 144px → 折り返し発生

## 修正方針

### 変更箇所: `src/web/static/js/sketch.js`

1. **Frame の gridTemplateColumns を `200px 1fr` に変更**
   - サイドバー幅 200px → 有効テキスト幅 = 200 - 18×2 = 164px
   - 「タイムスケジュール」9文字 × 17px = 153px < 164px → 収まる

2. **nav item div に `whiteSpace: 'nowrap'` を追加**
   - 将来的なラベル変更でも折り返しが起きないよう保険
