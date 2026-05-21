# 設計: 走行モードページ修正

## 変更ファイル

`src/web/static/js/screens/modes.js` のみ

## ModesScreen (一覧) — ModesA 準拠

```
[H2 "走行モード" + subtitle]            [+ 新規作成 Btn]
Box padding=0:
  Row header: 名前|説明|長さ|最高車速|プレビュー|操作
  Row×N: データ行
```

- 各行のプレビュー列: `Hatch` コンポーネント (width=160, height=44)
  一覧APIに `reference_speed` は含まれないため
- 選択中行: 背景 `PAPER_2`、名前横に「選択中」Pill
- 長さ: `${total_duration}s (${Math.round(total_duration/60)}分)` 形式
- 操作ボタン: 「選択」(primary, 非選択時のみ)・「削除」(ghost)
- ModePreviewChart / インライン展開は削除

## ModeCreate (新規作成) — ModeCreate 準拠

```
[← 一覧に戻る]  [H2 "走行モード · 新規作成"]

Grid (340px | 1fr):
左:
  Box "基本情報": Input(モード名), textarea(説明)
  Box "CSVファイル": drop zone + validation list
  Note
  spacer
  [キャンセル] [保存]
右:
  Box "プレビュー":
    SpeedGraph (csvData から描画)
    stats grid (総時間/最高車速/平均車速/停止比率)
    Box "CSVサンプル (先頭5行)"
```

## CSV クライアントサイド処理

- ファイル選択後に FileReader で読み込み
- `time_s,speed_kmh` ヘッダーを検証
- `{time_s, speed_kmh}[]` の配列を state に保持
- SpeedGraph 用の専用 `CsvSpeedGraph` コンポーネントを追加
- 統計: 総時間=最後のtime_s、最高車速=max、平均車速=mean、停止比率=(speed==0の行/全行)
- バリデーション結果を validation 配列 state で管理
