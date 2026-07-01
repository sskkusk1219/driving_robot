# 設計: ログの確認・ダウンロード機能

## バックエンド

### CSV ダウンロード API

`src/web/routers/sessions.py` に追加:

```
GET /api/v1/sessions/{session_id}/logs.csv
Response: text/csv (Content-Disposition: attachment)
         | 404（セッション不在）
```

- `repo.get_by_id()` でセッション存在を確認し、無ければ 404。
- `repo.list_logs(session_id, limit=1_000_000)` で全ログを取得（既定 1000 では
  長時間セッションが欠落するため明示的に大きな上限を渡す）。
- `csv` 標準ライブラリで `io.StringIO` に書き出し、`fastapi.responses.Response`
  で返す。列順は `ArchiveManager._export_to_csv` と一致させる:
  `timestamp, ref_speed_kmh, actual_speed_kmh, accel_opening, brake_opening,
   accel_pos, brake_pos, accel_current, brake_current`
- ファイル名は `session_{id}_{started_at:%Y%m%d_%H%M%S}.csv`。

ルート競合は無し（`/{session_id}`・`/{session_id}/logs` とパスが異なる）。

## フロントエンド (`src/web/static/js/screens/logs.js`)

詳細パネル（`isSelected`）を以下の構成に拡張する:

1. ヘッダ行: 「CSVダウンロード」ボタン（`<a download href=.../logs.csv>` を
   `Btn` 風スタイルで）。
2. サマリ: 最高車速 / 平均逸脱 / 記録時間 / サンプル数。
3. 速度グラフ（既存 `LogChart`）。
4. 明細テーブル（スクロール可・`maxHeight`）: 時刻・基準車速・実車速・
   アクセル開度・ブレーキ開度・アクセル電流・ブレーキ電流。

既存の `variant: 'ghost'` 指定は `Btn` が解釈しない（boolean `ghost` を取る）ため、
新規ボタンは `ghost: true` を使用し、既存「更新」ボタンも合わせて修正する。

## テスト

`tests/integration/test_web_api.py` に CSV エンドポイントのテストを追加
（in-memory リポジトリは空のため、ヘッダのみ・200・`text/csv`・添付ヘッダを検証）。
