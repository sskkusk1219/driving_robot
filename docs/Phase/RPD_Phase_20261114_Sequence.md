# シーケンスモード（作成・編集・一覧・保存）実装計画

作成日: 2026-11-14 / 対象: `driving_robot`
根拠: `docs/product-requirements.md`「2. シーケンス」・`docs/ideas/initial-requirements.md` Phase 12

---

## 0. 目的

走行モードとタイムスケジュールを任意の順序で並べた「シーケンス」を、WebUI で作成・編集・一覧・削除・保存できるようにする。

受け入れ条件（PRD）:
- 走行モードとタイムスケジュールを任意の順序で組み合わせられる
- シーケンス名を付けて保存・一覧管理できる

## 1. ユーザー決定（2026-11-14）

| # | 項目 | 決定 |
|---|---|---|
| A | 範囲 | 作成・編集・一覧・削除・保存まで（画面＋API＋DB）。**実行（自動運転画面から走らせる）は次の段** |
| B | 段の中身 | 種類（`mode`＝走行モード／`schedule`＝タイムスケジュール）＋参照先 ID＋繰り返し回数（1 以上の整数） |
| C | 段と段のあいだ | 今回は持たせない（待ち時間・停車確認は実行の段で決める） |
| D | 全体の繰り返し | シーケンス全体の繰り返し回数を 1 つ持つ（1 以上の整数） |
| E | 参照先が削除されたとき | 一覧・編集で「見つからない」と表示。見つからない段があるままでは保存できない（API も 422） |
| F | プレビュー | 段を順につないだ全体のタイムライン（段ごとの帯・長さ）と合計時間 |
| G | 画面の形 | 一覧 → 新規作成／編集。スケジュール画面と同じ並び（← 一覧に戻る・右下にキャンセル→保存・本体だけスクロール） |

## 2. データ

### モデル `src/models/sequence.py`

```python
@dataclass(frozen=True)
class SequenceStep:
    kind: Literal["mode", "schedule"]
    ref_id: str          # driving_modes.id または time_schedules.id
    repeat: int = 1      # この段の繰り返し回数（>=1）

@dataclass(frozen=True)
class Sequence:
    id: str
    name: str
    description: str
    steps: list[SequenceStep]
    repeat: int          # 全体の繰り返し回数（>=1）
    created_at: datetime
```

### テーブル `sequences`（`scripts/setup_db.py` に追加）

```sql
CREATE TABLE IF NOT EXISTS sequences (
    id          UUID PRIMARY KEY,
    name        TEXT NOT NULL UNIQUE,
    description TEXT NOT NULL DEFAULT '',
    steps       JSONB NOT NULL,     -- [{kind, ref_id, repeat}]
    repeat      INTEGER NOT NULL DEFAULT 1,
    created_at  TIMESTAMPTZ NOT NULL
)
```

参照先は外部キーにしない（モード・スケジュールを削除できなくなるため。決定 E のとおり表示側で「見つからない」にする）。

## 3. API `/api/v1/sequences`（`schedules.py` と同じ CRUD パターン）

| メソッド | パス | 内容 |
|---|---|---|
| GET | `/` | 一覧。各段の参照先を解決し `name`・`duration_s`・`missing` を付ける。シーケンスの `total_duration`（段の長さ×段の繰り返し の和 × 全体の繰り返し。見つからない段は 0）と `has_missing` |
| GET | `/{id}` | 詳細（一覧と同じ解決済みの段） |
| POST | `/` | 作成。名前空・段 0 個・repeat<1・kind 不正・参照先が見つからない → 422。名前重複 → 409 |
| PUT | `/{id}` | 更新（同じ検証）。無ければ 404 |
| DELETE | `/{id}` | 削除。204／404 |

段の長さ: 走行モード＝`total_duration`、スケジュール＝`total_duration`。

## 4. 実装の段

| 段 | 内容 | ファイル |
|---|---|---|
| 1 | モデル・リポジトリ（PostgreSQL・インメモリ）・テーブル | `src/models/sequence.py`, `src/infra/sequence_repository.py`, `src/app/stubs.py`, `scripts/setup_db.py` |
| 2 | スキーマ・ルーター・依存注入・app への登録 | `src/web/schemas.py`, `src/web/routers/sequences.py`, `src/web/deps.py`, `src/web/app.py` |
| 3 | API テスト | `tests/unit/` または `tests/integration/test_web_api.py` の流儀に合わせる |
| 4 | 画面（一覧・新規作成・編集・プレビュー） | `src/web/static/js/screens/schedule-sequence.js` の `SequenceScreen` |
| 5 | スタブ起動で画面の通し確認（作成→一覧→編集→参照先削除で「見つからない」→削除） | Playwright |

## 5. 対象外（次の段）

- シーケンスの実行（自動運転画面での選択・制御ループでの段の切り替え・段の間の扱い）
- 段のあいだの待ち時間・停車確認

## 6. 結果

2026-11-14 段1〜5 完了（未コミット）。

| 確認 | 結果 |
|---|---|
| `tests/unit/test_web_sequences.py`（インメモリ、PostgreSQL 不要） | 14 passed（CRUD・合計時間・422（段0・repeat<1・空名・kind 不正・参照先なし・種類違い・非UUID）・409・削除後の has_missing） |
| `ruff check`（変更した Python） | All checks passed |
| スタブ起動のブラウザ確認 | 作成（スケジュール→走行モード×2→スケジュール）→一覧 20:40 → 参照スケジュール削除で一覧に「見つからない段あり」・合計 20:20・編集で赤表示・保存は toast で止まる → 削除（確認ダイアログ）で一覧が空 |

メモ:
- 合計時間 = Σ(段の長さ × 段の繰り返し) × 全体の繰り返し。見つからない段は 0。
- PUT は全置き換え（スケジュールの PUT のような部分更新ではない）。
- 画面の参照先の選択肢は `GET /api/v1/modes/`（システムモードを含まない）と `GET /api/v1/schedules/`。
- 本番 DB に `scripts/setup_db.py` を実行し `sequences` テーブルを作成済み（2026-11-15、ユーザー許可）。
