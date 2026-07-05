# 設計書: タイムスケジュールUIの表形式エディタ刷新

> **🎨 実装開始時に必ず `/frontend-design` スキルを読み込むこと。**
> 本作業はフロントエンド UI の刷新であり、既存ダークテーマ・共有コンポーネント
> （`sketch.js` の Box/Btn/H2/Note/Pill/Row/RowActions/Input）との調和を保つこと。

## アーキテクチャ概要

変更は **`src/web/static/js/screens/schedule-sequence.js` の1ファイルのみ**。
バックエンド（API・モデル・ScheduleLoop）は完成済みで一切変更しない。

```
表エディタ（rows: 1行=1時刻点、ボタンはON/OFF状態）
   ⇅ フロントエンド純関数で相互変換
APIモデル（pedal_points: 開度点列 / button_events: 押下開始時刻+押下秒）
   → POST /api/v1/schedules/（新規） / PUT /api/v1/schedules/{id}（更新）
```

## 共有コンポーネントの正しい使い方（不具合の真因と修正）

### Input（sketch.js:292）— 名前が入力できない不具合
- `Input` の `onChange` は **値文字列** を渡す（`e => onChange(e.target.value)` 処理済み）
- ❌ `onChange: e => setName(e.target.value)` → ✅ `onChange: v => setName(v)`
- 名前・説明の両フィールドを修正

### Row（sketch.js:320）— 一覧表示崩れの不具合
- `Row` の cells は **タプル `[content, width, mono?]`** の配列
  （`c[0]` を描画、`c[1]` を gridTemplateColumns に使用）
- ❌ `cells: ['', '名前', ...]`（素の文字列/React要素）
- ✅ `cells: [[radioEl, '40px'], ['名前', '2fr'], ...]` のタプル形式、
  もしくは profiles.js の一覧のように素の div grid で組む

## コンポーネント設計

### 1. ScheduleScreen（一覧・走行制御）

**責務**:
- スケジュール一覧表示・ラジオ選択・削除・編集導線・走行開始/停止（既存機能の修正）

**実装の要点**:
- Row タプル形式へ修正（上記）
- `RowActions`（sketch.js:594、`{onSelect, isActive, onEdit, onCopy, onDelete}`）に
  `onEdit` を追加 → `GET /api/v1/schedules/{id}` で詳細取得 → 表エディタへ
- mode: `'list' | 'create' | 'edit'`（edit 時は取得した詳細を ScheduleEditForm へ渡す）

### 2. ScheduleEditForm（作成・編集共通の表エディタ、現 ScheduleCreateForm を置換）

**責務**:
- 基本情報（名前・説明・ループ再生）+ 統合タイムライン表の編集
- 保存時に rows → APIモデル変換、POST（新規）/ PUT（編集）

**画面レイアウト**:

```
名前 [________]  説明 [__________________]  □ ループ再生

時刻[s] | Acc[%] | Brk[%] | スタート | P | N | D |（＋列追加 ▾）| 操作
--------|--------|--------|---------|---|---|---|--------------|------
 0.0    |  0     |  0     |   —     | — | — | — |              | ＋ ✕
 1.0    |  0     |  0     |  ●ON    | — | — | — |              | ＋ ✕
 3.0    |  10    |  0     |   —     | — | — | — |              | ＋ ✕

総時間: 3.0 s（最終行の時刻を自動表示）
ボタン押下プレビュー: スタート: 1.0s〜3.0s（押下2.0s）
[保存] [キャンセル]
```

**実装の要点**:
- state: `rows: [{time_s, accel, brake, buttons: {ch: bool}}]` + `visibleChannels: number[]`
- 時刻・Acc・Brk は `type:'number'` 入力。ボタンセルはクリックでトグル
  （`●ON` はアクセント色、OFF は `—` を薄色で表示）
- 既定列: スタート(ch0)/P(ch1)/N(ch2)/D(ch3)。「＋列追加」ドロップダウンで
  オプション1-12（ch4-15）から選択して列追加（追加済み・使用中の ch は除外）。
  列削除はその列が全行 OFF のときのみ可
- 行操作: ＋=直下に行挿入（時刻は前行+1.0s 既定、ボタン状態は上の行を引き継ぐ）、✕=行削除
- チャンネル名定数: `{0:'スタート', 1:'P', 2:'N', 3:'D', 4:'オプション1', ... 15:'オプション12'}`
  （`docs/functional-design.md` の ch 割当に一致させる）
- クリック可能セルは素の `<div>` + `onClick`（共有 Box は onClick を握り潰すため使わない）

### 3. 変換純関数（schedule-sequence.js 内、window 非公開でよい）

**rowsToApi(rows, visibleChannels) → {pedal_points, button_events}**:
- `pedal_points` = 全行の `{time_s, accel_opening, brake_opening}`
- `button_events` = 各 ch の**連続 ON 区間**ごとに
  `{time_s: ON開始行の時刻, channel, press_duration_s: 次にOFFになる行の時刻 − ON開始時刻}`
- 例（ユーザー要求例）: 1.0s ON, 2.0s ON, 3.0s OFF →
  `{time_s: 1.0, channel: 0, press_duration_s: 2.0}`（イベントは1件。2.0s の行は
  ペダル点としてのみ意味を持つ）

**apiToRows(detail) → {rows, visibleChannels}**:
- 行 = pedal_points をベースに生成
- 各 button_event の開始時刻 `time_s`・終了時刻 `time_s + press_duration_s` が行に無ければ
  行を挿入（Acc/Brk は前後の pedal_points から**線形補間** = ScheduleLoop の補間と同一のため
  保存し直しても走行挙動は不変）
- `[開始, 終了)` に時刻が含まれる行の該当 ch セルを ON にする
- visibleChannels = 既定 [0,1,2,3] ∪ button_events に登場する ch

### 4. バリデーション（保存前チェック）

- 名前必須（trim 後）
- 時刻は狭義単調増加（同時刻・逆順はエラー、該当行を強調）
- Acc/Brk は 0-100
- **最終行はすべてのボタンが OFF**（ON のまま終わると押下終了時刻が定義できない）
- 行が1つ以上あること
- 警告（保存は可能）: 同一行で Acc>0 かつ Brk>0 → バックエンドの
  `enforce_pedal_exclusion` により同時踏みは排除される旨を showToast で警告

エラーは `window.showToast(…, 'error')` + 該当セルの border 強調（style 直書き。
共有 Box に borderColor を渡しても無効なため）。

## データフロー

### 新規作成
```
1. 一覧「＋ 新規作成」→ ScheduleEditForm（初期 rows: [{time_s:0, accel:0, brake:0, buttons:{}}]）
2. 表を編集 → プレビューで押下区間を確認
3. 保存 → バリデーション → rowsToApi → POST /api/v1/schedules/
4. 成功 → toast → 一覧へ戻り refresh / 409（名称重複）は apiFetch 側の共通エラー表示
```

### 既存編集
```
1. 一覧の RowActions「編集」→ GET /api/v1/schedules/{id}
2. apiToRows で表へ展開 → 編集
3. 保存 → rowsToApi → PUT /api/v1/schedules/{id}（name/description/loop も送信）
```

## エラーハンドリング戦略

- API エラー（409/404/422）は既存 `apiFetch` の共通ハンドリングに従う（r が falsy なら中断）
- フロントバリデーションは保存ボタン押下時に一括チェックし、最初のエラーを toast 表示

## テスト戦略

- バックエンド無変更のため **Python テストの追加・変更なし**。回帰確認のみ
  （`pytest` 762 passed 基準 / `ruff check`）
- フロントは Playwright MCP による実ブラウザ確認（tasklist の検証手順参照）

## 依存ライブラリ

追加なし（既存の React + Babel-standalone 構成のまま）

## ディレクトリ構造

```
src/web/static/js/screens/schedule-sequence.js   # 全面書き換え（唯一の変更ファイル）
```

## 実装の順序

1. 不具合修正（Input onChange / 一覧 Row タプル + onEdit 導線）
2. 変換純関数（rowsToApi / apiToRows）
3. ScheduleEditForm（表エディタ・列追加・行操作・バリデーション・プレビュー）
4. Playwright MCP で E2E 確認 → pytest / ruff 回帰確認

## パフォーマンス考慮事項

- 行数は高々数十行想定。全行 re-render で問題なし（最適化不要）

## 将来の拡張性

- rows 表現は波形グラフ等のグラフィカル可視化にそのまま流用可能
- 実行中の進行位置ハイライトは robotState + realtime WS を購読すれば追加できる（今回対象外）
