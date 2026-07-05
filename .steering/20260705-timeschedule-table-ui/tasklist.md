# タスクリスト: タイムスケジュールUIの表形式エディタ刷新

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

### 必須ルール
- **全てのタスクを`[x]`にすること**
- 「時間の都合により別タスクとして実施予定」は禁止
- 「実装が複雑すぎるため後回し」は禁止
- 未完了タスク（`[ ]`）を残したまま作業を終了しない

### タスクスキップが許可される唯一のケース
技術的理由（実装方針変更・アーキテクチャ変更・依存関係変更）のみ。
スキップ時は `- [x] ~~タスク名~~（理由）` と明記する。

---

> **前提**: 実装開始時に `/frontend-design` スキルを読み込むこと（design.md 冒頭参照）。
> 変更対象は `src/web/static/js/screens/schedule-sequence.js` の1ファイルのみ。
> バックエンドは変更しない。

## フェーズ1: 不具合修正

- [x] T1: ScheduleCreateForm の名前・説明入力を修正
  - [x] 共有 `Input` の `onChange` は値文字列を渡す仕様のため
        `e => setName(e.target.value)` → `v => setName(v)` に修正（名前・説明の2箇所）
- [x] T2: 一覧（ScheduleScreen）の Row 崩れを修正
  - [x] cells をタプル `[content, width, mono?]` 形式に修正
        （ヘッダ行・データ行とも。ラジオ・RowActions が描画されること）
  - [x] `RowActions` に `onEdit` を追加（フェーズ3の編集導線。フェーズ3実装まで
        は一覧→詳細取得→編集画面遷移のハンドラだけ用意）

## フェーズ2: 変換純関数

- [x] T3: チャンネル名定数を定義
  - [x] `{0:'スタート', 1:'P', 2:'N', 3:'D', 4-15:'オプション1-12'}`
- [x] T4: `rowsToApi(rows, visibleChannels)` を実装
  - [x] pedal_points = 全行の `{time_s, accel_opening, brake_opening}`
  - [x] button_events = 各 ch の連続 ON 区間 →
        `{time_s: ON開始時刻, channel, press_duration_s: OFF行時刻 − ON開始時刻}`
  - [x] 検証例: rows[1.0s ON, 2.0s ON, 3.0s OFF] →
        `[{time_s:1.0, channel:0, press_duration_s:2.0}]`
- [x] T5: `apiToRows(detail)` を実装
  - [x] pedal_points → 行。button_event の開始/終了時刻が行に無ければ挿入
        （Acc/Brk は前後の点から線形補間）
  - [x] `[開始, 終了)` の行の ch セルを ON、visibleChannels = [0-3] ∪ 登場 ch
  - [x] ラウンドトリップ確認: apiToRows → rowsToApi で button_events が一致すること

## フェーズ3: 表エディタ（ScheduleEditForm）

- [x] T6: ScheduleCreateForm を ScheduleEditForm に置換（作成・編集共通）
  - [x] props: `{ initial: detail | null, onCancel, onSaved }`（initial あり=編集/PUT、なし=新規/POST）
  - [x] JSON テキストエリアを廃止
- [x] T7: タイムライン表の描画
  - [x] 列: 時刻[s] / Acc[%] / Brk[%] / ボタン列（visibleChannels）/ 操作
  - [x] 数値セルは `type:'number'` 入力、ボタンセルは素の div + onClick でトグル
        （共有 Box は onClick を握り潰すため使わない）
  - [x] ON セルはアクセント色 `●ON`、OFF は薄色 `—`
- [x] T8: 行操作
  - [x] ＋: 直下に行挿入（時刻=前行+1.0s、ボタン状態は上の行を引き継ぐ）
  - [x] ✕: 行削除（最低1行は残す）
- [x] T9: ボタン列の追加・削除
  - [x] 「＋列追加」ドロップダウン（ch4-15 のうち未表示のもの）
  - [x] 列削除は全行 OFF のときのみ可
- [x] T10: 総時間表示・押下プレビュー
  - [x] 総時間 = 最終行の時刻を自動表示
  - [x] 各 ch の ON 区間を「ch名: 開始s〜終了s（押下X.Xs）」で一覧表示
        （水平バープレビューとして実装。未終端 ON は赤バーで警告表示）
- [x] T11: バリデーション
  - [x] 名前必須 / 時刻の狭義単調増加 / 開度0-100 / 最終行ボタン全OFF / 行1つ以上
  - [x] エラーは showToast + 該当セル border 強調（style 直書き）
  - [x] Acc・Brk 同一行両方>0 は警告のみ（enforce_pedal_exclusion の旨）
- [x] T12: 保存処理
  - [x] 新規: `POST /api/v1/schedules/`、編集: `PUT /api/v1/schedules/{id}`
        （name/description/loop/pedal_points/button_events を送信）
  - [x] 成功 toast → 一覧へ戻り refresh
- [x] T13: 一覧の編集導線を接続
  - [x] RowActions の「編集」→ `GET /api/v1/schedules/{id}` → ScheduleEditForm(initial)

## フェーズ4: 検証

- [x] T14: 回帰確認（バックエンド無変更の確認）
  - [x] `pytest`（916 passed、hardware 除く。ベースライン762から増加しているが
        本タスクとは無関係の既存コミットによる増加であり、backend は無変更）
  - [x] `ruff check`（All checks passed!）
- [x] T15: Playwright MCP による実UI確認
  - [x] サーバ起動確認（`/static/` の `no-cache, must-revalidate` ヘッダを確認。
        静的JSのみの変更のためサーバ再起動は不要と判断）
  - [x] 名前・説明が入力できる（不具合1の修正確認）— 「動作確認テスト」を入力し反映確認
  - [x] 一覧の表示・ラジオ選択・削除が機能する（不具合2の修正確認）— 名前/総時間/件数/
        ループ/ラジオ/編集/削除ボタンがすべて正しく描画されることを確認
  - [x] 表で「0s Acc0/Brk0 → 1.0s スタートON → 2.0s スタートON → 3.0s Acc10% スタートOFF」
        を作成し保存
  - [x] `GET /api/v1/schedules/{id}` の**生データ**で
        `button_events=[{time_s:1.0, channel:0, press_duration_s:2.0}]` と
        pedal_points（0/1/2/3s の4点、3s時点でaccel_opening=10.0）を確認（curl で生JSON確認）
  - [x] 編集で開き直し、表（ON セル位置含む）が復元されることを確認
        （4行・スタートON区間1.0〜3.0が完全復元）
  - [x] P/N/D とオプション列追加（例: ch4=オプション1）の ON/OFF が保存・復元されることを確認
        （0.0〜1.0s と 2.0〜3.0s の2区間を追加→PUT保存→生データで
        `button_events` に2件の ch4 イベントが正しく含まれることを確認、編集で再度開いて復元も確認）
  - [x] 追加確認: 最終行ボタンONのまま保存を試み、バリデーションで保存がブロックされ
        「最終行はすべてのボタンをOFFにしてください」のエラーが表示されることを確認
  - [x] 検証用に作成したテストスケジュールはAPI経由で削除しクリーンアップ済み
- [x] T16: 実装後の振り返り（このファイル下部に記録）

---

## 実装後の振り返り

### 実装完了日
2026-07-05

### 計画と実績の差分

**計画と異なった点**:
- design.md ではボタン押下プレビューを「ch名: 開始s〜終了s（押下X.Xs）」のテキスト一覧と
  想定していたが、実装ではチャンネルごとの水平バー（総時間に対する相対位置で ON 区間を
  ハイライトする簡易ピアノロール）+ テキスト併記に強化した。保存前の未終端 ON（バリデーション
  エラー対象）は赤バーで視覚的に区別できるようにし、テキストでも「未終了」と表示する。
  表だけでは分かりにくい押下タイミングの重なりが一目で分かるため、当初テキストのみ案より
  実用性が高いと判断した。
- 元の `ScheduleCreateForm`（React.createElement 記法）を JSX 記法で全面的に書き直した。
  同ディレクトリの他画面（calibration.js / manual.js 等）は JSX 中心のため、テーブル生成の
  可読性を優先して JSX に統一した（1ファイルのみの変更という設計方針は維持）。

**新たに必要になったタスク**:
- なし（design.md / tasklist.md の記載範囲内で完結）

### 学んだこと

**技術的な学び**:
- `rowsToApi` / `apiToRows` の変換はローカルで Node + `@babel/core`（システムにインストール
  済みの `/usr/share/nodejs/@babel`）を使い、ブラウザを介さずに純関数の入出力を検証できた。
  JS 単体テストのフレームワークが無いプロジェクトでも、babel-standalone と同じ変換結果を
  Node の vm モジュールで再現し、ラウンドトリップ（apiToRows → rowsToApi）や境界値
  （未終端 ON・時刻逆転・範囲外開度・複数区間）を型どおりに検証できた。
- Row/Btn/Box などの共有コンポーネントの「onClick を渡しても握り潰される」制約
  （[[web-ui-component-gotchas]] 参照）は今回も踏襲。ボタンセルのトグルは素の `<div onClick>`
  で実装し、行操作（＋/✕）は `Btn` に onClick を直接渡す形で問題なく動作した。
- `/static/` の no-cache ミドルウェアが有効な環境では、静的 JS のみの変更はサーバ再起動不要
  で反映される（[[playwright-mcp-setup]] 記載の「初回ハードリロード」は当時のキャッシュ問題
  対策で、今回はブラウザキャッシュも問題にならなかった）。

**プロセス上の改善点**:
- Playwright MCP でのクリック操作は、直前のクリックで state が更新され再レンダリングされた
  直後に古い ref を使うと不正確なセレクタにフォールバックすることがあった（今回は結果的に
  正しいセルを捉えていたが、確実性のためには各クリック後に `browser_snapshot` を取り直して
  最新の ref を使う方が安全）。

### 次回への改善提案
- ボタン列が多くなる場合（オプション1-12を複数追加した場合）、テーブルの横スクロールが
  発生する。`overflowX: 'auto'` で対応済みだが、実運用で列数が多い場合は列幅の圧縮や
  チャンネル名の短縮表示も検討余地あり（今回のスコープでは不要と判断）。
