# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

### 必須ルール
- **全てのタスクを`[x]`にすること**
- 「時間の都合により別タスクとして実施予定」は禁止
- 「実装が複雑すぎるため後回し」は禁止
- 未完了タスク（`[ ]`）を残したまま作業を終了しない

---

## フェーズ1: グローバルなナビロック基盤

- [x] app.js に `navLock` state を追加
  - [x] `const [navLock, setNavLock] = useState(false)` を追加
  - [x] `ctx` に `navLock, setNavLock` を公開
- [x] app.js の `onNav` をガード
  - [x] ロック中かつ別ページなら中断＋トースト「実行中は他のページに移動できません」
- [x] `Frame` に `locked={navLock}` を伝播

## フェーズ2: サイドバーの視覚表現

- [x] sketch.js `Frame` が `locked` を受け取り `Sidebar` に渡す
- [x] sketch.js `Sidebar` が `locked` で非アクティブ項目を `opacity:0.4 / cursor:not-allowed` 表示

## フェーズ3: DragSlider の無効化対応

- [x] calib-components.js `DragSlider` に `disabled` prop を追加
  - [x] `onPointerDown/Move` を `disabled` で早期 return
  - [x] `opacity:0.4 / cursor:not-allowed` のスタイル適用

## フェーズ4: キャリブレーション画面の開始ゲート

- [x] `AxisCal` に `enabled` prop を追加し操作系へ `disabled` を伝播
  - [x] JogKey（±10/±100）
  - [x] DragSlider
  - [x] ZERO/FULL 確定 Btn
  - [x] 原点へ戻す Btn
- [x] `started` / `confirmStart` state を追加
- [x] 下部ボタンを `started` で「▶ 開始」/「キャリブレーション保存」出し分け
- [x] `ConfirmStopPopup`（「開始しますか？」）を追加し、はい→`started=true`
- [x] 保存成功時に `started=false`（ロック解除）
- [x] `useEffect` で `setNavLock(started)`＋cleanup

## フェーズ5: 手動運転画面の開始ゲート

- [x] `AxisJog` に `enabled` prop を追加し操作系へ `disabled` を伝播
- [x] `enabled = robotState === 'MANUAL'` を定義
- [x] `confirmStart` state を追加
- [x] 「走行開始」押下を `setConfirmStart(true)` に変更
- [x] `ConfirmStopPopup`（「開始しますか？」）を追加し、はい→`handleStart()`
- [x] 既存の停止確認ポップアップを維持
- [x] `useEffect` で `setNavLock(enabled)`＋cleanup

## フェーズ6: 学習運転・自動運転のナビロック

- [x] `DriveMonitorScreen` で `setNavLock` を context から取得
- [x] `useEffect` で `RUNNING`/`PAUSED` 時に `setNavLock(true)`＋cleanup

## フェーズ7: 品質チェックと動作確認

- [x] Babel によるトランスパイルエラーがないことを確認（ブラウザコンソール 0 errors）
- [x] スタブモードでサーバ起動し Playwright で動作確認
  - [x] キャリブレーション: 初期グレー＋「開始」のみ活性
  - [x] キャリブレーション: 開始確認「開始しますか？」→ はい → 操作有効化＋ボタン「保存」化
  - [x] キャリブレーション: 実行中サイドバーがグレー化、他ページ押下で遷移ブロック
  - [x] 手動運転: 初期グレー＋「走行開始」活性 → 開始確認ポップアップ表示
  - [x] 学習運転・自動運転: 画面ロードでコンソールエラー無し
- [x] 検証用の一時ファイル（スクリーンショット）削除・サーバ停止

## フェーズ8: ドキュメント更新

- [x] 実装後の振り返り（このファイルの下部に記録）

---

## 実装後の振り返り

### 実装完了日
2026-06-20

### 計画と実績の差分

**計画と異なった点**:
- キャリブレーションのセッション終了方法（ナビロック解除手段）は計画段階で未確定だったため、`AskUserQuestion` で確認し「保存成功でロック解除」を採用した。
- 開始確認ポップアップは新規コンポーネントを作らず、既存の `ConfirmStopPopup`（メッセージ可変・学習/自動運転の開始確認で既に流用実績あり）を再利用した。

**新たに必要になったタスク**:
- `DragSlider` への `disabled` 対応（`JogKey`/`Btn` は既存対応だったが `DragSlider` のみ未対応だった）。

**技術的理由でスキップしたタスク**:
- なし。

### 学んだこと

**技術的な学び**:
- 本フロントは Babel standalone によるブラウザ内トランスパイルでビルド工程が無いため、構文確認は「ブラウザでロードしコンソールエラー 0」で代替できる。
- 実行状態を `AppContext` の単一 `navLock` に集約し、各画面が `useEffect` の cleanup で必ず解除する設計にすると、画面が同時に1つしかマウントされない（`renderScreen` の switch）ため競合せず安全。
- localStorage は生文字列で保存（`lsGet`/`lsSet` は JSON 変換しない）。検証時に `JSON.stringify` を入れると不一致になる点に注意。
- 手動開始の HTTP 409 は未初期化（STANDBY）での開始を拒否する既存バックエンド検証であり、フロントの開始ゲートとは独立。robotState が遷移しないため操作系は disabled のままで安全に機能する。

**プロセス上の改善点**:
- 計画（plan）で「再利用できる既存パターン（ConfirmStopPopup・Btn/JogKey の disabled・DriveMonitorScreen 共有）」を先に洗い出したことで、新規コードを最小化できた。

### 次回への改善提案
- キャリブレーションに将来バックエンドの開始/終了エンドポイントが追加された場合は、`started` をサーバ状態へ置き換える（クライアントローカル状態からの移行）。
- 学習/自動運転で arm 済み・未確定（confirmStart 表示中）もロック対象としたい要望が出たら、navLock 条件に `confirmStart` を加える拡張で対応可能。
