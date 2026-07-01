# 設計書

## アーキテクチャ概要

フロントエンド（Babel standalone でブラウザ実行する React JSX、ビルド工程なし）のみの変更。グローバルな実行状態を `AppContext` に集約し、各画面が自身の「実行中」状態を `setNavLock` で報告、ルート（`App`）がナビゲーション遷移をガードする。

```
AppContext
  ├─ navLock / setNavLock        ← 追加（実行中フラグ）
  └─ robotState (既存, WS由来)

各画面 ──setNavLock(実行中)──▶ App.onNav ガード ──▶ Sidebar(locked) グレー表示＋遷移ブロック＋トースト
```

## コンポーネント設計

### 1. App（`src/web/static/js/app.js`）

**責務**:
- `navLock` state を保持し `AppContext` で公開
- `onNav` で遷移をガード（ロック中かつ別ページなら中断＋トースト）
- `Frame` に `locked` を伝播

**実装の要点**:
- `onNav: key => { if (navLock && key !== nav) { showToast('実行中は他のページに移動できません', 'error'); return; } setNav(key); }`

### 2. Frame / Sidebar（`src/web/static/js/sketch.js`）

**責務**:
- `locked` を受け取り、非アクティブなナビ項目を `opacity:0.4 / cursor:not-allowed` で表示

**実装の要点**:
- クリック自体は従来通り `onNav` を呼び、ブロック判定とトーストは App 側で実施（単一責務）

### 3. DragSlider（`src/web/static/js/calib-components.js`）

**責務**:
- `disabled` 時はポインタ操作を抑止し、グレースケール表示

**実装の要点**:
- `onPointerDown/Move` を `disabled` で早期 return、`opacity:0.4 / cursor:not-allowed`
- `JogKey` と `Btn` は既存の `disabled` 対応を流用

### 4. CalibrationScreen（`src/web/static/js/screens/calibration.js`）

**責務**:
- 開始ゲート（`started`）と開始確認（`confirmStart`）を管理
- `started` をナビロックへ反映

**実装の要点**:
- `AxisCal` に `enabled` prop を追加し全操作系へ `disabled={!enabled}` を伝播
- 下部ボタンは `started` で「▶ 開始」/「キャリブレーション保存」を出し分け
- 開始確認には既存 `ConfirmStopPopup`（メッセージ可変）を流用
- セッション終了はユーザー決定により「保存成功時に `started=false`」でロック解除
- `useEffect(() => { setNavLock(started); return () => setNavLock(false); }, [started])`

### 5. ManualScreen（`src/web/static/js/screens/manual.js`）

**責務**:
- `robotState === 'MANUAL'` を実行状態とし、開始ゲート・確認・ナビロックを実現

**実装の要点**:
- `AxisJog` に `enabled` prop を追加
- 「走行開始」押下を `setConfirmStart(true)` に変更し、確認「はい」で `handleStart()`
- 既存の停止確認ポップアップは維持
- `useEffect(() => { setNavLock(enabled); return () => setNavLock(false); }, [enabled])`

### 6. DriveMonitorScreen（学習/自動運転共有, `src/web/static/js/screens/auto-drive.js`）

**責務**:
- `RUNNING`/`PAUSED` 中のナビロックのみ追加（開始確認フローは既存）

**実装の要点**:
- `useEffect(() => { const driving = robotState==='RUNNING'||robotState==='PAUSED'; setNavLock(driving); return () => setNavLock(false); }, [robotState])`

## データフロー

### キャリブレーション開始
```
1. 初期: started=false → 操作系 disabled / ボタン「▶ 開始」
2. 開始押下 → confirmStart=true → ConfirmStopPopup「開始しますか？」
3. はい → started=true → 操作系 enabled / ボタン「保存」/ navLock=true
4. 保存成功 → started=false → navLock=false（ロック解除）
```

### ナビゲーションロック
```
1. 画面の useEffect が実行状態に応じ setNavLock(true/false)
2. サイドバー項目クリック → App.onNav
3. navLock=true かつ別ページ → 中断＋トースト
4. 画面アンマウント時 cleanup で setNavLock(false)
```

## エラーハンドリング戦略

- 開始ゲートはクライアント側の表示制御。実APIの前提条件違反（例: 未初期化での手動開始）は既存のバックエンド検証（HTTP 409）と `apiFetch`/トーストで処理され、robotState が遷移しないため操作系は disabled のまま安全。

## テスト戦略

### ユニットテスト
- JSのユニットテスト基盤は無し（既存方針どおり）。バックエンド未変更のため新規バックエンドテストなし。

### 統合テスト（手動・ブラウザ）
- スタブモードでサーバ起動し Playwright で各画面を確認
  - キャリブレーション: 初期グレー → 開始確認 → 有効化 → ナビロック → 遷移ブロック
  - 手動運転: 初期グレー → 開始確認
  - 学習/自動運転: 画面ロード時のコンソールエラー無し

## 依存ライブラリ

なし（追加なし）。

## ディレクトリ構造

```
src/web/static/js/
├── app.js                      # navLock state / onNav ガード / Frame へ伝播
├── sketch.js                   # Frame・Sidebar の locked 表示
├── calib-components.js         # DragSlider の disabled 対応
└── screens/
    ├── calibration.js          # 開始ゲート＋確認＋無効化＋ナビロック
    ├── manual.js               # 開始ゲート＋確認＋無効化＋ナビロック
    └── auto-drive.js           # DriveMonitorScreen にナビロック（学習/自動共有）
```

## 実装の順序

1. app.js に navLock 追加・onNav ガード・Frame へ伝播
2. sketch.js の Frame/Sidebar に locked 表示
3. calib-components.js の DragSlider に disabled 対応
4. calibration.js の開始ゲート実装
5. manual.js の開始ゲート実装
6. auto-drive.js（DriveMonitorScreen）にナビロック追加

## セキュリティ考慮事項

- 特になし（UI操作の制御のみ。権限・認証には影響しない）

## パフォーマンス考慮事項

- navLock は単一の boolean state。再描画コストは無視できる。

## 将来の拡張性

- キャリブレーションに将来バックエンドの開始/終了エンドポイントを設ける場合も、`started` を API 応答に置き換えるだけで本設計の構造を維持できる。
- 学習/自動運転で arm 済み・未確定状態もロックしたい場合は `confirmStart` を navLock 条件に加える拡張が可能。
