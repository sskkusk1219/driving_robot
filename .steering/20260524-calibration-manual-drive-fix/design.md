# 設計: キャリブレーション・手動運転ページの修正

## 変更方針

### calibration.js

#### 1. ステップバー削除
- `CALIB_STEPS` 定数を削除
- `activeStep()` 関数を削除
- ステップバー描画部分（`{/* Step progress bar */}` ブロック）を削除
- `canSave` の算出を直接的なnullチェックに変更:
  ```js
  const canSave = brk.zero !== null && brk.full !== null && acc.zero !== null && acc.full !== null;
  ```
- Row コンポーネントのステップ依存ハイライト（`style={{ background: step <= 1 ? ... }}`）を削除

#### 2. AxisCal の余白改善
- JogKey + 現在位置表示の div に `flex: 1` を付与し、ボックス内の余白を埋める
- Box の `gap` は 10px のまま維持（コンテンツが自然に広がる）

### manual.js

#### 1. AxisJog の余白改善
- AxisCalと同様、JogKey + 現在位置表示の div に `flex: 1` を付与
- Box の `gap` は 10px のまま維持

#### 2. デザイン統一
- キャリブレーションと手動運転の AxisCal/AxisJog に同じ構造・パディングを使用
- キーボードショートカット `lineHeight` を `1.6` に統一（現在 `1.9` の箇所があれば修正）
- キーボードショートカット box のパディングを統一

## レイアウト変更後のイメージ

### 両画面共通
```
+-------------------+-------------------+
|   ブレーキ Box     |   アクセル Box     |
| [PosRuler]        | [PosRuler]        |
|                   |                   |
| [JogKeys] flex:1  | [JogKeys] flex:1  |  ← ここが縦に広がる
| (中央に配置)      |                   |
|                   |                   |
| [ZERO/FULL ボタン]| [opening bar]     |
| [状態表示]        | [状態表示]        |
+-------------------+-------------------+
| keyboard hints    | status + save/stop|
+-------------------+-------------------+
```

### calibration.js 変更点（ステップバーなし）
- 画面上部にステップバーがなくなり、2x2グリッドが全高を使える

## 変更ファイル
- `src/web/static/js/screens/calibration.js`
- `src/web/static/js/screens/manual.js`
