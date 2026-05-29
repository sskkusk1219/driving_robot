# タスクリスト: キャリブレーション・手動運転ページの修正

## フェーズ1: calibration.js の修正

- [x] CALIB_STEPS 定数を削除する
- [x] activeStep() 関数を削除する
- [x] CalibrationScreen から Step progress bar ブロックを削除する
- [x] canSave の算出を直接 null チェックに変更する
- [x] 記録済みRow のステップ依存ハイライトを削除する
- [x] AxisCal の JogKey 行に flex: 1 を付与する
- [x] AxisCal のキーボードショートカット lineHeight を 1.6 に変更する

## フェーズ2: manual.js の修正

- [x] AxisJog の JogKey 行に flex: 1 を付与する
- [x] AxisJog のキーボードショートカット lineHeight を 1.6 に変更する

## フェーズ3: 動作確認

- [x] calibration.js の JSX 構文エラーがないことを確認
- [x] manual.js の JSX 構文エラーがないことを確認
- [x] tasklist.md の実装後の振り返りを記載

---

## 実装後の振り返り

**実装完了日**: 2026-05-24

**計画と実績の差分**:
- 計画通りの変更を実施
- 追加で `handleSave` 内に残存していた `activeStep()` 参照バグを修正（計画外のバグ修正）
- `CalibrationScreen` の不要な destructure 変数（`INK`, `INK_SOFT`, `INK_MUTE`, `PAPER_2`）も整理

**変更内容まとめ**:
- `calibration.js`: ステップバー・`CALIB_STEPS`・`activeStep()` を完全削除、`canSave` を直接null チェックに変更、JogKey行に `flex: 1` 付与、lineHeight 1.9→1.6
- `manual.js`: JogKey行に `flex: 1` 付与、lineHeight 1.9→1.6（デザイン統一）

**学んだこと**:
- ステップバー削除後、`handleSave` 内に `activeStep()` 呼び出しが残っていたので注意が必要。削除対象の関数はすべての呼び出し箇所を必ず確認すること。

**次回への改善提案**:
- flex: 1 をJogKeyの親divに付与するだけでは、大きな画面では依然としてやや余白が生じる可能性がある。将来的にはPosRulerの高さをレスポンシブに対応させることも検討できる。
