# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

### 必須ルール
- **全てのタスクを`[x]`にすること**
- 「時間の都合により別タスクとして実施予定」は禁止
- 未完了タスク（`[ ]`）を残したまま作業を終了しない

---

## フェーズ1: ユニットテスト確認

- [x] `tests/unit/test_robot_controller.py` を実行して全パスを確認する
  - [x] `.venv/bin/python -m pytest tests/unit/test_robot_controller.py -v`
  - [x] 全テストが PASSED であることを確認

## フェーズ2: ハードウェアテストスクリプト作成

- [x] `tests/hardware/test_emergency_stop_home_return.py` を作成する
  - [x] 定数定義（ポート・slave_id・ボーレート・GPIOピン番号・デバウンス）
  - [x] `home_return_both_axes(accel, brake)` 非同期関数（両軸 asyncio.gather）
  - [x] lgpio コールバック → `asyncio.run_coroutine_threadsafe()` でスケジュール
  - [x] `main()` 非同期関数
    - [x] Modbus 接続（両軸）
    - [x] 起動時: `reset_alarm` → `servo_on` → `home_return_both_axes`
    - [x] GPIO17 監視開始（RISING エッジ、デバウンス 50ms）
    - [x] ループ待機
    - [x] Ctrl+C で GPIO クリーンアップ + Modbus 切断

## フェーズ3: ハードウェア実機確認（手動）

- [x] スクリプトを実行して起動時の原点復帰が動作することを確認
  - [x] `.venv/bin/python tests/hardware/test_emergency_stop_home_return.py`
  - [x] 「起動時原点復帰完了」が表示される
  - [x] アクチュエータが物理的に原点位置へ移動することを目視確認
- [x] 非常停止スイッチを押して原点復帰が起動することを確認
  - [x] 「非常停止検知 → 原点復帰開始」が表示される
  - [x] 「両軸 原点復帰完了」が表示される
  - [x] アクチュエータが物理的に原点位置へ移動することを目視確認
- [x] Ctrl+C で正常終了することを確認
  - [x] 「GPIOクリーンアップ完了」が表示される

## フェーズ4: PRD チェックボックス更新

- [x] `docs/product-requirements.md` の受け入れ条件を更新する
  - [x] `- [ ] 非常停止スイッチ（室内・操作エリアの2箇所）のいずれかを押すと全アクチュエータが即座に原点復帰する`
    → `- [x] 非常停止スイッチ（室内・操作エリアの2箇所）のいずれかを押すと全アクチュエータが即座に原点復帰する`

## フェーズ5: 振り返り

- [x] 実装後の振り返りを記録する

---

## 実装後の振り返り

### 実装完了日
2026-05-07

### 計画と実績の差分

**計画と異なった点**:
- ブレーキ軸の slave_id が設計書（2）と実機（1）で異なっていた。各軸が独立した RS-485 バスを持つため両軸とも slave_id=1 に統一した
- `actuator_driver.home_return()` に立ち上がりエッジ生成（False→True）が抜けており、2回目以降の呼び出しで物理移動が発生しないバグがあった

**新たに必要になったタスク**:
- lgpio が .venv の sys.path に含まれないため `/usr/lib/python3/dist-packages` を明示的に追加
- `factory.py` の slave_id 修正（brake: 2→1）
- `docs/architecture.md` の SLAVE_ID 表記修正
- `tests/unit/test_factory.py` のアサーション修正
- `actuator_driver.home_return()` の立ち上がりエッジ修正（False→True パターン）
- `tests/unit/infra/test_actuator_driver.py` の対応テスト修正
- Ctrl+C 後の CancelledError によるトレースバック修正

### 学んだこと

**技術的な学び**:
- P-CON-CB の HOME コイルはエッジ入力（立ち上がり）でトリガーされる。True のみ送信では2回目以降に移動が発生しない
- .venv 環境は `--system-site-packages` なしでは `/usr/lib/python3/dist-packages` を含まない
- asyncio.sleep() の CancelledError は KeyboardInterrupt と別に捕捉が必要

### 次回への改善提案
- コントローラーの slave_id は settings.toml で設定可能にすると実機差異に対応しやすい
- `home_return()` のエッジ生成パターンを `reset_alarm()` と同様にドキュメントコメントで明示する
