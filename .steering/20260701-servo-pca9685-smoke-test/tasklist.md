# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

未完了タスク（`[ ]`）を残したまま作業を終了しない。スキップは技術的理由のみ。

---

## フェーズ1: スクリプト作成

- [x] スクリプト骨子を作成（docstring に接続・操作方法・実行方法を記載）
  - [x] `test_calibration.py` に倣った import / sys.path 追加
  - [x] 接続設定・角度パラメータの定数定義
- [x] `_Pca9685` 最小ドライバを実装
  - [x] smbus2 で /dev/i2c-1 を開く（例外時に I2C 有効化を案内）
  - [x] 50Hz 初期化（MODE1/PRESCALE、SLEEP→RESTART）
  - [x] `set_angle(channel, angle)`（角度→us→12bit count）
  - [x] `all_off()`（ALL_LED_OFF で安全停止）
- [x] 対話 CLI（main）を実装
  - [x] 単一キー読み取り（termios raw）
  - [x] ジョグ（e/w ±5°, d/s ±1°）・角度プリセット
  - [x] 押下デモ（待機↔押下往復）
  - [x] q/Ctrl-C/例外時に finally で all_off する安全終了

## フェーズ2: 品質チェック

- [x] `.venv/bin/python -m py_compile tests/hardware/test_servo_pca9685.py` が通る
- [x] docstring の配線・操作方法が design.md と一致していることを確認
- [x] 追加依存が無い（smbus2 のみ）ことを確認（prescale(50Hz)=121=0x79 も検算）

## フェーズ3: 配線説明の提供

- [x] 6 つの永続ドキュメントから抽出した raspi↔PCA9685↔SG90 の配線をユーザーに提示
- [x] I2C 有効化 → i2cdetect 確認 → スクリプト実行の手順を提示

## フェーズ4: 振り返り

- [x] 実装後の振り返り（このファイル下部に記録）

---

## 実装後の振り返り

### 実装完了日
2026-07-01

### 計画と実績の差分

**計画と異なった点**:
- adafruit-circuitpython-pca9685 は未インストールだったため、`docs/architecture.md` が代替として許容する `smbus2`（既存・0.4.3）で PCA9685 レジスタを直接制御した。追加依存ゼロで実装。
- 実機の I2C1 が未有効（/dev/i2c-1 なし。i2c-13/14 は HDMI DDC のみ）だったため、スクリプト実行前提として I2C 有効化手順を docstring と design.md に明記した。

**新たに必要になったタスク**:
- I2C 有効化・i2cdetect 確認の案内（実機セットアップ前提のため追記）。

### 学んだこと

**技術的な学び**:
- PCA9685 は SLEEP ビットを立てないと PRESCALE を書き込めない。50Hz の prescale=121(0x79)。角度→count は count = pulse_us / (1e6/freq) * 4096。
- SG90 のパルス幅は 500-2500us を採用。個体差があるため定数で調整可能にした。

### 次回への改善提案
- 実機で角度基準がずれる場合は `_MIN_PULSE_US`/`_MAX_PULSE_US` を実測調整する。
- 本スクリプトの `_Pca9685` と角度変換は、将来 `src/infra/ButtonServoDriver`（Protocol・16ch・release_all）へ発展させられる。
