# 要求内容

## 概要

用意した PCA9685（I2C 16ch PWM ドライバ）と SG90 サーボモータ 1 個が正しく配線・駆動できることを、実機で確認するための手動テストスクリプトを作成する。

## 背景

ボタンサーボ機能（PCA9685 + SG90 ×16 でエンジンスタート・シフト等の物理ボタンを押下）は Post-MVP として仕様（`docs/functional-design.md` の `ButtonServoDriver`、`docs/architecture.md` の I2C 配線）のみ定義済みで、ハードウェアは未検証の状態。本格実装（`ButtonServoDriver`）に着手する前に、まず「PCA9685 経由で SG90 が期待どおり動くか」という最小の疎通確認を行い、配線・電源分離・PWM 諸元の妥当性を実機で押さえておきたい。

## 実装対象の機能

### 1. サーボ疎通確認スクリプト（tests/hardware）

- PCA9685 に I2C（0x40 / 50Hz）で接続し、指定チャンネルの SG90 を任意角度へ駆動する手動確認スクリプト
- 対話キー操作で角度をジョグし、目視でサーボの動作・可動範囲を確認できる
- 待機↔押下の 2 ポジション往復（本番の押下相当）を確認するデモ動作を実行できる
- 既存の `tests/hardware/test_calibration.py` と同じスタイル（standalone `.venv/bin/python` 実行、pytest 非依存、docstring に接続・操作方法を記載）

## 受け入れ条件

### サーボ疎通確認スクリプト

- [ ] `.venv/bin/python tests/hardware/test_servo_pca9685.py` で起動できる
- [ ] I2C0x40 の PCA9685 を検出し、疎通失敗時は原因（I2C 無効・アドレス違い・配線）が分かるエラーを表示する
- [ ] 指定チャンネル（既定 ch0）の SG90 が角度指定どおりに動く
- [ ] キー操作で角度を ±ジョグでき、目視で可動範囲を確認できる
- [ ] 待機角度↔押下角度の往復デモが動作する
- [ ] 終了時・中止時にサーボ出力を停止（PWM off）して安全に終える
- [ ] 追加の外部依存を増やさない（既にある `smbus2` で実装）

## 成功指標

- SG90 1 個が PCA9685 経由で角度指定どおりに動くことを実機で確認できる
- 配線図（本ステアリングの design.md）と実機が一致していることを検証できる

## スコープ外

以下はこのフェーズでは実装しません:

- `src/infra/ButtonServoDriver` 本体の実装（Post-MVP 本実装は別作業）
- 16ch 全チャンネル / タイムスケジュール連携
- 押下時間スケジュール・Web UI 連携
- 自動テスト（CI）化。あくまで実機目視の手動スクリプト

## 参照ドキュメント

- `docs/product-requirements.md` L259 - ボタンサーボ（PCA9685 + SG90 ×16）要求
- `docs/functional-design.md` L395-428 - `ButtonServoDriver` 責務・チャンネルマッピング・PWM 諸元
- `docs/architecture.md` L278-293, L306-316 - I2C 配線図・電源分離・I2C/サーボ設定
- `docs/glossary.md` / `docs/repository-structure.md` / `docs/development-guidelines.md`
