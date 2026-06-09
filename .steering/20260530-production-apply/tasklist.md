---
name: production-apply-tasklist
description: 本番環境適用タスクリスト
metadata:
  type: project
---

# タスクリスト: 本番環境への適用

## フェーズ1: 設定ファイル修正

- [x] config/settings.toml に `bitrate = 500000` を [can] セクションへ追加
- [x] config/settings.toml に [ups] セクション（nut_host/nut_port/ups_name/poll_interval_s）を追加
- [x] config/settings.toml に [safety] セクション（overcurrent_limit_ma）を追加
- [x] config/settings.toml の [gpio] コメントを NUT 方式確定後の記述に更新

## フェーズ2: factory.py 修正

- [x] src/app/factory.py の CANReader 初期化に bitrate=settings.can.bitrate を追加

## フェーズ3: test_calibration.py docstring 修正

- [x] tests/hardware/test_calibration.py のモジュールdocstringの操作キー表記を実装に合わせて修正
- [x] tests/hardware/test_calibration.py の定数インラインコメント（キー名・移動量）を修正（+/- → e/w、./,→ d/s、0.1mm → 0.5mm）

## 実装後の振り返り

**実装完了日**: 2026-05-30

**計画との差分**:
- 定数コメント修正（_JOG_STEP_SMALL の 0.1mm→0.5mm 誤りとキー名更新）は、implementation-validator によって検証中に発見されたため計画外タスクとして追加・実施した。

**学んだこと**:
- settings.toml は .gitignore 対象のため、settings.toml.example を更新しても settings.toml には自動反映されない。新機能追加時（UPS監視等）に settings.toml.example を更新した後、settings.toml も手動で同期する必要がある。
- factory.py で新しいインフラクラスにパラメータを追加した際、対応する check_*/test_* スクリプトとの引数対称性を確認する習慣が重要。

**次回への改善提案**:
- settings.toml と settings.toml.example の同期忘れを防ぐため、CI または make ターゲットで diff チェックを行う仕組みを検討する。
