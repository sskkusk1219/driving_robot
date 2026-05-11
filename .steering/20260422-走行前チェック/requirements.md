# 要求内容

## 概要

走行開始前に6項目のシステム状態チェックを実施し、全項目パス時のみ走行を許可するドメインクラス `PreCheckRunner` を実装する。

## 背景

`RobotController` の `start_auto_drive` / `start_manual` には走行前チェックの stub コメントが残っている。
現状は何のチェックも行わずに走行を開始してしまうため、通信断・アラーム状態・キャリブレーション未実施のまま走行が始まる危険がある。

## 実装対象の機能

### 1. PreCheckRunner ドメインクラス

- 6項目のチェックをまとめて実行する `run()` メソッドを提供
- 各チェック結果（項目名・合否・エラーメッセージ）を返す
- Protocol を使ってハードウェア依存を分離し、テスト可能にする

### 2. RobotController への統合

- `start_auto_drive` / `start_manual` の stub コメントを実際の `PreCheckRunner` 呼び出しに置き換え
- チェック NG 時は状態を READY に戻し `PreCheckFailed` を raise する

## 受け入れ条件

### PreCheckRunner

- [ ] 6項目すべてパス時に `PreCheckResult(passed=True)` を返す
- [ ] いずれか1項目でも NG の場合 `PreCheckResult(passed=False)` を返す
- [ ] 各 `PreCheckItemResult` に項目名・合否・エラーメッセージが含まれる
- [ ] 通信確認: actuator の read_position / CAN の read_speed が成功すること
- [ ] サーボ状態: 両軸の is_alarm_active() が False であること
- [ ] キャリブレーション: profile.calibration が存在し is_valid が True であること
- [ ] プロファイル: profile が None でないこと
- [ ] UPS残量: battery_pct >= 20.0% であること（Protocol 経由、実機実装は別途）
- [ ] アクチュエータ位置: 両軸の read_position() が tolerance 以内であること

### RobotController 統合

- [ ] start_auto_drive でチェック NG 時に PreCheckFailed が raise される
- [ ] start_auto_drive でチェック NG 時に状態が READY に戻る
- [ ] start_manual でチェック NG 時に PreCheckFailed が raise される
- [ ] start_manual でチェック NG 時に状態が READY に戻る

## スコープ外

以下はこのフェーズでは実装しません:

- UPS残量の実機取得（GPIOMonitor / NUT 連携）: 機種未確定のため Protocol stub のみ提供
- 学習運転 (start_learning_drive) への統合: 未実装
- Web API / GUI での走行前チェック結果表示

## 参照ドキュメント

- `docs/functional-design.md` - 走行前チェック仕様（6項目表）
- `docs/product-requirements.md` - 受け入れ条件
- `docs/architecture.md` - レイヤードアーキテクチャ
