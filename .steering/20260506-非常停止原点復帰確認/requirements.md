# 要求内容

## 概要

PRD「7. 安全システム」の受け入れ条件「非常停止スイッチ（室内・操作エリアの2箇所）のいずれかを押すと全アクチュエータが即座に原点復帰する」を満たすことを確認し、PRD チェックボックスを完了済みにする。

## 背景

### PRD 7. 安全システム（抜粋）

**ユーザーストーリー**: 実験エンジニアとして、万が一の際にも安全に停止できるように、非常停止・過電流検知・AC電源断対応が欲しい

**受け入れ条件（本作業スコープ）**:
- [ ] 非常停止スイッチ（室内・操作エリアの2箇所）のいずれかを押すと全アクチュエータが即座に原点復帰する

### 現在の実装状況

コード・ユニットテストは実装済み（84件パス）。ハードウェア実機での動作確認が未完了のため、PRD チェックボックスは未完了。

| レイヤー | ファイル | 状態 |
|---|---|---|
| インフラ | `src/infra/gpio_monitor.py` | 実装済み（GPIO17 RISING エッジ、NC接点）|
| アプリ | `src/app/robot_controller.py` | 実装済み（`emergency_stop()` → `home_return()` 両軸並列）|
| 配線 | `src/app/factory.py` | 実装済み（GPIOMonitor → SafetyMonitor → RobotController）|
| Web API | `src/web/routers/drive.py` | 実装済み（`POST /api/v1/drive/emergency`）|
| GUI | `src/web/static/` | 実装済み（緊急停止ボタン常時表示）|
| ユニットテスト | `tests/unit/test_robot_controller.py` | 実装済み（84件全パス）|
| ハードウェアテスト | `tests/hardware/test_emergency_stop.py` | 実施済み（GPIO信号のみ確認）|

### 参照ドキュメント

- `docs/product-requirements.md` § 7. 安全システム
- `docs/functional-design.md` § UC4: 非常停止（sequenceDiagram）
- `docs/architecture.md` § GPIO ピン配置（GPIO17、2スイッチ並列）、§ アクチュエータ制御
- `docs/glossary.md` § 原点復帰、§ フェイルセーフ
- `docs/development-guidelines.md` § 非常停止ハンドラが優先的に動作する
- `docs/repository-structure.md` § `tests/hardware/`

## 実装対象の機能

### 1. ハードウェアテストスクリプト（エンドツーエンド原点復帰確認）

functional-design.md UC4 のシーケンスをハードウェア実機で再現する:

```
SW(GPIO17) → SafetyMonitor → emergency_stop() → home_return() 両軸同時
```

- スイッチ検知時に `ActuatorDriver.home_return()` を両軸並列で実行する
- DSS1 HEND ビット確認まで完了を報告しない
- アプリケーション全体（FastAPI / RobotController）を起動せず単体で動作する

### 2. ユニットテスト実行確認

- 既存の `tests/unit/test_robot_controller.py` が全パスすることを確認する

### 3. PRD チェックボックス更新

- 実機確認完了後に `docs/product-requirements.md` の受け入れ条件を `[x]` にする

## 受け入れ条件

### ハードウェアテスト
- [ ] スクリプト起動時に両軸の `reset_alarm` → `servo_on` → `home_return` が実行される
- [ ] 非常停止スイッチを押すと「非常停止検知 → 原点復帰開始」が表示される
- [ ] 両軸の `home_return 完了` ログが表示される（DSS1 HEND ビット確認）
- [ ] アクチュエータが物理的に原点位置へ移動することを目視確認できる
- [ ] Ctrl+C で `GPIOクリーンアップ完了` が表示される

### ユニットテスト
- [ ] `tests/unit/test_robot_controller.py` が全パスする（84件以上）

### PRD 更新
- [ ] `docs/product-requirements.md` の該当チェックボックスが `[x]` になる

## スコープ外

- 過電流検知・AC電源断対応は別作業（PRD 7. 安全システムの他の受け入れ条件）
- GPIO27 (AC断検知) は本作業の対象外
