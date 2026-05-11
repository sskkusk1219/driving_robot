# 要求内容

## 概要

PRD「7. 安全システム」の受け入れ条件「電流値が閾値を超えた場合に自動停止する」を実装・確認し、PRD チェックボックスを完了済みにする。

## 背景

### PRD 7. 安全システム（抜粋）

**ユーザーストーリー**: 実験エンジニアとして、万が一の際にも安全に停止できるように、非常停止・過電流検知・AC電源断対応が欲しい

**受け入れ条件（本作業スコープ）**:
- [ ] 電流値が閾値を超えた場合に自動停止する

### 現在の実装状況と課題

| レイヤー | ファイル | 状態 |
|---|---|---|
| ドメイン | `src/domain/control/drive_loop.py` | 実装済み（L185-195: check_overcurrent → on_emergency）|
| ドメイン | `src/domain/safety_monitor.py` | 実装済み（check_overcurrent: 3000mA固定値）|
| アプリ | `src/app/robot_controller.py` | 実装済み（emergency_stop → home_return 両軸並列）|
| アプリ | `src/app/factory.py` | **バグ**: `safety_check=safety_monitor` が渡されていない → DriveLoop が起動しない |
| 設定 | `config/settings.toml.example` | **欠落**: `overcurrent_limit_ma` が設定ファイルにない（3000mAハードコード）|
| ユニットテスト | `tests/unit/test_drive_loop.py` | 実装済み（TestOvercurrentEmergency: 2件）|
| ハードウェアテスト | `tests/hardware/` | **未作成**: 実機での過電流 → 原点復帰確認スクリプトがない |

**重大な問題**: `factory.py` で `safety_check` が `RobotController` に渡されていないため、本番環境では `DriveLoop` が起動せず、過電流検知が機能していない。

### 参照ドキュメント

- `docs/product-requirements.md` § 7. 安全システム
- `docs/functional-design.md` § SafetyMonitor、§ エラーハンドリング（「過電流：[軸名] [値]mA」）
- `docs/architecture.md` § アクチュエータドライバ（電流値レジスタ 0x900C-0x900D）
- `docs/development-guidelines.md` § 安全に関わるコードの原則
- `.steering/20260506-非常停止原点復帰確認/` § 実装済みの emergency_stop フロー

## 実装対象の機能

### 1. factory.py の safety_check バグ修正

`build_real_controller()` で `safety_check=safety_monitor` を `RobotController` に渡す。これにより `DriveLoop` 起動条件を満たし、50ms制御ループ内の過電流検知が実際に動作するようになる。

### 2. 設定ファイルへの overcurrent_limit_ma 追加

`[safety]` セクションを `settings.toml.example`・`settings.py` に追加し、過電流閾値を設定ファイルから読み込めるようにする。

### 3. ハードウェアテストスクリプト作成

`tests/hardware/test_overcurrent_home_return.py` を作成する。実機で「過電流検知 → 原点復帰」のフローをエンドツーエンドで確認する：

```
ActuatorDriver（両軸）× SafetyMonitor（低い閾値）→ 制御ループ模擬
    └── 電流 > 閾値 → 「過電流検知 → 原点復帰開始」表示 → home_return() 両軸同時
```

### 4. ユニットテスト確認 + factory テスト更新

既存 `TestOvercurrentEmergency` の全パスを確認し、factory の `safety_check` 引数テストを追加する。

### 5. PRD チェックボックス更新

実機確認完了後に `docs/product-requirements.md` の受け入れ条件を `[x]` にする。

## 受け入れ条件

### factory バグ修正
- [ ] `RobotController._safety_check` が `None` でなく `SafetyMonitor` インスタンスになっている
- [ ] `test_factory.py` に `safety_check` が正しく設定されているかのテストが追加されている

### 設定ファイル
- [ ] `config/settings.toml.example` の `[safety]` セクションに `overcurrent_limit_ma = 3000` が存在する
- [ ] `src/infra/settings.py` に `SafetySettings` dataclass が追加されている
- [ ] `factory.py` が `settings.safety.overcurrent_limit_ma` を `SafetyMonitor` に渡している

### ハードウェアテスト
- [ ] スクリプト起動時に両軸の `reset_alarm` → `servo_on` → `home_return` が実行される
- [ ] 過電流が検知されると「過電流検知: [軸名] [値]mA → 原点復帰開始」が表示される
- [ ] 両軸の `home_return 完了` ログが表示される（DSS1 HEND ビット確認）
- [ ] アクチュエータが物理的に原点位置へ移動することを目視確認できる
- [ ] Ctrl+C で `クリーンアップ完了` が表示される

### ユニットテスト
- [ ] `tests/unit/test_drive_loop.py` の `TestOvercurrentEmergency` が全パスする
- [ ] `tests/unit/test_factory.py` に `safety_check` のテストが追加・パスする

### PRD 更新
- [ ] `docs/product-requirements.md` の該当チェックボックスが `[x]` になる

## スコープ外

- `ff_controller` の設定（運転モデルの学習・ロードは別作業）
- AC電源断対応（別作業）
- GUI での過電流エラー表示改善（現状ログのみ）
- キャリブレーション中の過電流検知（別実装）
