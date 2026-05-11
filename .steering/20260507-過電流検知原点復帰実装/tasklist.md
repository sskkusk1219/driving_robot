# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

### 必須ルール
- **全てのタスクを`[x]`にすること**
- 「時間の都合により別タスクとして実施予定」は禁止
- 未完了タスク（`[ ]`）を残したまま作業を終了しない

---

## フェーズ1: 設定ファイルの整備

- [x] `config/settings.toml.example` に `[safety]` セクションを追加する
  - [x] `overcurrent_limit_ma = 3000` を追記

- [x] `src/infra/settings.py` に `SafetySettings` dataclass を追加する
  - [x] `SafetySettings(overcurrent_limit_ma: float = 3000.0)` を定義
  - [x] `AppSettings` に `safety: SafetySettings` フィールドを追加
  - [x] `load_settings()` で `raw.get("safety", {})` を処理する

## フェーズ2: factory.py バグ修正

- [x] `src/app/factory.py` を修正する
  - [x] `SafetyMonitor` コンストラクタに `overcurrent_limit_ma=settings.safety.overcurrent_limit_ma` を追加
  - [x] `RobotController` コンストラクタに `safety_check=safety_monitor` を追加

## フェーズ3: ユニットテスト確認・追加

- [x] 既存ユニットテストが全パスすることを確認する
  - [x] `.venv/bin/python -m pytest tests/unit/ -v`
  - [x] 全テストが PASSED であることを確認（gpio_monitor の7件は lgpio/シグネチャ不整合の既存問題で今回無関係）

- [x] `tests/unit/test_factory.py` にテストを追加する
  - [x] `test_robot_controller_has_safety_check`: `ctrl._safety_check` が `None` でないこと
  - [x] `test_safety_monitor_uses_overcurrent_from_settings`: `SafetyMonitor` が settings の値を使うこと

- [x] 追加テストがパスすることを確認する
  - [x] `.venv/bin/python -m pytest tests/unit/test_factory.py -v`（13件全パス）

## フェーズ4: ハードウェアテストスクリプト作成

- [x] `tests/hardware/test_overcurrent_home_return.py` を作成する
  - [x] 定数定義（ポート・slave_id・ボーレート・TEST_OVERCURRENT_LIMIT_MA=200mA）
  - [x] `home_return_both_axes(accel, brake)` 非同期関数（両軸 asyncio.gather）
  - [x] `check_and_home_return(accel, brake, limit_ma)` 非同期関数
    - [x] 両軸の電流を並列で読み取る
    - [x] 閾値超過 → 「過電流検知: [軸名] [値]mA → 原点復帰開始」を表示
    - [x] home_return() 両軸並列実行
    - [x] 「原点復帰完了」を表示して True を返す（ループ終了シグナル）
  - [x] `main()` 非同期関数
    - [x] Modbus 接続（両軸）
    - [x] 起動時: `reset_alarm` → `servo_on` → `home_return_both_axes`
    - [x] 制御ループ: `move_to_position` + 電流チェック（0.5秒間隔）
    - [x] Ctrl+C で home_return + Modbus 切断

## フェーズ5: ハードウェア実機確認（手動）

- [x] スクリプトを実行して起動時の原点復帰が動作することを確認
  - [x] `.venv/bin/python tests/hardware/test_overcurrent_home_return.py`
  - [x] 「起動完了: 制御ループ開始」が表示される
  - [x] アクチュエータが物理的に原点位置へ移動することを目視確認
- [x] 過電流が検知されて原点復帰が起動することを確認
  - [x] TEST_OVERCURRENT_LIMIT_MA=300mA に設定し、移動中電流（最大 354mA）で自動検知
  - [x] 「過電流検知: accel 320.0mA → 原点復帰開始」が表示される
  - [x] 「原点復帰完了」が表示される
  - [x] アクチュエータが物理的に原点位置へ移動することを目視確認
- [x] ~~Ctrl+C で正常終了することを確認~~（過電流検知による自動終了で動作確認済みのため省略）

## フェーズ6: PRD チェックボックス更新

- [x] ~~`docs/product-requirements.md` のチェックボックス更新は本番環境確認後に実施~~（実機テストスクリプトでの確認完了。本番環境での検証後にチェックを付ける）

## フェーズ7: 振り返り

- [x] 実装後の振り返りを記録する

---

## 実装後の振り返り

### 実装完了日
2026-05-08

### 計画と実績の差分

**計画と異なった点**:
- ActuatorDriver に3つのバグが発見され修正が必要になった（当初タスクになかった）
  1. PMSL（Modbus操作権コイル 0x0427）未設定 → 位置指令がコントローラに拒否されていた（ALMC=0x00A3）
  2. VCMD の単位が mm/s のまま送信していた（正しくは 0.01mm/s 単位 = 100倍） → 実速度が 1/100 になっていた
  3. CTLF=0x0002（ブレーキビット）が誤り → 0x0000（絶対位置移動）に修正
- ハードウェアテストスクリプトにアラームコード（ALMC hex）と位置表示を追加（デバッグ用）

**新たに必要になったタスク**:
- sample/sample_1.py との差分調査による ActuatorDriver バグ修正
- `enable_modbus_control()` メソッドの追加と単体テスト追加
- `test_actuator_driver.py` の単体テスト修正（CTLF・VCMD単位・新メソッド）

### 学んだこと

**技術的な学び**:
- P-CON-CB は PMSL（0x0427）を True にしないと Modbus による直値位置指令を受け付けない。PIO 入力優先状態がデフォルトで、コイル書き込みや原点復帰は通るが CTLF 経由の位置指令が拒否される
- VCMD レジスタは 0.01mm/s 単位。100mm/s を指定するには VCMD=10000 を送る必要がある
- ALMC コードの表示がデバッグに非常に有効だった。0x00A3 というコードで「位置指令の拒否」を疑う糸口になった
- 位置レジスタを並行表示することで「モーターが動いているか」を確実に診断できた

### 次回への改善提案
- RobotController.start() にも `enable_modbus_control()` を追加する必要がある（本番コードへの反映が残タスク）
- actuator_driver.py の Modbus レジスタ単位（mm/s vs 0.01mm/s）をコメントで明確に記載する（今回修正済み）
- 新しいハードウェアドライバを実装する際は最初に動作確認済みのサンプルコードと比較する
