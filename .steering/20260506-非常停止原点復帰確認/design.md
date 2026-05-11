# 設計書

## アーキテクチャ概要

functional-design.md UC4 のシーケンスをスタンドアロンスクリプトで再現する。既存の `ActuatorDriver` を直接 import して再利用し、アプリケーション全体（FastAPI / RobotController）を起動せずに動作確認できる。

```
test_emergency_stop_home_return.py
    │
    ├── lgpio (GPIO17 RISING エッジ検知)    ← gpio_monitor.py と同等ロジック
    │       └── run_coroutine_threadsafe → asyncio ループ
    │
    └── ActuatorDriver × 2
            ├── /dev/ttyUSB0 (slave_id=1: アクセル軸)
            │       reset_alarm → servo_on → home_return
            └── /dev/ttyUSB1 (slave_id=2: ブレーキ軸)
                    reset_alarm → servo_on → home_return
```

## コンポーネント設計

### 1. Modbus RTU 接続・原点復帰

**責務**:
- `src/infra/actuator_driver.ActuatorDriver` を import して再利用する（ロジック重複なし）
- 起動時前処理: `reset_alarm` → `servo_on`（architecture.md § 初期化シーケンス準拠）
- 両軸並列原点復帰: `asyncio.gather(accel.home_return(), brake.home_return())`

**実装の要点**:
- `ActuatorDriver.connect()` / `close()` を明示的に呼ぶ（`async with` 不使用）
- タイムアウト `_HOME_RETURN_TIMEOUT_S = 30.0` は既存定数をそのまま使う

### 2. GPIO 割り込み監視

**責務**:
- GPIO17 の RISING エッジを検知する（architecture.md: NC接点、HIGH=停止）
- 検知時に asyncio イベントループへ原点復帰タスクをスケジュールする

**実装の要点**:
- `lgpio` はシステムパッケージ（`/usr/bin/python3` 不要、`.venv` は `pymodbus` を持つため `.venv/bin/python` で実行）
  - `.venv` 環境に `lgpio` がなければ `import lgpio` で ImportError になるため、冒頭に確認用コメントを入れる
- lgpio コールバックは別スレッドから呼ばれる → `asyncio.run_coroutine_threadsafe()` で橋渡し
- デバウンス: `gpio_set_debounce_micros(h, 17, 50_000)`（前回実績: 50ms）

## データフロー

### スクリプト起動時
```
1. asyncio イベントループ起動
2. Modbus 接続（両軸）
3. reset_alarm → servo_on（両軸並列）
4. home_return 実行（両軸並列）  ← UC4 と同等
5. 「起動時原点復帰完了」を表示
6. GPIO17 監視開始
7. 「スイッチを押すと原点復帰します。Ctrl+C で終了」を表示してループ待機
```

### 非常停止スイッチ押下時（UC4 再現）
```
1. lgpio コールバック（別スレッド）: GPIO17 RISING エッジ検知
2. run_coroutine_threadsafe で asyncio ループへスケジュール
3. 「非常停止検知 → 原点復帰開始」を表示
4. home_return 実行（両軸 asyncio 並列）
5. 「両軸 原点復帰完了」を表示
```

### Ctrl+C 終了時
```
1. KeyboardInterrupt キャッチ
2. GPIO コールバックキャンセル + lgpio クリーンアップ
3. Modbus 切断（両軸）
4. 「GPIOクリーンアップ完了」を表示
```

## エラーハンドリング戦略

| エラー | 対処 |
|---|---|
| `home_return` タイムアウト（30秒） | エラーログを表示してループ継続（スクリプト終了しない）|
| Modbus 接続失敗 | エラーを表示して即終了 |
| 原点復帰実行中に再度スイッチ押下 | asyncio.gather を新たにスケジュールしてよい（冪等）|

## 依存ライブラリ

追加不要。既存の依存関係のみ使用:
- `lgpio` (システムパッケージ: `apt install python3-lgpio`)
- `pymodbus` (`.venv` パッケージ)
- `src.infra.actuator_driver.ActuatorDriver` (プロジェクト内モジュール)

**実行コマンド**: `.venv/bin/python tests/hardware/test_emergency_stop_home_return.py`
（`.venv` は `pymodbus` を持ち、システムの `lgpio` は Python パスに含まれる）

## ディレクトリ構造

```
tests/hardware/
├── test_emergency_stop.py              # 既存: GPIO 信号単体確認（完了済み）
└── test_emergency_stop_home_return.py  # 新規: 原点復帰エンドツーエンド確認
```

## 実装の順序

1. `tests/hardware/test_emergency_stop_home_return.py` 作成
2. ユニットテスト (`tests/unit/test_robot_controller.py`) を実行して全パスを確認
3. ハードウェア実機でスクリプトを実行して動作確認
4. PRD チェックボックスを `[x]` に更新
