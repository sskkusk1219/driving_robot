# 設計書

## アーキテクチャ概要

`tests/hardware/` に置く単体の手動確認スクリプト。既存の `test_calibration.py` と同様、standalone で `.venv/bin/python` から実行し、pytest には依存しない。PCA9685 の PWM レジスタを `smbus2` で直接叩き、SG90 を駆動する。

```
[操作者] --キー入力--> test_servo_pca9685.py
                          │  smbus2 (I2C1, /dev/i2c-1)
                          ▼
                   PCA9685 (0x40, 50Hz)
                          │  PWM ch (既定 ch0)
                          ▼
                     SG90 サーボ
```

## 配線（6 つの永続ドキュメントから）

`docs/architecture.md`（L278-293, L306-316）と `docs/functional-design.md`（L395-428）に定義済みの接続をそのまま使う。

| 接続元（Raspberry Pi 5） | 物理ピン | 接続先（PCA9685） | 備考 |
|--------------------------|----------|-------------------|------|
| GPIO2 / SDA1             | 物理ピン 3 | SDA               | I2C1 データ線 |
| GPIO3 / SCL1             | 物理ピン 5 | SCL               | I2C1 クロック線 |
| 3.3V                     | 物理ピン 1 | VCC               | PCA9685 **ロジック**電源（本体 3.3V） |
| GND                      | 物理ピン 6（等） | GND         | ロジック GND。電源 GND と**共通**にする |
| （接続しない）           | —        | V+                | サーボ電源。**外部 5V** から給電（本体ロジックと分離） |

**SG90 → PCA9685 の出力側**（ch0 のサーボヘッダ、3 ピン）:

| SG90 リード線 | PCA9685 ch ヘッダ |
|---------------|-------------------|
| オレンジ（信号 PWM） | PWM（信号ピン） |
| 赤（+電源）    | V+（外部 5V） |
| 茶（GND）      | GND |

**重要な配線ルール（docs より）**:
- PCA9685 の `V+` は Raspberry Pi 本体ロジック電源とは**分離した外部 5V 電源**から供給する（SG90 ×16 の突入・保持電流対策）。1 個の動作確認でも本体 5V ピンからの給電は避け、外部 5V を推奨（`docs/architecture.md` L293）。
- 信号線（I2C）側の GND と外部 5V 電源の GND は**共通に接続**する（GND を繋がないと PWM 基準がずれてサーボが暴れる）。
- I2C アドレスは既定 `0x40`（A0-A5 ジャンパ全開放）。PWM 周波数 50Hz。

**前提（実機セットアップ）**:
- 現状 `/dev/i2c-1` が存在しない（I2C1 が無効）。`sudo raspi-config nonint do_i2c 0` もしくは `/boot/firmware/config.txt` に `dtparam=i2c_arm=on` を追加し再起動して有効化する。
- 有効化後 `i2cdetect -y 1` で `40` が見えることを確認してからスクリプトを実行する。

## コンポーネント設計

### 1. PCA9685 最小ドライバ（スクリプト内クラス `_Pca9685`）

**責務**:
- MODE1/MODE2、PRE_SCALE レジスタ設定で PWM 周波数 50Hz を設定
- 指定 ch の LEDn_ON/OFF レジスタに 12bit デューティを書き込み
- 角度 → パルス幅[us] → 12bit カウント の変換
- 全 ch オフ（`ALL_LED_OFF`）で安全停止

**実装の要点**:
- レジスタ: MODE1=0x00, PRESCALE=0xFE, LED0_ON_L=0x06, ALL_LED_ON_L=0xFA
- prescale = round(25MHz / (4096 * freq)) - 1。50Hz → 0x79 付近
- SLEEP ビットを立ててから PRESCALE を書き、再起動して RESTART
- 角度→us: 既定 500us(0°)〜2500us(180°)。SG90 個体差があるので定数で調整可能に
- us→count: count = us * 4096 * freq / 1_000_000（50Hz なら 1 周期 20000us=4096count）

### 2. 対話 CLI（`main`）

**責務**:
- 起動時に接続・周波数設定、既定角度へ移動
- キー操作: `e/w` ±5°, `d/s` ±1°, 数値プリセット、`p` 押下デモ（待機↔押下往復）、`q` 終了
- 終了・中止・例外時に必ず PWM off

**実装の要点**:
- `test_calibration.py` の `_read_single_key`（termios raw モード）を踏襲
- I2C 例外は握って「I2C が有効か / 0x40 配線 / 外部電源」を促すメッセージにする

## データフロー

### サーボ動作確認
```
1. smbus2 で /dev/i2c-1 を開く（失敗→I2C 有効化を案内して終了）
2. PCA9685 を 50Hz に初期化
3. 待機角度(既定)へ移動
4. キー入力ループ: ジョグ / プリセット / 押下デモ
5. q または Ctrl-C → ALL_LED_OFF して終了
```

## エラーハンドリング戦略

- `FileNotFoundError`（/dev/i2c-1 無し）→ I2C 有効化手順を表示
- `OSError`（Remote I/O error）→ 0x40 未検出。配線・アドレス・電源を確認する旨を表示
- どの経路でも `finally` で `all_off()` を呼びサーボ出力を止める

## テスト戦略

### 手動テスト（実機）
- ch0 に SG90 を接続し、ジョグで 0°/90°/180° 付近を目視確認
- 押下デモで待機↔押下の往復を確認
- Ctrl-C 中断でサーボ出力が止まることを確認

### ユニットテスト
- 対象外（実機 I2C 依存の手動スクリプトのため）。角度→count 変換は関数として切り出し、必要なら将来 pytest 化可能な形にしておく

## 依存ライブラリ

新規追加なし。既存の `smbus2` (0.4.3) を使用（`docs/architecture.md` が PCA9685 制御で smbus2 を代替として許容）。adafruit-circuitpython-pca9685 は未インストールのため採用しない。

## ディレクトリ構造

```
tests/hardware/
  └── test_servo_pca9685.py   # 新規（手動確認スクリプト）
.steering/20260701-servo-pca9685-smoke-test/
  ├── requirements.md
  ├── design.md
  └── tasklist.md
```

## 実装の順序

1. スクリプト骨子（docstring: 接続・操作方法・実行方法）
2. `_Pca9685` クラス（init/50Hz/set_angle/all_off）
3. 対話 CLI（ジョグ・プリセット・押下デモ・安全停止）
4. 構文チェック（`.venv/bin/python -m py_compile`）と静的確認

## セキュリティ考慮事項

- 特になし（ローカル I2C デバイス操作のみ）

## パフォーマンス考慮事項

- SG90 は 50Hz。連続で角度を送りすぎない（ジョグ間に短い待機）

## 将来の拡張性

- `_Pca9685` と角度変換は将来 `src/infra/ButtonServoDriver`（Protocol ベース、16ch、release_all）へ発展させられる形にしておく（本作業では移植しない）
