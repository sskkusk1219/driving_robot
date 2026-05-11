# タスクリスト

## 🚨 タスク完全完了の原則

**このファイルの全タスクが完了するまで作業を継続すること**

### 必須ルール
- **全てのタスクを`[x]`にすること**
- 「時間の都合により別タスクとして実施予定」は禁止
- 「実装が複雑すぎるため後回し」は禁止
- 未完了タスク（`[ ]`）を残したまま作業を終了しない

---

## フェーズ1: 診断スクリプト実装

- [x] `scripts/check_can.py` を実装する
  - [x] settings.toml から CAN 設定を読み込む（tomllib 使用、ファイル不在時はデフォルト値）
  - [x] CANReader を使って Kvaser に接続する
  - [x] ループで `read_speed()` を呼び出してターミナルに表示する
  - [x] TimeoutError / ValueError をキャッチして継続する
  - [x] Ctrl+C でクリーン終了する

## フェーズ2: ハードウェアテスト追加

- [x] `tests/hardware/test_can_receive.py` を実装する
  - [x] pytest マーク `hardware` を付ける
  - [x] CANReader.connect() が正常に完了することを確認するテストを書く
  - [x] read_speed() が float を返すことを確認するテストを書く

## フェーズ3: 品質チェック

- [x] ruff でリントエラーがないことを確認
  - [x] `.venv/bin/ruff check scripts/check_can.py tests/hardware/test_can_receive.py`
- [x] mypy で型エラーがないことを確認
  - [x] `.venv/bin/mypy scripts/check_can.py`
- [x] 既存ユニットテストが引き続き通ることを確認
  - [x] `.venv/bin/pytest tests/unit/ -q` → 380 passed（gpio_monitor テストも lgpio API に合わせて修正済み）

## フェーズ4: Kvaser ドライバセットアップ（実機作業）

- [x] Kvaser Linux SDK (linuxcan v5.51.461) をビルド・インストール
  - [x] `https://www.kvaser.com/downloads-kvaser/` から `linuxcan.tar.gz` をダウンロード
  - [x] `sudo make KV_NO_PCI=1 && sudo make install KV_NO_PCI=1` でビルド・インストール
  - [x] `libcanlib.so.1.10.12` が `/usr/lib/` に配置されたことを確認
  - [x] `sudo modprobe usbcanII leaf mhydra` でカーネルモジュールをロード
  - [x] `/dev/usbcanII0`, `/dev/usbcanII1` が作成されたことを確認
  - [x] `listChannels` で Kvaser Memorator HS/HS (ch0/ch1) が表示されたことを確認

- [x] python-can の aarch64 バグをパッチ（2箇所）
  - [x] **バグ1**: `c_long(TIMESTAMP_RESOLUTION)` + `size=4` → aarch64 では `c_long` が8バイトのため `canERR_PARAM`
    - 修正: `.venv/lib/.../can/interfaces/kvaser/canlib.py` line 509
    - `ctypes.c_long(TIMESTAMP_RESOLUTION)` → `ctypes.c_uint(TIMESTAMP_RESOLUTION)`
    - `4` → `ctypes.sizeof(ctypes.c_uint)`
  - [x] **バグ2**: `canIOCTL_SET_LOCAL_TXACK` が Kvaser Memorator で未サポート（`canERR_PARAM`）
    - 修正: `canIoCtlInit(SET_LOCAL_TXACK, ...)` を `try/except CANLIBError` で囲む
    - 注意: `canIoCtlInit` と `canIoCtl` は ctypes が同じ関数オブジェクトを返すため
      `errcheck` が `__check_status_operation` に上書きされ `CANLIBOperationError` が投げられる。
      `except CANLIBInitializationError` では捕捉できず `except CANLIBError` が必要

- [x] `CANReader` / `CanSettings` にビットレートパラメータを追加
  - [x] `CanSettings.bitrate: int = 500000` を追加
  - [x] `CANReader.__init__` に `bitrate: int = 500000` を追加
  - [x] `CANReader.connect()` で `can.Bus(..., bitrate=self._bitrate)` を渡すよう修正
  - [x] `scripts/check_can.py` で `bitrate=can.bitrate` を渡すよう修正
  - [x] `config/settings.toml.example` に `bitrate = 500000` を追加

## フェーズ5: 物理配線の確認（未完了・申し送り）

- [ ] **GND 接続**: RPi3B と RPi5 は別電源のため GND 接続が必要
  - Kvaser DB9 ピン3(GND) ↔ MCP2515 HAT GND を配線すること
- [ ] **終端抵抗の確認**:
  - MCP2515 HAT 側: ジャンパで内蔵終端が有効になっているか確認
  - Kvaser Memorator 側: 終端内蔵なし → DB9 ピン7(CANH) と ピン2(CANL) の間に 120Ω を外付け
  - 確認方法: テスターで CANH-CANL 間を測定 → 約60Ω なら両端OK
- [ ] **実機受信確認**: 上記配線完了後に `sudo .venv/bin/python -u scripts/check_can.py` で車速表示を確認

---

## 補足: 構成メモ

### ノード構成
| 機器 | CAN ドライバ | インターフェース |
|------|------------|----------------|
| RPi3B (driving_simulator) | SocketCAN (MCP2515 HAT) | `can0` @ 500kbps |
| RPi5 (driving_robot) | Kvaser canlib (usbcanII) | `/dev/usbcanII0` |

### RPi5 での注意点
- `ip link show` に `can0` は**表示されない**（Kvaser は SocketCAN ではないため正常）
- 実行は `sudo` が必要（udev rules 未設定のため）
- `python-can` のパッチは `.venv` 内に直接当てているため、`pip install --upgrade python-can` で上書きされたら再パッチ必要

---

## 実装後の振り返り

### 実装完了日
2026-05-10（フェーズ1〜4完了、フェーズ5は物理配線待ち）

### 計画と実績の差分

**計画と異なった点**:
- `test_gpio_monitor.py` が既存の未コミット変更（`RPi.GPIO` → `lgpio` 移行）によって7件失敗していたため、lgpio API に合わせてテストを修正した
- Kvaser Linux SDK のインストールが必要だった（当初想定外）
- python-can に aarch64 固有のバグが2件あり、venv 内のファイルを直接パッチした

**新たに必要になったタスク**:
- Kvaser Linux SDK ビルド・インストール
- python-can aarch64 バグ修正パッチ
- `CANReader` へのビットレートパラメータ追加

### 学んだこと

**技術的な学び**:
- lgpio コールバックのシグネチャは `(chip, gpio, level, timestamp)` で、RPi.GPIO の `channel` キーワード引数とは異なる
- Kvaser Memorator は SocketCAN ではなく canlib 経由のため `ip link show` に出ない（正常）
- aarch64 では `ctypes.c_long` が 8バイト。canlib ioctl に `c_long` + `size=4` を渡すと `canERR_PARAM`
- `canIoCtlInit` と `canIoCtl` は ctypes がキャッシュにより同一オブジェクトを返すため errcheck が上書きされる
- CAN バスは CANH/CANL に加え GND も異電源間では必須
- 2ノード構成では両端に 120Ω 終端が必要。CANH-CANL 間テスターで約 60Ω が正常値

### 次回への改善提案
- python-can の aarch64 バグを upstream に PR する
- udev rules を設定して `sudo` 不要にする
- Kvaser インストール手順を `docs/` または `scripts/setup_env.sh` に追記する
