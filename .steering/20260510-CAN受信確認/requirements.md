# 要求内容

## 概要

USB接続のKvaser Memorator CAN インターフェース経由でシャシダイナモからCAN信号を受信し、
ターミナルに車速をリアルタイム表示する診断スクリプトを実装する。

## 背景

CAN受信が正しく動作するか確認したい。本番のRobotControllerに組み込む前に、
単独で実行できる診断ツールとして車速受信を検証する必要がある。

## 実装対象の機能

### 1. CAN受信診断スクリプト (`scripts/check_can.py`)

- Kvaser USB-CAN に接続し、DBC ファイルをロードする
- CAN フレームを受信してターミナルに車速をリアルタイム表示する
- タイムスタンプ・フレームID・デコード済み車速 [km/h] を表示する
- タイムアウト（フレームが来ない）は警告表示して継続する
- Ctrl+C で接続を閉じてクリーンに終了する

## 受け入れ条件

### CAN受信診断スクリプト

- [ ] `python scripts/check_can.py` で実行できる
- [ ] Kvaser デバイスが接続されていれば CAN バスに接続できる
- [ ] 受信した車速 [km/h] がターミナルに表示される
- [ ] フレームが来ない場合は "タイムアウト" と表示して継続する
- [ ] Ctrl+C でクリーンに終了する（"接続を閉じました" などのメッセージを表示）
- [ ] DBC ファイルパスと CAN インターフェース設定は `config/settings.toml` から読む

## 成功指標

- スクリプト単独実行で CAN 受信動作を確認できる
- エラー発生時もスタックトレースでなく分かりやすいメッセージが表示される

## スコープ外

以下はこのフェーズでは実装しません:

- WebSocket / FastAPI への組み込み（本番統合は別タスク）
- ログファイルへの書き込み
- PostgreSQL への保存

## 参照ドキュメント

- `docs/architecture.md` - ハードウェア構成 (Kvaser USB-CAN)
- `src/infra/can_reader.py` - 既存 CANReader クラス
- `config/can/MEIDEN_MEIDACS.dbc` - DBC ファイル (Speed 信号: ID 0x120)
- `config/settings.toml.example` - CAN 設定例
