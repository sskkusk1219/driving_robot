# 要求仕様: Kvaser CANReader 実装・修正・テスト

## 背景

Kvaser USB-CAN インターフェースを Raspberry Pi 5 の USB ポートに接続した。
シャシダイナモ MEIDACS から CAN バス経由で車速を受信するための
`CANReader` をプロダクション使用可能な状態に整備する。

## DBC ファイル調査結果

- ファイル: `config/can/MEIDEN_MEIDACS.dbc`
- フレーム ID: 288 (0x120)、フレーム名: `MEIDACS_Frame0`
- 車速シグナル: `Speed`（32bit Motorola signed、係数 0.01、単位 km/h）

## 現状の問題点

1. **シグナル名不一致**: `can_reader.py` の `_SPEED_SIGNAL_NAME = "VehicleSpeed"` が
   DBC の実際のシグナル名 `Speed` と異なる → 実機で `ValueError` が発生する
2. **`cantools` 未インストール**: `pyproject.toml` に依存関係がない
3. **`dbc_path` の設定経路がない**: `CanSettings` に `dbc_path` フィールドがなく、
   `factory.py` が `CANReader` に DBC パスを渡していない
4. **既存テスト**: `"VehicleSpeed"` を期待しているためシグナル名修正後に更新が必要

## 対応要件

1. `cantools` を `pyproject.toml` の依存関係に追加し、仮想環境にインストールする
2. `_SPEED_SIGNAL_NAME` を `"Speed"` に修正する
3. `CanSettings` に `dbc_path` フィールドを追加する（デフォルト: `config/can/MEIDEN_MEIDACS.dbc`）
4. `settings.toml.example` に `dbc_path` を追記する
5. `factory.py` で `dbc_path=settings.can.dbc_path` を `CANReader` に渡す
6. 既存テスト (`tests/unit/infra/test_can_reader.py`) を修正してシグナル名変更に追従する
7. DBC ファイルを使った統合的なテストケースを追加する（`cantools` をモック化せず実際の DBC を使用）
8. `config/settings.toml` を `settings.toml.example` からコピーして作成する

## 完了条件

- `pytest tests/unit/infra/test_can_reader.py` が全件パスする
- `pytest tests/` 全体がパスする
- `ruff check src/infra/can_reader.py tests/unit/infra/test_can_reader.py` が エラーなし
