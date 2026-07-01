# タスクリスト: Modbus RTU desync カスケードの修正

## フェーズ1: 原因調査

- [x] エラーログのバイト列解析（`recv` / `extra data` / `No response`）
- [x] `actuator_driver.py` / `robot_controller.py` / `ws.py` の呼び出し経路確認
- [x] pymodbus 3.13.0 ソース精読（transaction.py / transport.py / serialtransport.py / client/serial.py）
  - [x] `execute()` の retries / recv_buffer クリア挙動を確認
  - [x] `handle_local_echo` の send/recv 挙動を確認（async でも transport.send が sent_buffer を積むこと）
  - [x] timeout が `timeout_connect` として使われることを確認

## フェーズ2: 修正

- [x] ~~`handle_local_echo=True` を追加~~（実装方針変更により不要: 非エコー機で FC05 の正常応答を誤破棄するため撤回）
- [x] `__init__` デフォルトを `timeout=0.3`, `retries=3` に変更
- [x] `_flush_input_buffer()` を追加（OS 受信バッファの残渣除去）
- [x] `_bus_op()` コンテキストマネージャ（Lock + フラッシュ）を追加し全 Modbus 操作をラップ
  - [x] enable_modbus_control / reset_alarm / servo_on / servo_off
  - [x] home_return / move_to_position / wait_for_position_complete
  - [x] read_position / read_current / is_alarm_active

## フェーズ3: 検証

- [x] `python -m py_compile src/infra/actuator_driver.py` で構文確認
- [x] 実機で `tests/hardware/test_calibration.py` を完走（両軸ゼロ/フル点設定、通信エラーなし）

## フェーズ4: ドキュメント

- [x] 実装後の振り返り（このファイル下部に記録）

---

## 実装後の振り返り

### 実装完了日
2026-06-16

### 計画と実績の差分

**計画と異なった点**:
- 当初は「複数コルーチンの同時アクセスによるフレーム破損」と推定し、`asyncio.Lock`
  による直列化を最初の修正とした。しかし pymodbus の `execute()` は既にクライアント単位の
  独自ロックを持っており、単一クライアント・単一スレッドのキャリブレーションテストでも
  エラーが再現したため、これは主因ではなかった。
- 2 番目に「FTDI の TX エコー」と推定し `handle_local_echo=True` を入れたが、これは逆効果で
  状況を悪化させた（下記スキップ参照）。
- 最終的に pymodbus ソースの精読により、`retries=0` による「単発タイムアウトからの
  復旧不能」が真因と判明した。

**技術的理由でスキップ（撤回）したタスク**:
- `handle_local_echo=True` の追加
  - 撤回理由: `transport.send()` は async クライアントでも `sent_buffer += data` を実行し、
    受信時に「先頭が送信バイトと一致したら破棄」する。FC05 コイル書き込みの正常応答は
    リクエストと完全同一バイト列のため、非エコー機である本アダプタでは本物の応答を
    エコー誤認して破棄し、`enable_modbus_control` すら応答なしになった。
  - 代替: `retries`/`timeout` の調整 + OS バッファフラッシュで対応。

### 学んだこと

**技術的な学び**:
- pymodbus `AsyncModbusSerialClient` の `retries=0` は危険。1 回のタイムアウトで即例外となり、
  遅延応答バイトが OS バッファに残って次トランザクションを汚染し desync が連鎖する。
  `retries>0` にすると再送のたびに `recv_buffer` がクリアされ自動復旧する。
- pymodbus の `timeout` 引数は `timeout_connect` に割り当てられ、応答待ちタイムアウトとして使われる。
- `handle_local_echo` は **エコーするアダプタ専用**。FC05/FC15 など「応答=リクエスト同型」の
  ファンクションでは、非エコー機で有効化すると正常応答を破棄する罠がある。
- pymodbus はライブラリ内部 `recv_buffer` はクリアするが、OS シリアル受信バッファ
  （`SerialTransport.sync_serial`）はクリアしない。残渣除去には `reset_input_buffer()` が要る。
- 症状の「応答値自体は正しい」点が、配線断ではなくタイミング/バッファ問題であることの手がかりだった。

**プロセス上の改善点**:
- 推測ベースで対症療法（Lock → echo）を重ねるより、早期にライブラリのソースを
  精読すべきだった。`execute()` のループ構造を読めば retries=0 の問題は一目だった。

### 次回への改善提案
- シリアル/Modbus 系のトラブルは、まず使用ライブラリの送受信ループとバッファ管理を
  ソースで確認することを最優先にする。
- `retries`/`timeout` のような通信パラメータは、制御ループ予算（50ms）との両立を
  別途検証する（現状は desync 回避を優先。定常状態でのタイムアウト発生率は要観測）。
