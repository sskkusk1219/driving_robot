# 設計: Modbus RTU desync カスケードの修正

## 原因分析（pymodbus 3.13.0 ソースを精読して確定）

### 真の根本原因: `retries=0`

`pymodbus/transaction/transaction.py` の `execute()`:

```python
while count_retries <= self.retries:
    self.recv_buffer = b""          # ← トランザクション開始/再送のたびにクリア
    self.response_future = asyncio.Future()
    self.pdu_send(request)
    try:
        response = await asyncio.wait_for(
            self.response_future, timeout=self.comm_params.timeout_connect)
        ...
    except asyncio.TimeoutError:
        count_retries += 1          # ← retries>0 のときだけ再送される
```

- `retries=0` だと、応答が 1 回でもタイムアウトすると即 `ModbusIOException`。
- タイムアウト後に遅れて届いた応答バイトが **OS シリアル受信バッファ** に残る。
- pymodbus は自身の `recv_buffer` はクリアするが OS バッファはクリアしない。
- 次トランザクションの応答先頭にこの残渣が混入 → RTU フレーマーが
  バイト数フィールドを誤読（例: `0x90`=144 バイト待ち）→ タイムアウト →
  以降のトランザクションが永続的に失敗する（desync カスケード）。

→ `retries>0` にすれば、タイムアウト時に再送＋`recv_buffer` クリアが走り、
  単発の遅延・分割応答から自動復旧できる。

### 寄与因子: timeout が短い（0.1s）

`timeout=self.comm_params.timeout_connect`（serial.py で `timeout` がそのまま割当）。
FTDI のレイテンシタイマー（デフォルト 16ms）により応答がバイト単位で分割到着し、
0.1s 以内に応答が揃わずタイムアウトする余地があった。

### 残渣バイトの除去

OS 受信バッファに残った遅延応答バイトはトランザクション開始前に明示的に
フラッシュしないと残り続ける。`SerialTransport.sync_serial.reset_input_buffer()` で除去する。

### 採用しなかった手段とその理由

- **`handle_local_echo=True`（不採用 / 一度入れて撤回）**:
  `pymodbus/transport/transport.py` の `send()` は `is_sync` に関係なく
  `sent_buffer += data` を実行し、受信時に「先頭が送信バイトと一致したら破棄」する。
  **FC05（コイル書き込み）の正常応答はリクエストと完全に同一バイト列**のため、
  非エコー機ではこのアダプタで本物の応答を丸ごとエコー誤認して破棄し、
  `enable_modbus_control` すら応答なしになった。→ このハードはエコーしないと確定し撤回。

## 修正方針（`src/infra/actuator_driver.py` に閉じる）

1. `__init__` のデフォルトを `timeout=0.3`, `retries=3` に変更（主たる修正）。
2. OS 受信バッファのフラッシュ `_flush_input_buffer()` を追加し、
   各トランザクション開始前に呼ぶ（残渣除去による desync 遮断）。
3. `_bus_op()` コンテキストマネージャ（`asyncio.Lock` + フラッシュ）で
   「フラッシュ→トランザクション」を不可分化し、全 Modbus 操作をラップ。
   - pymodbus 自体も `execute()` 内に独自ロックを持つが、フラッシュと送信の間に
     別コルーチンのトランザクションが割り込むのを防ぐ目的でアプリ側ロックを持つ。

## 再送の冪等性確認

- FC05 コイル（servo_on/off, PMSL, ALRS, HOME）: 同値の再書き込みは冪等。
  原点復帰・アラームリセットは False→True のエッジトリガだが、True 再送では
  新たな立ち上がりエッジが立たないため二重起動しない。
- FC10 move_to_position: 同一目標位置の再送は冪等。
- FC03 読み取り: 副作用なし。

→ `retries=3` による再送は安全。

## 影響範囲

- `src/infra/actuator_driver.py` のみ。
- 制御ループ（DriveLoop）の読み取りも同ドライバ経由だが、timeout 0.3s への増加は
  desync で全滅するより遥かに望ましく、定常状態では再送・フラッシュにより
  タイムアウト自体がほぼ発生しない想定。
