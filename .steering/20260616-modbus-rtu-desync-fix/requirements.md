# 要求: Modbus RTU 通信エラー（No response / desync カスケード）の調査と修正

## 背景

IAI P-CON-CB アクチュエータ（アクセル/ブレーキ軸）への Modbus RTU 通信で、
WebUI からの初期化（`/api/v1/drive/initialize`）およびキャリブレーション
テスト（`tests/hardware/test_calibration.py`）実行中に次のエラーが多発した。

```
pymodbus.exceptions.ModbusIOException:
  Modbus Error: [Input/Output] No response received after 0 retries, continue with next request
>>>>> recv: 0x1a 0x7a 0x98 extra data: 0x1 0x3 0x4 0x0 0x0 0x2
>>>>> extra:  unexpected data: ...
```

- 一度エラーが出始めると以降のトランザクションが連鎖的に全滅する（desync カスケード）。
- 応答値そのもの（電流 538、DSS1=0x3280 など）は正しく届いており、完全断線ではない。
- 発生箇所: `src/infra/actuator_driver.py` の各 Modbus 操作（FC03 読み取り / FC05・FC10 書き込み）。

## 要求内容

1. エラーの根本原因を特定する。
2. 実機（`/dev/actuator_accel`, `/dev/actuator_brake`、FTDI USB-RS485、38400bps）で
   初期化〜キャリブレーション（両軸ゼロ/フル点設定）が通信エラーなく完走すること。
3. 修正は `actuator_driver.py` に閉じ、制御ループ（DriveLoop）の挙動を壊さないこと。

## 制約

- pymodbus 3.13.0 / `AsyncModbusSerialClient` / `FramerType.RTU` を継続使用。
- 各軸は独立した RS-485 バス（両軸とも slave_id=1）。
- ハードウェア配線の変更は伴わない（ソフトウェア側で吸収する）。

## 完了条件

- `tests/hardware/test_calibration.py` が `No response received` を出さず最後まで完走する。
- 修正内容と原因が振り返りに記録されている。
