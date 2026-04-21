# 設計: ハードウェア抽象層

## ActuatorDriver

### Modbus レジスタマッピング（MJ0162-12A 第12版）

#### FC03 読み取り
| 機能 | アドレス | サイズ | 備考 |
|------|---------|--------|------|
| 現在位置 PNOW | 0x9000-0x9001 | 32bit 符号付き | 単位 0.01mm |
| アラームコード ALMC | 0x9002 | 16bit | 0=正常 |
| デバイスステータス1 DSS1 | 0x9005 | 16bit | bit12=SV, bit10=ALMH, bit4=HEND, bit3=PEND |
| 拡張デバイスステータス DSSE | 0x9007 | 16bit | bit5=MOVE |
| 電流値 CNOW | 0x900C-0x900D | 32bit 符号付き | 単位 mA |

#### FC05 コイル書き込み
| 機能 | アドレス | 備考 |
|------|---------|------|
| サーボON SON | 0x0403 | FF00=ON, 0000=OFF |
| アラームリセット ALRS | 0x0407 | エッジ入力、完了後0000に戻す |
| 原点復帰 HOME | 0x040B | DSS1 bit4(HEND)=1 で完了 |

#### FC10 直値移動指令
| 機能 | アドレス | サイズ | 備考 |
|------|---------|--------|------|
| 目標位置 PCMD | 0x9900-0x9901 | 32bit 符号付き | 単位 0.01mm |
| 速度指令 VCMD | 0x9904-0x9905 | 32bit | 単位 mm/s |
| 加減速指令 ACMD | 0x9906 | 16bit | 単位 mm/s² |
| 制御フラグ CTLF | 0x9908 | 16bit | |

### pymodbus 3.x 非同期 API

```python
from pymodbus.client import AsyncModbusSerialClient

client = AsyncModbusSerialClient(
    port=port,
    baudrate=38400,
    bytesize=8,
    parity="N",
    stopbits=1,
    framer="rtu",
)
await client.connect()
```

### 32bit 値の読み書き

pymodbus は 16bit ずつ返すため、上位・下位ワードを結合して符号付き32bitに変換:
```python
hi, lo = regs[0], regs[1]
raw = (hi << 16) | lo
# 符号付きに変換
if raw >= 0x80000000:
    raw -= 0x100000000
```

## CANReader

### python-can + cantools

```python
import can
import cantools

db = cantools.database.load_file("config/can/dynamometer.dbc")
bus = can.Bus(interface="kvaser", channel=0)
msg = bus.recv(timeout=0.1)
decoded = db.decode_message(msg.arbitration_id, msg.data)
speed = decoded["VehicleSpeed"]  # km/h
```

- `cantools` がインストール済みでない場合は、DBC デコードなしで生バイト解析にフォールバック
- `config/can/*.dbc` ファイルが存在しない場合は NotImplementedError を送出

### 非同期ラッパー

python-can の Bus は同期 API のため、`asyncio.get_event_loop().run_in_executor()` でスレッドプールに委譲。

## GPIOMonitor

### RPi.GPIO

```python
import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(17, GPIO.FALLING, callback=_emergency_cb, bouncetime=50)
GPIO.add_event_detect(27, GPIO.FALLING, callback=_ac_loss_cb, bouncetime=50)
```

- コールバックは非同期関数を受け取り、`asyncio.run_coroutine_threadsafe` でイベントループに投入

## テスト方針

- RPi.GPIO・pymodbus・python-can はすべてモック化
- `unittest.mock.patch` で各ライブラリの import をモック
- AsyncMock で非同期メソッドをテスト
