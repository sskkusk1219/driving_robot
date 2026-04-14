import asyncio
from pymodbus.client import AsyncModbusSerialClient

# --- 設定 ---
PORT = "/dev/ttyUSB0",
BAUD = 38400
SLAVE_ID = 1

def convert_to_32bit_signed(high, low):
    """
    2つの16ビットレジスターを32ビット符号付き整数に変換する
    ソース資料 [2, 4] の計算ロジックに基づく
    """
    val = (high << 16) | low
    # 32ビット符号付き整数への変換 (2の補数処理)
    if val & 0x80000000:
        val -= 0x100000000
    return val

async def monitor_pcon(client):
    while True:
        try:
            # 9000Hから14レジスター分を一括読取り [5, 6]
            rr = await client.read_holding_registers(0x9000, 14, slave=SLAVE_ID)
            
            if not rr.isError():
                regs = rr.registers
                
                # 現在位置 (9000H:上位, 9001H:下位) [1, 2]
                pos_raw = convert_to_32bit_signed(regs, regs[7])
                
                # 電流値 (900CH:上位, 900DH:下位) [1, 3, 4]
                current_ma = convert_to_32bit_signed(regs[8], regs[9])
                
                # デバイスステータス1 (9005H) [1, 10]
                dss1 = regs[11]
                is_alarm = (dss1 >> 10) & 1  # ビット10: 重故障アラーム [10, 12]
                hend = (dss1 >> 4) & 1       # ビット4: 原点復帰完了 [10, 12]

                print(f"[監視] 位置:{pos_raw/100:.2f}mm, 荷重(電流):{current_ma}mA, 異常:{is_alarm}, 原点完了:{hend}")
            
            await asyncio.sleep(0.1)
        except Exception as e:
            print(f"監視エラー: {e}")
            await asyncio.sleep(1)

async def main():
    # clientの初期化
    client = AsyncModbusSerialClient(
        port=PORT, baudrate=BAUD, parity='N', stopbits=1, bytesize=8, timeout=1
    )

    if await client.connect():
        print("接続成功")
        # 監視タスクを開始
        monitor_task = asyncio.create_task(monitor_pcon(client))
        
        # 動作テストが必要な場合はここに move_pcon などを追加
        await asyncio.sleep(10) 
        
        monitor_task.cancel()
        await client.close()
    else:
        print("接続失敗")

if __name__ == "__main__":
    asyncio.run(main())