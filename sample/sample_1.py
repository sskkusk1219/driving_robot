from pymodbus.client import ModbusSerialClient
import time

# --- 通信設定 ---
# ポート名やボーレートはPCONのパラメーター設定（デフォルト38400等）に合わせてください
client = ModbusSerialClient(
    port = "/dev/ttyUSB0",
    baudrate = 38400,
    parity = 'N',
    stopbits = 1,
    bytesize = 8,
    timeout = 1
)

SLAVE_ID = 1  # 軸No.0の場合、アドレスは 軸No.+1 = 1 [1]

def convert_to_32bit(val):
    """32ビット値を上位ワードと下位ワードのリストに変換 [2]"""
    val = int(val)
    # 負の数の場合（2の補数処理） [2]
    if val < 0:
        val = (1 << 32) + val
    high = (val >> 16) & 0xFFFF
    low = val & 0xFFFF
    return high, low

def pcon_control():
    # if not client.connect():
    #     print("接続失敗")
    #     return

    # try:
        # 1. Modbus操作権の有効化 (PMSL: 0427H) [3], [4]
        # PIO入力を無効化しModbus指令を優先させます
        client.write_coil(address=0x0427, value=True,  device_id=SLAVE_ID)
        print("Modbus操作権有効")

        # --- アラームリセット ---
        # 1. リセット実行 (0407H に True/FF00H を書込み)
        client.write_coil(address=0x0407, value=True, device_id=SLAVE_ID)
        # 2. 通常状態に戻す (0407H に False/0000H を書込み)
        client.write_coil(address=0x0407, value=False, device_id=SLAVE_ID)
        print("アラームリセット完了（要因未解消の場合は再発します）")

        # 2. サーボON (SON: 0403H) [3], [5]
        client.write_coil(address=0x0403, value=True, device_id=SLAVE_ID)
        print("サーボON送信")
        time.sleep(1) # サーボON完了を待機 [6]

        # 3. 原点復帰 (HOME: 040BH) [3], [7]
        # エッジ(0→1)が必要なため一度Falseを書いてからTrueを書く [8]
        client.write_coil(address=0x040B, value=False, device_id=SLAVE_ID)
        client.write_coil(address=0x040B, value=True, device_id=SLAVE_ID)
        print("原点復帰開始")
        
        # 原点復帰完了(HEND)待ち (DSS1: 9005Hのビット4) [9], [10]
        while True:
            res = client.read_holding_registers(address=0x9005, count=1, device_id=SLAVE_ID)
            if not res.isError() and (res.registers[0] >> 4) & 1:
                print("原点復帰完了")
                break
            time.sleep(0.5)

        # 4. 絶対位置移動 (50.00mmの位置へ移動) [11], [12]
        # 9900H〜9908Hを一括書き込み
        target_mm = 10.00
        speed_mms = 100.00
        p_high, p_low = convert_to_32bit(target_mm * 100) # 0.01mm単位 [13]
        v_high, v_low = convert_to_32bit(speed_mms * 100) # 0.01mm/s単位 [14]
        
        # ペイロード: PCMD(2), INP(2), VCMD(2), ACMD(1), PPOW(1), CTLF(1)
        # CTLF 0x0000 = 絶対位置移動 [15]
        payload = [p_high, p_low, 0, 10, v_high, v_low, 30, 0, 0x0000]
        client.write_registers(address=0x9900, values=payload, device_id=SLAVE_ID)
        print(f"絶対移動: {target_mm}mmへ")
        time.sleep(1)

        # 5. 相対位置移動 (現在位置から+10.00mm移動) [15], [16]
        # CTLFのビット3(INC)を1に設定 (0x0008)
        relative_mm = 10.00
        p_high, p_low = convert_to_32bit(relative_mm * 100) # +10mm
        payload_rel = [p_high, p_low, 0, 10, v_high, v_low, 30, 0, 0x0008]
        client.write_registers(address=0x9900, values=payload_rel, device_id=SLAVE_ID)
        print(f"相対移動: +{relative_mm}mm")
        time.sleep(1)

        # 6. 現在位置、電流値、エラーの監視 (9000H〜) [17], [18]
        print("\n--- 監視開始 (5回) ---")
        for _ in range(5):
            # 9000Hから14レジスター分を一括読取り (位置〜電流値)
            res = client.read_holding_registers(address=0x9000, count=14, device_id=SLAVE_ID)
            if not res.isError():
                regs = res.registers
                # 現在位置 (9000H-9001H)
                pos_raw = (regs[0] << 16) | regs[1]
                if pos_raw & 0x80000000: pos_raw -= 0x100000000 # 符号付き処理 [2]
                
                # アラームコード (9002H) [22]
                alarm_code = regs[2]
                
                # デバイスステータス1 (9005H)
                is_heavy_alarm = (regs[5] >> 10) & 1
                
                # 電流値 (900CH-900DH) [26]
                current_ma = (regs[12] << 16) | regs[13]

                print(f"位置:{pos_raw/100:.2f}mm | 電流:{current_ma}mA | アラーム:{alarm_code:03X} | 重故障:{is_heavy_alarm}")
            
            time.sleep(1)

        # --- サーボOFF ---
        # 0403H に False/0000H を書き込んでサーボを落とす
        client.write_coil(address=0x0403, value=False, device_id=SLAVE_ID)
        print("サーボOFF指令送信")

    # except Exception as e:
    #     print(f"エラー: {e}")
    # finally:
    #     client.close()

if __name__ == "__main__":
    pcon_control()