"""非常停止スイッチ（GPIO17）ハードウェア動作確認スクリプト。

接続: GPIO17 (物理ピン11) ─── NC接点スイッチ ─── GND (物理ピン9)
論理: LOW=通常（NC接点が閉じてGNDに落ちている）
      HIGH=非常停止（NC接点が開いてプルアップが有効になる）

NC接点はフェイルセーフ設計: ケーブル断線時も HIGH になり停止する。

Raspberry Pi 5 は lgpio (システムパッケージ) を使用。
実行方法:
    $ /usr/bin/python3 tests/hardware/test_emergency_stop.py
"""

import time

import lgpio

CHIP = 0
EMERGENCY_PIN = 17
DEBOUNCE_US = 50_000  # 50ms チャタリング除去


def on_edge(chip: int, gpio: int, level: int, timestamp: int) -> None:  # noqa: ARG001
    if level == 1:
        print(f"[GPIO{gpio}] 非常停止スイッチ 押下検知 (HIGH) ← 停止トリガー")
    else:
        print(f"[GPIO{gpio}] 非常停止スイッチ 解除検知 (LOW)  ← 通常状態に復帰")


def main() -> None:
    h = lgpio.gpiochip_open(CHIP)
    lgpio.gpio_claim_alert(h, EMERGENCY_PIN, lgpio.BOTH_EDGES, lgpio.SET_PULL_UP)
    lgpio.gpio_set_debounce_micros(h, EMERGENCY_PIN, DEBOUNCE_US)

    # デバウンス安定待ち（起動直後のノイズを読み飛ばす）
    time.sleep(0.1)

    current = lgpio.gpio_read(h, EMERGENCY_PIN)
    state_str = "LOW（通常: NC接点が閉じている）" if current == 0 else "HIGH（非常停止中: NC接点が開いている）"
    print(f"GPIO{EMERGENCY_PIN} 初期状態: {state_str}")
    print("スイッチを押すと HIGH になります。終了は Ctrl+C")
    print("-" * 40)

    cb = lgpio.callback(h, EMERGENCY_PIN, lgpio.BOTH_EDGES, on_edge)

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        cb.cancel()
        lgpio.gpiochip_close(h)
        print("\nGPIO クリーンアップ完了")


if __name__ == "__main__":
    main()
