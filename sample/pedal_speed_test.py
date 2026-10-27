"""ペダルアクチュエータの実際の速さを測る（原点 ⇔ 上限開度 −5%）。

各軸を片方ずつ、速度指令（VCMD）を変えて原点から目標まで動かし、現在位置を連続で
読んで実際の速さを出す。戻りは 100 mm/s・0.3G。もう片方の軸は原点のまま。
加減速度（ACMD、0.01G 単位）も引数で変えられる。

実行（リポジトリの最上位で。WebUI・走行が動いていないこと）:
    PYTHONPATH=. .venv/bin/python sample/pedal_speed_test.py
    PYTHONPATH=. .venv/bin/python sample/pedal_speed_test.py \
        --accel-speeds 225 --brake-speeds 190 --acmd 30

出力: 1 回ごとの「動き出し・着いた時刻・20〜80% 区間の平均の速さ・最大の速さ・アラーム」と、
生データの CSV（--out、既定 pedal_speed_test.csv）。2026-10-27 に Claude が書いて実機で確認。
"""

import argparse
import asyncio
import csv
import time
from pathlib import Path

from tests.research.actuator_driver import ActuatorDriver
from tests.research.app_settings import load_settings
from tests.research.vehicle import opening_to_pulse

ACCEL_TARGET_PCT = 61.0 - 5.0  # max_accel_opening_pct − 5
BRAKE_TARGET_PCT = 47.3 - 5.0  # max_brake_opening_pct − 5
# 仕様の最高速度（RCP6-ROD 1.2.1、高出力設定なし）。これより速い指令は受け付けない
ACCEL_SPEED_LIMIT = 250
BRAKE_SPEED_LIMIT = 210
# 仕様の加減速度別可搬質量の表（1.2.2、高出力設定なし・水平）の最大 0.7G
ACMD_LIMIT = 70
RETURN_SPEED = 100
RETURN_ACMD = 30
TIMEOUT_S = 3.0


async def track(drv: ActuatorDriver, target: int) -> list[tuple[float, int]]:
    """目標に着くまで (経過秒, 位置 [0.01 mm]) を読み続ける。"""
    t0 = time.perf_counter()
    rows = []
    while True:
        pos = await drv.read_position()
        t = time.perf_counter() - t0
        rows.append((t, pos))
        if abs(pos - target) <= 5 and t > 0.05:
            break
        if t > TIMEOUT_S:
            break
    return rows


def summarize(rows: list[tuple[float, int]], start: int, target: int) -> dict:
    dist = abs(target - start) / 100.0  # mm
    t_arrive = next((t for t, p in rows if abs(p - target) <= 20), None)  # 0.2 mm 以内
    t_move = next((t for t, p in rows if abs(p - start) >= 20), None)

    def t_at(frac: float) -> float | None:
        lvl = start + (target - start) * frac
        for (ta, pa), (tb, pb) in zip(rows, rows[1:]):
            if (pa - lvl) * (pb - lvl) <= 0 and pa != pb:
                return ta + (tb - ta) * (lvl - pa) / (pb - pa)
        return None

    t20, t80 = t_at(0.2), t_at(0.8)
    mid = dist * 0.6 / (t80 - t20) if t20 is not None and t80 is not None else None
    peak = 0.0
    for (ta, pa), (tb, pb) in zip(rows, rows[2:]):
        if tb > ta:
            peak = max(peak, abs(pb - pa) / 100.0 / (tb - ta))
    return dict(dist_mm=dist, t_move=t_move, t_arrive=t_arrive, mid_mm_s=mid, peak_mm_s=peak,
                n=len(rows))


def fmt(x: float | None, nd: int = 3) -> str:
    return "—" if x is None else f"{x:.{nd}f}"


async def run_axis(name: str, drv: ActuatorDriver, target_pct: float, speeds: list[int],
                   acmd: int, log: list) -> None:
    target = opening_to_pulse(target_pct)
    for v in speeds:
        start = await drv.read_position()
        await drv.move_to_position(target, speed_mm_s=v, accel=acmd)
        rows = await track(drv, target)
        s = summarize(rows, start, target)
        alarm = await drv.is_alarm_active()
        print(f"{name} 速度 {v:>3} mm/s・加減速度 {acmd / 100:.2f}G: 距離 {s['dist_mm']:.1f} mm  "
              f"動き出し {fmt(s['t_move'])} s  着いた {fmt(s['t_arrive'])} s  "
              f"20〜80% 平均 {fmt(s['mid_mm_s'], 1)} mm/s  最大 {s['peak_mm_s']:.1f} mm/s  "
              f"(読み {s['n']} 回)  アラーム {'あり' if alarm else 'なし'}", flush=True)
        for t, p in rows:
            log.append((name, v, acmd, "out", round(t, 4), p))
        if alarm:
            print(f"{name}: アラーム → リセットしてこの軸は中止", flush=True)
            await drv.reset_alarm()
            await drv.servo_on()
            break
        await asyncio.sleep(0.5)
        await drv.move_to_position(0, speed_mm_s=RETURN_SPEED, accel=RETURN_ACMD)
        back = await track(drv, 0)
        for t, p in back:
            log.append((name, v, RETURN_ACMD, "back", round(t, 4), p))
        await asyncio.sleep(0.5)


def speed_list(text: str, limit: int) -> list[int]:
    vals = [int(x) for x in text.split(",") if x.strip()]
    bad = [v for v in vals if not 0 < v <= limit]
    if bad:
        raise SystemExit(f"速度 {bad} は仕様の範囲外（1〜{limit} mm/s）")
    return vals


async def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--accel-speeds", default="100,150,200,250", help="アクセルの速度指令 [mm/s]")
    ap.add_argument("--brake-speeds", default="100,150,200", help="ブレーキの速度指令 [mm/s]")
    ap.add_argument("--acmd", type=int, default=30, help="加減速度 [0.01G]（30 ＝ 0.3G）")
    ap.add_argument("--axis", choices=["both", "accel", "brake"], default="both")
    ap.add_argument("--out", default="pedal_speed_test.csv")
    args = ap.parse_args()
    accel_speeds = speed_list(args.accel_speeds, ACCEL_SPEED_LIMIT)
    brake_speeds = speed_list(args.brake_speeds, BRAKE_SPEED_LIMIT)
    if not 1 <= args.acmd <= ACMD_LIMIT:
        raise SystemExit(f"--acmd {args.acmd} は範囲外（1〜{ACMD_LIMIT}）")

    st = load_settings(Path("config/settings.toml"))

    def make(port: str, axis: str, lead: float) -> ActuatorDriver:
        return ActuatorDriver(
            port=port, slave_id=1, baud_rate=st.serial.baud_rate, timeout=st.serial.timeout_s,
            retries=st.serial.retries, axis_name=axis, lead_mm=lead)

    accel = make(st.serial.accel_port, "accel", st.actuator.accel.lead_mm)
    brake = make(st.serial.brake_port, "brake", st.actuator.brake.lead_mm)
    log: list = []
    try:
        for d in (accel, brake):
            await d.connect()
            await d.enable_modbus_control()
            await d.reset_alarm()
            await d.servo_on()
            await asyncio.sleep(0.2)
            await d.home_return()
        print("両軸 原点復帰 完了", flush=True)
        if args.axis in ("both", "accel"):
            await run_axis("accel", accel, ACCEL_TARGET_PCT, accel_speeds, args.acmd, log)
        if args.axis in ("both", "brake"):
            await run_axis("brake", brake, BRAKE_TARGET_PCT, brake_speeds, args.acmd, log)
    finally:
        for d in (accel, brake):
            try:
                await d.move_to_position(0, speed_mm_s=RETURN_SPEED, accel=RETURN_ACMD)
                await asyncio.sleep(1.0)
                await d.home_return()
            except Exception as e:  # noqa: BLE001
                print(f"後始末で失敗: {e}", flush=True)
            await d.close()
        with open(args.out, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["axis", "vcmd_mm_s", "acmd_001g", "dir", "t_s", "pos_0p01mm"])
            w.writerows(log)
        print(f"保存: {args.out}", flush=True)


if __name__ == "__main__":
    asyncio.run(main())
