"""CAN 車速を CSV へ記録するスクリプト（優先E: 0.57Hz 変動の出所切り分け用）。

**アクチュエータには一切触らない。** CAN を読むだけなので、ダイナモ側だけで車速を維持した
状態（ペダル無操作）の車速信号そのものを記録できる。

背景: 120km/h プラトーで偏差 std 0.57〜0.72km/h・主要周波数 0.57Hz の鋸歯が観測されており、
これは第2ラウンドから同じ大きさで存在する＝制御が作ったものではない可能性が高い。
`CANReader.read_speed` は補間も外挿もせず最新値をそのまま返すので、この鋸歯が
「車速信号の実効更新レート」由来なのか「ダイナモ／車両の実挙動」なのかを切り分ける必要がある。
p95 0.4km/h の物理的な達成可能性に直結する（docs/Problem/引き継ぎ20260909.md 優先E）。

記録する列:
    t_s          記録開始からの経過秒（単調時計）
    speed_kmh    read_speed() が返した値（制御が実際に見る値）
    frame_age_s  その時点でのキャッシュ経過時間（受信からの経過。0 に近いほど新鮮）
    updated      このサンプルで受信時刻が更新されたか（1=新フレーム）

`updated` 列があると、実効更新レート（何 Hz で新しい値が来ているか）を後から数えられる。
DBC には送信周期の属性が無いため、周期は実測するしかない。

使い方:
    .venv/bin/python -m scripts.record_can_speed --seconds 60 --out /tmp/can_120.csv

解析の目安:
    - `updated` が立つ間隔 → 車速信号の実効更新レート。制御周期 50ms より遅ければ
      read_speed は同じ値を返し続け、車速が階段状に見える（＝鋸歯の説明になる）。
    - `speed_kmh` の std → ペダル無操作時の計測床。制御ありの 0.57〜0.72km/h と同等なら
      計測／ダイナモ由来と判断できる。
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(_PROJECT_ROOT))

from src.infra.can_reader import CANReader  # noqa: E402
from src.infra.settings import CanSettings, load_settings  # noqa: E402

# 記録周期 [s]。制御ループ（50ms）より細かく取り、受信の立ち上がりを取りこぼさない。
_SAMPLE_DT_S = 0.01


def _load_can_settings() -> CanSettings:
    settings_path = _PROJECT_ROOT / "config" / "settings.toml"
    try:
        return load_settings(settings_path).can
    except FileNotFoundError:
        print(f"[警告] {settings_path} が見つかりません。デフォルト設定を使用します。")
        return CanSettings()


async def _record(reader: CANReader, seconds: float, out_path: Path) -> int:
    loop = asyncio.get_running_loop()
    started = loop.time()
    prev_updated_at: float | None = None
    rows = 0
    with out_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["t_s", "speed_kmh", "frame_age_s", "updated"])
        while loop.time() - started < seconds:
            now = loop.time()
            try:
                speed = await reader.read_speed()
            except TimeoutError:
                print("[警告] CAN 無音（キャッシュが古い）。記録を中断します。")
                break
            updated_at = reader.latest_updated_at
            age = 0.0 if updated_at is None else max(0.0, now - updated_at)
            is_new = int(updated_at is not None and updated_at != prev_updated_at)
            prev_updated_at = updated_at
            writer.writerow([f"{now - started:.4f}", f"{speed:.4f}", f"{age:.4f}", is_new])
            rows += 1
            await asyncio.sleep(_SAMPLE_DT_S)
    return rows


async def _main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=60.0, help="記録時間 [s]")
    ap.add_argument("--out", type=Path, required=True, help="出力 CSV パス")
    args = ap.parse_args()

    can = _load_can_settings()
    reader = CANReader(
        interface=can.interface,
        channel=can.channel,
        bitrate=can.bitrate,
        dbc_path=str(_PROJECT_ROOT / can.dbc_path),
    )
    print(f"CAN 接続中... interface={can.interface} channel={can.channel}")
    try:
        await reader.connect()
    except Exception as exc:  # noqa: BLE001 - 接続失敗の理由をそのまま見せる
        print(f"[エラー] CAN 接続失敗: {exc}")
        return
    print(f"接続完了。{args.seconds:.0f}s 記録します（アクチュエータは操作しません）。")

    try:
        rows = await _record(reader, args.seconds, args.out)
    finally:
        await reader.close()
    print(f"{rows} サンプルを {args.out} に書き出しました。")


if __name__ == "__main__":
    asyncio.run(_main())
