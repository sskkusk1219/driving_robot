"""走行ログ CSV から指定したパターン番号の行を除いた CSV を書く小さな CLI。

2026-10-17 段2o-8: G ランプの 2 周目（例: 200434 のパターン 9・10）を抜いたログで
`--retrain` し、2 周目が要るかを比べるために使う。標準ライブラリの csv のみ。

使い方:
    python -m tests.research.drop_patterns <csv> --drop 9,10 [--out PATH]

`pattern` 列は "<番号>:<名前>" の形（例 "9:RAMP_0.045G"）。先頭の番号が --drop に
含まれる行を除く。他の行・ヘッダはそのまま。
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path


def pattern_number(label: str) -> int | None:
    """`"9:xxx"` → 9。番号が無い（空・形式違い）なら None。"""
    head = label.split(":", 1)[0].strip()
    return int(head) if head.isdigit() and ":" in label else None


def summarize(rows: list[dict[str, str]]) -> list[dict[str, object]]:
    """パターン値ごと（出現順）の開始・終了 elapsed_s、秒数、最高速。"""
    out: dict[str, dict[str, object]] = {}
    for r in rows:
        p = r.get("pattern", "")
        if not p:
            continue
        try:
            t = float(r["elapsed_s"])
        except (KeyError, ValueError):
            continue
        try:
            v = float(r.get("actual_speed_kmh", "") or "nan")
        except ValueError:
            v = float("nan")
        s = out.setdefault(p, {"pattern": p, "start": t, "end": t, "vmax": float("-inf")})
        s["end"] = t
        if v == v and v > s["vmax"]:  # type: ignore[operator]
            s["vmax"] = v
    return list(out.values())


def drop_rows(
    rows: list[dict[str, str]], drop: set[int]
) -> list[dict[str, str]]:
    return [r for r in rows if pattern_number(r.get("pattern", "")) not in drop]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="指定したパターン番号の行を除いた CSV を書く")
    ap.add_argument("csv", type=Path)
    ap.add_argument("--drop", required=True, help="除くパターン番号（カンマ区切り。例 9,10）")
    ap.add_argument("--out", type=Path, default=None, help="出力先（既定: <入力>_lap1.csv）")
    args = ap.parse_args(argv)
    drop = {int(x) for x in args.drop.split(",") if x.strip()}
    with args.csv.open(newline="") as f:
        reader = csv.DictReader(f)
        header = list(reader.fieldnames or [])
        rows = list(reader)
    if "pattern" not in header:
        print("エラー: pattern 列がありません", file=sys.stderr)
        return 2
    print(f"{'pattern':<28}{'start_s':>9}{'end_s':>9}{'dur_s':>8}{'vmax_kmh':>10}  drop")
    found: set[int] = set()
    for s in summarize(rows):
        n = pattern_number(str(s["pattern"]))
        mark = "DROP" if n in drop else ""
        if n in drop:
            found.add(n)
        print(
            f"{s['pattern']!s:<28}{s['start']:>9.1f}{s['end']:>9.1f}"
            f"{s['end'] - s['start']:>8.1f}{s['vmax']:>10.1f}  {mark}"  # type: ignore[operator]
        )
    if found != drop:
        print(f"エラー: 見つからないパターン番号 {sorted(drop - found)}", file=sys.stderr)
        return 2
    kept = drop_rows(rows, drop)
    out = args.out or args.csv.with_name(f"{args.csv.stem}_lap1.csv")
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=header, lineterminator="\n")
        w.writeheader()
        w.writerows(kept)
    print(f"\n{len(rows)} 行 → {len(kept)} 行（{len(rows) - len(kept)} 行除去）: {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
