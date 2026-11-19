"""実機・スタブ走行用の yaml のコピーを本体 yaml から作る（読み取り専用にする）。

使い方（リポジトリ直下）:
    .venv/bin/python .claude/skills/research-run-analysis/scripts/make_run_yaml.py \\
        --out-dir <scratchpad> --prefix s6e --modes W,U,US06 \\
        --set coast_assist_accel_no_play=true --set coast_fill_brake_wait_floor=true

- 毎回本体（tests/research/config_testVehicle.yaml）から作る（前のコピーから作ると、決定とずれた値が残る）
- --set の KEY は yaml の中で 1 回だけ出るキー名（節は問わない）。行のコメントは消して値だけ書き換える。
  0 回・2 回以上ならエラーで止まる（打ち間違い・別の節の同名キーを書き換える事故を防ぐ）
- モードは modes.wltp_mode_name を書き換える。W・U・US06・AMA の略記か、モード名そのもの
- 作ったファイルは chmod a-w（走行を任せたサブエージェントが上書きしてモードを取り違えた事故の対策）
- 最後に本体との差（コメントを除く）を出す。--set した行とモードの行以外に差が無いことを見る
"""

from __future__ import annotations

import argparse
import difflib
import os
import re
import stat
from pathlib import Path

from _runs import REPO

MAIN = REPO / "tests/research/config_testVehicle.yaml"
MODES = {"W": "01_WLTP_Low,Mid,Hi,ExHi", "U": "07_UDDS", "US06": "09_US06", "AMA": "12_AMA_short"}
TAGS = {"01_WLTP_Low,Mid,Hi,ExHi": "wltp", "07_UDDS": "udds", "09_US06": "us06", "12_AMA_short": "ama"}
LINE = re.compile(r"^(\s*)([A-Za-z_][\w]*):\s*([^#]*?)(\s*#.*)?$")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--prefix", required=True, help="ファイル名の頭（例 s6e → s6e_wltp.yaml）")
    ap.add_argument("--modes", default="W,U,US06")
    ap.add_argument("--set", action="append", default=[], help="KEY=VALUE（何度でも）")
    a = ap.parse_args()
    src = MAIN.read_text(encoding="utf-8").split("\n")
    sets = dict(kv.split("=", 1) for kv in a.set)
    out_dir = Path(a.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    for m in a.modes.split(","):
        mode = MODES.get(m.strip(), m.strip())
        hit = dict.fromkeys(sets, 0)
        nmode = 0
        out, section = [], ""
        for line in src:
            if re.match(r"^[A-Za-z_]\w*:", line):
                section = line.split(":")[0]
            g = LINE.match(line)
            if g and g.group(2) in sets:
                line = f"{g.group(1)}{g.group(2)}: {sets[g.group(2)]}"
                hit[g.group(2)] += 1
            if g and g.group(2) == "wltp_mode_name" and section == "modes":
                line = f'{g.group(1)}wltp_mode_name: "{mode}"'
                nmode += 1
            out.append(line)
        bad = {k: n for k, n in hit.items() if n != 1}
        if bad or nmode != 1:
            raise SystemExit(f"書き換えの数が 1 でない: {bad}・モードの行 {nmode}（本体 yaml を確かめる）")
        p = out_dir / f"{a.prefix}_{TAGS.get(mode, re.sub(r'\W+', '_', mode).lower())}.yaml"
        if p.exists():
            os.chmod(p, p.stat().st_mode | stat.S_IWUSR)
        p.write_text("\n".join(out), encoding="utf-8")
        os.chmod(p, p.stat().st_mode & ~(stat.S_IWUSR | stat.S_IWGRP | stat.S_IWOTH))
        strip = [ln.split("#")[0].rstrip() for ln in src if ln.split("#")[0].strip()]
        new = [ln.split("#")[0].rstrip() for ln in out if ln.split("#")[0].strip()]
        diff = [d for d in difflib.unified_diff(strip, new, lineterm="", n=0) if d[:1] in "+-" and d[:3] not in ("+++", "---")]
        print(f"{p}（読み取り専用）: 本体との差 {len(diff) // 2} 行")
        for d in diff:
            print("   " + d.strip())


if __name__ == "__main__":
    main()
