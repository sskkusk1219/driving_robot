"""研究ハーネス（tests.research）の走行 CSV を読む共通部品（分析専用）。

走行の指定は「ラベル:トークン」か「トークン」。トークンは CSV のパス・`20261109_130454`・`130454`
（`drive_log_real_*<トークン>*.csv` の最新。`_lap` は除く）。`stub:125814` のように `stub:` を前に付けると
スタブのログを探す。走行時の設定は CSV と同じ名前の .yaml（main が保存）から読む。
"""

from __future__ import annotations

import glob
import sys
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[4]
RES = REPO / "tests/research/results"
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))


def find_csv(token: str) -> Path:
    kind = "real"
    if token.startswith("stub:"):
        kind, token = "stub", token[5:]
    p = Path(token)
    if p.suffix == ".csv" and p.exists():
        return p
    hits = [h for h in sorted(glob.glob(str(RES / f"drive_log_{kind}_*{token}*.csv"))) if "_lap" not in h]
    if not hits:
        sys.exit(f"CSV が見つからない: {token}（{RES} の drive_log_{kind}_*）")
    return Path(hits[-1])


def parse_run(arg: str) -> tuple[str, Path]:
    """「ラベル:トークン」→ (ラベル, CSV)。ラベルが無ければ時刻（HHMMSS）をラベルにする。"""
    if ":" in arg and not arg.startswith("stub:") and not Path(arg).exists():
        lab, tok = arg.split(":", 1)
    else:
        lab, tok = "", arg
    path = find_csv(tok)
    return (lab or path.stem.split("_")[-1]), path


def load(path: Path) -> pd.DataFrame:
    """モード走行の行だけ（section 列があれば MODE_DRIVE、無ければ mode_time_s がある行）。"""
    d = pd.read_csv(path, low_memory=False)
    if "section" in d and (d.section == "MODE_DRIVE").any():
        d = d[d.section == "MODE_DRIVE"]
    else:
        d = d[d.mode_time_s.notna()]
    return d.reset_index(drop=True)


def run_config(path: Path):
    """走行時の設定（CSV と同じ名前の .yaml）。無ければ本体 yaml（その旨を stderr に出す）。"""
    from tests.research.config import load_config

    y = path.with_suffix(".yaml")
    if not y.exists():
        print(f"注意: {y.name} が無いので本体 yaml を使う", file=sys.stderr)
        y = REPO / "tests/research/config_testVehicle.yaml"
    return load_config(y)


def dt_of(d: pd.DataFrame) -> float:
    return float(np.median(np.diff(d.mode_time_s.to_numpy())))


def runs_of(mask: np.ndarray) -> list[tuple[int, int]]:
    """True が続く区間 [i0, i1)（行番号）の一覧。"""
    m = np.asarray(mask, dtype=bool)
    if not m.any():
        return []
    e = np.flatnonzero(np.diff(np.r_[0, m.astype(int), 0]))
    return list(zip(e[::2], e[1::2]))


def col(d: pd.DataFrame, name: str, fill: float = 0.0) -> np.ndarray:
    """列が無い（古いログ）ときは fill の配列。"""
    if name in d:
        return d[name].fillna(fill).to_numpy()
    return np.full(len(d), fill)
