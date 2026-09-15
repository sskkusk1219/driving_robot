"""定速階段（CRUISE_HOLD）の保持窓から「定速開度の実測テーブル」を作る。

背景（docs/memo.md「2026-09-15: 段4-2 関門確認 → C6 の骨格を実測テーブルに変更」）:
2次式の骨格は 30〜130 km/h の実測に対して上に凸（頂点 120 km/h 付近）になり、130 km/h 超で
開度が下がる向きに延びてしまう（C6 が外挿を支えるという狙いと逆）。区分線形の実測テーブル
（端は最後の区間の傾きで直線延長）の方が RMSE・外挿とも素直だったため、C6 の骨格をこちらに
差し替える。本モジュールはその実測テーブル（`CruiseCurve`）を作るところまで（C6 本体・
`ff_candidate.py` への組み込みは対象外）。

引用元（判定ロジックを再現。クラス自体は import・実行しない）:
    tests/research/pattern_loop.py の `CruiseStairPattern` / `_advance_cruise_hold` … 定速階段の
    段の前進判定（settle_tol_kmh 以内に settle_s 続いたら保持タイマー開始 → hold_s で次へ、
    step_timeout_s で打ち切り）。本物は `now`（ループの絶対時刻）を使うが、ここでは CSV の
    `elapsed_s` 列をそのまま `now` として使う。

CSV から読み取る材料（`tests/research/drive_log.py` と同じ列・関数）:
    section 列（`SECTION_PATTERN_DRIVE` の行だけ）、phase 列（`CRUISE_HOLD` の行だけ）、
    pattern 列（複数の定速階段パターンがあれば列ごとに独立して段を再現する）、
    elapsed_s・actual_speed_kmh、実開度は `actual_opening(row, "accel")`。

CLI:
    .venv/bin/python -m tests.research.cruise_curve \
        tests/research/results/drive_log_real_<日時>.csv

    段ごとの表（目標車速・終わり方・保持窓の行数・実車速中央値・実開度中央値・保持中の
    |v-目標| 最大・保持中の dv/dt）を出し、実測テーブルの点数を出す。ファイルは書かない
    （図も無い）。CRUISE_HOLD の行が無い CSV は「保持段なし」と表示して exit 1。
"""

from __future__ import annotations

import argparse
import csv
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass
from enum import StrEnum
from pathlib import Path
from typing import Any

import numpy as np

from tests.research.config import DEFAULT_CONFIG_PATH, LearningSection, load_config
from tests.research.debug_process23 import md_table
from tests.research.drive_log import SECTION_PATTERN_DRIVE, actual_opening

PHASE_CRUISE_HOLD = "CRUISE_HOLD"


class HoldOutcome(StrEnum):
    """定速階段 1 段の終わり方。"""

    HELD = "held"  # 保持タイマー開始から hold_s 経過（実測テーブルに使える）
    TIMED_OUT = "timed_out"  # step_timeout_s 経過（保持できないまま打ち切り。捨てる）
    INCOMPLETE = "incomplete"  # CSV の行が尽きた（最高速超え・停車・CSV の途中終了など）


_OUTCOME_JA: dict[HoldOutcome, str] = {
    HoldOutcome.HELD: "保持",
    HoldOutcome.TIMED_OUT: "打ち切り",
    HoldOutcome.INCOMPLETE: "途中終了",
}


@dataclass(frozen=True)
class HoldStep:
    """定速階段 1 段の再現結果。

    Attributes:
        target_kmh: この段の目標車速 [km/h]（`LearningSection.cruise_hold_speeds_kmh` の 1 つ）。
        outcome: 終わり方。
        rows: 保持タイマー開始後の行だけ（`HoldOutcome.HELD` 以外は保持タイマーが動かないまま
            終わっていれば空）。csv.DictReader の 1 行（列名 → 文字列）。
    """

    target_kmh: float
    outcome: HoldOutcome
    rows: tuple[Mapping[str, str], ...]


@dataclass(frozen=True)
class CruiseCurve:
    """定速開度の実測テーブル（区分線形）。車速昇順・重複なし。

    Attributes:
        speeds_kmh: 各段の実車速の中央値 [km/h]（昇順）。
        openings_pct: 各段の実開度（アクセル）の中央値 [%]。
        n_rows: 各段の保持窓の行数。
    """

    speeds_kmh: tuple[float, ...]
    openings_pct: tuple[float, ...]
    n_rows: tuple[int, ...]

    def __post_init__(self) -> None:
        n = len(self.speeds_kmh)
        if len(self.openings_pct) != n or len(self.n_rows) != n:
            raise ValueError(
                "CruiseCurve の speeds_kmh・openings_pct・n_rows は同じ長さにしてください: "
                f"{len(self.speeds_kmh)} / {len(self.openings_pct)} / {len(self.n_rows)}"
            )
        if n < 2:
            raise ValueError(f"CruiseCurve は 2 点以上必要です（{n} 点）")
        if any(a >= b for a, b in zip(self.speeds_kmh, self.speeds_kmh[1:], strict=False)):
            raise ValueError(
                f"CruiseCurve.speeds_kmh は昇順・重複なしにしてください: {self.speeds_kmh}"
            )

    def opening_at(self, v: float | np.ndarray, floor_pct: float) -> float | np.ndarray:
        """車速 `v` [km/h] での定速開度 [%] を返す。

        表の範囲内は区分線形（`np.interp`）。範囲外は端の区間の傾きで直線延長する
        （下側は `speeds_kmh[0:2]`、上側は `speeds_kmh[-2:]` の傾き）。結果は `floor_pct`
        （アクセル不感帯を想定）で下限にする（上限クランプはしない）。`v` は float・
        `np.ndarray` のどちらでもよく、float を渡せば float を返す。
        """
        speeds = np.asarray(self.speeds_kmh, dtype=float)
        openings = np.asarray(self.openings_pct, dtype=float)
        v_arr = np.atleast_1d(np.asarray(v, dtype=float))

        result = np.interp(v_arr, speeds, openings)

        low_slope = (openings[1] - openings[0]) / (speeds[1] - speeds[0])
        below = v_arr < speeds[0]
        result = np.where(below, openings[0] + low_slope * (v_arr - speeds[0]), result)

        high_slope = (openings[-1] - openings[-2]) / (speeds[-1] - speeds[-2])
        above = v_arr > speeds[-1]
        result = np.where(above, openings[-1] + high_slope * (v_arr - speeds[-1]), result)

        result = np.maximum(result, floor_pct)
        if np.ndim(v) == 0:
            return float(result[0])
        return result

    def to_dict(self) -> dict[str, Any]:
        """pkl 保存用のプレーンな dict（list/float/int のみ）。"""
        return {
            "speeds_kmh": [float(v) for v in self.speeds_kmh],
            "openings_pct": [float(v) for v in self.openings_pct],
            "n_rows": [int(v) for v in self.n_rows],
        }

    @classmethod
    def from_dict(cls, d: Mapping[str, Any]) -> CruiseCurve:
        return cls(
            speeds_kmh=tuple(float(v) for v in d["speeds_kmh"]),
            openings_pct=tuple(float(v) for v in d["openings_pct"]),
            n_rows=tuple(int(v) for v in d["n_rows"]),
        )


def _extract_steps_for_group(
    rows: Sequence[Mapping[str, str]], targets: tuple[float, ...], learning: LearningSection
) -> list[HoldStep]:
    """1 パターン分の CRUISE_HOLD 行から `HoldStep` の列を作る（`_advance_cruise_hold` の再現）。"""
    steps: list[HoldStep] = []
    if not rows or not targets:
        return steps

    step_index = 0
    step_started_at = float(rows[0]["elapsed_s"])
    settle_since: float | None = None
    hold_start: float | None = None
    step_rows: list[Mapping[str, str]] = []
    finished_last = False

    for row in rows:
        if step_index >= len(targets):
            break
        now = float(row["elapsed_s"])
        speed = float(row["actual_speed_kmh"])
        target = targets[step_index]

        if hold_start is not None:
            step_rows.append(row)

        within_tol = abs(speed - target) <= learning.cruise_hold_settle_tol_kmh
        if within_tol:
            if settle_since is None:
                settle_since = now
            if hold_start is None and now - settle_since >= learning.cruise_hold_settle_s:
                hold_start = now
                step_rows = [row]  # 保持タイマーが始まったこの行から数える
        elif hold_start is None:
            settle_since = None  # 保持タイマー開始前は連続条件をやり直す

        held_enough = (
            hold_start is not None and now - hold_start >= learning.cruise_hold_hold_s
        )
        timed_out = now - step_started_at >= learning.cruise_hold_step_timeout_s

        if held_enough or timed_out:
            outcome = HoldOutcome.HELD if held_enough else HoldOutcome.TIMED_OUT
            steps.append(HoldStep(target_kmh=target, outcome=outcome, rows=tuple(step_rows)))
            step_index += 1
            if step_index >= len(targets):
                finished_last = True
                break
            step_started_at = now  # 前の段が終わった行の時刻
            settle_since = None
            hold_start = None
            step_rows = []

    if not finished_last and step_index < len(targets):
        steps.append(
            HoldStep(
                target_kmh=targets[step_index],
                outcome=HoldOutcome.INCOMPLETE,
                rows=tuple(step_rows),
            )
        )

    return steps


def extract_hold_steps(
    rows: Iterable[Mapping[str, str]], learning: LearningSection
) -> list[HoldStep]:
    """CSV の全行から、定速階段（PATTERN_DRIVE・phase=CRUISE_HOLD）の段を再現する。

    `rows` は csv.DictReader が返す dict 列（全行を渡してよい。PATTERN_DRIVE 以外・
    CRUISE_HOLD 以外の行はここで捨てる）。1 つの CSV に定速階段パターンが複数あれば
    `pattern` 列ごとに独立して再現する（登場順）。
    """
    targets = tuple(learning.cruise_hold_speeds_kmh)
    if not targets:
        return []

    order: list[str] = []
    groups: dict[str, list[Mapping[str, str]]] = {}
    for row in rows:
        if "section" in row and row["section"] != SECTION_PATTERN_DRIVE:
            continue
        if row.get("phase") != PHASE_CRUISE_HOLD:
            continue
        pattern = row.get("pattern", "")
        if pattern not in groups:
            groups[pattern] = []
            order.append(pattern)
        groups[pattern].append(row)

    steps: list[HoldStep] = []
    for pattern in order:
        steps.extend(_extract_steps_for_group(groups[pattern], targets, learning))
    return steps


def _median(values: Sequence[float]) -> float | None:
    return float(np.median(np.asarray(values, dtype=float))) if values else None


def _linear_slope(xs: Sequence[float], ys: Sequence[float]) -> float | None:
    """最小二乗の傾き（xs に対する ys）。点が 2 未満・xs が全て同じ値なら None。"""
    if len(xs) < 2 or len(set(xs)) < 2:
        return None
    coeffs = np.polyfit(np.asarray(xs, dtype=float), np.asarray(ys, dtype=float), 1)
    return float(coeffs[0])


def build_cruise_curve(
    rows: Iterable[Mapping[str, str]], learning: LearningSection
) -> CruiseCurve:
    """定速階段の保持段（`HoldOutcome.HELD`）から `CruiseCurve` を作る。

    段ごとに保持窓の実車速の中央値・実開度（`actual_opening(row, "accel")`、読めない行は
    除く）の中央値・行数を求め、車速の中央値で昇順に並べる。実開度が 1 つも読めなかった段は
    使わない。使える段が 2 未満なら ValueError。
    """
    hold_steps = [s for s in extract_hold_steps(rows, learning) if s.outcome is HoldOutcome.HELD]

    speeds: list[float] = []
    openings: list[float] = []
    n_rows: list[int] = []
    for step in hold_steps:
        speed_values = [float(r["actual_speed_kmh"]) for r in step.rows]
        opening_values = [
            v for r in step.rows if (v := actual_opening(r, "accel")) is not None
        ]
        speed_median = _median(speed_values)
        opening_median = _median(opening_values)
        if speed_median is None or opening_median is None:
            continue
        speeds.append(speed_median)
        openings.append(opening_median)
        n_rows.append(len(step.rows))

    if len(speeds) < 2:
        raise ValueError(
            f"定速階段の保持段（使える段）が足りません: {len(speeds)} 段（2 段以上必要）。"
            "CRUISE_HOLD の保持窓（打ち切り・途中終了ではない段）を増やしてください。"
        )

    order = sorted(range(len(speeds)), key=lambda i: speeds[i])
    return CruiseCurve(
        speeds_kmh=tuple(speeds[i] for i in order),
        openings_pct=tuple(openings[i] for i in order),
        n_rows=tuple(n_rows[i] for i in order),
    )


# ─────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────


def _fmt(value: float | None, spec: str) -> str:
    return "—" if value is None else format(value, spec)


def _step_table(steps: Sequence[HoldStep]) -> str:
    header = [
        "目標 [km/h]",
        "終わり方",
        "保持窓の行数",
        "実車速中央値 [km/h]",
        "実開度中央値 [%]",
        "保持中 |v-目標| 最大 [km/h]",
        "保持中 dv/dt [km/h/s]",
    ]
    table_rows: list[list[str]] = []
    for step in steps:
        speeds = [float(r["actual_speed_kmh"]) for r in step.rows]
        elapsed = [float(r["elapsed_s"]) for r in step.rows]
        openings = [v for r in step.rows if (v := actual_opening(r, "accel")) is not None]
        max_dev = max((abs(v - step.target_kmh) for v in speeds), default=None)
        table_rows.append(
            [
                f"{step.target_kmh:.1f}",
                _OUTCOME_JA[step.outcome],
                str(len(step.rows)),
                _fmt(_median(speeds), ".2f"),
                _fmt(_median(openings), ".2f"),
                _fmt(max_dev, ".2f"),
                _fmt(_linear_slope(elapsed, speeds), ".3f"),
            ]
        )
    return md_table(header, table_rows)


def run(csv_path: Path, config_path: Path) -> int:
    config = load_config(config_path)
    with csv_path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    if not any(row.get("phase") == PHASE_CRUISE_HOLD for row in rows):
        print(f"保持段なし（{csv_path} に CRUISE_HOLD の行が見つかりませんでした）")
        return 1

    steps = extract_hold_steps(rows, config.learning)
    print(_step_table(steps))

    try:
        curve = build_cruise_curve(rows, config.learning)
    except ValueError as e:
        print(f"\n{e}")
        return 1

    print(f"\n実測テーブルの点数: {len(curve.speeds_kmh)}")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("csv", type=Path, help="定速階段（CRUISE_HOLD）を含む走行ログ CSV（手順2）")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    args = ap.parse_args(argv)
    return run(args.csv, args.config)


if __name__ == "__main__":
    raise SystemExit(main())
