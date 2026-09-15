"""A2・A5（低開度の ACCEL_SWEEP と 60 km/h からの BRAKE_HOLD を追加）の手順 2 を A6 と比べる。

車両・アクチュエータには触らない。手順 2 の走行ログ CSV（PATTERN_DRIVE 区間）を 2 本読み、
表（Markdown）をターミナルに出し、図を --out に保存する。WLTP の基準車速は DB から読む。

    .venv/bin/python -m tests.research.debug_a2a5 \
        --csv tests/research/results/drive_log_real_<日時>.csv

出すもの（KAIZEN 表5-5 順4 の関門と、A2・A5 で足したところ）:
    gate      関門: 所要時間（900s との差）・表 4-1 の空白セル数（前 → 新）・
              最高速（140 km/h 以下か）
    coverage  速度帯 × 開度の網羅表（kaizen.coverage_table）を前 / 新で
    sweep     追加した ACCEL_SWEEP: 開度・加速時間・終わりの車速・終わり 5s の車速の変化・
              ガバナー作動・停車復帰
    hold      60 km/h から保持する BRAKE_HOLD: 保持を始めた車速・保持時間・保持中に停車したか・
              効くブレーキ行
    rows      学習に効く行（開度 ≥ 不感帯）を 5% 刻みで（debug_a6.rows_table）
    figure    新しい CSV の網羅マップ（kaizen.fig_coverage_map）

追加した段はパターン番号で見分ける（config から build_patterns を組み直す。
本数と順は config だけで決まる）。
"""

from __future__ import annotations

import argparse
import asyncio
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from tests.research.config import ResearchConfig, load_config
from tests.research.debug_a6 import (
    DT_S,
    Row,
    blocks_of,
    effective_row_bins,
    gate_summary,
    pattern_gates,
    read_pattern_rows,
    rows_table,
)
from tests.research.debug_process23 import md_table
from tests.research.kaizen import (
    COVER_OPEN_EDGES_PCT,
    DEFAULT_FEATURE_SPEC,
    PEDAL_ACCEL,
    PEDAL_BRAKE,
    SIM_DT_S,
    EffectiveRows,
    _is_hole,
    coverage_grid,
    coverage_table,
    effective_rows,
    fig_coverage_map,
    ref_frames,
    wltp_demand,
    worst_holes,
)
from tests.research.mode_drive import ReferenceSpeed, load_mode
from tests.research.pattern_drive import build_patterns
from tests.research.pattern_loop import STOP_SPEED_KMH, SpeedTargetPattern
from tests.research.vehicle import build_vehicle_profile, feedforward_params

RESULTS = Path("tests/research/results")
DEFAULT_CONFIG_PATH = Path("tests/research/config_testVehicle.yaml")
OLD_CSV = RESULTS / "drive_log_real_20260913_145445.csv"  # 9/13 14:54 手順 2（A6）
GATE_TIME_S = 900.0  # KAIZEN 5.6 の関門「所要時間が 900s に収まるか」
SETTLE_WINDOW_S = 5.0  # 「一定になった」を見る加速の終わりの窓 [s]


@dataclass(frozen=True)
class AddedPatterns:
    """build_patterns で研究側が足した段の CSV 上の名前（"12:ACCEL_SWEEP" など）。"""

    sweeps: tuple[str, ...]
    low_holds: tuple[str, ...]


def added_patterns(cfg: ResearchConfig) -> AddedPatterns:
    patterns = build_patterns(cfg, build_vehicle_profile(cfg))
    n_add = len(cfg.learning.accel_sweep_add_offsets_pct)
    sweeps = [f"{i}:{p.kind.name}" for i, p in enumerate(patterns, start=1)
              if p.kind.name == "ACCEL_SWEEP"][:n_add]
    lows = [f"{i}:{p.kind.name}" for i, p in enumerate(patterns, start=1)
            if isinstance(p, SpeedTargetPattern)
            and p.accel_target_kmh == cfg.learning.brake_hold_low_start_kmh]  # A4 の段と分ける
    return AddedPatterns(tuple(sweeps), tuple(lows))


# ─────────────────────────────────────────────────────────────────────
# 追加した段
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class SweepStat:
    pattern: str
    opening: float  # DRIVE_ACCEL で出した最大開度 [%]
    accel_s: float
    v_end: float  # 加速を終えた車速
    v_change_end: float  # 加速の終わり SETTLE_WINDOW_S の車速の変化（小さいほど一定）
    governor_s: float
    stop_return_s: float | None


def sweep_stats(rows: Sequence[Row], names: Sequence[str]) -> list[SweepStat]:
    blocks = blocks_of(rows)
    out = []
    for name in names:
        accel = next((b for b in blocks if b.pattern == name and b.phase == "DRIVE_ACCEL"), None)
        if accel is None:
            continue
        brake = next((b for b in blocks if b.pattern == name and b.phase == "DRIVE_BRAKE"), None)
        t_end = accel.rows[-1].t
        window = [r for r in accel.rows if r.t >= t_end - SETTLE_WINDOW_S]
        out.append(SweepStat(
            pattern=name,
            opening=max(r.accel for r in accel.rows),
            accel_s=accel.duration_s,
            v_end=accel.rows[-1].v,
            v_change_end=window[-1].v - window[0].v,
            governor_s=sum(r.governor for r in accel.rows) * DT_S,
            stop_return_s=brake.duration_s if brake else None,
        ))
    return out


def sweep_table(stats: Sequence[SweepStat]) -> str:
    body = [[
        s.pattern, f"{s.opening:.1f}", f"{s.accel_s:.1f}", f"{s.v_end:.1f}",
        f"{s.v_change_end:+.1f}", f"{s.governor_s:.1f}",
        "—" if s.stop_return_s is None else f"{s.stop_return_s:.1f}",
    ] for s in stats]
    return md_table(
        ["パターン", "開度[%]", "加速[s]", "加速を終えた車速[km/h]",
         f"終わり {SETTLE_WINDOW_S:g}s の車速の変化[km/h]", "ガバナー作動[s]", "停車復帰[s]"],
        body,
    )


@dataclass(frozen=True)
class LowHoldStat:
    pattern: str
    brake: float  # BRAKE_HOLD で出した最大開度 [%]
    v_hold_start: float
    hold_s: float
    v_hold_end: float
    effective_rows: int  # 保持中にブレーキ開度 ≥ 不感帯だった行


def low_hold_stats(
    rows: Sequence[Row], names: Sequence[str], brake_deadband_pct: float
) -> list[LowHoldStat]:
    blocks = blocks_of(rows)
    out = []
    for name in names:
        hold = next((b for b in blocks if b.pattern == name and b.phase == "BRAKE_HOLD"), None)
        if hold is None:
            continue
        out.append(LowHoldStat(
            pattern=name,
            brake=max(r.brake for r in hold.rows),
            v_hold_start=hold.rows[0].v,
            hold_s=hold.duration_s,
            v_hold_end=hold.rows[-1].v,
            effective_rows=sum(r.brake >= brake_deadband_pct for r in hold.rows),
        ))
    return out


def low_hold_table(stats: Sequence[LowHoldStat]) -> str:
    body = [[
        s.pattern, f"{s.brake:.2f}", f"{s.v_hold_start:.1f}", f"{s.hold_s:.1f}",
        f"{s.v_hold_end:.2f}", "○" if s.v_hold_end <= STOP_SPEED_KMH else "×",
        str(s.effective_rows),
    ] for s in stats]
    return md_table(
        ["パターン", "ブレーキ[%]", "保持を始めた車速[km/h]", "保持[s]", "保持を終えた車速[km/h]",
         "保持中に停車", "効くブレーキ行"],
        body,
    )


# ─────────────────────────────────────────────────────────────────────
# 関門
# ─────────────────────────────────────────────────────────────────────


def hole_count(grid: Sequence[Sequence[tuple[float, int]]]) -> int:
    return sum(_is_hole(sec, cnt) for row in grid for sec, cnt in row)


def a2a5_gate_table(
    *, span_s: float, holes_old: tuple[int, int], holes_new: tuple[int, int],
    v_max: float, max_speed_kmh: float,
) -> str:
    body = [
        ["パターン走行の所要時間", f"{span_s:.1f}s",
         f"{GATE_TIME_S:.0f}s に対し {GATE_TIME_S - span_s:+.1f}s",
         "○" if span_s <= GATE_TIME_S else "×"],
        ["表 4-1 の空白（アクセル）", f"{holes_old[0]} → {holes_new[0]} 個", "前（A6）→ 新",
         "○" if holes_new[0] < holes_old[0] else "×"],
        ["表 4-1 の空白（ブレーキ）", f"{holes_old[1]} → {holes_new[1]} 個", "前（A6）→ 新",
         "○" if holes_new[1] < holes_old[1] else "×"],
        ["最高速", f"{v_max:.1f} km/h", f"vehicle.max_speed_kmh {max_speed_kmh:g} km/h",
         "○" if v_max <= max_speed_kmh else "×"],
    ]
    return md_table(["関門", "値", "補足", "判定"], body)


# ─────────────────────────────────────────────────────────────────────


def run(new_csv: Path, old_csv: Path, cfg_path: Path, out: Path) -> int:
    cfg = load_config(cfg_path)
    p = feedforward_params(cfg)
    old, new = read_pattern_rows(old_csv), read_pattern_rows(new_csv)
    added = added_patterns(cfg)
    print(f"前（A6）: {old_csv}（{len(old)} 行）\n新: {new_csv}（{len(new)} 行）")
    print(f"不感帯 アクセル {p.accel_deadband_pct:g}% / ブレーキ {p.brake_deadband_pct:g}%"
          f"（{cfg_path} の値。前後で共通に使う）")
    print(f"追加した ACCEL_SWEEP: {', '.join(added.sweeps) or 'なし'} / "
          f"60 km/h からの BRAKE_HOLD: {', '.join(added.low_holds) or 'なし'}")

    mode = asyncio.run(load_mode(cfg, cfg.modes.wltp_mode_name))
    t = np.round(np.arange(0.0, mode.total_duration + SIM_DT_S / 2, SIM_DT_S), 4)
    frames = ref_frames(ReferenceSpeed(mode), t, DEFAULT_FEATURE_SPEC)
    demand = wltp_demand(p, frames)
    rows_old = effective_rows(old_csv, p)
    rows_new = effective_rows(new_csv, p)

    def holes(rows: tuple[EffectiveRows, EffectiveRows]) -> tuple[int, int]:
        return (hole_count(coverage_grid(demand, frames, rows[0], PEDAL_ACCEL)),
                hole_count(coverage_grid(demand, frames, rows[1], PEDAL_BRAKE)))

    span = new[-1].t - new[0].t if new else 0.0
    print("\n### 関門\n")
    print(a2a5_gate_table(span_s=span, holes_old=holes(rows_old), holes_new=holes(rows_new),
                          v_max=max((r.v for r in new), default=0.0),
                          max_speed_kmh=cfg.vehicle.max_speed_kmh))
    print()
    print(gate_summary(new, pattern_gates(new), GATE_TIME_S))

    for label, idx, pedal in (("アクセル", 0, PEDAL_ACCEL), ("ブレーキ", 1, PEDAL_BRAKE)):
        for tag, rows in (("前（A6）", rows_old), ("新", rows_new)):
            print(f"\n### 網羅（{label}・{tag}。セルは「WLTP が要る時間 / 学習の効く行数」、"
                  f"太字が空白。開度ビンの下端 {COVER_OPEN_EDGES_PCT[0]:g}%）\n")
            print(coverage_table(demand, frames, rows[idx], pedal))
        print(f"\n- 新で残った大きい空白: {worst_holes(demand, frames, rows_new[idx], pedal)}")

    print("\n### 追加した ACCEL_SWEEP\n")
    print(sweep_table(sweep_stats(new, added.sweeps)))
    print("\n### 60 km/h から保持する BRAKE_HOLD\n")
    print(low_hold_table(low_hold_stats(new, added.low_holds, p.brake_deadband_pct)))

    for label, accel, db in (("アクセル", True, p.accel_deadband_pct),
                             ("ブレーキ", False, p.brake_deadband_pct)):
        print(f"\n### 学習に効く{label}行（開度 ≥ 不感帯 {db:g}%。前 = A6）\n")
        print(rows_table(effective_row_bins(old, db, accel=accel),
                         effective_row_bins(new, db, accel=accel), label))

    out.mkdir(parents=True, exist_ok=True)
    path = out / "coverage_map.png"
    fig_coverage_map(demand, frames, rows_new[0], rows_new[1], path)
    print(f"\n図: {path}")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--csv", type=Path, required=True, help="A2・A5 の手順 2 の走行ログ CSV")
    ap.add_argument("--old", type=Path, default=OLD_CSV, help="比べる手順 2 の CSV（既定: A6）")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    ap.add_argument(
        "--out", type=Path, default=None, help="図の保存先（既定: CSV と同じ名前のフォルダ）"
    )
    args = ap.parse_args(argv)
    out = args.out or args.csv.with_name(f"{args.csv.stem}_a2a5")
    return run(args.csv, args.old, args.config, out)


if __name__ == "__main__":
    raise SystemExit(main())
