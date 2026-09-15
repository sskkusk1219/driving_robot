"""A3・A4（トリム階段と、20 km/h からの高ブレーキ保持を追加）の手順 2 を A2・A5 と比べる。

車両・アクチュエータには触らない。手順 2 の走行ログ CSV（PATTERN_DRIVE 区間）を 2 本読み、
表（Markdown）をターミナルに出し、図を --out に保存する。WLTP の基準車速は DB から読む。

    .venv/bin/python -m tests.research.debug_a3a4 \
        --csv tests/research/results/drive_log_real_<日時>.csv

出すもの（KAIZEN 表5-5 順5 の関門と、A3・A4 で足したところ）:
    gate      関門: 表 4-1 の空白セル数（前 → 新）・最高速（140 km/h 以下か）・
              所要時間（learning.timeout_s 以内か。900s・見積りとの差は記録だけ）
    coverage  速度帯 × 開度の網羅表（kaizen.coverage_table）を前 / 新で
    stair     トリム階段: 段ごとの開度・保持・始めた / 終えた車速・終えた理由・
              効くアクセル行（速度帯別）。パターンごとの加速を終えた車速・最高速・停車復帰
    hard      20 km/h から保持する BRAKE_HOLD: 加速を終えた車速・保持を始めた車速・停車までの秒数・
              0〜20 km/h × 40% 以上の行・保持中のガバナー行
    rows      学習に効く行（開度 ≥ 不感帯）を 5% 刻みで（debug_a6.rows_table）
    figure    新しい CSV の網羅マップ（kaizen.fig_coverage_map）

足した段はパターン番号で見分ける（config から build_patterns を組み直す。
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
from tests.research.debug_a2a5 import hole_count
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
from tests.research.pattern_loop import (
    STOP_SPEED_KMH,
    PatternLoopConfig,
    SpeedTargetPattern,
    TrimStairPattern,
)
from tests.research.vehicle import build_vehicle_profile, feedforward_params

RESULTS = Path("tests/research/results")
DEFAULT_CONFIG_PATH = Path("tests/research/config_testVehicle.yaml")
OLD_CSV = RESULTS / "drive_log_real_20260913_172758.csv"  # 9/13 17:27 手順 2（A2・A5）
RECORD_TIME_S = 900.0  # 本番 learning_timeout_s。A3・A4 では記録だけ（ユーザー決定 3）
PLAN_ESTIMATE_S = 1020.0  # この計画の見積り（実測の単位時間の積算）
KAIZEN_ESTIMATE_S = 661.8  # KAIZEN 表 4-6 の見積り
BAND_KMH = 20.0  # 効く行を数える速度帯の幅
HARD_LOW_KMH = 20.0  # A4 の空白の速度帯（0〜20 km/h）
HARD_OPENING_PCT = 40.0  # A4 の空白の開度（40% 以上）

REASON_STEP = "保持時間"
REASON_CAP = "cap"
REASON_SLOW = "低速"
REASON_OVERSPEED = "最高速超え"


@dataclass(frozen=True)
class A3A4Patterns:
    """build_patterns で A3・A4 として足した段の CSV 上の名前と、階段の段の開度。"""

    stairs: tuple[str, ...]
    hard_holds: tuple[str, ...]
    stair_steps_pct: tuple[float, ...]
    stair_step_s: float


def a3a4_patterns(cfg: ResearchConfig) -> A3A4Patterns:
    patterns = build_patterns(cfg, build_vehicle_profile(cfg))
    stairs = [(f"{i}:{p.kind.name}", p) for i, p in enumerate(patterns, start=1)
              if isinstance(p, TrimStairPattern)]
    hard = [f"{i}:{p.kind.name}" for i, p in enumerate(patterns, start=1)
            if isinstance(p, SpeedTargetPattern)
            and p.accel_target_kmh == cfg.learning.brake_hold_hard_start_kmh]
    first = stairs[0][1] if stairs else None
    return A3A4Patterns(
        stairs=tuple(name for name, _ in stairs),
        hard_holds=tuple(hard),
        stair_steps_pct=first.trim_steps_pct if isinstance(first, TrimStairPattern) else (),
        stair_step_s=cfg.learning.trim_stair_step_s,
    )


# ─────────────────────────────────────────────────────────────────────
# トリム階段（A3）
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class StepStat:
    pattern: str
    step: int  # 1 始まり
    opening: float
    hold_s: float
    v_start: float
    v_end: float
    reason: str
    band_rows: dict[int, int]  # 速度帯の下端 [km/h] → 効くアクセル行


@dataclass(frozen=True)
class StairStat:
    pattern: str
    v_accel_end: float | None
    v_max: float  # 階段の保持中の最高速
    stop_return_s: float | None
    overspeed: bool  # 最高速超えの回復に入ったか
    steps: tuple[StepStat, ...]


def _runs_by_opening(rows: Sequence[Row]) -> list[list[Row]]:
    runs: list[list[Row]] = []
    for r in rows:
        if runs and runs[-1][-1].accel == r.accel:
            runs[-1].append(r)
        else:
            runs.append([r])
    return runs


def _step_reason(
    run: Sequence[Row], hold_s: float, *, step_s: float, cap_kmh: float, max_speed_kmh: float,
    stop_kmh: float,
) -> str:
    v = run[-1].v  # 状態機械はこの行の車速で段を進めた
    if v > max_speed_kmh:
        return REASON_OVERSPEED
    if v <= stop_kmh:
        return REASON_SLOW
    if v >= cap_kmh and hold_s < step_s - DT_S:
        return REASON_CAP
    return REASON_STEP


def stair_stats(
    rows: Sequence[Row], names: Sequence[str], *, accel_deadband_pct: float, step_s: float,
    cap_kmh: float, max_speed_kmh: float, stop_kmh: float,
) -> list[StairStat]:
    blocks = blocks_of(rows)
    out = []
    for name in names:
        trim = next((b for b in blocks if b.pattern == name and b.phase == "CRUISE_TRIM"), None)
        if trim is None:
            continue
        accel = next((b for b in blocks if b.pattern == name and b.phase == "DRIVE_ACCEL"), None)
        brake = next((b for b in blocks if b.pattern == name and b.phase == "DRIVE_BRAKE"), None)
        steps = []
        for k, run in enumerate(_runs_by_opening(trim.rows), start=1):
            hold_s = len(run) * DT_S
            bands: dict[int, int] = {}
            for r in run:
                if r.accel >= accel_deadband_pct:
                    band = int(r.v // BAND_KMH * BAND_KMH)
                    bands[band] = bands.get(band, 0) + 1
            steps.append(StepStat(
                pattern=name, step=k, opening=run[0].accel, hold_s=hold_s,
                v_start=run[0].v, v_end=run[-1].v,
                reason=_step_reason(run, hold_s, step_s=step_s, cap_kmh=cap_kmh,
                                    max_speed_kmh=max_speed_kmh, stop_kmh=stop_kmh),
                band_rows=bands,
            ))
        out.append(StairStat(
            pattern=name,
            v_accel_end=accel.rows[-1].v if accel else None,
            v_max=max(r.v for r in trim.rows),
            stop_return_s=brake.duration_s if brake else None,
            overspeed=trim.rows[-1].v > max_speed_kmh,
            steps=tuple(steps),
        ))
    return out


def _bands_text(bands: dict[int, int]) -> str:
    if not bands:
        return "—"
    ordered = sorted(bands.items(), reverse=True)
    return " / ".join(f"{b:g}〜{b + BAND_KMH:g}: {n}" for b, n in ordered)


def stair_table(stats: Sequence[StairStat]) -> str:
    body = [[
        s.pattern, "—" if s.v_accel_end is None else f"{s.v_accel_end:.1f}", f"{s.v_max:.1f}",
        str(len(s.steps)), "○" if s.overspeed else "—",
        "—" if s.stop_return_s is None else f"{s.stop_return_s:.1f}",
    ] for s in stats]
    return md_table(
        ["パターン", "加速を終えた車速[km/h]", "階段中の最高速[km/h]",
         "段数（開度が変わった回数+1）",
         "最高速超え", "停車復帰[s]"],
        body,
    )


def step_table(stats: Sequence[StairStat]) -> str:
    body = [[
        st.pattern, str(st.step), f"{st.opening:.1f}", f"{st.hold_s:.1f}", f"{st.v_start:.1f}",
        f"{st.v_end:.1f}", st.reason, _bands_text(st.band_rows),
    ] for s in stats for st in s.steps]
    return md_table(
        ["パターン", "段", "開度[%]", "保持[s]", "始めた車速[km/h]", "終えた車速[km/h]",
         "終えた理由", "効くアクセル行（速度帯[km/h]: 行）"],
        body,
    )


# ─────────────────────────────────────────────────────────────────────
# 低速 × 高ブレーキ（A4）
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class HardHoldStat:
    pattern: str
    brake: float  # BRAKE_HOLD で出した最大開度 [%]
    v_accel_end: float | None
    v_max: float
    v_hold_start: float
    hold_s: float
    v_hold_end: float
    hard_rows: int  # 0〜20 km/h × 40% 以上の行
    effective_rows: int  # 保持中にブレーキ開度 ≥ 不感帯だった行
    governor_rows: int  # 保持中にガバナーが開度を削っていた行


def hard_hold_stats(
    rows: Sequence[Row], names: Sequence[str], brake_deadband_pct: float
) -> list[HardHoldStat]:
    blocks = blocks_of(rows)
    out = []
    for name in names:
        hold = next((b for b in blocks if b.pattern == name and b.phase == "BRAKE_HOLD"), None)
        if hold is None:
            continue
        accel = next((b for b in blocks if b.pattern == name and b.phase == "DRIVE_ACCEL"), None)
        driven = (accel.rows if accel else ()) + hold.rows
        out.append(HardHoldStat(
            pattern=name,
            brake=max(r.brake for r in hold.rows),
            v_accel_end=accel.rows[-1].v if accel else None,
            v_max=max(r.v for r in driven),
            v_hold_start=hold.rows[0].v,
            hold_s=hold.duration_s,
            v_hold_end=hold.rows[-1].v,
            hard_rows=sum(r.v < HARD_LOW_KMH and r.brake >= HARD_OPENING_PCT for r in hold.rows),
            effective_rows=sum(r.brake >= brake_deadband_pct for r in hold.rows),
            governor_rows=sum(r.governor for r in hold.rows),
        ))
    return out


def hard_hold_table(stats: Sequence[HardHoldStat]) -> str:
    body = [[
        s.pattern, f"{s.brake:.2f}", "—" if s.v_accel_end is None else f"{s.v_accel_end:.1f}",
        f"{s.v_max:.1f}", f"{s.v_hold_start:.1f}", f"{s.hold_s:.1f}", f"{s.v_hold_end:.2f}",
        "○" if s.v_hold_end <= STOP_SPEED_KMH else "×", str(s.effective_rows), str(s.hard_rows),
        str(s.governor_rows),
    ] for s in stats]
    return md_table(
        ["パターン", "ブレーキ[%]", "加速を終えた車速[km/h]", "最高速[km/h]",
         "保持を始めた車速[km/h]", "保持[s]", "保持を終えた車速[km/h]", "保持中に停車",
         "効くブレーキ行", f"0〜{HARD_LOW_KMH:g} km/h × {HARD_OPENING_PCT:g}% 以上の行",
         "保持中のガバナー行"],
        body,
    )


# ─────────────────────────────────────────────────────────────────────
# 関門
# ─────────────────────────────────────────────────────────────────────


def a3a4_gate_table(
    *, span_s: float, timeout_s: float, holes_old: tuple[int, int], holes_new: tuple[int, int],
    v_max: float, max_speed_kmh: float,
) -> str:
    body = [
        ["表 4-1 の空白（アクセル）", f"{holes_old[0]} → {holes_new[0]} 個", "前（A2・A5）→ 新",
         "○" if holes_new[0] < holes_old[0] else "×"],
        ["表 4-1 の空白（ブレーキ）", f"{holes_old[1]} → {holes_new[1]} 個", "前（A2・A5）→ 新",
         "○" if holes_new[1] < holes_old[1] else "×"],
        ["最高速", f"{v_max:.1f} km/h", f"vehicle.max_speed_kmh {max_speed_kmh:g} km/h",
         "○" if v_max <= max_speed_kmh else "×"],
        ["パターン走行の所要時間", f"{span_s:.1f}s",
         f"learning.timeout_s {timeout_s:.0f}s に対し {timeout_s - span_s:+.1f}s",
         "○" if span_s <= timeout_s else "×"],
        ["（記録）本番の 900s との差", f"{span_s - RECORD_TIME_S:+.1f}s",
         f"見積り {PLAN_ESTIMATE_S:.0f}s との差 {span_s - PLAN_ESTIMATE_S:+.1f}s / "
         f"KAIZEN 表 4-6 の {KAIZEN_ESTIMATE_S:g}s との差 {span_s - KAIZEN_ESTIMATE_S:+.1f}s",
         "記録"],
    ]
    return md_table(["関門", "値", "補足", "判定"], body)


# ─────────────────────────────────────────────────────────────────────


def run(new_csv: Path, old_csv: Path, cfg_path: Path, out: Path) -> int:
    cfg = load_config(cfg_path)
    p = feedforward_params(cfg)
    profile = build_vehicle_profile(cfg)
    loop_cfg = PatternLoopConfig(coast_timeout_s=cfg.learning.coast_timeout_s)
    cap_kmh = profile.max_speed * loop_cfg.accel_speed_cap_frac
    old, new = read_pattern_rows(old_csv), read_pattern_rows(new_csv)
    added = a3a4_patterns(cfg)
    print(f"前（A2・A5）: {old_csv}（{len(old)} 行）\n新: {new_csv}（{len(new)} 行）")
    print(f"不感帯 アクセル {p.accel_deadband_pct:g}% / ブレーキ {p.brake_deadband_pct:g}%"
          f"（{cfg_path} の値。前後で共通に使う）。cap {cap_kmh:.1f} km/h")
    print(f"トリム階段: {', '.join(added.stairs) or 'なし'}"
          f"（段 {' → '.join(f'{v:g}' for v in added.stair_steps_pct)}% "
          f"各 {added.stair_step_s:g}s）"
          f" / {cfg.learning.brake_hold_hard_start_kmh:g} km/h からの BRAKE_HOLD: "
          f"{', '.join(added.hard_holds) or 'なし'}")

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
    print(a3a4_gate_table(span_s=span, timeout_s=cfg.learning.timeout_s,
                          holes_old=holes(rows_old), holes_new=holes(rows_new),
                          v_max=max((r.v for r in new), default=0.0),
                          max_speed_kmh=cfg.vehicle.max_speed_kmh))
    print()
    print(gate_summary(new, pattern_gates(new), cfg.learning.timeout_s))

    for label, idx, pedal in (("アクセル", 0, PEDAL_ACCEL), ("ブレーキ", 1, PEDAL_BRAKE)):
        for tag, rows in (("前（A2・A5）", rows_old), ("新", rows_new)):
            print(f"\n### 網羅（{label}・{tag}。セルは「WLTP が要る時間 / 学習の効く行数」、"
                  f"太字が空白。開度ビンの下端 {COVER_OPEN_EDGES_PCT[0]:g}%）\n")
            print(coverage_table(demand, frames, rows[idx], pedal))
        print(f"\n- 新で残った大きい空白: {worst_holes(demand, frames, rows_new[idx], pedal)}")

    stairs = stair_stats(
        new, added.stairs, accel_deadband_pct=p.accel_deadband_pct, step_s=added.stair_step_s,
        cap_kmh=cap_kmh, max_speed_kmh=cfg.vehicle.max_speed_kmh,
        stop_kmh=loop_cfg.coast_down_stop_speed_kmh,
    )
    print("\n### トリム階段（パターンごと）\n")
    print(stair_table(stairs))
    print("\n### トリム階段（段ごと）\n")
    print(step_table(stairs))
    print(f"\n### {cfg.learning.brake_hold_hard_start_kmh:g} km/h から保持する BRAKE_HOLD\n")
    print(hard_hold_table(hard_hold_stats(new, added.hard_holds, p.brake_deadband_pct)))

    for label, accel, db in (("アクセル", True, p.accel_deadband_pct),
                             ("ブレーキ", False, p.brake_deadband_pct)):
        print(f"\n### 学習に効く{label}行（開度 ≥ 不感帯 {db:g}%。前 = A2・A5）\n")
        print(rows_table(effective_row_bins(old, db, accel=accel),
                         effective_row_bins(new, db, accel=accel), label))

    out.mkdir(parents=True, exist_ok=True)
    path = out / "coverage_map.png"
    fig_coverage_map(demand, frames, rows_new[0], rows_new[1], path)
    print(f"\n図: {path}")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--csv", type=Path, required=True, help="A3・A4 の手順 2 の走行ログ CSV")
    ap.add_argument("--old", type=Path, default=OLD_CSV,
                    help="比べる手順 2 の CSV（既定: A2・A5）")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    ap.add_argument(
        "--out", type=Path, default=None, help="図の保存先（既定: CSV と同じ名前のフォルダ）"
    )
    args = ap.parse_args(argv)
    out = args.out or args.csv.with_name(f"{args.csv.stem}_a3a4")
    return run(args.csv, args.old, args.config, out)


if __name__ == "__main__":
    raise SystemExit(main())
