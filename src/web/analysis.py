# ruff: noqa: E501
"""走行ログ（DB の drive_logs）から、ログ画面の「運転指標」「偏差」「走行グラフ（3 段の図）」を作る（純粋関数。段11）。

対象はモード走行（detail の section が MODE_DRIVE の行）だけ。基準・実車速・モード時刻は
drive_logs の 9 列と detail（CSV の列。文字列）から読む。運転指標は core の
``drive_index.compute_drive_index``、偏差の区間別・KPI は tests の走行レポート（3・4 章）と同じ定義
（偏差 = 実車速 − 基準車速。区間は pattern 列。1.0 超えの秒数は行数 × 刻み）。
合否・基準の線は出さない。
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import Any

import numpy as np

from src.robot.core.drive_index import DriveIndexRow, compute_drive_index

SECTION_MODE_DRIVE = "MODE_DRIVE"

#: KPI の |偏差| のしきい値 [km/h]（0.5・1.0）
OVER_THRESHOLDS = (0.5, 1.0)


def _f(value: Any) -> float | None:
    if value in (None, ""):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def mode_rows(rows: Sequence[Any]) -> list[dict[str, Any]]:
    """LogCsvRow の並びからモード走行の行（t・ref・act・segment）だけを取り出す。"""
    out: list[dict[str, Any]] = []
    for r in rows:
        d = r.detail or {}
        if d.get("section") != SECTION_MODE_DRIVE:
            continue
        t = _f(d.get("mode_time_s"))
        if t is None or r.ref_speed_kmh is None:
            continue
        out.append(
            {
                "t": t,
                "ref": float(r.ref_speed_kmh),
                "act": float(r.actual_speed_kmh),
                "segment": str(d.get("pattern") or ""),
            }
        )
    return out


def _step_s(t: np.ndarray) -> float:
    if len(t) < 2:
        return 0.1
    d = np.diff(t)
    d = d[d > 0]
    return float(np.median(d)) if len(d) else 0.1


def _index_row(row: DriveIndexRow) -> dict[str, Any]:
    return {
        "label": row.label,
        "distance_ref_km": round(row.distance_ref_km, 3),
        "distance_act_km": round(row.distance_act_km, 3),
        "dr_pct": row.dr_pct,
        "er_pct": row.er_pct,
        "eer_pct": row.eer_pct,
        "iwr_pct": row.iwr_pct,
        "ascr_pct": row.ascr_pct,
        "rmsse_kmh": row.rmsse_kmh,
    }


def _stats(dev: np.ndarray, dt: float) -> dict[str, float]:
    a = np.abs(dev)
    return {
        "duration_s": round(len(dev) * dt, 1),
        "p95": round(float(np.percentile(a, 95)), 3),
        "max": round(float(a.max()), 3),
        "mean": round(float(dev.mean()), 3),
        "over_1_s": round(float(np.count_nonzero(a > OVER_THRESHOLDS[1])) * dt, 1),
    }


def build_analysis(
    rows: Sequence[Any],
    *,
    mass_kg: float | None,
    a0: float | None,
    a1: float | None,
    a2: float | None,
    segment_names: Sequence[str] = (),
) -> dict[str, Any]:
    """モード走行の行から ``{available, drive_index, kpi, sections}`` を作る。

    モード走行の行が無ければ ``{"available": False}``。区間別の表は、行の区間名が全部
    ``segment_names``（共通設定 modes.segment_names）に入っているときだけ出す（WLTP の Low/Mid/High/ExHi）。
    """
    mr = mode_rows(rows)
    if len(mr) < 2:
        return {"available": False}
    t = np.array([r["t"] for r in mr])
    ref = np.array([r["ref"] for r in mr])
    act = np.array([r["act"] for r in mr])
    segs = [r["segment"] for r in mr]
    dev = act - ref
    dt = _step_s(t)
    seg_set = {s for s in segs if s}
    segmented = bool(seg_set) and seg_set <= set(segment_names)

    index = compute_drive_index(
        t.tolist(),
        ref.tolist(),
        act.tolist(),
        segments=segs if segmented else None,
        mass_kg=mass_kg,
        road_load_a0_n=a0,
        road_load_a1_n_per_kmh=a1,
        road_load_a2_n_per_kmh2=a2,
    )
    abs_dev = np.abs(dev)
    kpi = {
        "max": round(float(abs_dev.max()), 3),
        "p95": round(float(np.percentile(abs_dev, 95)), 3),
        "over_0_5_s": round(float(np.count_nonzero(abs_dev > OVER_THRESHOLDS[0])) * dt, 2),
        "over_1_s": round(float(np.count_nonzero(abs_dev > OVER_THRESHOLDS[1])) * dt, 2),
        "mean": round(float(dev.mean()), 3),
        "duration_s": round(len(dev) * dt, 1),
    }
    sections: list[dict[str, Any]] = []
    if segmented:
        order = [n for n in segment_names if n in seg_set]
        total = len(dev)
        for name in order:
            mask = np.array([s == name for s in segs])
            sections.append({"name": name, "share": round(float(mask.sum()) / total, 3), **_stats(dev[mask], dt)})
    return {
        "available": True,
        "drive_index": {
            "rows": [_index_row(r) for r in [*index.phases, index.total]],
            "mass_kg": mass_kg,
            "road_load": {"a0": a0, "a1": a1, "a2": a2},
        },
        "kpi": kpi,
        "sections": sections,
    }


def build_figure(
    rows: Sequence[Any],
    *,
    is_auto: bool,
    presses: Sequence[dict[str, Any]] = (),
) -> dict[str, Any] | None:
    """走行グラフ（車速・ペダル開度＋ボタン・偏差の 3 段）の点列とボタンの区間を作る。

    x はモード走行（auto で MODE_DRIVE の行が 2 行以上）ならモード時間 [s]、それ以外（学習など）は
    最初の行からの経過時間 [s]。``presses`` は ``button_events.parse_button_presses`` の戻り値で、
    壁時計を x に直して ``buttons`` に入れる。行が 2 行未満なら None。
    """
    from src.infra.button_events import presses_to_intervals  # noqa: PLC0415

    rows = list(rows)
    use_mode = False
    picked: list[tuple[float, Any]] = []
    if is_auto:
        for r in rows:
            d = r.detail or {}
            t = _f(d.get("mode_time_s")) if d.get("section") == SECTION_MODE_DRIVE else None
            if t is not None and r.ref_speed_kmh is not None:
                picked.append((t, r))
        use_mode = len(picked) >= 2
    if not use_mode:
        if len(rows) < 2:
            return None
        t0 = rows[0].timestamp
        picked = [((r.timestamp - t0).total_seconds(), r) for r in rows]
    has_ref = any(r.ref_speed_kmh is not None for _, r in picked)
    xs = [t for t, _ in picked]
    figure: dict[str, Any] = {
        "x_label": "モード時間 [s]" if use_mode else "経過時間 [s]",
        "t": [round(t, 2) for t in xs],
        "ref": [None if r.ref_speed_kmh is None else round(float(r.ref_speed_kmh), 2) for _, r in picked] if has_ref else None,
        "act": [round(float(r.actual_speed_kmh), 2) for _, r in picked],
        "accel": [round(float(getattr(r, "accel_opening", 0.0) or 0.0), 2) for _, r in picked],
        "brake": [round(float(getattr(r, "brake_opening", 0.0) or 0.0), 2) for _, r in picked],
        "buttons": [],
    }
    if presses and all(hasattr(r, "timestamp") for _, r in picked):
        figure["buttons"] = presses_to_intervals(presses, [r.timestamp for _, r in picked], xs)
    return figure
