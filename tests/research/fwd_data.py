"""順モデル（車速 × 効く開度 → 加速度）の学習用の行を走行 CSV から作る（段2f）。

効く開度 u [%]（アクセル＋・ブレーキ−の 1 本）:
    アクセル実開度 > アクセル不感帯 なら +(実開度 − 不感帯)
    ブレーキ実開度 > ブレーキ不感帯 なら −(実開度 − 不感帯)
    どちらも不感帯以下なら 0（惰行）。両方が不感帯を超える行は捨てる。

表の定義（Apollo の形）: 「時刻 t の車速 × 時刻 t の開度 u → 時刻 t + L の加速度」。
u の元になる開度は opening_source で選ぶ（既定 "cmd" = 最終指令の開度 `*_cmd_pct`。待機位置を含む。
"actual" = 実開度 `*_actual_pct`）。

加速度 [km/h/s]: 実車速の、時刻 t + 遅れ L を中心にした幅 accel_window_s の最小二乗の傾き。
遅れ L は u > 0 の行が accel_lag_s、u < 0 の行が brake_lag_s、u = 0（惰行）の行が coast_lag_s
（None なら accel_lag_s）。
傾きの窓はセグメント（section が同じで行間が SEGMENT_GAP_S 未満の連続区間）の中だけで取る。

マスの集計（cell_counts）と「モード（検証ログ）が通るのに学習が薄いマス」（thin_cells）も持つ。
"""

from __future__ import annotations

import csv
import math
from collections.abc import Collection, Sequence
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from tests.research.drive_log import (
    SECTION_MODE_DRIVE,
    SECTION_PATTERN_DRIVE,
    actual_opening,
    cmd_opening,
)
from tests.research.ff_bias import window_slope

DEFAULT_SECTIONS: tuple[str, ...] = (SECTION_PATTERN_DRIVE, SECTION_MODE_DRIVE)
SEGMENT_GAP_S = 0.5  # 仮値: 行間がこれ以上空いたらセグメントを切る
STOP_SPEED_KMH = 0.5  # 実車速がこれ未満の行は停車として捨てる
DEFAULT_ACCEL_WINDOW_S = 1.0  # 加速度の窓幅（前後 0.5 秒）
# 指令 → 加速度の遅れ L [s]。データ由来（指令ベースの振り。検証 052422、2026-10-03）:
# そのペダルの行の「外れの 3 秒平均 p95」が最小になる値（0.05 刻み。fwd_tool score の (iii)）。
# 選んだ値: アクセル 0.05・ブレーキ 0.05（どちらも 3 秒平均 p95 が最小。惰行行は 0 が最小だが
# アクセルの L を使う）。ブレーキは 0.05〜0.15 でほぼ平ら（p95 0.479〜0.483）。
DEFAULT_ACCEL_LAG_S = 0.05
DEFAULT_BRAKE_LAG_S = 0.05
OPENING_CMD = "cmd"
OPENING_ACTUAL = "actual"

DROP_STOPPED = "停車（実車速 < 0.5 km/h）"
DROP_NO_OPENING = "実開度が空"
DROP_NO_SPEED = "実車速が空"
DROP_GOVERNOR = "減速Gガバナー作動"
DROP_BOTH_PEDALS = "両ペダルが不感帯超え"
DROP_NO_WINDOW = "加速度の窓が取れない（セグメント端）"
DROP_REASONS: tuple[str, ...] = (
    DROP_NO_OPENING,
    DROP_NO_SPEED,
    DROP_STOPPED,
    DROP_GOVERNOR,
    DROP_BOTH_PEDALS,
    DROP_NO_WINDOW,
)

Cell = tuple[int, int]  # (車速のマス番号, u のマス番号)


@dataclass(frozen=True)
class FwdRows:
    """学習用の行。配列はすべて同じ長さ。"""

    speed_kmh: np.ndarray
    u_pct: np.ndarray
    accel_kmhs: np.ndarray
    t_s: np.ndarray
    dt_s: float  # ログの刻みの中央値 [s]（行数 → 秒数の換算）
    source: str
    dropped: dict[str, int] = field(default_factory=dict)
    sections: tuple[str, ...] = ()  # 使えた行の section
    dt_row_s: np.ndarray = field(default_factory=lambda: np.zeros(0))  # 行ごとの秒数

    def __len__(self) -> int:
        return len(self.speed_kmh)

    @property
    def seconds(self) -> float:
        """使えた行の秒数。"""
        return float(np.sum(self.dt_row_s))

    def dropped_seconds(self) -> dict[str, float]:
        return {k: n * self.dt_s for k, n in self.dropped.items()}


def effective_opening(
    accel_pct: float,
    brake_pct: float,
    accel_deadband_pct: float,
    brake_deadband_pct: float,
) -> float | None:
    """効く開度 u [%]。両ペダルが不感帯を超えるときは None（捨てる行）。"""
    accel_on = accel_pct > accel_deadband_pct
    brake_on = brake_pct > brake_deadband_pct
    if accel_on and brake_on:
        return None
    if accel_on:
        return accel_pct - accel_deadband_pct
    if brake_on:
        return -(brake_pct - brake_deadband_pct)
    return 0.0


def _num(text: str | None) -> float | None:
    if text is None or text.strip() == "":
        return None
    try:
        v = float(text)
    except ValueError:
        return None
    return v if math.isfinite(v) else None


def _truthy(text: str | None) -> bool:
    return (text or "").strip().lower() not in ("", "0", "false", "0.0")


def _cmd_or_none(row: dict[str, str], axis: str) -> float | None:
    """最終指令の開度 [%]。列が無い・空・読めない行は None。"""
    try:
        return cmd_opening(row, axis)
    except (KeyError, ValueError):
        return None


def read_fwd_rows(
    path: Path,
    *,
    accel_deadband_pct: float,
    brake_deadband_pct: float,
    sections: Collection[str] = DEFAULT_SECTIONS,
    accel_window_s: float = DEFAULT_ACCEL_WINDOW_S,
    accel_lag_s: float = DEFAULT_ACCEL_LAG_S,
    brake_lag_s: float = DEFAULT_BRAKE_LAG_S,
    coast_lag_s: float | None = None,
    opening_source: str = OPENING_CMD,
) -> FwdRows:
    """走行 CSV 1 本から学習用の行を作る。捨てた行は理由ごとに dropped に数える。"""
    if opening_source not in (OPENING_CMD, OPENING_ACTUAL):
        raise ValueError(f"opening_source は cmd か actual: {opening_source!r}")
    pick = _cmd_or_none if opening_source == OPENING_CMD else actual_opening
    lag_coast = accel_lag_s if coast_lag_s is None else coast_lag_s
    dropped = dict.fromkeys(DROP_REASONS, 0)
    # セグメントごとの行: (t, 実車速, accel実開度, brake実開度, ガバナー, section)
    segments: list[list[tuple[float, float, float | None, float | None, bool, str]]] = []
    prev_t = math.nan
    prev_section = ""
    with path.open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            section = r.get("section") or ""
            t = _num(r.get("elapsed_s"))
            if section not in sections or t is None:
                continue
            speed = _num(r.get("actual_speed_kmh"))
            new_seg = (
                not segments
                or section != prev_section
                or not (0.0 < t - prev_t < SEGMENT_GAP_S)
            )
            if new_seg:
                segments.append([])
            prev_t, prev_section = t, section
            segments[-1].append(
                (
                    t,
                    math.nan if speed is None else speed,
                    pick(r, "accel"),
                    pick(r, "brake"),
                    _truthy(r.get("governor_active")),
                    section,
                )
            )

    speed_o: list[float] = []
    u_o: list[float] = []
    acc_o: list[float] = []
    t_o: list[float] = []
    secs: set[str] = set()
    all_dt: list[float] = []
    for seg in segments:
        ts = np.array([x[0] for x in seg])
        all_dt.extend(np.diff(ts).tolist())
        # 傾きは実車速が読めた点だけで取る
        ok = np.array([np.isfinite(x[1]) for x in seg])
        sp = np.array([x[1] for x in seg])
        us: list[float | None] = []
        for i, (_t, v, ap, bp, gov, _s) in enumerate(seg):
            if ap is None or bp is None:
                dropped[DROP_NO_OPENING] += 1
                us.append(None)
            elif not ok[i]:
                dropped[DROP_NO_SPEED] += 1
                us.append(None)
            elif v < STOP_SPEED_KMH:
                dropped[DROP_STOPPED] += 1
                us.append(None)
            elif gov:
                dropped[DROP_GOVERNOR] += 1
                us.append(None)
            else:
                u = effective_opening(ap, bp, accel_deadband_pct, brake_deadband_pct)
                if u is None:
                    dropped[DROP_BOTH_PEDALS] += 1
                us.append(u)
        keep = [i for i, u in enumerate(us) if u is not None]
        if not keep or int(ok.sum()) == 0:
            continue
        u_arr = np.array([us[i] for i in keep], dtype=float)
        t_keep = ts[keep]
        lag = np.where(u_arr > 0.0, accel_lag_s, np.where(u_arr < 0.0, brake_lag_s, lag_coast))
        slope = window_slope(ts[ok], sp[ok], t_keep + lag, accel_window_s)
        for j, i in enumerate(keep):
            if not np.isfinite(slope[j]):
                dropped[DROP_NO_WINDOW] += 1
                continue
            speed_o.append(float(sp[i]))
            u_o.append(float(u_arr[j]))
            acc_o.append(float(slope[j]))
            t_o.append(float(t_keep[j]))
            secs.add(seg[i][5])
    dt = float(np.median(all_dt)) if all_dt else 0.0
    n = len(speed_o)
    return FwdRows(
        speed_kmh=np.array(speed_o, dtype=float),
        u_pct=np.array(u_o, dtype=float),
        accel_kmhs=np.array(acc_o, dtype=float),
        t_s=np.array(t_o, dtype=float),
        dt_s=dt,
        source=path.name,
        dropped=dropped,
        sections=tuple(sorted(secs)),
        dt_row_s=np.full(n, dt),
    )


def concat_rows(parts: Sequence[FwdRows], source: str = "") -> FwdRows:
    """複数ログの行を連結する。dropped は足し、dt_s は行ごとの秒数の中央値。"""
    dropped: dict[str, int] = dict.fromkeys(DROP_REASONS, 0)
    for p in parts:
        for k, n in p.dropped.items():
            dropped[k] = dropped.get(k, 0) + n
    dt_row = np.concatenate([p.dt_row_s for p in parts]) if parts else np.zeros(0)

    def cat(name: str) -> np.ndarray:
        return np.concatenate([getattr(p, name) for p in parts]) if parts else np.zeros(0)

    return FwdRows(
        speed_kmh=cat("speed_kmh"),
        u_pct=cat("u_pct"),
        accel_kmhs=cat("accel_kmhs"),
        t_s=cat("t_s"),
        dt_s=float(np.median(dt_row)) if len(dt_row) else 0.0,
        source=source or "+".join(p.source for p in parts),
        dropped=dropped,
        sections=tuple(sorted({s for p in parts for s in p.sections})),
        dt_row_s=dt_row,
    )


def speed_cell(speed_kmh: np.ndarray | float, speed_step_kmh: float) -> np.ndarray:
    """車速のマス番号 k = floor(v / step)。マスは [k*step, (k+1)*step)（下端を含む）。"""
    return np.floor(np.asarray(speed_kmh) / speed_step_kmh + 1e-9).astype(int)


# 車速の境界（不均一）: 15 km/h 以下は 2.5 km/h、それより上は 5 km/h（ユーザー決定 2026-10-03）
FINE_SPEED_STEP_KMH = 2.5
FINE_SPEED_MAX_KMH = 15.0
COARSE_SPEED_STEP_KMH = 5.0


def make_speed_edges(
    max_kmh: float,
    fine_step_kmh: float = FINE_SPEED_STEP_KMH,
    fine_max_kmh: float = FINE_SPEED_MAX_KMH,
    coarse_step_kmh: float = COARSE_SPEED_STEP_KMH,
) -> np.ndarray:
    """車速のマスの境界の列 [0, 2.5, 5, ..., 15, 20, 25, ...]（max_kmh を含む最初の境界まで）。

    fine_max_kmh 以下は fine_step_kmh、それより上は coarse_step_kmh。
    全部同じ刻みにしたいときは fine_step_kmh = coarse_step_kmh を渡す。
    """
    fine = np.arange(0.0, fine_max_kmh + 1e-9, fine_step_kmh)
    n = max(int(math.ceil((max_kmh - fine_max_kmh) / coarse_step_kmh - 1e-9)), 1)
    coarse = fine_max_kmh + coarse_step_kmh * np.arange(1, n + 1)
    return np.concatenate([fine, coarse])


def speed_cell_edges(speed_kmh: np.ndarray | float, edges: np.ndarray) -> np.ndarray:
    """境界の列 edges で引いた車速のマス番号。マスは [edges[k], edges[k+1])（下端を含む）。

    最初の境界より下は 0、最後の境界以上は最後のマスに丸める。
    """
    k = np.searchsorted(edges, np.asarray(speed_kmh, dtype=float) + 1e-9, side="right") - 1
    return np.clip(k, 0, len(edges) - 2).astype(int)


def u_cell(u_pct: np.ndarray | float, u_step_pct: float) -> np.ndarray:
    """u のマス番号。0 は惰行（u = 0 ちょうど）だけの独立のマス。

    u > 0: k = ceil(u / step) ≥ 1 → マスは ((k-1)*step, k*step]（0 を含まず上端を含む）
    u < 0: k = −ceil(−u / step) ≤ −1 → マスは [−|k|*step, −(|k|−1)*step)（0 を含まず下端を含む）
    例: step=2 のとき u=2.0 は番号 1（0〜2）、u=2.1 は番号 2（2〜4）、u=−2.0 は番号 −1（−2〜0）。
    """
    u = np.asarray(u_pct, dtype=float)
    mag = np.ceil(np.abs(u) / u_step_pct - 1e-9).astype(int)
    return np.where(u > 0.0, np.maximum(mag, 1), np.where(u < 0.0, -np.maximum(mag, 1), 0))


def cell_counts(
    rows: FwdRows,
    speed_step_kmh: float = 5.0,
    u_step_pct: float = 2.0,
    speed_edges: np.ndarray | None = None,
) -> dict[Cell, float]:
    """マスごとの秒数（行数 × 刻み）。行の無いマスは含まない。

    speed_edges（車速のマスの境界の列）を渡すと speed_step_kmh は使わず不均一なマスで数える。
    """
    out: dict[Cell, float] = {}
    if len(rows) == 0:
        return out
    if speed_edges is not None:
        sc = speed_cell_edges(rows.speed_kmh, speed_edges)
    else:
        sc = speed_cell(rows.speed_kmh, speed_step_kmh)
    uc = u_cell(rows.u_pct, u_step_pct)
    for s, u, d in zip(sc.tolist(), uc.tolist(), rows.dt_row_s.tolist(), strict=True):
        out[(s, u)] = out.get((s, u), 0.0) + d
    return out


@dataclass(frozen=True)
class ThinCell:
    """モードが通るのに学習が薄いマス 1 つ。"""

    speed_idx: int
    u_idx: int
    check_s: float  # 検証ログがそこに居た秒数
    train_s: float  # 学習ログの秒数（無ければ 0）


def thin_cells(
    train: dict[Cell, float], check: dict[Cell, float], min_s: float = 1.0
) -> list[ThinCell]:
    """check が通ったマスのうち、学習側の秒数が min_s 未満のマス（車速・u の順に並べる）。"""
    out = [
        ThinCell(s, u, sec, train.get((s, u), 0.0))
        for (s, u), sec in check.items()
        if train.get((s, u), 0.0) < min_s
    ]
    out.sort(key=lambda c: (c.speed_idx, c.u_idx))
    return out
