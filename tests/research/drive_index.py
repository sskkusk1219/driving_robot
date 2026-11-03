"""運転指標（SAE J2951 ／ WLTP の DR・ER・EER・IWR・ASCR・RMSSE）の計算。

基準車速と実車速の 2 本に同じ計算（0.1s 刻みで 5 点の中心移動平均 → 加速度 → 慣性の力・走行抵抗 →
エネルギー）をして、比（実 ÷ 基準 − 1）を出す。numpy だけを使う（pandas は使わない）。
手順 3 のレポート（mode_report.py）が使う。参考の指標で、総合判定には含めない。

ユーザー提供のノート（docs/ideas/sim_ver6.ipynb。リポジトリ外）の calculate_driving_metrics を
元にしているが、次の点を直してある:
    - フェーズ別は、そのフェーズの行だけで足す（ノートは累積値を使い、High の距離は Mid の行を
      読んでいる）
    - RMSSE は丸めない（ノートは差の二乗を整数に丸めている）。フェーズ別もフェーズ内の全行で
      平均する
    - EER_IW はやめて IWR を出す
    - RMSSE の単位は km/h（ノートは mph）
    - 平滑化の端は、点の数が足りないので、ある点だけで平均する（ノートは端を 0 にしている）

指標の定義（いずれも [%]。RMSSE だけ [km/h]）:
    DR    = 実の距離 ÷ 基準の距離 − 1
    ER    = 実の CE ÷ 基準の CE − 1                （CE: 走行に要るエネルギー）
    EER   = 1 − (ER + 1) ÷ (DR + 1)                （距離のずれを引いたエネルギーのずれ）
    IWR   = 実の IWE ÷ 基準の IWE − 1              （IWE: 慣性仕事のうち正の分）
    ASCR  = 実の ASC ÷ 基準の ASC − 1              （ASC: 加速度の絶対値の積分）
    RMSSE = √平均((平滑化した実車速 − 平滑化した基準車速)²)
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

SMOOTH_WINDOW = 5  # 中心移動平均の点数（ノートの window=5）
GRID_STEP_S = 0.1  # 計算の時間刻み [s]（ノートは 0.1s 刻みのデータで window=5 ＝ 0.5s）


@dataclass(frozen=True)
class DriveIndexRow:
    """1 つのフェーズ（または合計）の運転指標。計算できない値は None。"""

    label: str
    distance_ref_km: float
    distance_act_km: float
    dr_pct: float | None
    er_pct: float | None  # 質量・走行抵抗がそろわないとき None
    eer_pct: float | None  # 同上
    iwr_pct: float | None
    ascr_pct: float | None
    rmsse_kmh: float | None
    n_rows: int


@dataclass(frozen=True)
class DriveIndexResult:
    phases: list[DriveIndexRow]  # 区間名の指定がないときは空
    total: DriveIndexRow


@dataclass(frozen=True)
class _Series:
    """1 本の車速から作る中間量（全行ぶん）。"""

    speed_kmh: np.ndarray  # 平滑化前
    smooth_kmh: np.ndarray  # 平滑化後
    dt_s: np.ndarray  # 各行の刻み（先頭は 0）
    acc: np.ndarray  # 加速度 [m/s^2]（先頭は 0）
    dist_km: np.ndarray  # 各行の距離 [km]（平滑化前の車速 × dt）
    ce_j: np.ndarray | None  # 各行の CE [J]（質量・抵抗がそろったとき）
    iwe_j: np.ndarray  # 各行の IWE [J]（質量 mass_for_iw で計算）
    asc: np.ndarray  # 各行の |a|·dt


def smooth_centered(values: np.ndarray, window: int = SMOOTH_WINDOW) -> np.ndarray:
    """中心移動平均。端は点の数が足りないので、ある点だけで平均する。"""
    n = len(values)
    if n == 0:
        return values.astype(float)
    half = window // 2
    csum = np.concatenate(([0.0], np.cumsum(values, dtype=float)))
    idx = np.arange(n)
    lo = np.clip(idx - half, 0, n)
    hi = np.clip(idx + half + 1, 0, n)
    return (csum[hi] - csum[lo]) / (hi - lo)


def _build_series(
    t_s: np.ndarray,
    speed_kmh: np.ndarray,
    mass_kg: float | None,
    a0: float | None,
    a1: float | None,
    a2: float | None,
) -> _Series:
    smooth = smooth_centered(speed_kmh)
    v_ms = smooth / 3.6
    dt = np.diff(t_s, prepend=t_s[0]) if len(t_s) else np.zeros(0)
    acc = np.zeros_like(v_ms)
    if len(v_ms) > 1:
        dv = np.diff(v_ms)
        step = dt[1:]
        with np.errstate(divide="ignore", invalid="ignore"):
            acc[1:] = np.where(step > 0, dv / step, 0.0)
    # IWR は質量が割り算で消えるので、未設定なら 1 kg で計算する
    m_iw = mass_kg if mass_kg is not None else 1.0
    f_inertia_iw = m_iw * acc
    iwe = np.where(f_inertia_iw > 0, v_ms * f_inertia_iw * dt, 0.0)
    ce: np.ndarray | None = None
    if mass_kg is not None and a0 is not None and a1 is not None and a2 is not None:
        f_road = np.where(smooth > 0, a0 + a1 * smooth + a2 * smooth**2, 0.0)
        f_total = mass_kg * acc + f_road
        ce = np.where(f_total > 0, v_ms * f_total * dt, 0.0)
    return _Series(
        speed_kmh=speed_kmh,
        smooth_kmh=smooth,
        dt_s=dt,
        acc=acc,
        dist_km=speed_kmh / 3.6 * dt / 1000.0,
        ce_j=ce,
        iwe_j=iwe,
        asc=np.abs(acc) * dt,
    )


def _ratio_pct(act: float, ref: float) -> float | None:
    if ref <= 0.0 or not np.isfinite(ref) or not np.isfinite(act):
        return None
    return (act / ref - 1.0) * 100.0


def _row(label: str, ref: _Series, act: _Series, mask: np.ndarray) -> DriveIndexRow:
    n = int(mask.sum())
    d_ref = float(ref.dist_km[mask].sum())
    d_act = float(act.dist_km[mask].sum())
    dr = _ratio_pct(d_act, d_ref)
    er: float | None = None
    eer: float | None = None
    if ref.ce_j is not None and act.ce_j is not None:
        er = _ratio_pct(float(act.ce_j[mask].sum()), float(ref.ce_j[mask].sum()))
        if er is not None and dr is not None and (dr / 100.0 + 1.0) != 0.0:
            eer = (1.0 - (er / 100.0 + 1.0) / (dr / 100.0 + 1.0)) * 100.0
    iwr = _ratio_pct(float(act.iwe_j[mask].sum()), float(ref.iwe_j[mask].sum()))
    ascr = _ratio_pct(float(act.asc[mask].sum()), float(ref.asc[mask].sum()))
    rmsse: float | None = None
    if n > 0:
        diff = act.smooth_kmh[mask] - ref.smooth_kmh[mask]
        rmsse = float(np.sqrt(np.mean(diff**2)))
    return DriveIndexRow(
        label=label,
        distance_ref_km=d_ref,
        distance_act_km=d_act,
        dr_pct=dr,
        er_pct=er,
        eer_pct=eer,
        iwr_pct=iwr,
        ascr_pct=ascr,
        rmsse_kmh=rmsse,
        n_rows=n,
    )


def compute_drive_index(
    t_s: Sequence[float],
    ref_kmh: Sequence[float],
    actual_kmh: Sequence[float],
    segments: Sequence[str] | None = None,
    mass_kg: float | None = None,
    road_load_a0_n: float | None = None,
    road_load_a1_n_per_kmh: float | None = None,
    road_load_a2_n_per_kmh2: float | None = None,
    total_label: str = "合計",
) -> DriveIndexResult:
    """運転指標を計算する。

    segments: 各行の区間名（フェーズ名）。None ならフェーズ別は出さない。出てくる順に並べる。
    入力が 0.1s 刻みでなければ 0.1s の格子に補間してから計算する（GRID_STEP_S）。
    mass_kg と走行抵抗の 3 係数がそろわなければ ER・EER は None（IWR は M=1 で計算）。
    """
    t = np.asarray(t_s, dtype=float)
    ref = np.asarray(ref_kmh, dtype=float)
    act = np.asarray(actual_kmh, dtype=float)
    if not (len(t) == len(ref) == len(act)):
        raise ValueError("t_s・ref_kmh・actual_kmh の長さが違います")
    if segments is not None and len(segments) != len(t):
        raise ValueError("segments の長さが t_s と違います")
    # 平滑化の 5 点＝0.5s・加速度の刻みをノートと同じにするため、0.1s 刻みでなければ
    # 0.1s の格子に直す（実機の走行ログは 50ms 刻みの行を含む）。区間名は一番近い元の行のもの
    if len(t) > 1 and not np.allclose(np.diff(t), GRID_STEP_S, atol=0.005):
        grid = np.arange(t[0], t[-1] + 1e-9, GRID_STEP_S)
        if segments is not None:
            near = np.clip(np.searchsorted(t, grid), 0, len(t) - 1)
            prev = np.clip(near - 1, 0, len(t) - 1)
            near = np.where(np.abs(t[prev] - grid) <= np.abs(t[near] - grid), prev, near)
            segments = [segments[i] for i in near]
        ref, act, t = np.interp(grid, t, ref), np.interp(grid, t, act), grid
    s_ref = _build_series(t, ref, mass_kg, road_load_a0_n, road_load_a1_n_per_kmh,
                          road_load_a2_n_per_kmh2)
    s_act = _build_series(t, act, mass_kg, road_load_a0_n, road_load_a1_n_per_kmh,
                          road_load_a2_n_per_kmh2)
    total = _row(total_label, s_ref, s_act, np.ones(len(t), dtype=bool))
    phases: list[DriveIndexRow] = []
    if segments is not None:
        seg = np.asarray(list(segments), dtype=object)
        order: list[str] = []
        for name in seg:
            if name not in order:
                order.append(name)
        for name in order:
            phases.append(_row(str(name), s_ref, s_act, seg == name))
    return DriveIndexResult(phases=phases, total=total)
