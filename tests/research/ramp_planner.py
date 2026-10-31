"""G ランプ走行の段の計画（段2k 段A。車は走らせない純ロジック）。

手順2 の学習運転は「狙いの G で加速 → 同じ G で減速」を G の段ごとに繰り返す。
どの G の段をどの車速まで走るかは、選んだ走行モード（modes.coverage_mode_names）の
「車速 × 加速度」の秒数から自動で決める。このモジュールはその計算と表示だけを行う。

定義:
- 加速度: 基準車速を 1 秒刻みに線形補間し、隣り合う点の差 [km/h/s]。G = 加速度 / G_TO_KMHS。
  車速は区間の中点。中点が STOP_SPEED_KMH 未満（停車）の区間は数えない。
- 車速帯: SPEED_BIN_KMH 幅（0–10, 10–20, …）。
- G の帯: 段の値 levels（昇順）に対し、段 i の帯は |a| が [下端_i, 上端_i)。境界は隣り合う段の
  中点、いちばん下の下端は levels[0] / 2、いちばん上の上端は無限大。加速側（a > 0）と
  減速側（a < 0）は別々に数える。|a| が最下端未満（ほぼ定速）は cruise_s に合計だけ持つ。
- 段の上端の車速: その段の帯で、加速側または減速側の秒数が mode_min_s 以上ある車速帯のうち
  いちばん高い帯の上端 [km/h]（max_speed_kmh で頭打ち。呼ぶ側は top_speed_cap_kmh の上端を渡す）。
  該当が無い段は「走らない」。
- 最小の上端（2026-10-10 段2l-11）: 強い G の段で基準が狙いの G にいる時間を確保するため、
  上端 ＝ max(モードから決めた上端, 最小の上端) を max_speed_kmh で丸める。最小の上端 ＝
  ceil((立ち上げで進む車速 ＋ G × G_TO_KMHS × (min_at_g_s ＋ turn_s ÷ 2)) ÷ 10) × 10。
  立ち上げで進む車速 ＝ (min(edge_g, G) ＋ G) ÷ 2 × G_TO_KMHS × launch_s。min_at_g_s が
  0 以下なら使わない。引き上げた段は表の上端に「*」が付く。
- 1 本の見込み秒数: 2 × 上端 / (g × G_TO_KMHS) + RAMP_OVERHEAD_S + 保持 + 切り替え 2 回
  （2026-10-10 段2l-10。保持・切り替えは呼ぶ側が yaml の値を渡す。既定 0）。
- 周回: 走る段すべてを 1 周（合計が枠 budget_s を超えても 1 周目は削らず over_budget=True）。
  残りの枠に、モードの秒数（加速側＋減速側）が多い段から 2 周目を足す
  （見込み秒数が残りに入る段だけ。入らない段はとばす）。1 段あたり最大 MAX_PASSES 周。

- 戻し（2026-10-14 段2o-5）: 下りの途中でブレーキを山から離す車速。モードの減速の区間（段あり かつ
  acc < 0 が連続）が クリープ車速 ＋ 5 km/h 以上で終わるものについて、いちばん減速が強い 1 秒から
  終わりまでの秒数を `release_s`（車速帯 × 段）に数える。段（いちばん弱い段以外）ごとに
  `plan_release_stations` で車速を決める（戻しの間に下がる車速を引いても クリープ ＋ 5 以上・
  最小の間隔・上端からの間隔）。見込み秒数に戻しの流れの分を足す。

- 小さな動き（2026-10-28 段2。learning.ramp_dither_*）: 有効なら、目標減速度が `dither_max_g` 以下
  の段には下りのブレーキの dither を測る車速の数だけ、保持（上りの途中）のある段にはアクセルの
  dither を min(`dither_accel_max_n`, 保持の数) 回、見込み秒数に足す（上限の見積り。帯に入らなければ
  実際は少ない）。1 回 ＝ 4 つの開度 × `dither_hold_s` ＋ 切り替え 1 回（`turn_s`）。枠
  `budget_s` の扱いは変えず、足した結果 1 周目が枠を超えれば `over_budget` になる（表示の警告・
  「うち dither」で分かる）。

使い方:
    .venv/bin/python -m tests.research.ramp_planner [--config PATH]
  config の modes.coverage_mode_names の各モードを DB から読み、(a) 車速帯 × G の帯の秒数、
  (b) 段の表を出す。調整値は config の learning.ramp_g_levels / ramp_mode_min_s /
  ramp_time_budget_s。
"""

from __future__ import annotations

import argparse
import asyncio
import math
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from tests.research.research_types import G_TO_KMHS, DrivingMode  # G_TO_KMHS = 9.81 × 3.6

SPEED_BIN_KMH = 10.0  # 車速帯の幅 [km/h]（走行モードの車速の見やすさから決めた値。実測根拠なし）
STOP_SPEED_KMH = 0.5  # 区間の中点の車速がこれ未満なら停車として数えない [km/h]
# 仮値。データの根拠なし。2l の実機で決め直す。
# 1 本あたりの停車復帰・つなぎの見込み [s]。段ごとの見込み秒数に足す（周回の割り当てに効く）
RAMP_OVERHEAD_S = 15.0
# 2026-10-13 段2o-2: 保持する車速の下限 ＝ クリープ車速 ＋ これ [km/h]（クリープ域は保持しない）
CRUISE_MIN_MARGIN_KMH = 5.0
MAX_PASSES = 2  # 1 段あたりの最大周回数（仮値。枠に余りがあれば 2 周目を足す）


@dataclass(frozen=True)
class BandCounts:
    """車速帯 × G の帯の秒数。accel_s / decel_s の形は (車速帯の数, 段の数)。"""

    levels: tuple[float, ...]
    speed_bin_kmh: float
    accel_s: np.ndarray
    decel_s: np.ndarray
    cruise_s: float  # |a| が最下端未満（ほぼ定速）の合計秒数
    # 2026-10-13 段2o-2: ほぼ定速の秒数の車速帯別（形は (車速帯の数,)）。保持する車速の決定用
    cruise_by_speed_s: np.ndarray = field(default_factory=lambda: np.zeros(0))
    # 2026-10-14 段2o-4: 「走行中（停車でない車速）から始まる加速」の区間の秒数。形は
    # (始まりの車速帯の数, 段の数)。区間の中の各段の秒数を、始まりの車速の帯に足す
    accel_start_s: np.ndarray = field(default_factory=lambda: np.zeros((0, 0)))
    # 2026-10-14 段2o-5: 「走行中に終わる減速」の戻しの秒数。形は (車速帯の数, 段の数)。減速の区間
    # （段あり かつ acc < 0 が連続）のうち終わりの車速が release_min_kmh 以上のものについて、区間の
    # 中でいちばん減速が強い 1 秒から終わりまでの秒数を、その 1 秒の車速帯・段に足す
    release_s: np.ndarray = field(default_factory=lambda: np.zeros((0, 0)))

@dataclass(frozen=True)
class RampLevel:
    g: float  # 狙いの G
    top_kmh: float  # 上端の車速 [km/h]
    mode_accel_s: float  # モードがこの段の帯で加速していた秒数
    mode_decel_s: float  # 同・減速
    est_s: float  # 1 本の見込み秒数
    passes: int  # 周回数
    raised: bool = False  # 最小の上端でモードから決めた上端を引き上げた段（2026-10-10 段2l-11）
    # 2026-10-13 段2o-2: 上り下りの途中で定速を保持する車速 [km/h]（いちばん弱い段だけ。他は空）
    stations: tuple[float, ...] = ()
    # 2026-10-14 段2o-4: True なら stations は上りだけで保持する（いちばん弱い段以外）
    stations_up_only: bool = False
    # 2026-10-14 段2o-5: 下りで山からブレーキを離す「戻し」を入れる車速 [km/h]（いちばん弱い段以外）
    release_stations: tuple[float, ...] = ()
    # 2026-10-28 段2: 見込み秒数に足した dither の回数の上限（ブレーキ・アクセル）。0 は入れない
    dither_brake_n: int = 0
    dither_accel_n: int = 0


@dataclass(frozen=True)
class RampPlan:
    levels: tuple[RampLevel, ...]  # 走る段（G 昇順）
    skipped: tuple[float, ...]  # 走らない段の G
    total_est_s: float
    budget_s: float
    over_budget: bool  # 1 周目だけで枠を超えた
    cruise_s: float
    dither_s: float = 0.0  # 2026-10-28 段2: total_est_s のうち dither の見込み分（周回を含む）

    def execution_order(
        self,
    ) -> list[tuple[float, float, tuple[float, ...], bool, tuple[float, ...]]]:
        """走る順: 1 周目を G の昇順 → 2 周目を G の昇順。

        要素は (g, top_kmh, stations, stations_up_only)。
        2026-10-13 段2o-2: 3 つ目に保持する車速 stations を足した（保持のない段は空）。
        2026-10-14 段2o-4: 4 つ目に stations_up_only（True なら上りだけで保持）を足した。
        2026-10-14 段2o-5: 5 つ目に戻しを入れる車速 release_stations を足した（戻しのない段は空）。
        """
        order: list[tuple[float, float, tuple[float, ...], bool, tuple[float, ...]]] = []
        # 2026-10-17 段2o-8: 定数ではなく各段の passes の最大まで回す（ramp_max_passes で変わる）
        max_p = max((lv.passes for lv in self.levels), default=0)
        for p in range(1, max_p + 1):
            order.extend(
                (lv.g, lv.top_kmh, lv.stations, lv.stations_up_only, lv.release_stations)
                for lv in self.levels
                if lv.passes >= p
            )
        return order


def level_lower_edges(levels: Sequence[float]) -> list[float]:
    """段 i の帯の下端 [G]（|a| / G_TO_KMHS）。levels[0]/2 と隣り合う段の中点。"""
    edges = [levels[0] / 2.0]
    edges.extend((a + b) / 2.0 for a, b in zip(levels, levels[1:], strict=False))
    return edges


def count_bands(
    speeds: Sequence[tuple[np.ndarray, np.ndarray]], levels: Sequence[float],
    release_min_kmh: float = STOP_SPEED_KMH,
) -> BandCounts:
    """(時刻 [s], 車速 [km/h]) の組のリスト（モード複数）から車速帯 × G の帯の秒数を数える。

    2026-10-14 段2o-5: `release_s`（戻しの秒数）は、減速の区間の終わりの車速が `release_min_kmh`
    以上のものだけ数える。呼ぶ側は クリープ車速 ＋ 5 km/h（`cruise_min_kmh`）を渡す（既定は停車の
    しきい値で、停車で終わる区間だけを除く）。
    """
    lower = np.asarray(level_lower_edges(levels))
    n_lv = len(levels)
    series: list[tuple[np.ndarray, np.ndarray]] = []
    # 走行中から始まる加速の区間 (始まりの車速, 区間内の各 1 秒の段の番号)
    accel_runs: list[tuple[float, list[int]]] = []
    # 2026-10-14 段2o-5: 走行中に終わる減速の区間の戻し (最も強い 1 秒の車速, その段, 戻しの秒数)
    release_runs: list[tuple[float, int, float]] = []
    max_mid = 0.0
    for t_arr, v_arr in speeds:
        t = np.asarray(t_arr, dtype=float)
        v = np.asarray(v_arr, dtype=float)
        if len(t) < 2:
            continue
        grid = np.arange(t[0], t[-1] + 1e-9, 1.0)
        vg = np.interp(grid, t, v)
        mid = (vg[:-1] + vg[1:]) / 2.0
        acc = np.diff(vg)  # 1 秒刻みなので [km/h/s]
        # 2026-10-14 段2o-4: 加速の 1 秒（段あり かつ acc > 0）が連続する区間を時間順で拾う
        lv_all = np.searchsorted(lower, np.abs(acc) / G_TO_KMHS, side="right") - 1
        is_acc = (lv_all >= 0) & (acc > 0)
        j = 0
        while j < len(acc):
            if not is_acc[j]:
                j += 1
                continue
            k = j
            while k < len(acc) and is_acc[k]:
                k += 1
            if vg[j] >= STOP_SPEED_KMH:  # 停車から始まる区間は数えない
                accel_runs.append((float(vg[j]), [int(x) for x in lv_all[j:k]]))
            j = k
        # 2026-10-14 段2o-5: 減速の 1 秒（段あり かつ acc < 0）が連続する区間。終わりの車速が
        # release_min_kmh 以上なら、いちばん減速が強い 1 秒（p）から終わりまでの秒数を数える
        is_dec = (lv_all >= 0) & (acc < 0)
        j = 0
        while j < len(acc):
            if not is_dec[j]:
                j += 1
                continue
            k = j
            while k < len(acc) and is_dec[k]:
                k += 1
            if vg[k] >= release_min_kmh:
                p = j + int(np.argmin(acc[j:k]))  # 減速が最も強い（acc が最小）1 秒
                release_runs.append((float(vg[p]), int(lv_all[p]), float(k - p)))
            j = k
        keep = mid >= STOP_SPEED_KMH
        series.append((mid[keep], acc[keep]))
        if keep.any():
            max_mid = max(max_mid, float(mid[keep].max()))
    n_sp = max(1, int(math.floor(max_mid / SPEED_BIN_KMH)) + 1)
    accel = np.zeros((n_sp, n_lv))
    decel = np.zeros((n_sp, n_lv))
    cruise = 0.0
    cruise_by_speed = np.zeros(n_sp)
    for mid, acc in series:
        g = np.abs(acc) / G_TO_KMHS
        sp = np.minimum((mid // SPEED_BIN_KMH).astype(int), n_sp - 1)
        lv = np.searchsorted(lower, g, side="right") - 1  # 下端を含む。-1 は最下端未満
        for s_i, l_i, a in zip(sp, lv, acc, strict=True):
            if l_i < 0:
                cruise += 1.0
                cruise_by_speed[s_i] += 1.0
            elif a > 0:
                accel[s_i, l_i] += 1.0
            else:
                decel[s_i, l_i] += 1.0
    accel_start = np.zeros((n_sp, n_lv))
    for v_start, lvs in accel_runs:
        s_i = min(int(v_start // SPEED_BIN_KMH), n_sp - 1)
        for l_i in lvs:
            accel_start[s_i, l_i] += 1.0
    release = np.zeros((n_sp, n_lv))
    for v_peak, l_i, sec in release_runs:
        release[min(int(v_peak // SPEED_BIN_KMH), n_sp - 1), l_i] += sec
    return BandCounts(
        tuple(levels), SPEED_BIN_KMH, accel, decel, cruise, cruise_by_speed, accel_start, release
    )


def count_bands_for_modes(
    modes: Sequence[DrivingMode], levels: Sequence[float],
    release_min_kmh: float = STOP_SPEED_KMH,
) -> BandCounts:
    """DrivingMode（reference_speed が SpeedPoint のリスト）を受ける薄いラッパー。"""
    series = [
        (
            np.array([p.time_s for p in m.reference_speed], dtype=float),
            np.array([p.speed_kmh for p in m.reference_speed], dtype=float),
        )
        for m in modes
    ]
    return count_bands(series, levels, release_min_kmh)


def top_speed_cap_kmh(
    modes: Sequence[DrivingMode], margin_kmh: float, vehicle_max_kmh: float
) -> float:
    """コーストダウンと G ランプの上端の車速 [km/h]。

    上端 = min(選んだモードの最高車速の最大 + 余裕, 車両の最高速)。2026-10-07 段2l-4
    （ユーザー指定: 最高速は選んだモードの最高車速 + 5 km/h で十分）。モードが空なら車両の最高速。
    """
    if not modes:
        return vehicle_max_kmh
    return min(max(m.max_speed for m in modes) + margin_kmh, vehicle_max_kmh)


def estimate_s(g: float, top_kmh: float, *, hold_s: float = 0.0, turn_s: float = 0.0) -> float:
    """1 本の見込み秒数: 往復の走行 2 × 上端 / (g × G_TO_KMHS) + つなぎ + 保持と切り替え 2 回。

    2026-10-10 段2l-10: 最高速での保持 `hold_s`（learning.ramp_hold_time_s）と、切り替え
    `turn_s`（learning.ramp_turn_time_s。+G → 0 と 0 → -G の 2 回）を足す。既定 0 なら従来と同じ。
    """
    return 2.0 * top_kmh / (g * G_TO_KMHS) + RAMP_OVERHEAD_S + hold_s + 2.0 * turn_s


def stop_approach_top_kmh(
    g: float, *, release_start_kmh: float, turn_s: float, settle_s: float
) -> float:
    """停止の手前の測定（一定 G で停車まで下りる G ランプ）の上端の車速 [km/h]
    （2026-10-31 段2。ProblemReport_20261031_停止間際 B）。

    ceil((release_start_kmh ＋ G × G_TO_KMHS × (turn_s ÷ 2 ＋ settle_s)) ÷ 5) × 5。
    下りの頭で 0 → −G に切り替える間（turn_s ÷ 2 ぶん）とブレーキの遅れ・立ち上がり（settle_s）に
    車が進むぶんを見込み、停止前の戻しの開始車速（7.5 km/h。この測定では戻さないが比べる目安）を
    切る前に一定 G へ着かせる。5 km/h 刻みに切り上げる。
    """
    raw = release_start_kmh + g * G_TO_KMHS * (turn_s / 2.0 + settle_s)
    return float(math.ceil(raw / 5.0 - 1e-9) * 5)


def stop_approach_est_s(
    g: float, accel_g: float, top_kmh: float, *, hold_s: float, turn_s: float
) -> float:
    """停止の手前の測定 1 本の見込み秒数: 上り top ÷ (accel_g × G_TO_KMHS) ＋ 下り
    top ÷ (g × G_TO_KMHS) ＋ つなぎ ＋ 保持 ＋ 切り替え 2 回（2026-10-31 段2）。"""
    return (
        top_kmh / (accel_g * G_TO_KMHS) + top_kmh / (g * G_TO_KMHS)
        + RAMP_OVERHEAD_S + hold_s + 2.0 * turn_s
    )


def format_stop_approach(cfg: Any) -> str:
    """停止の手前の測定（learning.stop_approach_*）の別行の説明。無効なら空文字。"""
    lr = cfg.learning
    if not lr.stop_approach_enabled:
        return ""
    lines = [
        "停止の手前の測定（一定 G で停車まで。パターン列の先頭。"
        f"上りは {lr.stop_approach_accel_g:g}G・上端で {lr.stop_approach_hold_s:g} 秒保持）:"
    ]
    total = 0.0
    for g, n in zip(lr.stop_approach_g_levels, lr.stop_approach_counts, strict=True):
        top = stop_approach_top_kmh(
            g, release_start_kmh=lr.stop_release_start_kmh, turn_s=lr.ramp_turn_time_s,
            settle_s=lr.stop_approach_settle_s,
        )
        est = stop_approach_est_s(
            g, lr.stop_approach_accel_g, top, hold_s=lr.stop_approach_hold_s,
            turn_s=lr.ramp_turn_time_s,
        )
        total += est * n
        lines.append(f"  下り {g:g}G × {n} 本: 上端 {top:g} km/h、1 本の見込み {est:.0f} 秒")
    lines.append(f"  見込みの合計 {total:.0f} 秒（{total / 60.0:.1f} 分。上の段の表には含まない）")
    return "\n".join(lines)


def _band_station_kmh(k: int, width: float, fine_max_kmh: float, coarse_step_kmh: float) -> float:
    """車速帯の番号 k → 保持する車速 [km/h]（2026-10-14 段2o-4 で共通化）。

    帯の中心。`fine_max_kmh` より上は `coarse_step_kmh` 幅の帯の中心。
    """
    lo = k * width
    centre = lo + width / 2.0
    if centre <= fine_max_kmh:
        return centre
    # 細かい刻みの上限より上は coarse_step_kmh 幅の帯の中心（fine_max から数える）
    idx = math.floor((lo - fine_max_kmh) / coarse_step_kmh + 1e-9)
    return fine_max_kmh + (idx + 0.5) * coarse_step_kmh


def plan_cruise_stations(
    counts: BandCounts, *, mode_min_s: float, fine_max_kmh: float, coarse_step_kmh: float,
    min_kmh: float, top_kmh: float,
) -> tuple[float, ...]:
    """途中で定速を保持する車速 [km/h]（2026-10-13 段2o-2）。

    10 km/h の車速帯ごとに、モードの「ほぼ定速」が `mode_min_s` 秒以上ある帯の中心（15, 25, …）。
    `fine_max_kmh` より上は `coarse_step_kmh` 刻みにまとめ（既定 20: 100〜120 → 110、120〜140 →
    130）、どちらかの帯に定速があれば採る。保持する車速は `min_kmh` 未満（クリープ車速 ＋ 5 km/h
    未満）と、段の上端 `top_kmh` 以上を外す。車速はモードと設定から決まり、車に依存しない。
    """
    width = counts.speed_bin_kmh
    cruise = counts.cruise_by_speed_s
    picked: set[float] = set()
    for k, sec in enumerate(cruise):
        if sec < mode_min_s:
            continue
        picked.add(_band_station_kmh(k, width, fine_max_kmh, coarse_step_kmh))
    return tuple(sorted(v for v in picked if min_kmh <= v < top_kmh))


def plan_accel_stations(
    counts: BandCounts, level_idx: int, *, mode_min_s: float, fine_max_kmh: float,
    coarse_step_kmh: float, min_kmh: float, top_kmh: float, min_gap_kmh: float,
) -> tuple[float, ...]:
    """段 `level_idx` で保持する車速 [km/h]（2026-10-14 段2o-4。上りだけで保持する）。

    モードの「走行中から始まる加速」（`accel_start_s`）が `mode_min_s` 秒以上ある始まりの車速帯の
    中心（`plan_cruise_stations` と同じ規則）。`min_kmh` 未満を外す。さらに、昇順に見て直前に採った
    点から `min_gap_kmh` 未満しか離れていない点は外し（低い車速の側を残す）、上端 `top_kmh` までの
    間隔が `min_gap_kmh` 未満の点も外す。間隔の理由: 保持を出て狙いの G に戻り、次の保持の手前で
    緩め始めるので、狭いと狙いの G にいる時間が無くなる。
    """
    arr = counts.accel_start_s
    if arr.ndim != 2 or arr.shape[0] == 0 or level_idx >= arr.shape[1]:
        return ()
    width = counts.speed_bin_kmh
    cand = sorted(
        {
            _band_station_kmh(k, width, fine_max_kmh, coarse_step_kmh)
            for k, sec in enumerate(arr[:, level_idx])
            if sec >= mode_min_s
        }
    )
    eps = 1e-9
    out: list[float] = []
    for v in cand:
        if v < min_kmh:
            continue
        if out and v - out[-1] < min_gap_kmh - eps:
            continue
        if top_kmh - v < min_gap_kmh - eps:
            continue
        out.append(v)
    return tuple(out)


def release_drop_kmh(
    g: float, fracs: Sequence[float], hold_s: Sequence[float], move_s: float
) -> float:
    """戻しの流れの間に下がる車速の見込み [km/h]（2026-10-14 段2o-5）。

    G × G_TO_KMHS × Σ(各区間の秒 × その区間の平均の効き割合)。効き割合は山の開度で 1、移動中は
    直線に変わり、保持中はその割合（効きは開度に比例するとした見積もり）。既定の流れ
    （割合 0.5 → 0、移動 1 秒、保持 1.5・2 秒）は 1×0.75 ＋ 1.5×0.5 ＋ 1×0.25 ＋ 2×0 ＝ 1.75 秒分。
    """
    prev = 1.0
    eff_s = 0.0
    for f, h in zip(fracs, hold_s, strict=True):
        eff_s += move_s * (prev + f) / 2.0 + h * f
        prev = f
    return g * G_TO_KMHS * eff_s


def release_est_s(
    n_stations: int, *, fracs: Sequence[float], hold_s: Sequence[float], move_s: float,
    turn_s: float,
) -> float:
    """戻しの見込み秒数（2026-10-14 段2o-5）。1 回 ＝ 移動 × 割合の数 ＋ 保持の合計 ＋ 切り替え。"""
    return n_stations * (move_s * len(fracs) + sum(hold_s) + turn_s)


def plan_release_stations(
    counts: BandCounts, level_idx: int, *, mode_min_s: float, fine_max_kmh: float,
    coarse_step_kmh: float, min_kmh: float, top_kmh: float, min_gap_kmh: float,
    drop_kmh: float,
) -> tuple[float, ...]:
    """段 `level_idx` の下りで戻す車速 [km/h]（2026-10-14 段2o-5）。

    モードの `release_s`（走行中に終わる減速の戻しの秒数）が `mode_min_s` 秒以上ある車速帯の中心。
    外す: 車速 − `drop_kmh` ＜ `min_kmh`（戻しの間に下がる車速を引いても クリープ ＋ 5 km/h 以上
    で終わること）。間隔は `plan_accel_stations` と同じ（昇順に見て低い側を残す・上端 `top_kmh` ま
    での間隔も `min_gap_kmh` 以上。下りの始めと戻しの後に −G へ戻って `min_at_g_s` いるため）。
    """
    arr = counts.release_s
    if arr.ndim != 2 or arr.shape[0] == 0 or level_idx >= arr.shape[1]:
        return ()
    width = counts.speed_bin_kmh
    cand = sorted(
        {
            _band_station_kmh(k, width, fine_max_kmh, coarse_step_kmh)
            for k, sec in enumerate(arr[:, level_idx])
            if sec >= mode_min_s
        }
    )
    eps = 1e-9
    out: list[float] = []
    for v in cand:
        if v - drop_kmh < min_kmh - eps:
            continue
        if out and v - out[-1] < min_gap_kmh - eps:
            continue
        if top_kmh - v < min_gap_kmh - eps:
            continue
        out.append(v)
    return tuple(out)


def cruise_hold_est_s(
    n_stations: int, *, cruise_hold_s: float, turn_s: float, settle_s: float = 3.0,
    both_ways: bool = True,
) -> float:
    """保持の見込み秒数: 1 回 ＝ 保持 ＋ 切り替え 2 回 ＋ 落ち着き × 回数（× 2 は上り下り）。

    `both_ways` が False（上りだけで保持する段。2026-10-14 段2o-4）なら × 2 しない。
    """
    return n_stations * (cruise_hold_s + 2.0 * turn_s + settle_s) * (2.0 if both_ways else 1.0)


def dither_est_s(n: int, *, hold_s: float, turn_s: float) -> float:
    """dither の見込み秒数: 1 回 ＝ 4 つの開度 × 保持 ＋ 切り替え 1 回（g_ref の戻し）。"""
    return max(0, n) * (4.0 * hold_s + turn_s)


def min_top_kmh(
    g: float, *, min_at_g_s: float, launch_s: float, turn_s: float, edge_g: float
) -> float:
    """最小の上端 [km/h]（2026-10-10 段2l-11）。基準が狙いの G に `min_at_g_s` 秒いられる車速。

    立ち上げで進む車速 ＝ (min(edge_g, G) ＋ G) ÷ 2 × G_TO_KMHS × launch_s。それに
    G × G_TO_KMHS × (min_at_g_s ＋ turn_s ÷ 2) を足し、SPEED_BIN_KMH 刻みに切り上げる
    （浮動小数の誤差で境目が 1 段上がらないよう 1e-9 を引く）。`min_at_g_s` が 0 以下なら 0。
    """
    if min_at_g_s <= 0.0:
        return 0.0
    launch_kmh = (min(edge_g, g) + g) / 2.0 * G_TO_KMHS * launch_s
    need = launch_kmh + g * G_TO_KMHS * (min_at_g_s + turn_s / 2.0)
    return math.ceil(need / SPEED_BIN_KMH - 1e-9) * SPEED_BIN_KMH


def plan_ramps(
    counts: BandCounts, *, mode_min_s: float, max_speed_kmh: float, budget_s: float,
    hold_s: float = 0.0, turn_s: float = 0.0,
    min_at_g_s: float = 0.0, launch_s: float = 0.0, edge_g: float = 0.0,
    cruise_hold_s: float = 0.0, cruise_fine_max_kmh: float = 100.0,
    cruise_coarse_step_kmh: float = 20.0, cruise_min_kmh: float = 0.0,
    release_fracs: Sequence[float] = (), release_move_s: float = 1.0,
    release_hold_s: Sequence[float] = (),
    max_passes: int = MAX_PASSES,
    dither_enabled: bool = False, dither_hold_s: float = 1.5, dither_brake_n: int = 0,
    dither_max_g: float = 0.1, dither_accel_max_n: int = 0,
) -> RampPlan:
    """段ごとの上端の車速・見込み秒数・周回数を決める。

    `hold_s`・`turn_s` は `estimate_s` へ。2026-10-10 段2l-11: 上端 ＝ max(モードから決めた上端,
    最小の上端 `min_top_kmh`) を `max_speed_kmh` で丸める。見込み秒数は引き上げ後の上端で計算する。
    2026-10-13 段2o-2: いちばん弱い段（counts.levels の最小）だけ、`plan_cruise_stations` で
    決めた車速で途中の定速保持を入れる（`cruise_hold_s` が 0 以下なら入れない）。見込み秒数に
    保持の分を足す。
    2026-10-14 段2o-4: それ以外の段は `plan_accel_stations`（走行中から始まる加速の数から。上り
    だけで保持）。最小の間隔 ＝ G × G_TO_KMHS × (min_at_g_s ＋ turn_s)。見込みは上りの分だけ。
    2026-10-14 段2o-5: いちばん弱い段以外に、下りの戻し（`plan_release_stations`）を入れる。
    `release_fracs` が空なら入れない。最小の車速は `cruise_min_kmh`（クリープ ＋ 5 km/h）。
    見込み秒数に戻しの分（`release_est_s`）を足す。
    2026-10-17 段2o-8: `max_passes` ＝ 1 段あたりの最大周回数（learning.ramp_max_passes）。1 なら
    枠が余っても 2 周目を足さない。
    2026-10-28 段2: `dither_enabled` なら dither の見込み（上限）を足す。ブレーキは `dither_brake_n`
    ＝ 測る車速の数を、目標減速度 g ≤ `dither_max_g` の段に。アクセルは保持のある段に
    min(`dither_accel_max_n`, 保持の数) 回。
    """
    runs: list[RampLevel] = []
    skipped: list[float] = []
    for i, g in enumerate(counts.levels):
        enough = np.flatnonzero(
            (counts.accel_s[:, i] >= mode_min_s) | (counts.decel_s[:, i] >= mode_min_s)
        )
        if len(enough) == 0:
            skipped.append(g)
            continue
        mode_top = (int(enough.max()) + 1) * counts.speed_bin_kmh
        floor_top = min_top_kmh(
            g, min_at_g_s=min_at_g_s, launch_s=launch_s, turn_s=turn_s, edge_g=edge_g
        )
        top = min(max(mode_top, floor_top), max_speed_kmh)
        raised = top > min(mode_top, max_speed_kmh)
        stations: tuple[float, ...] = ()
        up_only = False
        if i == 0 and cruise_hold_s > 0.0 and len(counts.cruise_by_speed_s) > 0:
            stations = plan_cruise_stations(
                counts, mode_min_s=mode_min_s, fine_max_kmh=cruise_fine_max_kmh,
                coarse_step_kmh=cruise_coarse_step_kmh, min_kmh=cruise_min_kmh, top_kmh=top,
            )
        elif i > 0 and cruise_hold_s > 0.0 and counts.accel_start_s.size > 0:
            # 2026-10-14 段2o-4: 他の段は「走行中から始まる加速」の数から決め、上りだけ保持する
            stations = plan_accel_stations(
                counts, i, mode_min_s=mode_min_s, fine_max_kmh=cruise_fine_max_kmh,
                coarse_step_kmh=cruise_coarse_step_kmh, min_kmh=cruise_min_kmh, top_kmh=top,
                min_gap_kmh=g * G_TO_KMHS * (min_at_g_s + turn_s),
            )
            up_only = bool(stations)
        release: tuple[float, ...] = ()
        if i > 0 and len(release_fracs) > 0 and counts.release_s.size > 0:
            release = plan_release_stations(
                counts, i, mode_min_s=mode_min_s, fine_max_kmh=cruise_fine_max_kmh,
                coarse_step_kmh=cruise_coarse_step_kmh, min_kmh=cruise_min_kmh, top_kmh=top,
                min_gap_kmh=g * G_TO_KMHS * (min_at_g_s + turn_s),
                drop_kmh=release_drop_kmh(g, release_fracs, release_hold_s, release_move_s),
            )
        d_brake = dither_brake_n if dither_enabled and g <= dither_max_g else 0
        d_accel = min(dither_accel_max_n, len(stations)) if dither_enabled else 0
        est = estimate_s(g, top, hold_s=hold_s, turn_s=turn_s) + cruise_hold_est_s(
            len(stations), cruise_hold_s=cruise_hold_s, turn_s=turn_s, both_ways=not up_only
        ) + release_est_s(
            len(release), fracs=release_fracs, hold_s=release_hold_s, move_s=release_move_s,
            turn_s=turn_s,
        ) + dither_est_s(d_brake + d_accel, hold_s=dither_hold_s, turn_s=turn_s)
        runs.append(
            RampLevel(
                g=g,
                top_kmh=top,
                mode_accel_s=float(counts.accel_s[:, i].sum()),
                mode_decel_s=float(counts.decel_s[:, i].sum()),
                est_s=est,
                passes=1,
                raised=raised,
                stations=stations,
                stations_up_only=up_only,
                release_stations=release,
                dither_brake_n=d_brake,
                dither_accel_n=d_accel,
            )
        )
    remaining = budget_s - sum(r.est_s for r in runs)
    over_budget = remaining < 0.0
    passes = {r.g: 1 for r in runs}
    by_mode_s = sorted(runs, key=lambda r: (-(r.mode_accel_s + r.mode_decel_s), r.g))
    for _ in range(2, max_passes + 1):
        for r in by_mode_s:
            if passes[r.g] < max_passes and r.est_s <= remaining:
                passes[r.g] += 1
                remaining -= r.est_s
    final = tuple(
        RampLevel(
            r.g, r.top_kmh, r.mode_accel_s, r.mode_decel_s, r.est_s, passes[r.g], r.raised,
            r.stations, r.stations_up_only, r.release_stations,
            r.dither_brake_n, r.dither_accel_n,
        )
        for r in runs
    )
    return RampPlan(
        levels=final,
        skipped=tuple(skipped),
        total_est_s=sum(r.est_s * r.passes for r in final),
        budget_s=budget_s,
        over_budget=over_budget,
        cruise_s=counts.cruise_s,
        dither_s=sum(
            dither_est_s(r.dither_brake_n + r.dither_accel_n, hold_s=dither_hold_s, turn_s=turn_s)
            * r.passes
            for r in final
        ),
    )


def cruise_kwargs(cfg: Any) -> dict[str, Any]:
    """`plan_ramps` へ渡す保持・戻しの設定（2026-10-13 段2o-2。呼び出し側 3 か所で共有）。

    2026-10-14 段2o-5: 戻しの設定（release_fracs・release_move_s・release_hold_s）も含める。
    """
    lr = cfg.learning
    return {
        "release_fracs": tuple(lr.ramp_release_fracs),
        "release_move_s": lr.ramp_release_move_s,
        "release_hold_s": tuple(lr.ramp_release_hold_s),
        "cruise_hold_s": lr.ramp_cruise_hold_s,
        "cruise_fine_max_kmh": lr.ramp_cruise_fine_max_kmh,
        "cruise_coarse_step_kmh": lr.ramp_cruise_coarse_step_kmh,
        "cruise_min_kmh": cfg.feedforward.creep_speed_kmh + CRUISE_MIN_MARGIN_KMH,
        # 2026-10-28 段2: 小さな動き（dither）の見込み
        "dither_enabled": lr.ramp_dither_enabled,
        "dither_hold_s": lr.ramp_dither_hold_s,
        "dither_brake_n": len(lr.ramp_dither_brake_speeds_kmh),
        "dither_max_g": lr.ramp_dither_max_g,
        "dither_accel_max_n": lr.ramp_dither_accel_max_n,
    }


def format_band_table(counts: BandCounts) -> str:
    """(a) 車速帯 × G の帯の秒数（加速側・減速側）のマークダウン表。"""
    head = "| 車速帯 [km/h] | " + " | ".join(f"{g:g}G" for g in counts.levels) + " |"
    sep = "|---|" + "---|" * len(counts.levels)
    lower = level_lower_edges(counts.levels)
    out = [
        f"帯の下端 [G]: {', '.join(f'{x:.4f}' for x in lower)}"
        f"（上端は次の段の下端。最後は無限大）。ほぼ定速（最下端未満）: {counts.cruise_s:.0f} 秒",
        "",
    ]
    tables = [("加速側", counts.accel_s), ("減速側", counts.decel_s)]
    if counts.accel_start_s.size > 0:
        tables.append(("走行中から始まる加速（始まりの車速帯）", counts.accel_start_s))
    if counts.release_s.size > 0:
        tables.append(("走行中に終わる減速の戻し（山の車速帯）", counts.release_s))
    for title, arr in tables:
        out += [f"{title}の秒数（形式: 秒）", "", head, sep]
        for k in range(arr.shape[0]):
            lo, hi = k * counts.speed_bin_kmh, (k + 1) * counts.speed_bin_kmh
            cells = " | ".join(f"{x:.0f}" if x else "-" for x in arr[k])
            out.append(f"| {lo:g}–{hi:g} | {cells} |")
        out.append("")
    return "\n".join(out)


def format_plan_table(plan: RampPlan) -> str:
    """(b) 段の表と合計・枠・超過の警告のマークダウン。"""
    out = [
        "| 狙いの G | 上端の車速 [km/h] | モード秒数 加速 | モード秒数 減速 | 1 本の見込み [s] "
        "| 周回数 | 小計 [s] |",
        "|---|---|---|---|---|---|---|",
    ]
    rows = {lv.g: lv for lv in plan.levels}
    for g in sorted([*rows, *plan.skipped]):
        if g in rows:
            lv = rows[g]
            top = f"{lv.top_kmh:g}*" if lv.raised else f"{lv.top_kmh:g}"
            out.append(
                f"| {g:g} | {top} | {lv.mode_accel_s:.0f} | {lv.mode_decel_s:.0f} "
                f"| {lv.est_s:.0f} | {lv.passes} | {lv.est_s * lv.passes:.0f} |"
            )
        else:
            out.append(f"| {g:g} | 走らない | - | - | - | - | - |")
    out += [
        "",
        f"合計の見込み: {plan.total_est_s:.0f} 秒 / 枠: {plan.budget_s:.0f} 秒"
        f"（走行 {plan.total_est_s / 60.0:.1f} 分）",
    ]
    for lv in plan.levels:
        if lv.stations:
            way = (
                f"上りだけ {len(lv.stations)} 回" if lv.stations_up_only
                else f"上り下りの各 {len(lv.stations)} 回"
            )
            out.append(f"{lv.g:g}G: 保持 {', '.join(f'{v:g}' for v in lv.stations)} km/h（{way}）")
    for lv in plan.levels:
        if lv.release_stations:
            out.append(
                f"{lv.g:g}G: 下りの戻し {', '.join(f'{v:g}' for v in lv.release_stations)} km/h"
                f"（{len(lv.release_stations)} 回）"
            )
    for lv in plan.levels:
        if lv.dither_brake_n or lv.dither_accel_n:
            out.append(
                f"{lv.g:g}G: 小さな動き（dither）ブレーキ 最大 {lv.dither_brake_n} 回・"
                f"アクセル 最大 {lv.dither_accel_n} 回"
            )
    if plan.dither_s > 0.0:
        over = "（枠を超えている）" if plan.total_est_s > plan.budget_s else ""
        out.append(
            f"うち dither の見込み（上限）: {plan.dither_s:.0f} 秒"
            f"（合計 {plan.total_est_s:.0f} 秒 / 枠 {plan.budget_s:.0f} 秒{over}）"
        )
    if any(lv.raised for lv in plan.levels):
        out.append(
            "* 上端の車速は、モードから決めた上端より高い「最小の上端」"
            "（learning.ramp_min_at_g_s）で引き上げた値。"
        )
    if plan.over_budget:
        out.append(
            "警告: 1 周目だけで枠を超えています（1 周目は削らない）。"
            "learning.ramp_time_budget_s か段の設定を見直してください。"
        )
    return "\n".join(out)


async def _run(config_path: Path) -> int:
    from tests.research.config import load_config  # noqa: PLC0415 - CLI のときだけ
    from tests.research.mode_drive import load_mode  # noqa: PLC0415 - 循環と重い import を避ける

    cfg = load_config(config_path)
    names = cfg.modes.coverage_mode_names
    modes = [await load_mode(cfg, n) for n in names]
    lr = cfg.learning
    counts = count_bands_for_modes(modes, lr.ramp_g_levels, cruise_kwargs(cfg)["cruise_min_kmh"])
    top_cap = top_speed_cap_kmh(modes, lr.ramp_top_margin_kmh, cfg.vehicle.max_speed_kmh)
    plan = plan_ramps(
        counts,
        mode_min_s=lr.ramp_mode_min_s,
        max_speed_kmh=top_cap,
        budget_s=lr.ramp_time_budget_s,
        max_passes=lr.ramp_max_passes,  # 2026-10-17 段2o-8
        hold_s=lr.ramp_hold_time_s,
        turn_s=lr.ramp_turn_time_s,
        min_at_g_s=lr.ramp_min_at_g_s,
        launch_s=lr.ramp_launch_time_s,
        edge_g=lr.ramp_edge_g,
        **cruise_kwargs(cfg),
    )
    print(f"モード: {', '.join(names)}  最高速 {top_cap:g} km/h"
          f"（モードの最高車速 + {lr.ramp_top_margin_kmh:g}・車両 {cfg.vehicle.max_speed_kmh:g}"
          f" で頭打ち）")
    print(f"最小秒数 {lr.ramp_mode_min_s:g} s / 1 本のつなぎ {RAMP_OVERHEAD_S:g} s（仮値）\n")
    print("## (a) 車速帯 × G の帯の秒数\n")
    print(format_band_table(counts))
    print("## (b) 段の表\n")
    print(format_plan_table(plan))
    round_trip = sum(lv.est_s - RAMP_OVERHEAD_S for lv in plan.levels)
    print(f"\n1 周の往復秒数の合計（つなぎ抜き）: {round_trip:.0f} 秒")
    order = ", ".join(f"({g:g}, {t:g})" for g, t, *_ in plan.execution_order())
    print(f"実行順 (G, 上端 km/h): {order}")
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    from tests.research.config import DEFAULT_CONFIG_PATH  # noqa: PLC0415

    ap = argparse.ArgumentParser(description="G ランプ走行の段の計画（モードの秒数から）")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    args = ap.parse_args(argv)
    return asyncio.run(_run(args.config))


if __name__ == "__main__":
    raise SystemExit(main())
