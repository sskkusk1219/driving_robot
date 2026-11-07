"""発進のブレーキの帯の中・帯の終わりの FB（2026-11-06 ProblemReport_20261106_発進で速すぎでも
ブレーキを戻す 段1。forward_map.launch_band_step_fb）。

帯（hold_scene 3）の中の保持の PI を使わず、「L 先の偏差の見込み（偏差 ＋ 偏差の傾き × L。
偏差 ＝ 実車速 − 基準、＋が速すぎ）」で 踏み増す・保つ・FF へ少しずつ離す を決める純粋な関数と、
走る前の設定の読み出し。停止側の mode_drive.step_release_brake・step_release_setup と同じ形。
2026-11-06 段8: 帯の中は開度そのものでなく「FF からの上乗せ（出す開度 − FF）」を保つ・増やす・
減らす形（FF の先読みが通る）。帯の終わりの延長は無い。段12: 見込み > +band の間だけは開度
そのものを保つ。ブレーキ開度は正 [%]。
tests は src を import しない（自己完結）。"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from tests.research.config import ConfigError, ResearchConfig
from tests.research.creep_floor import creep_floor_opening_pct
from tests.research.hold_scene import HoldInterval

# launch_band_step が返す印（ログ launch_step_code）
LBS_CODE_NONE = 0  # 何もしない（上乗せ 0 ＝ FF のまま）
LBS_CODE_HELD = 1  # 保った（浅くしなかった）
LBS_CODE_RAISED = 2  # 踏み増した
LBS_CODE_RELEASED_TO_FF = 3  # 上乗せを 0 へ向けて減らした
LBS_CODE_WAIT = 4  # 動かした後の待ち
LBS_CODE_RELEASED_BELOW_FF = 5  # 上乗せを 0 より下（FF より浅く）へ減らした

# 2026-11-07 ProblemReport_20261106_発進で速すぎでもブレーキを戻す 段16（型 A。
# forward_map.launch_breakaway_observed）: 「動き出した」とみなす CAN 車速 [km/h]。
# ユーザー決定 2026-11-07。半分データ由来（シミュレータは停車で 0.000・今日の発進は
# 0.001〜0.004 から動き出しが見える）。実車ではノイズを見て決め直す
LAUNCH_OBS_MOVE_KMH = 0.01

DEV_BAND_KMH = 0.3  # 踏み増す・FF より浅く離す境目（ユーザー決定 2026-11-06。停止側と同じ幅）


def launch_band_forecast(dev_kmh: float, slope_kmhs: float | None, lag_s: float) -> float:
    """L 先の偏差の見込み ＝ 偏差 ＋ 傾き × L（傾きが無ければ偏差のまま）。"""
    return dev_kmh if slope_kmhs is None else dev_kmh + slope_kmhs * lag_s


def _finalize(
    ff_pct: float, offset_pct: float, cap_pct: float, min_pct: float
) -> tuple[float, float]:
    """上乗せ → (上乗せ, 出す開度)。出す開度 ＝ FF ＋ 上乗せを [min, cap] に収め、頭打ちの分は
    上乗せも丸める。FF が cap より深いときは FF をそのまま出す（上限は増やす分の上限で、FF は
    削らない。上乗せは動かさない）。"""
    if ff_pct > cap_pct:
        return offset_pct, ff_pct
    out = min(max(ff_pct + offset_pct, min_pct), cap_pct)
    return out - ff_pct, out


def launch_band_step(
    prev_out_pct: float,
    prev_ff_pct: float,
    ff_pct: float,
    *,
    dev_kmh: float,
    slope_kmhs: float | None,
    lag_s: float,
    dev_band_kmh: float = DEV_BAND_KMH,
    gain_kmhs_pct: float,
    cap_pct: float,
    min_pct: float,
    wait_left: int,
    wait_cycles: int,
) -> tuple[float, float, int, int]:
    """帯の中の 1 周期の判定（2026-11-06 段12。速すぎが大きいときは開度そのものを保つ）。戻り値
    (新しい上乗せ [%] ＝ 出す開度 − FF, 出すブレーキ開度 [%], 印, 残りの待ち周期)。
    前の周期の出す開度 prev_out・FF prev_ff を受け取る（段8 の上乗せ ＝ prev_out − prev_ff）。
    見込み p ＝ dev ＋ slope × L（傾きが無ければ p ＝ dev。増減はしない）。
    - p > +band: 開度そのものを保つ（out ＝ max(prev_out, FF)。FF が下がっても浅くしない・
      FF の方が深ければ FF）。待ちでなく slope > 0 なら prev_out ＋ slope ÷ 効き（上限 cap、
      FF 未満にしない）を試し、増えたら印 2・待ち。増えなければ印 1。待ち中は印 4。
      FF が cap より深いときは out ＝ FF（印 0）
    - p ≤ +band（待ち中も含む）: 段8 と同じ FF ＋ 上乗せ。待ち中は上乗せを動かさない（印 4）。
      0 〜 +band は保つ・p ≤ 0 かつ slope < 0 は 0 まで減らす（印 3）・p < −band かつ slope < 0 は
      0 より下（FF より浅く。下限 min）へ減らす（印 5）。動かしたら wait_cycles 待つ。
      保つ ＝ 上乗せそのまま（上乗せ 0 なら印 0、そうでなければ印 1）。"""
    eps = 1e-9
    p = launch_band_forecast(dev_kmh, slope_kmhs, lag_s)
    if p > dev_band_kmh:
        if ff_pct > cap_pct:  # FF は削らない・増やさない
            return 0.0, ff_pct, LBS_CODE_NONE, max(wait_left - 1, 0)
        hold_out = max(prev_out_pct, ff_pct)
        if wait_left > 0:
            return hold_out - ff_pct, hold_out, LBS_CODE_WAIT, wait_left - 1
        if slope_kmhs is not None and slope_kmhs > 0.0 and gain_kmhs_pct > 0.0:
            cand = max(min(prev_out_pct + slope_kmhs / gain_kmhs_pct, cap_pct), ff_pct)
            if cand > prev_out_pct + eps:
                return cand - ff_pct, cand, LBS_CODE_RAISED, wait_cycles
        return hold_out - ff_pct, hold_out, LBS_CODE_HELD, 0
    prev_offset_pct = prev_out_pct - prev_ff_pct
    off, out = _finalize(ff_pct, prev_offset_pct, cap_pct, min_pct)
    if wait_left > 0:
        return off, out, LBS_CODE_WAIT, wait_left - 1
    hold = (off, out, LBS_CODE_NONE if abs(off) <= eps else LBS_CODE_HELD, 0)
    if slope_kmhs is None or gain_kmhs_pct <= 0.0:
        return hold
    if p > 0.0:
        return hold
    if p < -dev_band_kmh and slope_kmhs < 0.0:
        new, new_out = _finalize(
            ff_pct, prev_offset_pct - (-slope_kmhs) / gain_kmhs_pct, cap_pct, min_pct
        )
        if new < prev_offset_pct - eps:
            return new, new_out, LBS_CODE_RELEASED_BELOW_FF, wait_cycles
        return hold
    if slope_kmhs < 0.0 and prev_offset_pct > 0.0:
        new, new_out = _finalize(
            ff_pct, max(prev_offset_pct - (-slope_kmhs) / gain_kmhs_pct, 0.0), cap_pct, min_pct
        )
        if new < prev_offset_pct - eps:
            return new, new_out, LBS_CODE_RELEASED_TO_FF, wait_cycles
    return hold


def launch_band_raise_wanted(
    *, dev_kmh: float, slope_kmhs: float | None, lag_s: float,
    dev_band_kmh: float = DEV_BAND_KMH,
) -> bool:
    """踏み増したい状態か（見込み > +band かつ傾き > 0）。上限で頭打ちの集計に使う。"""
    return bool(
        slope_kmhs is not None and slope_kmhs > 0.0
        and launch_band_forecast(dev_kmh, slope_kmhs, lag_s) > dev_band_kmh
    )


def launch_band_step_setup(
    cfg: ResearchConfig, table: Any
) -> tuple[float, float, int, float, float] | None:
    """(効き [km/h/s per %], 遅れ L [s], 待つ周期数, 踏み増しの上限 [%], 離す下限 min [%])。
    off は None。on で pkl の meta に効き（stop_release_gain_kmhs_pct）か動き出す開度
    （launch_breakaway_u_pct）、または brake_lag_s（yaml か meta）が無ければ ConfigError
    （走る前に止める）。待ち ＝ round((L ＋ 0.1) ÷ 周期)。上限 ＝ 動き出す開度（min ＋
    |meta launch_breakaway_u_pct|、停車保持より深くしない。停止側の下限と同じ式）。"""
    fm = cfg.forward_map
    if not fm.launch_band_step_fb:
        return None
    meta = getattr(table, "meta", None) or {}
    gain = meta.get("stop_release_gain_kmhs_pct")
    if gain is None or not math.isfinite(float(gain)) or float(gain) <= 0.0:
        raise ConfigError(
            "forward_map.launch_band_step_fb には pkl の meta stop_release_gain_kmhs_pct"
            "（正の戻す側の効き）が要ります。fwd_tool attach-release-gain で付けてください"
        )
    lag = fm.brake_lag_s if fm.brake_lag_s is not None else meta.get("brake_lag_s")
    if lag is None or not math.isfinite(float(lag)):
        raise ConfigError(
            "forward_map.launch_band_step_fb には brake_lag_s（yaml か pkl の meta）が要ります"
        )
    wait = max(1, round((float(lag) + 0.1) / cfg.control.loop_interval_s))
    cap = creep_floor_opening_pct(
        meta, cfg.vehicle.min_brake_opening_pct, cfg.stop_brake_opening_pct
    )
    if cap is None:
        raise ConfigError(
            "forward_map.launch_band_step_fb には pkl の meta launch_breakaway_u_pct"
            "（動き出す開度）が要ります"
        )
    return float(gain), float(lag), wait, float(cap), float(cfg.vehicle.min_brake_opening_pct)


def launch_band_seq_rows(prep: np.ndarray | None, band: np.ndarray | None) -> np.ndarray | None:
    """2026-11-07 段16: 帯のある発進の並びの行。launch_prep の連なり（帯の行も launch_prep）のうち、
    帯の行を 1 つでも含むものの全行を True にする（帯の無い発進＝型 C は対象外）。どちらかが
    None なら None。"""
    if prep is None or band is None:
        return None
    n = len(prep)
    out = np.zeros(n, dtype=bool)
    i = 0
    while i < n:
        if not prep[i]:
            i += 1
            continue
        s = i
        while i < n and prep[i]:
            i += 1
        if bool(band[s:i].any()):
            out[s:i] = True
    return out


class LaunchBreakawayObserver:
    """2026-11-07 段16（型 A）: 帯のある発進ごとに「その場で動き出す開度」u_obs を 1 回測る。

    発進の並び（帯のある launch_prep の連なり）の中で、元の基準が停車速度以下のまま CAN 車速が
    LAUNCH_OBS_MOVE_KMH を超えた最初の周期の、前の周期のブレーキ実開度（絶対の開度 [%]）を
    u_obs とする。基準が停車速度を超えた後は測らない（先に動かなかった発進は None のまま）。
    並びが新しく始まった周期で u_obs を捨てて測り直す。並びを出たら u_obs も捨てる。
    2026-11-08 段20: u_obs は帯の中の踏み増しの上限にだけ使う（前の離しの行の床・帯の最初の
    周期の持ち上げは無くした。深くすると US06 48・U 346 で遅すぎが出たため）。"""

    def __init__(self, move_kmh: float = LAUNCH_OBS_MOVE_KMH) -> None:
        self.move_kmh = move_kmh
        self.u_obs: float | None = None
        self._in_seq = False
        self._done = False  # 基準が動いた後（もう測らない）
        self.measured: list[float] = []  # 測れた発進の u_obs（集計用）
        self.n_sequences = 0

    def update(
        self, *, active: bool, ref_kmh: float, speed_kmh: float,
        prev_brake_actual_pct: float | None, stop_speed_kmh: float,
    ) -> float | None:
        """1 周期の更新。active ＝ 帯のある発進の並びの行。戻り値は今の u_obs。"""
        if not active:
            self._in_seq = False
            self.u_obs = None
            return None
        if not self._in_seq:
            self._in_seq = True
            self.n_sequences += 1
            self.u_obs = None
            self._done = False
        if self.u_obs is None and not self._done:
            if ref_kmh > stop_speed_kmh:
                self._done = True
            elif speed_kmh > self.move_kmh and prev_brake_actual_pct is not None:
                self.u_obs = float(prev_brake_actual_pct)
                self.measured.append(self.u_obs)
        return self.u_obs

    def summary(self) -> str:
        if not self.measured:
            return f"測れた発進 0 / {self.n_sequences} 回"
        return (
            f"測れた発進 {len(self.measured)} / {self.n_sequences} 回・u_obs "
            f"{min(self.measured):.2f}〜{max(self.measured):.2f}%"
        )


def launch_band_obs_inputs(u_obs_pct: float | None, *, cap_pct: float) -> float:
    """2026-11-08 段20（型 A の直し）: 帯の 1 周期の launch_band_step へ渡す踏み増しの上限 cap を
    決める。u_obs が無ければ入力のまま（pkl の動き出す開度）。u_obs があれば cap ＝ u_obs
    （pkl の値との深い方にはしない）。前の出す開度・前の FF は触らない（測った開度まで
    持ち上げない・FF を上書きしない。段16 の持ち上げは US06 48 で車を止め直し、U 346 で上乗せを
    帯へ持ち込んで遅すぎにした）。"""
    return cap_pct if u_obs_pct is None else u_obs_pct


class BandReturnObserver:
    """2026-11-08 段20（B-1）: 帯の発進ごとに「ブレーキが min へ戻る周期数 R」を測る。

    帯の終わり ＝ 帯の最後の周期の次の周期を 0 番とし、周期 k（0 番から数える）の指令を出した
    直後のブレーキ実開度（mode_drive では周期の終わりに読む _last_brake_actual。次の周期の頭で
    安全網が見る値）が min ＋ tol 以下になった最初の k を R とする。帯の終わりの周期で min へ
    離す指令を出すので、11-07 の実機では 0 番の指令 → 3 番の直後の読みが ≤ 4.25% で R ＝ 3。
    発進ごとに 1 回測り、走行の中の最大 r_max を持つ。B-1 で帯を伸ばした発進も、伸ばした後の
    帯の終わりから測る（in_band が伸ばした周期も True だから）。帯に入り直したら測りかけを
    捨てる。max_wait 周期たっても戻らなければ捨てる（測れなかった扱い）。"""

    def __init__(self, tol_pct: float, min_pct: float, max_wait: int = 40) -> None:
        self.thr = min_pct + tol_pct
        self.max_wait = max_wait
        self.measured: list[int] = []
        self._was_band = False
        self._k: int | None = None  # 帯の終わりからの周期数（測っている間）

    @property
    def r_max(self) -> int | None:
        return max(self.measured) if self.measured else None

    def update(self, *, in_band: bool, brake_actual_pct: float | None) -> None:
        """周期の終わりに 1 回呼ぶ。in_band ＝ この周期が帯（B-1 で伸ばした周期を含む）。
        brake_actual_pct ＝ この周期の指令を出した直後のブレーキ実開度。"""
        if in_band:
            self._was_band = True
            self._k = None
            return
        if self._was_band:
            self._was_band = False
            self._k = 0
        if self._k is None:
            return
        if brake_actual_pct is not None and brake_actual_pct <= self.thr:
            self.measured.append(self._k)
            self._k = None
        else:
            self._k += 1
            if self._k > self.max_wait:
                self._k = None


def late_release_rows(gap_rows: int, r_max: int | None) -> int:
    """2026-11-08 段20（B-1）: 帯を伸ばす周期数 s ＝ max(G − (R_max ＋ 1), 0)。G ＝ 帯の終わり
    （0 番）から計画のアクセルの最初の行までの周期数。＋1 は、安全網が見る「前の周期の直後の
    実開度」の分（0 番で離すと R 番の直後に min に着き、アクセルは R ＋ 1 番以降なら止まらない）。
    測った R が無ければ 0（最初の帯の発進は今のまま）。"""
    if r_max is None:
        return 0
    return max(int(gap_rows) - (int(r_max) + 1), 0)


def plan_gap_rows(effort_pct: np.ndarray, last_band_idx: int) -> int:
    """帯の最後の行 last_band_idx の次の行（0 番）から、計画のアクセルの最初の行（effort > 0）
    までの行数 G。間に 0 でない行（ブレーキの行など）があるか、アクセルの行が無ければ 0
    （伸ばさない）。"""
    n = len(effort_pct)
    j = last_band_idx + 1
    while j < n and effort_pct[j] == 0.0:
        j += 1
    if j >= n or effort_pct[j] <= 0.0:
        return 0
    return j - (last_band_idx + 1)


def extend_launch_band(
    bands: list[HoldInterval], last_cycle_s: float, s: int, step_s: float
) -> tuple[list[HoldInterval], int | None]:
    """最後の周期の時刻が last_cycle_s の帯の区間を s 周期だけ後ろへ伸ばす。戻り値 (区間の一覧,
    伸ばした区間の番号［合うものが無い・s ≦ 0 は None で一覧はそのまま］)。"""
    if s <= 0:
        return bands, None
    for k, iv in enumerate(bands):
        if abs((iv.end_s - step_s) - last_cycle_s) <= 0.5 * step_s:
            out = list(bands)
            out[k] = HoldInterval(iv.start_s, iv.length_s + s * step_s, iv.mean_kmh)
            return out, k
    return bands, None


def extend_band_ext(
    ext: list[tuple[float, float]], last_cycle_s: float, s: int, step_s: float
) -> list[tuple[float, float]]:
    """帯の FF の時刻の差し替え表 ext（B-2 と同じ band_ff_time）へ、B-1 で伸ばした s 周期を足す。
    last_cycle_s が B-2 で伸ばした区間の最後の周期なら、その項を伸ばす（FF は元の最後の周期の値
    のまま）。無ければ (last_cycle_s, last_cycle_s ＋ s × 周期) を足す（伸ばした周期の FF は
    今までの最後の周期の値）。"""
    out = list(ext)
    for k, (t_o, t_n) in enumerate(out):
        if abs(t_n - last_cycle_s) <= 0.5 * step_s:
            out[k] = (t_o, t_n + s * step_s)
            return out
    out.append((last_cycle_s, last_cycle_s + s * step_s))
    return out


def stretch_launch_bands(
    bands: list[HoldInterval], events: list[tuple[int, int]], step_s: float
) -> tuple[list[HoldInterval], list[tuple[float, float]]]:
    """2026-11-07 段16 B-2: 計画が選んだ帯の発進の δ（events ＝ (帯の終わりの行 a, δ)）だけ、
    実行中の帯の区間（終わり ≈ a × 周期）を後ろへ伸ばす。戻り値 (伸ばした区間の一覧,
    伸ばした区間ごとの (元の最後の周期の時刻, 伸ばした後の最後の周期の時刻))。
    δ ＝ 0 の発進・合う区間が無い発進はそのまま。伸ばした行の帯の FF は元の最後の周期の値のまま
    （band_ff_time で時刻を差し替える）。"""
    out = list(bands)
    ext: list[tuple[float, float]] = []
    for a, d in events:
        if d <= 0:
            continue
        end_s = a * step_s
        best = min(
            range(len(out)), key=lambda k: abs(out[k].end_s - end_s), default=None
        )
        if best is None or abs(out[best].end_s - end_s) > 2.0 * step_s + 1e-9:
            continue
        iv = out[best]
        out[best] = HoldInterval(iv.start_s, iv.length_s + d * step_s, iv.mean_kmh)
        ext.append((iv.end_s - step_s, iv.end_s - step_s + d * step_s))
    return out, ext


def band_ff_time(t_s: float, ext: list[tuple[float, float]], step_s: float) -> float:
    """帯の FF を引く時刻。B-2 で伸ばした行（元の最後の周期の後〜伸ばした最後の周期）は元の最後の
    周期の時刻（FF は元の帯の終わりの値のまま）、それ以外は t_s のまま。"""
    tol = 0.5 * step_s
    for t_orig, t_new in ext:
        if t_orig + tol < t_s <= t_new + tol:
            return t_orig
    return t_s
