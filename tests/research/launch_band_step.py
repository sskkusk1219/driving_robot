"""発進のブレーキの帯の中・帯の終わりの FB（2026-11-06 ProblemReport_20261106_発進で速すぎでも
ブレーキを戻す 段1。forward_map.launch_band_step_fb）。

帯（hold_scene 3）の中の保持の PI を使わず、「L 先の偏差の見込み（偏差 ＋ 偏差の傾き × L。
偏差 ＝ 実車速 − 基準、＋が速すぎ）」で 踏み増す・保つ・FF へ少しずつ離す を決める純粋な関数と、
走る前の設定の読み出し。停止側の mode_drive.step_release_brake・step_release_setup と同じ形。
ブレーキ開度は正 [%]。tests は src を import しない（自己完結）。"""

from __future__ import annotations

import math
from typing import Any

from tests.research.config import ConfigError, ResearchConfig
from tests.research.creep_floor import creep_floor_opening_pct

# launch_band_step が返す印（ログ launch_step_code）
LBS_CODE_NONE = 0  # 何もしない（FF のまま・FF の方が深い）
LBS_CODE_HELD = 1  # 保った（浅くしなかった）
LBS_CODE_RAISED = 2  # 踏み増した
LBS_CODE_RELEASED_TO_FF = 3  # FF へ少し離した
LBS_CODE_WAIT = 4  # 動かした後の待ち
LBS_CODE_RELEASED_BELOW_FF = 5  # FF より浅く離した
LBS_CODE_END_WAIT = 6  # 帯の終わりで min へ離すのを待った（mode_drive が付ける）

DEV_BAND_KMH = 0.3  # 踏み増す・FF より浅く離す境目（ユーザー決定 2026-11-06。停止側と同じ幅）


def launch_band_forecast(dev_kmh: float, slope_kmhs: float | None, lag_s: float) -> float:
    """L 先の偏差の見込み ＝ 偏差 ＋ 傾き × L（傾きが無ければ偏差のまま）。"""
    return dev_kmh if slope_kmhs is None else dev_kmh + slope_kmhs * lag_s


def launch_band_step(
    prev_pct: float,
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
) -> tuple[float, int, int]:
    """帯の中の 1 周期の判定。戻り値 (出すブレーキ開度 [%], 印, 残りの待ち周期)。
    待ちの間（wait_left > 0）は前の周期のまま（印 4）。傾きが無い（履歴不足）ときは保つ。
    見込み p ＝ dev ＋ slope × L。
    - p > +band かつ slope > 0: prev ＋ slope ÷ 効き だけ踏み増す（上限 cap_pct。印 2）。上限で
      増えなければ保つ
    - 0 < p ≤ +band: 保つ（浅くしない）
    - p < −band かつ slope < 0: prev − (−slope) ÷ 効き（下限 min_pct。FF より浅くてよい。印 5）
    - p ≤ 0 かつ slope < 0（上以外）: FF へ向けて離す（FF より浅くしない。印 3）
    - 上のどれでもない: 保つ
    保つ ＝ max(prev, FF)。FF の方が深ければ FF に従う（印 0）、そうでなければ印 1。
    動かしたら wait_cycles 周期待つ。"""
    if wait_left > 0:
        return prev_pct, LBS_CODE_WAIT, wait_left - 1
    hold_pct = max(prev_pct, ff_pct)
    hold_code = LBS_CODE_NONE if ff_pct > prev_pct else LBS_CODE_HELD
    if slope_kmhs is None or gain_kmhs_pct <= 0.0:
        return hold_pct, hold_code, 0
    p = launch_band_forecast(dev_kmh, slope_kmhs, lag_s)
    if p > dev_band_kmh and slope_kmhs > 0.0:
        new = min(prev_pct + slope_kmhs / gain_kmhs_pct, cap_pct)
        if new > prev_pct:
            return max(new, ff_pct), LBS_CODE_RAISED, wait_cycles
        return hold_pct, hold_code, 0
    if p > 0.0:
        return hold_pct, hold_code, 0
    if p < -dev_band_kmh and slope_kmhs < 0.0:
        new = max(prev_pct - (-slope_kmhs) / gain_kmhs_pct, min_pct)
        if new < prev_pct:
            return new, LBS_CODE_RELEASED_BELOW_FF, wait_cycles
        return hold_pct, hold_code, 0
    if slope_kmhs < 0.0:
        new = max(prev_pct - (-slope_kmhs) / gain_kmhs_pct, ff_pct)
        if new < prev_pct:
            return new, LBS_CODE_RELEASED_TO_FF, wait_cycles
    return hold_pct, hold_code, 0


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
