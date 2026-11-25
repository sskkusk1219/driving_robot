"""手順 2 走行後の緩減速 → 停車保持（本番 RobotController._decelerate_to_stop の研究用置き換え）。

2026-11-24（ProblemReport_20261124_停車の0.2G減速）: 「0.5mm 踏んでは 1 秒待つ」刻みをやめ、
0.1 s 周期の PI にした（刻みは最大でも 0.11〜0.13G しか出ず 0.2G に届かなかった）。
昔の 0.1 s 差分（標準偏差 2〜3 km/h/s）で踏んでは戻すを繰り返した反省から、減速度は直近
pi_slope_window_s の車速の最小二乗の傾きで測り、P・I の強さは局所の効き（表の meta）で割る
（設定は decel_stop セクション）:
    1. アクセルを原点へ
    2. 接近   … ブレーキを効き始め（min ＋ meta brake_onset_u_pct。無ければ min）まで最高速度で
               動かす（遊びの中で減速は出ない）。すでに深く踏んでいればその位置（最大開度まで）から
               start_pulse を渡せば、その開度から始める（CAN 断の刻みから移るとき。接近を飛ばす）
    3. PI     … 誤差 e ＝ 目標 G − 測った減速 G。開度 ＝ 基準開度 ＋ Kp·e ＋ (Kp/Ti)·∫e dt
               Kp [%/G] ＝ p_fraction ÷ k（k ＝ 局所の効き [G/%]。表の傾き × meta local_gain_brake。
               無ければ default_local_gain_g_per_pct）、Ti ＝ ti_lag_multiple × ブレーキの遅れ
               （meta dither_brake_lag_s。無ければ default_lag_s）。
               枠は 効き始め〜最大開度。枠に当たって e がさらに押す向きの間は積分を止める。
               release_above_g 超えは e < 0 なので積分が減る向きだけに働き、開度を戻す。
               窓が埋まる（pi_slope_window_s 経つ）までは開度を保つ。
               車速を読むたびに VEHICLE_STOP_SPEED_KMH 未満なら停止確認
    4. 停車保持 … 停車保持開度まで刻んで動かす（浅い所にいれば踏み、max まで踏んでいれば戻す）
"""

from __future__ import annotations

import asyncio
import math
from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

import numpy as np

from tests.research import local_gain
from tests.research.config import ResearchConfig
from tests.research.drive_log import SECTION_DECEL_TO_STOP, SessionLog, mark
from tests.research.hardware import DriveError, ResearchHardware
from tests.research.pedal_search import HOLD_STEP_DWELL_S, SPEED_SAMPLE_INTERVAL_S, step_to_position
from tests.research.research_types import G_TO_KMHS, VEHICLE_STOP_SPEED_KMH, VehicleProfile
from tests.research.term import say
from tests.research.vehicle import opening_to_pulse, pulse_to_opening

PHASE_APPROACH = "APPROACH"
PHASE_PI = "PI"
PHASE_STOP_HOLD = "STOP_HOLD"

# 表の傾きを引く開度の幅 [%]（x ± これ の差分）。仮値（local_gain.CRUISE_TABLE_H_PCT と同じ）。
# 効く先: 局所の効き k の求め方だけ
_SLOPE_H_PCT = 0.25
# 局所の効きを引く開度の下限は「効き始め ＋ これ」。効き始めの真上では差分の半分が遊びに掛かって
# 傾きが小さく出るため。仮値。効く先: 踏み始めの Kp だけ
_SLOPE_ABOVE_ONSET_PCT = 0.5
# 経過を say に出す間隔 [s]（制御には使わない）
_SAY_INTERVAL_S = 1.0


@dataclass(frozen=True)
class StopDecelResult:
    hold_pct: float
    duration_s: float
    max_decel_g: float  # PI が測った減速度の最大
    cycles: int  # PI の周期数


def decel_from_slope(history: Sequence[tuple[float, float]]) -> float:
    """(時刻 [s], 車速 [km/h]) 列の最小二乗の傾きから減速度 [km/h/s] を返す（正が減速）。"""
    n = len(history)
    if n < 2:
        return 0.0
    t_mean = sum(t for t, _ in history) / n
    v_mean = sum(v for _, v in history) / n
    den = sum((t - t_mean) ** 2 for t, _ in history)
    if den <= 0.0:
        return 0.0
    return -sum((t - t_mean) * (v - v_mean) for t, v in history) / den


def _finite(x: Any) -> float | None:
    try:
        f = float(x)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def load_brake_table(cfg: ResearchConfig) -> Any | None:
    """FF モデル（順モデルの表の pkl）を読んで返す。無い・読めない・順モデルでないなら None
    （緩減速はいちばん弱い既定で動く）。表を持たない呼び出し元（非常停止・自動運転の終わり）用。"""
    from pathlib import Path  # noqa: PLC0415

    from tests.research.fwd_map import load_forward_map  # noqa: PLC0415

    ff_cfg = cfg.feedforward
    try:
        if not ff_cfg.is_model_trained:
            return None
        return load_forward_map(Path(ff_cfg.model_path))
    except Exception:  # noqa: BLE001 - 表が読めなくても止められる側（既定）に倒す
        return None


def brake_onset_pct(table: Any | None) -> float:
    """表の meta brake_onset_u_pct（min からの量 [%]）。無ければ 0（＝ min から始める）。"""
    meta = getattr(table, "meta", None) or {}
    onset = _finite(meta.get("brake_onset_u_pct"))
    return max(0.0, onset) if onset is not None else 0.0


def brake_lag_s(table: Any | None, default_s: float) -> float:
    """表の meta dither_brake_lag_s（手順 2 の dither で測ったブレーキの遅れ）。無ければ既定。"""
    meta = getattr(table, "meta", None) or {}
    lag = _finite(meta.get("dither_brake_lag_s"))
    return lag if lag is not None and lag > 0.0 else default_s


def local_gain_k_g_per_pct(
    table: Any | None, v_kmh: float, opening_pct: float, *, min_pct: float, default_k: float,
) -> float:
    """今の車速・ブレーキ開度での局所の効き k [G/%]（1% 踏み増すと増える減速度）。

    ＝ 表の減速度の開度方向の傾き × meta local_gain_brake の倍率（mode_drive の
    `_local_gain_ratio` と同じ引き方: 効き始めからの量 × 車速）。表が無い・傾きが出ない・
    0 以下のときは default_k（強めの効きを仮定した値＝Kp が小さく弱く踏む側）。"""
    if table is None:
        return default_k
    meta = getattr(table, "meta", None) or {}
    onset = brake_onset_pct(table)
    try:
        x = max(opening_pct - min_pct, onset + _SLOPE_ABOVE_ONSET_PCT)  # min からの量 [%]
        lo, hi = x - _SLOPE_H_PCT, x + _SLOPE_H_PCT
        # 表の u はブレーキが負（u ＝ −(開度 − min)）、加速度も負（減速）
        a_lo = float(np.asarray(table.accel(float(v_kmh), -lo)))
        a_hi = float(np.asarray(table.accel(float(v_kmh), -hi)))
        slope_kmhs = (a_lo - a_hi) / (hi - lo)  # 正 ＝ 踏むほど減速が増える
        ratio = local_gain.local_gain_at(meta.get("local_gain_brake"), float(v_kmh), x - onset)
    except Exception:  # noqa: BLE001 - 表が読めなければ既定（停車は止められる側に倒す）
        return default_k
    k = slope_kmhs * ratio / G_TO_KMHS
    if not math.isfinite(k) or k <= 0.0:
        return default_k
    return k


async def decelerate_to_stop(
    hw: ResearchHardware,
    cfg: ResearchConfig,
    profile: VehicleProfile,
    *,
    log: SessionLog | None = None,
    table: Any | None = None,
    start_pulse: int | None = None,
) -> StopDecelResult:
    """0.2G の PI で停車させ、停車保持開度で保持した状態で返す。停車しなければ DriveError。

    table: 順モデルの表（ForwardTable。.meta と .accel を使う）。None ならいちばん弱い既定で動く。
    start_pulse: 今のブレーキ位置 [pulse]（途中の開度から PI を始める。None なら読む）。
    """
    d = cfg.decel_stop
    ff = profile.feedforward_params
    loop = asyncio.get_running_loop()
    started = loop.time()
    step = max(1, round(d.step_mm * 100))
    hold_pct = max(0.0, min(ff.stop_brake_opening_pct, profile.max_brake_opening))
    hold_pulse = opening_to_pulse(hold_pct)
    # 上限はブレーキの最大開度（効きが足りなければ停車保持開度より深く踏む）
    ceiling = opening_to_pulse(max(0.0, profile.max_brake_opening))
    ceiling_pct = pulse_to_opening(ceiling)
    min_pct = ff.brake_deadband_pct
    onset_pct = brake_onset_pct(table)
    approach = min(ceiling, opening_to_pulse(min_pct + onset_pct))
    floor_pct = pulse_to_opening(approach)
    lag_s = brake_lag_s(table, d.default_lag_s)
    ti_s = d.ti_lag_multiple * lag_s
    target_kmhs = d.target_decel_g * G_TO_KMHS
    window_s = d.pi_slope_window_s

    mark(log, SECTION_DECEL_TO_STOP, PHASE_APPROACH)
    say(f"緩減速: 目標 {d.target_decel_g:g}G の PI（{d.release_above_g:g}G 超えで戻す）、"
        f"減速度の窓 {window_s:g}s・Ti {ti_s:g}s（遅れ {lag_s:g}s）、"
        f"枠 {floor_pct:.2f}〜{ceiling_pct:.2f}%"
        f"（効き始め {'表あり' if table is not None else '表なし＝min'}）")
    await hw.accel.move_to_position(0)
    current = start_pulse if start_pulse is not None else await hw.brake.read_position()
    pos = min(max(current, approach), ceiling)
    if current > approach:
        say(f"  接近: ブレーキはすでに {pulse_to_opening(current):.2f}% を踏んでいるため "
            f"{pulse_to_opening(pos):.2f}% から始めます")
    else:
        say(f"  接近: ブレーキを {pos} pulse（{pulse_to_opening(pos):.2f}% = min {min_pct:.2f}% ＋ "
            f"効き始め {onset_pct:.2f}%）まで最高速度で動かします")
    await hw.brake.move_to_position(pos)

    mark(log, SECTION_DECEL_TO_STOP, PHASE_PI)
    base_pct = pulse_to_opening(pos)  # 基準開度（PI の出力 0 のときの開度）
    integral_pct = 0.0
    history: deque[tuple[float, float]] = deque()
    max_decel_kmhs = 0.0
    cycles = 0
    t_first: float | None = None
    last_t = loop.time()
    last_say = -_SAY_INTERVAL_S
    while True:
        try:
            speed = await hw.can.read_speed()
        except Exception as exc:
            raise DriveError(
                f"緩減速中に CAN 車速を読めません（{type(exc).__name__}: {exc}）"
            ) from exc
        now = loop.time()
        history.append((now, speed))
        while history and now - history[0][0] > window_s:
            history.popleft()
        if speed < VEHICLE_STOP_SPEED_KMH:
            break
        if t_first is None:
            t_first = now
        dt = now - last_t
        last_t = now
        cycles += 1
        decel_kmhs: float | None = None
        action = "窓待ち（保持）"
        if now - t_first >= window_s:
            decel_kmhs = decel_from_slope(history)
            max_decel_kmhs = max(max_decel_kmhs, decel_kmhs)
            e_g = (target_kmhs - decel_kmhs) / G_TO_KMHS
            k = local_gain_k_g_per_pct(
                table, speed, pulse_to_opening(pos), min_pct=min_pct,
                default_k=d.default_local_gain_g_per_pct,
            )
            kp = d.p_fraction / k  # [%/G]
            inc = kp / ti_s * e_g * dt  # 積分の増分 [%]
            tentative = base_pct + kp * e_g + integral_pct + inc
            # 枠に当たっていて e がさらに押す向きなら積分を止める（ワインドアップ防止）。
            # 0.3G 超えは e < 0 なので inc ≤ 0（積分は戻す向きにだけ働く）
            if (tentative > ceiling_pct and inc > 0.0) or (tentative < floor_pct and inc < 0.0):
                inc = 0.0
            integral_pct += inc
            out_pct = base_pct + kp * e_g + integral_pct
            out_pct = min(ceiling_pct, max(floor_pct, out_pct))
            new_pos = min(ceiling, max(approach, opening_to_pulse(out_pct)))
            if new_pos != pos:
                pos = new_pos
                await hw.brake.move_to_position(pos, smooth_over_s=SPEED_SAMPLE_INTERVAL_S)
            action = "戻し" if decel_kmhs > d.release_above_g * G_TO_KMHS else "PI"
        if now - last_say >= _SAY_INTERVAL_S:
            last_say = now
            g_text = "  —  " if decel_kmhs is None else f"{decel_kmhs / G_TO_KMHS:5.3f}"
            say(f"  車速 {speed:6.2f} km/h  減速 {g_text}G  "
                f"ブレーキ {pos:5d} pulse ({pulse_to_opening(pos):5.2f}%)  {action}")
        if now - started >= d.timeout_s:
            raise DriveError(
                f"緩減速で decel_stop.timeout_s={d.timeout_s:g}s 以内に停車しません"
                f"（車速 {speed:.2f} km/h、ブレーキ {pulse_to_opening(pos):.2f}%）"
            )
        await asyncio.sleep(SPEED_SAMPLE_INTERVAL_S)

    mark(log, SECTION_DECEL_TO_STOP, PHASE_STOP_HOLD)
    say(f"  停止確認: ブレーキ {pulse_to_opening(pos):.2f}% → "
        f"停車保持 {hold_pct:.2f}% まで刻みます")
    await step_to_position(hw.brake, pos, hold_pulse, step_pulse=step, dwell_s=HOLD_STEP_DWELL_S)
    result = StopDecelResult(
        hold_pct=hold_pct,
        duration_s=loop.time() - started,
        max_decel_g=max_decel_kmhs / G_TO_KMHS,
        cycles=cycles,
    )
    say(f"緩減速完了: {result.duration_s:.1f}s、最大減速 {result.max_decel_g:.3f}G、"
        f"PI {cycles} 周期")
    return result
