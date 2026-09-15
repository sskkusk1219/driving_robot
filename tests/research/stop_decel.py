"""手順 2 走行後の緩減速 → 停車保持（本番 RobotController._decelerate_to_stop の研究用置き換え）。

本番は 0.1s ごとに「車速差 ÷ 0.1s」で減速度を出し、0.2G との大小でブレーキを ±1% 動かす。
実機では 0.1s 差分のノイズ（標準偏差 2〜3 km/h/s、目標 7 km/h/s）と、ブレーキ→減速の遅れ
（効き始め約 0.3s、ピーク 0.6〜1.0s）のため、ブレーキが押し戻しを繰り返した。

ここでは一方向に踏み進め、行き過ぎそうなら止めて待つ（設定は decel_stop セクション）:
    1. アクセルを原点へ
    2. 接近   … ブレーキ不感帯 − approach_margin_pct まで最高速度で動かす（遊びの中で減速は出ない）
    3. 刻み   … step_mm 踏む → dwell_s 待つ → 直近 slope_window_s の車速の傾き（最小二乗）で
                 減速度を出す
                 目標 − press_margin_g 未満 → 1 刻み踏み増し（上限 = 停車保持開度）
                 release_above_g 超え     → 1 刻み戻す（接近位置より浅くはしない）
                 それ以外                 → 保持
                 車速を読むたびに VEHICLE_STOP_SPEED_KMH 未満なら停止確認
    4. 停車保持 … 停車保持開度まで刻んで踏む（通常はすでに到達している）

連続移動にしないのは、実機の効き（効き始め以降 1% ≈ 0.065G）とむだ時間から、ブレーキの最低速度
10mm/s でも減速が出たのを見てから止めるまでに 0.35〜0.6G まで行き過ぎる見込みのため。
"""

from __future__ import annotations

import asyncio
from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass

from src.domain.control.conversions import G_TO_KMHS, VEHICLE_STOP_SPEED_KMH
from src.models.profile import VehicleProfile
from tests.research.config import ResearchConfig
from tests.research.drive_log import SECTION_DECEL_TO_STOP, SessionLog, mark
from tests.research.hardware import DriveError, ResearchHardware
from tests.research.pedal_search import HOLD_STEP_DWELL_S, SPEED_SAMPLE_INTERVAL_S, step_to_position
from tests.research.term import say
from tests.research.vehicle import opening_to_pulse, pulse_to_opening

PHASE_APPROACH = "APPROACH"
PHASE_STEP = "STEP"
PHASE_STOP_HOLD = "STOP_HOLD"


@dataclass(frozen=True)
class StopDecelResult:
    hold_pct: float
    duration_s: float
    max_decel_g: float  # 判定に使った減速度の最大
    presses: int
    releases: int


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


async def decelerate_to_stop(
    hw: ResearchHardware,
    cfg: ResearchConfig,
    profile: VehicleProfile,
    *,
    log: SessionLog | None = None,
) -> StopDecelResult:
    """緩減速で停車させ、停車保持開度で保持した状態で返す。停車しなければ DriveError。"""
    d = cfg.decel_stop
    ff = profile.feedforward_params
    loop = asyncio.get_running_loop()
    started = loop.time()
    step = max(1, round(d.step_mm * 100))
    hold_pct = max(0.0, min(ff.stop_brake_opening_pct, profile.max_brake_opening))
    ceiling = opening_to_pulse(hold_pct)
    approach_pct = max(0.0, ff.brake_deadband_pct - d.approach_margin_pct)
    approach = min(ceiling, opening_to_pulse(approach_pct))
    press_below_kmhs = (d.target_decel_g - d.press_margin_g) * G_TO_KMHS
    release_above_kmhs = d.release_above_g * G_TO_KMHS

    mark(log, SECTION_DECEL_TO_STOP, PHASE_APPROACH)
    say(f"緩減速: 目標 {d.target_decel_g:g}G"
        f"（{d.target_decel_g - d.press_margin_g:g}G 未満で踏み増し・"
        f"{d.release_above_g:g}G 超えで戻す）、{d.step_mm:g}mm 刻み・待ち {d.dwell_s:g}s、"
        f"上限 = 停車保持 {hold_pct:.2f}%")
    await hw.accel.move_to_position(0)
    current = await hw.brake.read_position()
    pos = min(max(current, approach), ceiling)
    if current > approach:
        # 最後のパターンでブレーキを踏んだまま終わった場合は、そこ（上限まで）から刻む
        say(f"  接近: ブレーキはすでに {pulse_to_opening(current):.2f}% を踏んでいるため "
            f"{pulse_to_opening(pos):.2f}% から刻みます")
    else:
        say(f"  接近: ブレーキを {pos} pulse（{pulse_to_opening(pos):.2f}% = 不感帯 "
            f"{ff.brake_deadband_pct:.2f}% − {d.approach_margin_pct:g}%）まで最高速度で動かします")
    await hw.brake.move_to_position(pos)

    mark(log, SECTION_DECEL_TO_STOP, PHASE_STEP)
    history: deque[tuple[float, float]] = deque()
    max_decel_kmhs = 0.0
    presses = releases = 0
    while True:
        speed = await _watch_speed(hw, history, d.dwell_s, d.slope_window_s)
        if speed < VEHICLE_STOP_SPEED_KMH:
            break
        decel = decel_from_slope(history)
        max_decel_kmhs = max(max_decel_kmhs, decel)
        if decel > release_above_kmhs and pos > approach:
            pos = max(approach, pos - step)
            releases += 1
            action = "戻し"
        elif decel < press_below_kmhs and pos < ceiling:
            pos = min(ceiling, pos + step)
            presses += 1
            action = "踏み増し"
        elif decel < press_below_kmhs:
            action = "上限で待機"
        else:
            action = "保持"
        if action in {"踏み増し", "戻し"}:
            await hw.brake.move_to_position(pos, smooth_over_s=d.dwell_s)
        say(f"  車速 {speed:6.2f} km/h  減速 {decel / G_TO_KMHS:5.3f}G  "
            f"ブレーキ {pos:5d} pulse ({pulse_to_opening(pos):5.2f}%)  {action}")
        if loop.time() - started >= d.timeout_s:
            raise DriveError(
                f"緩減速で decel_stop.timeout_s={d.timeout_s:g}s 以内に停車しません"
                f"（車速 {speed:.2f} km/h、ブレーキ {pulse_to_opening(pos):.2f}%）"
            )

    mark(log, SECTION_DECEL_TO_STOP, PHASE_STOP_HOLD)
    say(f"  停止確認: ブレーキ {pulse_to_opening(pos):.2f}% → "
        f"停車保持 {hold_pct:.2f}% まで刻みます")
    await step_to_position(hw.brake, pos, ceiling, step_pulse=step, dwell_s=HOLD_STEP_DWELL_S)
    result = StopDecelResult(
        hold_pct=hold_pct,
        duration_s=loop.time() - started,
        max_decel_g=max_decel_kmhs / G_TO_KMHS,
        presses=presses,
        releases=releases,
    )
    say(f"緩減速完了: {result.duration_s:.1f}s、最大減速 {result.max_decel_g:.3f}G、"
        f"踏み増し {presses} 回・戻し {releases} 回")
    return result


async def _watch_speed(
    hw: ResearchHardware,
    history: deque[tuple[float, float]],
    dwell_s: float,
    window_s: float,
) -> float:
    """dwell_s の間 車速を読んで履歴に積み、最後の車速を返す（停車しきい値未満なら即返す）。"""
    loop = asyncio.get_running_loop()
    end = loop.time() + dwell_s
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
        if speed < VEHICLE_STOP_SPEED_KMH or now >= end:
            return speed
        await asyncio.sleep(SPEED_SAMPLE_INTERVAL_S)
