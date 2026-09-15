"""手順 2-0: ペダル探索（不感帯と停車保持開度を車速応答で測る）。

tests 環境では本番のキャリブレーションを使わず、開度 0% = 原点、100% = 9500 pulse とする
（tests/research/vehicle.py）。原点からペダルに触れるまでの隙間と遊びは設置で変わるため、
走行前に毎回ここで測る。

走行前チェック（tests/research/pre_drive_check.py）で停車を確認した状態から始める。

手順（1 刻み step_mm。刻むたびに dwell_s 待ち、その間の平均車速で判定する）:
    1. クリープ安定待ち … 両ペダル原点（走行前チェックのブレーキを離す）。平均車速が落ち着いたら
                           基準車速とする
    2. アクセル探索     … 刻んで踏み、基準 + margin を confirm_count 回連続で超えたら、最初に
                           超えた刻みの位置をアクセル不感帯とする → 原点へ戻して 1. をやり直す
    3. ブレーキ探索     … 刻んで踏み、基準 − margin を連続で割ったらブレーキ不感帯。車速が
                           下がっている間は踏み増さず、0.02 km/h 未満になった位置で停止確認
    4. 停車保持         … 停止確認開度 + stop_hold_margin_pct まで刻んで踏み、そのまま保持する

いきなり目標開度を踏まないのは、クリープ中の急制動とペダルへの衝撃を避けるため。連続移動に
しないのは、アクチュエータの最低速度（ブレーキ 10mm/s）で踏み続けると、むだ時間 0.6〜0.9s と
停車までの数秒の間に大きく踏み過ぎるため。

既知のバイアス: 車速の立ち上がり遅れにより、検出位置は真値より 1〜2 刻み深く出うる。
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass, replace
from typing import Any

from src.domain.control.conversions import VEHICLE_STOP_SPEED_KMH
from src.models.profile import VehicleProfile
from tests.research.config import ResearchConfig
from tests.research.drive_log import SECTION_PEDAL_SEARCH, SessionLog, mark
from tests.research.hardware import ActuatorProtocol, DriveError, ResearchHardware
from tests.research.term import say
from tests.research.vehicle import STROKE_LIMIT_PULSE, opening_to_pulse, pulse_to_opening

CREEP_WINDOW_S = 3.0  # クリープ安定判定に使う平均車速の窓 [s]
SPEED_SAMPLE_INTERVAL_S = 0.1  # 平均車速を取るときの読み取り間隔（CAN 10Hz）
HOLD_STEP_DWELL_S = 0.2  # 停車した後に保持位置まで踏むときの 1 刻みの待ち [s]

# 走行ログ（drive_log.SessionLog）の phase 列
PHASE_CREEP_WAIT = "CREEP_WAIT"
PHASE_ACCEL_SEARCH = "ACCEL_SEARCH"
PHASE_BRAKE_SEARCH = "BRAKE_SEARCH"
PHASE_STOP_HOLD = "STOP_HOLD"


@dataclass(frozen=True)
class PedalSearchResult:
    creep_speed_kmh: float
    accel_deadband_pct: float
    brake_deadband_pct: float
    stop_confirm_pct: float
    stop_brake_opening_pct: float

    def apply_to_profile(self, profile: VehicleProfile) -> VehicleProfile:
        """実測した不感帯・停車保持開度をプロファイルに反映したコピーを返す。"""
        ffp = replace(
            profile.feedforward_params,
            accel_deadband_pct=self.accel_deadband_pct,
            brake_deadband_pct=self.brake_deadband_pct,
            stop_brake_opening_pct=self.stop_brake_opening_pct,
        )
        return replace(profile, feedforward_params=ffp)

    def config_updates(self) -> dict[str, Any]:
        return {
            "feedforward.accel_deadband_pct": self.accel_deadband_pct,
            "feedforward.brake_deadband_pct": self.brake_deadband_pct,
            "feedforward.stop_brake_opening_pct": self.stop_brake_opening_pct,
        }


def search_step_pulse(cfg: ResearchConfig) -> int:
    """1 刻みの移動量 [pulse]（位置指令は 0.01mm 単位）。"""
    return max(1, round(cfg.pedal_search.step_mm * 100))


async def mean_speed(hw: ResearchHardware, duration_s: float) -> float:
    """duration_s の間 CAN 車速を 10Hz で読み、平均を返す。"""
    loop = asyncio.get_running_loop()
    end = loop.time() + duration_s
    total = 0.0
    count = 0
    while True:
        try:
            total += await hw.can.read_speed()
        except Exception as exc:
            raise DriveError(f"CAN 車速を読めません（{type(exc).__name__}: {exc}）") from exc
        count += 1
        if loop.time() >= end:
            return total / count
        await asyncio.sleep(SPEED_SAMPLE_INTERVAL_S)


async def step_to_position(
    axis: ActuatorProtocol, start_pos: int, target_pos: int, *, step_pulse: int, dwell_s: float
) -> int:
    """start_pos から target_pos まで step_pulse ずつ動かす（1 刻みごとに dwell_s 待つ）。"""
    pos = start_pos
    while pos != target_pos:
        if target_pos > pos:
            pos = min(target_pos, pos + step_pulse)
        else:
            pos = max(target_pos, pos - step_pulse)
        await axis.move_to_position(pos, smooth_over_s=dwell_s)
        await asyncio.sleep(dwell_s)
    return pos


async def wait_creep_stable(
    hw: ResearchHardware, cfg: ResearchConfig, *, window_s: float = CREEP_WINDOW_S
) -> float:
    """両ペダルを離した状態で車速が落ち着くのを待ち、基準車速を返す。"""
    s = cfg.pedal_search
    loop = asyncio.get_running_loop()
    deadline = loop.time() + s.creep_timeout_s
    say(f"クリープ安定待ち（{window_s:g}s 平均の変化 < {s.creep_stable_kmh:g} km/h かつ "
        f"{s.creep_min_speed_kmh:g} km/h 以上、最大 {s.creep_timeout_s:g}s）…")
    prev: float | None = None
    while True:
        mean = await mean_speed(hw, window_s)
        change = "" if prev is None else f"（変化 {mean - prev:+.2f}）"
        say(f"  平均車速 {mean:6.2f} km/h{change}")
        if (
            prev is not None
            and abs(mean - prev) < s.creep_stable_kmh
            and mean >= s.creep_min_speed_kmh
        ):
            say(f"クリープ安定: 基準車速 {mean:.2f} km/h")
            return mean
        if loop.time() >= deadline:
            raise DriveError(
                f"クリープで車速が安定しません"
                f"（{s.creep_timeout_s:g}s、最後の平均 {mean:.2f} km/h）。"
                "両ペダルを離した状態で車両がクリープで動くか確認してください"
            )
        prev = mean


def _step_line(label: str, pos: int, speed: float, base: float, mark: str) -> str:
    return (f"  {label} {pos:5d} pulse ({pulse_to_opening(pos):5.2f}%)  "
            f"平均車速 {speed:6.2f} km/h（基準比 {speed - base:+.2f}） {mark}")


async def _search_accel(
    hw: ResearchHardware, cfg: ResearchConfig, base: float, step: int
) -> int:
    s = cfg.pedal_search
    limit = opening_to_pulse(s.accel_max_pct)
    say(f"アクセル探索: 平均車速が基準 {base:.2f} + {s.onset_margin_kmh:g} km/h を "
        f"{s.confirm_count} 刻み連続で超えるまで踏みます（上限 {s.accel_max_pct:g}%）")
    pos = 0
    streak = 0
    first = 0
    while True:
        if pos + step > limit:
            raise DriveError(
                f"アクセルを {s.accel_max_pct:g}% まで踏んでも車速が上がりません"
                f"（基準 {base:.2f} km/h）。pedal_search.accel_max_pct を見直してください"
            )
        pos += step
        await hw.accel.move_to_position(pos, smooth_over_s=s.dwell_s)
        speed = await mean_speed(hw, s.dwell_s)
        rising = speed >= base + s.onset_margin_kmh
        streak = streak + 1 if rising else 0
        if streak == 1:
            first = pos
        say(_step_line("アクセル", pos, speed, base, "↑ 反応" if rising else ""))
        if streak >= s.confirm_count:
            say(f"アクセル不感帯: {first} pulse = {pulse_to_opening(first):.2f}%")
            return first


async def _search_brake(
    hw: ResearchHardware, cfg: ResearchConfig, base: float, step: int
) -> tuple[int, int]:
    """ブレーキ不感帯の位置と、停止確認できた位置を返す。"""
    s = cfg.pedal_search
    limit = opening_to_pulse(s.brake_max_pct)
    say(f"ブレーキ探索: 基準 {base:.2f} − {s.onset_margin_kmh:g} km/h を割ったら不感帯、"
        f"{VEHICLE_STOP_SPEED_KMH:g} km/h 未満で停止確認（上限 {s.brake_max_pct:g}%）")
    pos = 0
    streak = 0
    first = 0
    onset: int | None = None
    last_speed = base
    last_drop = 0.0
    while True:
        # 効き始めた後、直前の待ちで車速が下がっているうちは踏み増さない（踏み過ぎ防止）
        holding = onset is not None and last_drop >= s.onset_margin_kmh
        if not holding:
            if pos + step > limit:
                raise DriveError(
                    f"ブレーキを {s.brake_max_pct:g}% まで踏んでも停車しません"
                    f"（車速 {last_speed:.2f} km/h）。pedal_search.brake_max_pct を見直してください"
                )
            pos += step
            await hw.brake.move_to_position(pos, smooth_over_s=s.dwell_s)
        speed = await mean_speed(hw, s.dwell_s)
        last_drop, last_speed = last_speed - speed, speed

        mark = "減速中のため保持" if holding else ""
        if onset is None:
            falling = speed <= base - s.onset_margin_kmh
            streak = streak + 1 if falling else 0
            if streak == 1:
                first = pos
            if falling:
                mark = "↓ 反応"
            if streak >= s.confirm_count:
                onset = first
                mark = f"↓ ブレーキ不感帯 {pulse_to_opening(onset):.2f}%"
        say(_step_line("ブレーキ", pos, speed, base, mark))

        if speed < VEHICLE_STOP_SPEED_KMH:
            if onset is None:  # 確定前に止まった（効きが急）→ 最初に反応した位置を使う
                onset = first if streak > 0 else pos
            say(f"停止確認: {pos} pulse = {pulse_to_opening(pos):.2f}%")
            return onset, pos


async def run_pedal_search(
    hw: ResearchHardware,
    cfg: ResearchConfig,
    *,
    window_s: float = CREEP_WINDOW_S,
    log: SessionLog | None = None,
) -> PedalSearchResult:
    """不感帯と停車保持開度を測り、停車保持の状態で返す。実機のときだけ YAML へ保存する。"""
    s = cfg.pedal_search
    step = search_step_pulse(cfg)
    say(f"開度の定義: 原点 0 pulse = 0% / {STROKE_LIMIT_PULSE} pulse = 100%"
        f"（1 刻み {step} pulse = {pulse_to_opening(step):.2f}%、待ち {s.dwell_s:g}s）")
    mark(log, SECTION_PEDAL_SEARCH, PHASE_CREEP_WAIT)
    say("両ペダルを原点へ戻してクリープさせます …")
    await asyncio.gather(hw.accel.move_to_position(0), hw.brake.move_to_position(0))

    base = await wait_creep_stable(hw, cfg, window_s=window_s)
    mark(log, SECTION_PEDAL_SEARCH, PHASE_ACCEL_SEARCH)
    accel_pos = await _search_accel(hw, cfg, base, step)
    mark(log, SECTION_PEDAL_SEARCH, PHASE_CREEP_WAIT)
    say("アクセルを原点へ戻し、クリープが落ち着くのを待ちます …")
    await hw.accel.move_to_position(0)
    base = await wait_creep_stable(hw, cfg, window_s=window_s)
    mark(log, SECTION_PEDAL_SEARCH, PHASE_BRAKE_SEARCH)
    brake_pos, stop_pos = await _search_brake(hw, cfg, base, step)

    mark(log, SECTION_PEDAL_SEARCH, PHASE_STOP_HOLD)
    stop_pct = pulse_to_opening(stop_pos)
    hold_pct = round(min(stop_pct + s.stop_hold_margin_pct, cfg.vehicle.max_brake_opening_pct), 2)
    say(f"停車保持: 停止確認 {stop_pct:.2f}% + {s.stop_hold_margin_pct:g}% → "
        f"{hold_pct:.2f}% まで刻んで踏みます …")
    await step_to_position(
        hw.brake, stop_pos, opening_to_pulse(hold_pct), step_pulse=step, dwell_s=HOLD_STEP_DWELL_S
    )

    result = PedalSearchResult(
        creep_speed_kmh=round(base, 2),
        accel_deadband_pct=round(pulse_to_opening(accel_pos), 2),
        brake_deadband_pct=round(pulse_to_opening(brake_pos), 2),
        stop_confirm_pct=round(stop_pct, 2),
        stop_brake_opening_pct=hold_pct,
    )
    _print_result(result, s.stop_hold_margin_pct)
    if hw.is_real:
        save_to_config(cfg, result)
    else:
        say(f"スタブのため {cfg.source_path} は更新しません（実機の値を模擬値で上書きしないため）")
    return result


def save_to_config(cfg: ResearchConfig, result: PedalSearchResult) -> list[str]:
    changed = cfg.save(result.config_updates())
    say(f"{cfg.source_path} に保存しました:")
    for line in changed:
        say(f"  {line}")
    return changed


def _print_result(result: PedalSearchResult, margin_pct: float) -> None:
    rows = (
        ("クリープ車速（基準）", f"{result.creep_speed_kmh:6.2f} km/h"),
        ("アクセル不感帯", f"{result.accel_deadband_pct:6.2f} %"),
        ("ブレーキ不感帯", f"{result.brake_deadband_pct:6.2f} %"),
        ("停止確認開度", f"{result.stop_confirm_pct:6.2f} %"),
        ("停車保持開度", f"{result.stop_brake_opening_pct:6.2f} %（停止確認 + {margin_pct:g}%）"),
    )
    say("── ペダル探索の結果 ──")
    for label, value in rows:
        say(f"  {label:<12} {value}")
