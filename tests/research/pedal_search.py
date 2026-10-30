"""手順 2-0: ペダル探索（クリープ車速と停車保持開度を車速応答で測る）。

tests 環境では本番のキャリブレーションを使わず、開度 0% = 原点、100% = 9500 pulse とする
（tests/research/vehicle.py）。原点からペダルに触れるまでの隙間は、手で測った
「ペダルに触れた位置」vehicle.min_accel_opening_pct・min_brake_opening_pct（
tests/hardware/test_calibration.py で測る）で表し、待機位置かつ効く開度 u の原点にする。

走行前チェック（tests/research/pre_drive_check.py）で停車を確認した状態から始める。

手順（ブレーキは min から 1 刻み brake_step_mm ずつ踏み、刻むたびに dwell_s 待って平均車速で判定）:
    1. クリープ安定待ち … 両ペダルを min（待機位置）に置く。平均車速が落ち着いたら
                           クリープ平衡車速とする（wait_creep_stable）
    2. ブレーキ踏み込み … min から刻んで踏み、平均車速が VEHICLE_STOP_SPEED_KMH 未満になった位置を
                           停止確認開度とする。車速が下がっている間は踏み増さない。
                           vehicle.max_brake_opening_pct まで踏んでも停車しなければエラー
    3. 停車保持         … 停止確認開度 + stop_hold_margin_pct まで刻んで踏み、そのまま保持する
4. 低速の階段       … （pedal_search.low_stair_enabled のときだけ。2026-10-28 段1）停車保持の後、
                       止まった開度 − k × 刻み（k=1,2,…）を順に下げて保持し（LOW_RELEASE_STAIR）、
                       クリープに着いたら同じ開度を逆順に踏んで保持し（LOW_PRESS_STAIR）、停車で終わる。
                       これを low_stair_repeats 回。最後は停車保持の開度へ戻して G ランプへ渡す

いきなり目標開度を踏まないのは、クリープ中の急制動とペダルへの衝撃を避けるため。連続移動に
しないのは、アクチュエータの最低速度（ブレーキ 10mm/s）で踏み続けると、むだ時間 0.6〜0.9s と
停車までの数秒の間に大きく踏み過ぎるため。

2026-10-08 段2l-6（ProblemReport_20261001）: 不感帯の探索（アクセル・ブレーキを刻んで車速の
反応を見る `_search_accel`・`OnsetDetector`・反応の判定）をやめた。不感帯は走行ごとに動き
（アクセル 4.42 → 14.0 → 7.68 → 10.53%）、誤検出（091434 の 0.11%）と判定の作り直しが続いて
いたため、手で測った min に置き換えた。PedalSearchResult はクリープ平衡車速・停止確認開度・
停車保持開度の 3 つで、yaml に書き戻すのは停車保持開度とクリープ平衡車速の 2 つ。

2026-09-20 クリープ安定判定（1. の停止条件）は「連続する2つの窓平均の差 < 閾値」から
「窓平均の傾き < 閾値 かつ 最短経過時間」に変えた。クリープ車速は漸近的に上がるため、旧判定は
まだ上がっている途中で差が閾値を割ってしまう（実機ログで、真の平衡 5.00 km/h に対し
4.79→4.88→4.93→4.96→4.98 km/h と 17.9 秒かけて推移する途中の 4.88 km/h で確定していた）。

2026-09-21 クリープ平衡車速（wait_creep_stable が返し PedalSearchResult.creep_speed_kmh に
入る値）を yaml に保存する対象へ昇格した。従来は表示のみで、yaml の
feedforward.creep_speed_kmh は model_training.estimate_dynamics_params が出す「ペダルオフ・
|dv|<0.3 km/h/s のサンプルの中央値」のままだった。この中央値は手順2末尾のクリープ発進の上昇
途中のサンプルが多く、平衡より 0.24 km/h 低く出ていた（実機ログで実証済み）。2-0 の
wait_creep_stable は 20.1 秒かけて 5.00 km/h のプラトーまで待てており、こちらを正とする。
config_updates() で yaml に保存し、pattern_drive.MEASURED_KEYS に creep_speed_kmh を加えて
estimate_dynamics_params の推定値で上書きされないようにした。
"""

from __future__ import annotations

import asyncio
import math
from collections import deque
from collections.abc import Callable
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from tests.research.config import ResearchConfig
from tests.research.drive_log import SECTION_PEDAL_SEARCH, SessionLog, mark
from tests.research.hardware import ActuatorProtocol, DriveError, ResearchHardware
from tests.research.hold_scene import HoldScenePI
from tests.research.pattern_loop import AXIS_SMOOTH_DUTY, fit_slope
from tests.research.research_types import G_TO_KMHS, VEHICLE_STOP_SPEED_KMH, VehicleProfile
from tests.research.term import say
from tests.research.vehicle import STROKE_LIMIT_PULSE, opening_to_pulse, pulse_to_opening

SPEED_SAMPLE_INTERVAL_S = 0.1  # 平均車速を取るときの読み取り間隔（CAN 10Hz）
HOLD_STEP_DWELL_S = 0.2  # 停車した後に保持位置まで踏むときの 1 刻みの待ち [s]

# 2-0 のブレーキ踏み込みの 1 刻みの移動（smooth_over_s）にかける時間の下限 [s]。刻み幅
# （brake_step_mm）に比例させて短くするが、これ未満にするとアクチュエータの位置決めが追いつかず
# 暴れるおそれがある
MOVE_DWELL_MIN_S = 0.2

# 走行ログ（drive_log.SessionLog）の phase 列
PHASE_CREEP_WAIT = "CREEP_WAIT"
PHASE_BRAKE_SEARCH = "BRAKE_SEARCH"
PHASE_STOP_HOLD = "STOP_HOLD"
# 2026-10-28 段1（ProblemReport_20261028）: 停車保持の後に入れる低速の階段（low_stair_enabled）
PHASE_LOW_RELEASE_STAIR = "LOW_RELEASE_STAIR"  # 離す向き: 浅い方へ（停車 → クリープ）
PHASE_LOW_PRESS_STAIR = "LOW_PRESS_STAIR"  # 踏む向き: 同じ開度を戻す方へ（クリープ → 停車）
STAIR_TAIL_S = 2.0  # 各段の保持の終わりに車速・加速度を測る時間 [s]
# 2026-10-30 段6（ProblemReport_20261030）: 手順13 の 4 点の保持
PHASE_LOW_HOLD = "LOW_HOLD"
# 基準を上げる傾き [km/h/s]。UDDS 1251→1252 s の基準が 0→1.6 km/h と 1 s で上がるのと同じ傾き
# （その後 1256 s まで 1.6 km/h 一定。この立ち上がり方で保てるかを見る点なので同じにする）
HOLD_REF_RAMP_KMHS = 1.6
HOLD_OVERSPEED_MARGIN_KMH = 1.0  # クリープ車速 + これを超えたら、その点は保てずで打ち切る [km/h]
# 1 刻み寄せの 1 周期の移動量 [pulse] ＝ アクチュエータの最小刻み 1 個（100 ÷ ストローク [pulse] ＝
# 約 0.0105%）。pattern_loop の _ramp_min_step_pct（キャリブレーションのストロークから求める）と同じ
HOLD_MIN_STEP_PULSE = 1


@dataclass(frozen=True)
class HoldPointResult:
    """4 点の保持の 1 点の結果。speed_kmh・opening_pct は帯の中だった行の平均
    （帯の中の行が無いときは None）。"""

    target_kmh: float
    held: bool
    speed_kmh: float | None
    opening_pct: float | None
    in_band_s: float  # 帯の中だった時間の合計 [s]
    elapsed_s: float  # 基準が上がり始めてから終わるまで [s]
    initial_opening_pct: float  # 初めの開度（BRAKE_SEARCH の補間）[%]
    note: str = ""


@dataclass(frozen=True)
class PedalSearchResult:
    creep_speed_kmh: float
    stop_confirm_pct: float
    stop_brake_opening_pct: float  # 絶対開度（内部は絶対開度のまま。yaml へは min からの量で書く）
    # 2026-10-08 段2l-7: yaml へ「止まった開度＋マージン − ブレーキの min」で書くための min [%]
    brake_min_pct: float = 0.0
    # 2026-10-30 段6: 手順13 の 4 点の保持の結果（低い順。走らなければ空）と、BRAKE_SEARCH の
    # (開度 [%], 平均車速 [km/h]) の組（_search_brake が段ごとに記録する）
    hold_points: tuple[HoldPointResult, ...] = ()
    brake_search_pairs: tuple[tuple[float, float], ...] = ()

    def apply_to_profile(self, profile: VehicleProfile) -> VehicleProfile:
        """実測した停車保持開度・クリープ平衡車速をプロファイルに反映したコピーを返す。"""
        ffp = replace(
            profile.feedforward_params,
            stop_brake_opening_pct=self.stop_brake_opening_pct,
            creep_speed_kmh=self.creep_speed_kmh,
        )
        return replace(profile, feedforward_params=ffp)

    def config_updates(self) -> dict[str, Any]:
        return {
            "feedforward.stop_brake_offset_pct": round(
                self.stop_brake_opening_pct - self.brake_min_pct, 2
            ),
            "feedforward.creep_speed_kmh": self.creep_speed_kmh,
        }


def search_step_pulse(cfg: ResearchConfig) -> int:
    """停車保持位置への刻み送り（step_to_position）の1刻みの移動量 [pulse]（0.01mm単位）。

    pre_drive_check.py・stop_decel.py も自分の刻み送りにこの関数を使う。2-0 のブレーキ踏み込みの
    刻みは brake_step_pulse を使う（停車保持まで遅くならないよう分離している）。
    """
    return max(1, round(cfg.pedal_search.step_mm * 100))


def brake_step_pulse(cfg: ResearchConfig) -> int:
    """2-0 のブレーキ踏み込みの1刻みの移動量 [pulse]（0.01mm 単位。brake_step_mm を使う）。"""
    return max(1, round(cfg.pedal_search.brake_step_mm * 100))


def _move_dwell_s(cfg: ResearchConfig) -> float:
    """2-0 のブレーキ踏み込みの1刻みの移動（smooth_over_s）にかける時間 [s]。

    刻み幅 brake_step_mm に比例させる（例: brake_step_mm=0.1・step_mm=0.5・dwell_s=1.0 なら
    0.2s）。車速を測る待ち時間（dwell_s そのもの）とは別で、動かす時間だけを短くする。
    刻みが小さいのに大きい刻み用の dwell_s を丸ごと移動にかけるのは無駄なため。
    下限は MOVE_DWELL_MIN_S。
    """
    s = cfg.pedal_search
    return max(MOVE_DWELL_MIN_S, s.dwell_s * s.brake_step_mm / s.step_mm)


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
    hw: ResearchHardware, cfg: ResearchConfig, *, window_s: float | None = None
) -> float:
    """両ペダルを離した状態で車速が落ち着くのを待ち、基準車速を返す。

    停止条件は「窓平均の傾き < creep_settle_kmhs かつ経過時間 >= creep_settle_min_s」。
    クリープ車速は漸近的に上がるので、旧来の「連続する2つの窓平均の差 < 閾値」では
    まだ上がっている途中で差が閾値を割ってしまう（実機ログで、真の平衡に対し収束前の値で
    確定していたことが判明）。傾きなら漸近の最終段階でしか閾値を割らず、最短時間ガードで
    立ち上がり直後の偶然の小さい傾きも拾わない。
    """
    s = cfg.pedal_search
    window_s = s.creep_window_s if window_s is None else window_s
    loop = asyncio.get_running_loop()
    started = loop.time()
    deadline = started + s.creep_timeout_s
    say(f"クリープ安定待ち（{window_s:g}s 平均の傾き < {s.creep_settle_kmhs:g} km/h/s かつ "
        f"{s.creep_min_speed_kmh:g} km/h 以上、最短 {s.creep_settle_min_s:g}s・"
        f"最大 {s.creep_timeout_s:g}s）…")
    prev: float | None = None
    while True:
        mean = await mean_speed(hw, window_s)
        elapsed = loop.time() - started
        slope = None if prev is None else (mean - prev) / window_s
        trend = "" if slope is None else f"（傾き {slope:+.3f} km/h/s）"
        say(f"  平均車速 {mean:6.2f} km/h{trend} [{elapsed:5.1f}s]")
        if (
            slope is not None
            and abs(slope) < s.creep_settle_kmhs
            and mean >= s.creep_min_speed_kmh
            and elapsed >= s.creep_settle_min_s
        ):
            say(f"クリープ安定: 基準車速 {mean:.2f} km/h（{elapsed:.1f}s）")
            return mean
        if loop.time() >= deadline:
            raise DriveError(
                f"クリープで車速が安定しません"
                f"（{s.creep_timeout_s:g}s、最後の平均 {mean:.2f} km/h）。"
                "両ペダルを離した状態で車両がクリープで動くか確認してください"
            )
        prev = mean


def _step_line(pos: int, speed: float, base: float, mark: str) -> str:
    return (f"  ブレーキ {pos:5d} pulse ({pulse_to_opening(pos):5.2f}%)  "
            f"平均車速 {speed:6.2f} km/h（基準比 {speed - base:+.2f}） {mark}")


async def _search_brake(
    hw: ResearchHardware, cfg: ResearchConfig, base: float, start_pos: int, step: int,
    record: list[tuple[float, float]] | None = None,
) -> int:
    """ブレーキを start_pos（min）から刻んで踏み、停止確認できた位置 [pulse] を返す。

    車速が下がっている間（直前の刻みで onset_margin_kmh 以上下がった）は踏み増さない
    （踏み過ぎ防止）。vehicle.max_brake_opening_pct まで踏んでも停車しなければ DriveError。
    record を渡すと、刻みごとの (その開度 [%], 平均車速 [km/h]) を追記する（2026-10-30 段6。
    手順13 の 4 点の保持の初めの開度の補間に使う）。
    """
    s = cfg.pedal_search
    max_pct = cfg.vehicle.max_brake_opening_pct  # 機構保護の上限を探索の上限にも使う
    stop_limit = opening_to_pulse(max_pct)
    move_dwell = _move_dwell_s(cfg)
    say(f"ブレーキ踏み込み: min {pulse_to_opening(start_pos):.2f}% から "
        f"{pulse_to_opening(step):.2f}% 刻み、待ち {s.dwell_s:g}s の平均車速が "
        f"{VEHICLE_STOP_SPEED_KMH:g} km/h 未満になったら停止確認"
        f"（上限 {max_pct:g}%。車速が {s.onset_margin_kmh:g} km/h 以上下がる間は"
        "踏み増さない）")
    pos = start_pos
    last_speed = base
    last_drop = 0.0
    while True:
        holding = last_drop >= s.onset_margin_kmh  # 減速中は踏み増さない
        if not holding:
            if pos + step > stop_limit:
                raise DriveError(
                    f"ブレーキを開度上限 {max_pct:g}%（vehicle.max_brake_opening_pct）まで踏んでも"
                    f"停車しません（車速 {last_speed:.2f} km/h）。"
                    "取り付け位置・max の測定を確認してください"
                )
            pos += step
            await hw.brake.move_to_position(pos, smooth_over_s=move_dwell)
        speed = await mean_speed(hw, s.dwell_s)
        last_drop, last_speed = last_speed - speed, speed
        if record is not None:
            entry = (pulse_to_opening(pos), speed)
            if record and record[-1][0] == entry[0]:  # 減速中の保持＝同じ開度の測り直し（後を採る）
                record[-1] = entry
            else:
                record.append(entry)
        say(_step_line(pos, speed, base, "減速中のため保持" if holding else ""))
        if speed < VEHICLE_STOP_SPEED_KMH:
            say(f"停止確認: {pos} pulse = {pulse_to_opening(pos):.2f}%")
            return pos


def low_stair_openings(
    stop_confirm_pct: float,
    step_pct: float,
    max_steps: int,
    *,
    min_pct: float,
    max_pct: float,
) -> list[float]:
    """低速の階段の開度の列 [%]（離す向きの順＝浅くなる順）を返す。

    止まった開度（stop_confirm_pct）から k × step_pct（k=1..max_steps）下げた開度。
    [min_pct, max_pct] に切る。min に当たったらそれ以上は同じ値になるので、そこで列を終える。
    車の固定値は持たず、止まった開度からの量だけで決める。
    """
    out: list[float] = []
    for k in range(1, max_steps + 1):
        pct = round(min(max_pct, max(min_pct, stop_confirm_pct - k * step_pct)), 4)
        if out and pct == out[-1]:
            break
        out.append(pct)
    return out


def low_stair_press_openings(
    start_pct: float,
    step_pct: float,
    max_steps: int,
    *,
    hold_pct: float,
    max_pct: float,
) -> list[float]:
    """踏む向きの開度の列 [%]（深くなる順）。start_pct（離す向きで最後に居た開度）から
    k × step_pct（k=1..max_steps）深くした開度。停車保持の開度（hold_pct）と max_pct の浅いほうで
    切り、そこに当たったらそこで列を終える（同じ値を繰り返さない）。
    """
    cap = min(hold_pct, max_pct)
    out: list[float] = []
    for k in range(1, max_steps + 1):
        pct = round(min(cap, start_pct + k * step_pct), 4)
        if pct <= start_pct or (out and pct == out[-1]):
            break
        out.append(pct)
    return out


def stair_release_done(speed_kmh: float, creep_kmh: float, margin_kmh: float) -> bool:
    """離す向きの終わり: 車速がクリープ車速 − 余裕 以上（クリープに着いた）。"""
    return speed_kmh >= creep_kmh - margin_kmh


def stair_press_done(speed_kmh: float) -> bool:
    """踏む向きの終わり: 車速が停車の閾値未満（止まった）。"""
    return speed_kmh < VEHICLE_STOP_SPEED_KMH


def low_stair_estimate_s(cfg: ResearchConfig) -> float:
    """低速の階段の最大所要時間 [s]（全段・全繰り返しを使い切ったとき。say の見込み用）。"""
    s = cfg.pedal_search
    per_step = s.low_stair_hold_s + _move_dwell_s(cfg)
    steps = s.low_stair_max_steps + s.low_stair_press_max_steps
    return s.low_stair_repeats * steps * per_step


async def _stair_direction(
    hw: ResearchHardware,
    cfg: ResearchConfig,
    openings: list[float],
    *,
    done: Callable[[float], bool],
    label: str,
) -> tuple[int, bool]:
    """openings を順に踏み、各段で low_stair_hold_s 保持して車速を測る。

    各段の結果（開度・保持の終わり STAIR_TAIL_S の平均車速と加速度）を say で出す。
    done(平均車速) が True になった段で終わる。返り値は (最後に踏んだ段の添字, done で終わったか)。
    開度は呼び出し側で [min, max] に切ってある前提。アクセルには触れない（min のまま）。
    """
    s = cfg.pedal_search
    move_dwell = _move_dwell_s(cfg)
    tail = min(STAIR_TAIL_S, s.low_stair_hold_s)
    lead = max(0.0, s.low_stair_hold_s - tail)
    half = tail / 2.0
    last = -1
    for i, pct in enumerate(openings):
        await hw.brake.move_to_position(opening_to_pulse(pct), smooth_over_s=move_dwell)
        last = i
        if lead > 0.0:
            await asyncio.sleep(lead)
        first = await mean_speed(hw, half)
        second = await mean_speed(hw, half)
        speed = (first + second) / 2.0
        accel = (second - first) / half  # 前半・後半の窓の中心は half 離れている
        finished = done(speed)
        say(f"  {label} {i + 1}/{len(openings)}: ブレーキ {pct:5.2f}%  "
            f"保持の終わり {tail:g}s 平均 {speed:6.2f} km/h・加速度 {accel:+.3f} km/h/s"
            f"{'  → 終わり' if finished else ''}")
        if finished:
            return last, True
    return last, False


async def run_low_stair(
    hw: ResearchHardware,
    cfg: ResearchConfig,
    *,
    creep_kmh: float,
    stop_confirm_pct: float,
    hold_pct: float,
    log: SessionLog | None = None,
) -> None:
    """停車保持の後の低速の階段（離す向き → 踏む向き）を low_stair_repeats 回行う。

    終わったら停車保持の開度（hold_pct）まで刻んで踏み直して返す（G ランプへ渡す状態）。
    """
    s = cfg.pedal_search
    v = cfg.vehicle
    openings = low_stair_openings(
        stop_confirm_pct, s.low_stair_step_pct, s.low_stair_max_steps,
        min_pct=v.min_brake_opening_pct, max_pct=v.max_brake_opening_pct,
    )
    est = low_stair_estimate_s(cfg)
    say(f"低速の階段: 止まった開度 {stop_confirm_pct:.2f}% から {s.low_stair_step_pct:g}% 刻みで "
        f"最大 {len(openings)} 段（{openings[0]:.2f}% → {openings[-1]:.2f}%）、保持 "
        f"{s.low_stair_hold_s:g}s、{s.low_stair_repeats} 回繰り返し。クリープ車速 "
        f"{creep_kmh:.2f} − {s.low_stair_creep_margin_kmh:g} km/h 以上で離す向きの終わり、"
        f"{VEHICLE_STOP_SPEED_KMH:g} km/h 未満で踏む向きの終わり。"
        f"所要の見込み 最大 約 {est / 60.0:.1f} 分（途中で終われば短くなる）")
    hold_pulse = opening_to_pulse(hold_pct)
    stopped = True  # 停車保持から始まる
    pos = hold_pulse
    for rep in range(1, s.low_stair_repeats + 1):
        if not stopped:  # 前の繰り返しが停車で終わらなかったときは、止まった状態へ戻してから始める
            pos = await step_to_position(
                hw.brake, pos, hold_pulse,
                step_pulse=search_step_pulse(cfg), dwell_s=HOLD_STEP_DWELL_S,
            )
        mark(log, SECTION_PEDAL_SEARCH, PHASE_LOW_RELEASE_STAIR)
        say(f"低速の階段 {rep}/{s.low_stair_repeats} 離す向き")
        last, reached = await _stair_direction(
            hw, cfg, openings,
            done=lambda sp: stair_release_done(sp, creep_kmh, s.low_stair_creep_margin_kmh),
            label="離す",
        )
        if not reached:
            say(f"  段数の上限（{len(openings)} 段）でクリープに着きませんでした")
        # 踏む向き: 離す向きで最後に居た開度から刻みずつ深く踏み、止まるまで続ける
        # （離す向きの列の長さに縛られない。上限は low_stair_press_max_steps と停車保持の開度）
        press = low_stair_press_openings(
            openings[last], s.low_stair_step_pct, s.low_stair_press_max_steps,
            hold_pct=hold_pct, max_pct=v.max_brake_opening_pct,
        )
        mark(log, SECTION_PEDAL_SEARCH, PHASE_LOW_PRESS_STAIR)
        say(f"低速の階段 {rep}/{s.low_stair_repeats} 踏む向き")
        pos = opening_to_pulse(openings[last])
        if press:
            last_press, stopped = await _stair_direction(
                hw, cfg, press, done=stair_press_done, label="踏む"
            )
            pos = opening_to_pulse(press[last_press])
        else:
            stopped = False
        if not stopped:
            say(f"  踏む向きの上限（最大 {s.low_stair_press_max_steps} 段・"
                f"停車保持 {hold_pct:.2f}% まで）でも停車しませんでした。停車保持へ戻します")
    # 停車保持の開度へ戻す（G ランプの前提。今までの停車保持と同じ位置）
    mark(log, SECTION_PEDAL_SEARCH, PHASE_STOP_HOLD)
    await step_to_position(
        hw.brake, pos, hold_pulse,
        step_pulse=search_step_pulse(cfg), dwell_s=HOLD_STEP_DWELL_S,
    )


# ─────────────────────────────────────────────────────────────────────
# 手順13: 4 点の保持の開度（2026-10-30 段6。ProblemReport_20261030）
# ─────────────────────────────────────────────────────────────────────


def hold_point_speeds(
    creep_kmh: float, fractions: list[float], extra_kmh: list[float]
) -> list[float]:
    """4 点の保持の車速 [km/h]（低い順）。クリープ車速 × fractions ＋ extra_kmh のうち、
    停車速度より大きくクリープ車速より小さいものだけ。重複（小数 3 桁で同じ）は 1 つにする。"""
    cands = [creep_kmh * f for f in fractions] + list(extra_kmh)
    out: list[float] = []
    for v in sorted(round(c, 3) for c in cands):
        if VEHICLE_STOP_SPEED_KMH < v < creep_kmh and (not out or v != out[-1]):
            out.append(v)
    return out


def hold_initial_opening(
    pairs: list[tuple[float, float]], speed_kmh: float, fallback_pct: float
) -> float:
    """初めの開度 [%]: BRAKE_SEARCH の (開度, 平均車速) の組を、車速の昇順に並べて speed_kmh で
    線形補間する（端の外は端の値）。車速は開度に対して単調に下がる前提。組が 2 つ未満なら
    fallback_pct（動き出す開度）。"""
    if len(pairs) < 2:
        return fallback_pct
    ordered = sorted((v, u) for u, v in pairs)
    return float(np.interp(speed_kmh, [p[0] for p in ordered], [p[1] for p in ordered]))


def hold_band_ok(accel_kmhs: float | None, band_g: float) -> bool:
    """実測の加速度の大きさが ±band_g [G] 以内か（測れていない＝None は False）。
    pattern_loop の RAMP_CRUISE と同じ判定（abs(加速度 [km/h/s]) ÷ G_TO_KMHS ≤ band_g）。"""
    return accel_kmhs is not None and abs(accel_kmhs) / G_TO_KMHS <= band_g


def hold_dead(error: float, deadband: float) -> float:
    """不感帯を引いた偏差 e′。|e| ≤ deadband なら 0、外側は e − deadband × sign(e)（連続）。
    pattern_loop の _dead と同じ（持ち込み）。"""
    if abs(error) <= deadband:
        return 0.0
    return error - math.copysign(deadband, error)


def hold_min_step_delta_pulse(dev_kmh: float, deadband_kmh: float, dt_s: float) -> int:
    """1 刻み寄せ（PI を使わない保持。手順2 用）の 1 周期のブレーキ指令の動き [pulse]。

    pattern_loop の RAMP_CRUISE で感度が未知のとき（no_sens）の動かし方と同じ: e′ ＝ 不感帯を
    引いた偏差（dev ＝ 基準 − 実車速）の向きへ最小刻み 1 個（HOLD_MIN_STEP_PULSE）だけ動かす。
    e′ ＝ 0 または dt ≤ 0 なら動かさない。遅すぎ（e′ ＞ 0）は浅く（開度が減る向き ＝ 負）、
    速すぎ（e′ ＜ 0）は深く（正）。"""
    e_dead = hold_dead(dev_kmh, deadband_kmh)
    if e_dead == 0.0 or dt_s <= 0.0:
        return 0
    return -int(math.copysign(HOLD_MIN_STEP_PULSE, e_dead))


def hold_verdict(
    in_band_s: float, since_ref_start_s: float, hold_s: float, timeout_s: float
) -> bool | None:
    """保てたか。帯の中の合計が hold_s に達したら True（途切れても合計は戻さない）、
    基準が上がり始めてから timeout_s 経っても達しなければ False、まだなら None。"""
    if in_band_s >= hold_s:
        return True
    if since_ref_start_s >= timeout_s:
        return False
    return None


def hold_breakaway_pct(meta: dict[str, Any], min_pct: float) -> float:
    """0 km/h の端の開度 [%] ＝ ブレーキの min ＋ |pkl meta の launch_breakaway_u_pct|
    （動き出す開度。無ければ KeyError）。手順2 の自動付与と手順13 が共有する。"""
    return float(min_pct) + abs(float(meta["launch_breakaway_u_pct"]))


def hold_opening_meta(
    points: list[HoldPointResult] | tuple[HoldPointResult, ...],
    *,
    creep_kmh: float,
    breakaway_pct: float,
    min_pct: float,
    source_csv: str,
) -> dict[str, Any]:
    """pkl の meta["hold_opening"]（hold_scene.hold_opening_points が読む形）を作る。
    speed_kmh・opening_pct は帯の中の平均（保てなかった点は狙い・初めの開度。補間に使われない）。"""
    return {
        "points": [
            {
                "speed_kmh": float(p.target_kmh if p.speed_kmh is None else p.speed_kmh),
                "opening_pct": float(
                    p.initial_opening_pct if p.opening_pct is None else p.opening_pct
                ),
                "held": bool(p.held),
                "target_kmh": float(p.target_kmh),
                "in_band_s": float(p.in_band_s),
                "elapsed_s": float(p.elapsed_s),
                "initial_opening_pct": float(p.initial_opening_pct),
            }
            for p in points
        ],
        "creep_kmh": float(creep_kmh),
        "breakaway_pct": float(breakaway_pct),
        "min_pct": float(min_pct),
        "source_csv": source_csv,
    }


async def _wait_stopped(hw: ResearchHardware, cfg: ResearchConfig) -> None:
    """平均車速が停車速度未満になるまで待つ（stop_confirm_max_wait_s まで）。"""
    limit = cfg.pedal_search.stop_confirm_max_wait_s
    waited = 0.0
    while True:
        speed = await mean_speed(hw, 1.0)
        waited += 1.0
        if speed < VEHICLE_STOP_SPEED_KMH:
            return
        if waited >= limit:
            raise DriveError(
                f"停車保持へ戻しても {limit:g}s 以内に停車しません（車速 {speed:.2f} km/h）"
            )


async def _run_hold_point(
    hw: ResearchHardware,
    cfg: ResearchConfig,
    *,
    target_kmh: float,
    creep_kmh: float,
    hold_pct: float,
    initial_pct: float,
    pre_pct: float,
    kp: float | None,
    ki: float | None,
) -> tuple[HoldPointResult, int]:
    """1 点を保つ。戻り値は (結果, 終わったときの指令位置 [pulse])。

    基準 = 0 から HOLD_REF_RAMP_KMHS で target_kmh まで上げ、その後一定。ブレーキだけで保つ
    （アクセルは min のまま）。指令 = −初めの開度 ＋ HoldScenePI の上乗せ（偏差 = 基準 − 実車速）。
    停車保持の開度から、基準を上げる前に max(pre_pct, 初めの開度)（pre_pct ＝ 動き出す開度）まで
    刻み送り（step_to_position）で寄せておく（止まったまま。手順3 の launch_release_ramp が
    動き出す開度まで離してから発進するのと同じ考え）。その後は刻み送りと同じ速さ
    （search_step_pulse ÷ HOLD_STEP_DWELL_S）を上限に 1 周期ずつ寄せ、上限で追いつかない周期は
    PI の積分を進めない（止まったまま積分だけがたまって、離れた後に行き過ぎるのを防ぐ）。

    kp・ki が None のとき（手順2。まだ PI が無い）は PI を使わず、G ランプの定速の保持で感度が
    未知のときと同じ 1 刻み寄せにする: 1 周期ごとに、不感帯（learning.ramp_dev_deadband_kmh）を
    引いた偏差の向きへ、指令を最小刻み 1 個（HOLD_MIN_STEP_PULSE）だけ動かす（遅すぎなら浅く）。
    動かす範囲は [min のブレーキ, 停車保持の開度]。判定・記録・停車保持への戻しは PI のときと同じ。
    """
    lr = cfg.learning
    interval = cfg.control.loop_interval_s
    window_s = cfg.decel_stop.slope_window_s
    timeout_s = lr.ramp_turn_time_s + lr.ramp_cruise_timeout_s
    ramp_end_s = target_kmh / HOLD_REF_RAMP_KMHS
    slew_pulse = max(1, round(search_step_pulse(cfg) / HOLD_STEP_DWELL_S * interval))
    pi = (
        HoldScenePI(
            kp=kp, ki=ki, dt_s=interval,
            lo_pct=cfg.vehicle.min_brake_opening_pct, hi_pct=hold_pct,
        )
        if kp is not None and ki is not None else None
    )
    deadband = max(0.0, lr.ramp_dev_deadband_kmh)
    lo_pulse = opening_to_pulse(cfg.vehicle.min_brake_opening_pct)
    hi_pulse = opening_to_pulse(hold_pct)
    ff_effort = -initial_pct
    cmd_pos = opening_to_pulse(max(pre_pct, initial_pct))
    await step_to_position(
        hw.brake, opening_to_pulse(hold_pct), cmd_pos,
        step_pulse=search_step_pulse(cfg), dwell_s=HOLD_STEP_DWELL_S,
    )
    hist: deque[tuple[float, float]] = deque()
    in_band_s = 0.0
    sum_speed = sum_open = 0.0
    n_band = 0
    note = ""
    loop = asyncio.get_running_loop()
    t0 = loop.time()
    last = t0
    held: bool | None = None
    while held is None:
        now = loop.time()
        t = now - t0
        dt = interval if now == t0 else now - last
        last = now
        try:
            speed = await hw.can.read_speed()
        except Exception as exc:
            raise DriveError(f"CAN 車速を読めません（{type(exc).__name__}: {exc}）") from exc
        if speed > creep_kmh + HOLD_OVERSPEED_MARGIN_KMH:
            note = f"車速 {speed:.2f} km/h がクリープ車速 + {HOLD_OVERSPEED_MARGIN_KMH:g} を超えた"
            held = False
            break
        ref = min(t * HOLD_REF_RAMP_KMHS, target_kmh)
        if pi is None:
            cmd_pos += hold_min_step_delta_pulse(ref - speed, deadband, dt)
            cmd_pos = min(hi_pulse, max(lo_pulse, cmd_pos))
        else:
            i_before = pi.integral if pi.active else 0.0
            effort, _ = pi.step(ref - speed, ff_effort)  # 範囲 [min, 停車保持] はここで収まる
            want = opening_to_pulse(-effort)
            if abs(want - cmd_pos) > slew_pulse:
                pi.integral = i_before  # 寄せの上限で追いつかない周期は積分を進めない
            cmd_pos += max(-slew_pulse, min(slew_pulse, want - cmd_pos))
        await hw.brake.move_to_position(cmd_pos, smooth_over_s=interval * AXIS_SMOOTH_DUTY)
        actual_pct = pulse_to_opening(await hw.brake.read_position())
        # 実測の加速度: pattern_loop の RAMP_CRUISE と同じ（直近 slope_window_s の最小二乗の傾き、
        # 3 点そろってから）
        hist.append((t, speed))
        while hist and t - hist[0][0] > window_s:
            hist.popleft()
        accel = fit_slope([a for a, _ in hist], [b for _, b in hist]) if len(hist) >= 3 else None
        # 帯を数えるのは基準が一定になってから。止まった車も加速度 0 なので、停車速度以下の行は
        # 数えない（G ランプの定速の保持は走っている車速でしか判定しないので、そこだけ足した条件。
        # スタブで 1.25 km/h の点が 0.018 km/h のまま「保てた」になった）
        if (
            t >= ramp_end_s
            and speed > VEHICLE_STOP_SPEED_KMH
            and hold_band_ok(accel, lr.ramp_cruise_band_g)
        ):
            in_band_s += dt
            sum_speed += speed
            sum_open += actual_pct
            n_band += 1
        held = hold_verdict(in_band_s, t, lr.ramp_cruise_hold_s, timeout_s)
        if held is None:
            await asyncio.sleep(max(0.0, t0 + t + interval - loop.time()))
    elapsed = loop.time() - t0
    result = HoldPointResult(
        target_kmh=round(target_kmh, 3), held=bool(held),
        speed_kmh=sum_speed / n_band if n_band else None,
        opening_pct=sum_open / n_band if n_band else None,
        in_band_s=round(in_band_s, 3), elapsed_s=round(elapsed, 3),
        initial_opening_pct=round(initial_pct, 3), note=note,
    )
    return result, cmd_pos


async def run_hold_points(
    hw: ResearchHardware,
    cfg: ResearchConfig,
    *,
    creep_kmh: float,
    hold_pct: float,
    brake_search_pairs: list[tuple[float, float]],
    kp: float | None,
    ki: float | None,
    fallback_opening_pct: float,
    log: SessionLog | None = None,
) -> list[HoldPointResult]:
    """停車保持の状態から 4 点（hold_point_speeds）を低い順に保つ（run_low_stair と同じ位置づけ）。

    点ごとに _run_hold_point → 停車保持の開度へ刻み送りで戻す → 停車を待つ、を繰り返し、
    最後は停車保持で返す。初めの開度は BRAKE_SEARCH の組（hold_initial_opening）。
    kp・ki が None なら PI を使わず 1 刻み寄せで保つ（手順2。_run_hold_point 参照）。
    """
    s = cfg.pedal_search
    lr = cfg.learning
    speeds = hold_point_speeds(creep_kmh, s.hold_points_fractions, s.hold_points_extra_kmh)
    if not speeds:
        say("4 点の保持: 点がありません（クリープ車速が低すぎる）")
        return []
    listed = ", ".join(f"{v:.3f}" for v in speeds)
    cut_s = lr.ramp_turn_time_s + lr.ramp_cruise_timeout_s
    how = (
        f"PI（Kp {kp:.4f}・Ki {ki:.4f}）" if kp is not None and ki is not None else
        f"1 刻み寄せ（1 周期ごとに偏差の向きへ {HOLD_MIN_STEP_PULSE} pulse・不感帯 "
        f"{lr.ramp_dev_deadband_kmh:g} km/h）"
    )
    say(f"4 点の保持: 点 {listed} km/h（クリープ {creep_kmh:.2f} km/h）。"
        f"基準は {HOLD_REF_RAMP_KMHS:g} km/h/s で上げて一定・ブレーキだけで {how}。"
        f"加速度 ±{lr.ramp_cruise_band_g:g} G の帯が合計 "
        f"{lr.ramp_cruise_hold_s:g}s で保てた・{cut_s:g}s で打ち切り")
    hold_pulse = opening_to_pulse(hold_pct)
    out: list[HoldPointResult] = []
    for i, target in enumerate(speeds, 1):
        initial = hold_initial_opening(brake_search_pairs, target, fallback_opening_pct)
        initial = min(max(initial, cfg.vehicle.min_brake_opening_pct), hold_pct)
        mark(log, SECTION_PEDAL_SEARCH, PHASE_LOW_HOLD)
        say(f"  点 {i}/{len(speeds)}: {target:.3f} km/h・初めの開度 {initial:.2f}%")
        res, pos = await _run_hold_point(
            hw, cfg, target_kmh=target, creep_kmh=creep_kmh, hold_pct=hold_pct,
            initial_pct=initial, pre_pct=fallback_opening_pct, kp=kp, ki=ki,
        )
        out.append(res)
        got = "" if res.speed_kmh is None else (
            f"帯の平均 {res.speed_kmh:.3f} km/h・{res.opening_pct:.2f}%")
        say(f"    → {'保てた' if res.held else '保てず'}  {got}  帯 {res.in_band_s:.1f}s・"
            f"{res.elapsed_s:.1f}s{'  ' + res.note if res.note else ''}")
        mark(log, SECTION_PEDAL_SEARCH, PHASE_STOP_HOLD)
        await step_to_position(
            hw.brake, pos, hold_pulse,
            step_pulse=search_step_pulse(cfg), dwell_s=HOLD_STEP_DWELL_S,
        )
        await _wait_stopped(hw, cfg)
    return out


async def run_pedal_search(
    hw: ResearchHardware,
    cfg: ResearchConfig,
    *,
    window_s: float | None = None,
    log: SessionLog | None = None,
    save: bool = True,
    hold_points: bool | None = None,
    hold_pi: tuple[float, float] | None = None,
    breakaway_pct: float | None = None,
) -> PedalSearchResult:
    """クリープ車速と停車保持開度を測り、停車保持の状態で返す。実機のときだけ YAML へ保存する。

    2026-10-30 段6: save=False なら実機でも yaml へ書き戻さない（手順13 は読み取り専用の yaml）。
    hold_points（None なら pedal_search.hold_points_enabled）が真なら、最後に 4 点の保持
    （run_hold_points）を走る。hold_pi = (Kp, Ki)（pkl meta["hold_pi"]）があれば PI で保つ
    （手順13）。None なら PI を使わず 1 刻み寄せで保つ（手順2。PI は後でそのログから求める）。
    breakaway_pct は
    BRAKE_SEARCH の組が足りないときの初めの開度（動き出す開度。None なら停止確認開度）。
    """
    s = cfg.pedal_search
    v = cfg.vehicle
    brake_step = brake_step_pulse(cfg)
    hold_step = search_step_pulse(cfg)
    accel_min = opening_to_pulse(v.min_accel_opening_pct)
    brake_min = opening_to_pulse(v.min_brake_opening_pct)
    say(f"開度の定義: 原点 0 pulse = 0% / {STROKE_LIMIT_PULSE} pulse = 100%"
        f"（ブレーキ 1 刻み {brake_step} pulse = {pulse_to_opening(brake_step):.2f}%、"
        f"待ち {s.dwell_s:g}s）")
    mark(log, SECTION_PEDAL_SEARCH, PHASE_CREEP_WAIT)
    say(f"両ペダルを待機位置（min: アクセル {v.min_accel_opening_pct:g}% / "
        f"ブレーキ {v.min_brake_opening_pct:g}%）へ置いてクリープさせます …")
    await asyncio.gather(
        hw.accel.move_to_position(accel_min), hw.brake.move_to_position(brake_min)
    )

    base = await wait_creep_stable(hw, cfg, window_s=window_s)
    mark(log, SECTION_PEDAL_SEARCH, PHASE_BRAKE_SEARCH)
    pairs: list[tuple[float, float]] = [(v.min_brake_opening_pct, base)]  # min ＝ クリープ車速
    stop_pos = await _search_brake(hw, cfg, base, brake_min, brake_step, record=pairs)

    mark(log, SECTION_PEDAL_SEARCH, PHASE_STOP_HOLD)
    stop_pct = pulse_to_opening(stop_pos)
    hold_pct = round(min(stop_pct + s.stop_hold_margin_pct, cfg.vehicle.max_brake_opening_pct), 2)
    say(f"停車保持: 停止確認 {stop_pct:.2f}% + {s.stop_hold_margin_pct:g}% → "
        f"{hold_pct:.2f}% まで刻んで踏みます …")
    await step_to_position(
        hw.brake, stop_pos, opening_to_pulse(hold_pct),
        step_pulse=hold_step, dwell_s=HOLD_STEP_DWELL_S,
    )

    if s.low_stair_enabled:
        await run_low_stair(
            hw, cfg, creep_kmh=base, stop_confirm_pct=stop_pct, hold_pct=hold_pct, log=log
        )

    points: list[HoldPointResult] = []
    if s.hold_points_enabled if hold_points is None else hold_points:
        points = await run_hold_points(
            hw, cfg, creep_kmh=base, hold_pct=hold_pct, brake_search_pairs=pairs,
            kp=None if hold_pi is None else hold_pi[0],
            ki=None if hold_pi is None else hold_pi[1],
            fallback_opening_pct=stop_pct if breakaway_pct is None else breakaway_pct,
            log=log,
        )

    result = PedalSearchResult(
        creep_speed_kmh=round(base, 2),
        stop_confirm_pct=round(stop_pct, 2),
        stop_brake_opening_pct=hold_pct,
        brake_min_pct=cfg.vehicle.min_brake_opening_pct,
        hold_points=tuple(points),
        brake_search_pairs=tuple(pairs),
    )
    _print_result(result, s.stop_hold_margin_pct)
    if not save:
        say(f"save=False のため {cfg.source_path} は更新しません")
    elif hw.is_real:
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
    # config_updates() が yaml へ書くのは stop_brake_offset_pct（min からの量）・
    # creep_speed_kmh の2項目（apply_to_profile() は絶対開度で反映）。停止確認開度は
    # G ランプの発進の開始位置に使うが保存しない
    rows = (
        ("クリープ車速（基準）", f"{result.creep_speed_kmh:6.2f} km/h（yaml に保存。"
                                 f"手順2 末尾の推定値では上書きしない）"),
        ("停止確認開度", f"{result.stop_confirm_pct:6.2f} %（G ランプの発進の開始位置）"),
        ("停車保持開度", f"{result.stop_brake_opening_pct:6.2f} %（停止確認 + {margin_pct:g}%。"
                         f"min {result.brake_min_pct:.2f} ＋ "
                         f"{result.stop_brake_opening_pct - result.brake_min_pct:.2f}%）"),
    )
    say("── ペダル探索の結果 ──")
    for label, value in rows:
        say(f"  {label:<12} {value}")
