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
from collections.abc import Callable
from dataclasses import dataclass, replace
from typing import Any

from tests.research.config import ResearchConfig
from tests.research.drive_log import SECTION_PEDAL_SEARCH, SessionLog, mark
from tests.research.hardware import ActuatorProtocol, DriveError, ResearchHardware
from tests.research.research_types import VEHICLE_STOP_SPEED_KMH, VehicleProfile
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


@dataclass(frozen=True)
class PedalSearchResult:
    creep_speed_kmh: float
    stop_confirm_pct: float
    stop_brake_opening_pct: float  # 絶対開度（内部は絶対開度のまま。yaml へは min からの量で書く）
    # 2026-10-08 段2l-7: yaml へ「止まった開度＋マージン − ブレーキの min」で書くための min [%]
    brake_min_pct: float = 0.0

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
    hw: ResearchHardware, cfg: ResearchConfig, base: float, start_pos: int, step: int
) -> int:
    """ブレーキを start_pos（min）から刻んで踏み、停止確認できた位置 [pulse] を返す。

    車速が下がっている間（直前の刻みで onset_margin_kmh 以上下がった）は踏み増さない
    （踏み過ぎ防止）。vehicle.max_brake_opening_pct まで踏んでも停車しなければ DriveError。
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


async def run_pedal_search(
    hw: ResearchHardware,
    cfg: ResearchConfig,
    *,
    window_s: float | None = None,
    log: SessionLog | None = None,
) -> PedalSearchResult:
    """クリープ車速と停車保持開度を測り、停車保持の状態で返す。実機のときだけ YAML へ保存する。"""
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
    stop_pos = await _search_brake(hw, cfg, base, brake_min, brake_step)

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

    result = PedalSearchResult(
        creep_speed_kmh=round(base, 2),
        stop_confirm_pct=round(stop_pct, 2),
        stop_brake_opening_pct=hold_pct,
        brake_min_pct=cfg.vehicle.min_brake_opening_pct,
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
