"""手順 2-0: ペダル探索（不感帯と停車保持開度を車速応答で測る）。

tests 環境では本番のキャリブレーションを使わず、開度 0% = 原点、100% = 9500 pulse とする
（tests/research/vehicle.py）。原点からペダルに触れるまでの隙間と遊びは設置で変わるため、
走行前に毎回ここで測る。

走行前チェック（tests/research/pre_drive_check.py）で停車を確認した状態から始める。

手順（1 刻み search_step_mm。刻むたびに dwell_s 待ち、その間の車速の傾きで判定する）:
    1. クリープ安定待ち … 両ペダル原点（走行前チェックのブレーキを離す）。平均車速が落ち着いたら
                           基準車速とする
    2. アクセル探索     … 刻んで踏み、車速の傾きが +onset_accel_kmhs 以上を confirm_count 回
                           連続で超えたら、最初に超えた刻みの位置をアクセル不感帯とする →
                           原点へ戻して 1. をやり直す
    3. ブレーキ探索     … 刻んで踏み、車速の傾きが -onset_accel_kmhs 以下を連続で割ったら
                           ブレーキ不感帯。車速が下がっている間は踏み増さず、
                           VEHICLE_STOP_SPEED_KMH 未満になった位置で停止確認
    4. 停車保持         … 停止確認開度 + stop_hold_margin_pct まで刻んで踏み、そのまま保持する

いきなり目標開度を踏まないのは、クリープ中の急制動とペダルへの衝撃を避けるため。連続移動に
しないのは、アクチュエータの最低速度（ブレーキ 10mm/s）で踏み続けると、むだ時間 0.6〜0.9s と
停車までの数秒の間に大きく踏み過ぎるため。

2026-09-17 段1b（ProblemReport_20260916）で反応判定を「待ち時間の平均車速が基準を超えたか」から
「車速の傾き」に変えた。旧判定は (a) 踏んでから車速が動くまでの遅れを 1 秒平均で薄めてしまう、
(b) 基準が探索開始時の固定値なので、クリープのドリフト（実測で−0.4 km/h/s）を拾う、という
2つの理由で検出位置が真値より 1〜2 刻み深く出ていた（実機ログで実証済み）。傾きなら固定基準との
比較が要らないのでドリフトを拾わず、待ち時間の後半だけを使うので遅れの影響も減る。

2026-09-20 クリープ安定判定（1. の停止条件）も「連続する2つの窓平均の差 < 閾値」から
「窓平均の傾き < 閾値 かつ 最短経過時間」に変えた。クリープ車速は漸近的に上がるため、旧判定は
まだ上がっている途中で差が閾値を割ってしまう（実機ログで、真の平衡 5.00 km/h に対し
4.79→4.88→4.93→4.96→4.98 km/h と 17.9 秒かけて推移する途中の 4.88 km/h で確定していた）。

2026-09-21 クリープ平衡車速（wait_creep_stable が返し PedalSearchResult.creep_speed_kmh に
入る値）を yaml に保存する
対象へ昇格した。従来は表示のみで、yaml の feedforward.creep_speed_kmh は
model_training.estimate_dynamics_params が出す「ペダルオフ・|dv|<0.3 km/h/s のサンプルの中央値」
（4.7645）のままだった。この中央値は母数の69%が手順2末尾のクリープ発進
（pattern_loop._advance_creep_launch。傾き 0.1 km/h/s・最短5s でおよそ10.3秒で打ち切り、終端
4.88 km/h・まだ +0.065 km/h/s で上昇中）の上昇途中のサンプルで、平衡より 0.24 km/h 低く出て
いた（実機ログで実証済み）。一方 2-0 の wait_creep_stable は 20.1 秒かけて 5.00 km/h のプラト
ーまで待てており、指数外挿（時定数 τ≈2.70s）で検算しても 9/21・9/20 両日のクリープ発進12本
すべて平衡 5.00 km/h で、冷間・暖機後の差もない。この値を config_updates() で yaml に保存し、
pattern_drive.MEASURED_KEYS に creep_speed_kmh を加えて estimate_dynamics_params の推定値で
上書きされないようにした（ff_params.free_accel_at が 4.77〜5.00 km/h の帯を惰行域と誤認する
不具合の修正を兼ねる）。クリープ発進パターン自体と learning.creep_launch_* の設定はクリープ
加速カーブのデータ源のため現状維持。
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

SPEED_SAMPLE_INTERVAL_S = 0.1  # 平均車速を取るときの読み取り間隔（CAN 10Hz）
HOLD_STEP_DWELL_S = 0.2  # 停車した後に保持位置まで踏むときの 1 刻みの待ち [s]

# 傾き算出に使う、待ち時間サンプル列のうち後半の割合。踏んでから車速が動くまでの遅れ
# （むだ時間）が前半に乗るため、前半は捨てて後半だけを最小二乗にかける
SLOPE_TAIL_FRACTION = 0.5

# 不感帯探索の1刻みの移動（smooth_over_s）にかける時間の下限 [s]。刻み幅（search_step_mm）に
# 比例させて短くするが、これ未満にするとアクチュエータの位置決めが追いつかず暴れるおそれがある
MOVE_DWELL_MIN_S = 0.2

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
        """実測した不感帯・停車保持開度・クリープ平衡車速をプロファイルに反映したコピーを返す。"""
        ffp = replace(
            profile.feedforward_params,
            accel_deadband_pct=self.accel_deadband_pct,
            brake_deadband_pct=self.brake_deadband_pct,
            stop_brake_opening_pct=self.stop_brake_opening_pct,
            creep_speed_kmh=self.creep_speed_kmh,
        )
        return replace(profile, feedforward_params=ffp)

    def config_updates(self) -> dict[str, Any]:
        return {
            "feedforward.accel_deadband_pct": self.accel_deadband_pct,
            "feedforward.brake_deadband_pct": self.brake_deadband_pct,
            "feedforward.stop_brake_opening_pct": self.stop_brake_opening_pct,
            "feedforward.creep_speed_kmh": self.creep_speed_kmh,
        }


def search_step_pulse(cfg: ResearchConfig) -> int:
    """停車保持位置への刻み送り（step_to_position）の1刻みの移動量 [pulse]（位置指令は0.01mm単位）。

    pre_drive_check.py・stop_decel.py も自分の刻み送りにこの関数を使う。不感帯探索専用の刻みは
    deadband_search_step_pulse を使う（停車保持まで遅くならないよう分離している）。
    """
    return max(1, round(cfg.pedal_search.step_mm * 100))


def deadband_search_step_pulse(cfg: ResearchConfig) -> int:
    """不感帯探索の1刻みの移動量 [pulse]（位置指令は 0.01mm 単位。search_step_mm を使う）。"""
    return max(1, round(cfg.pedal_search.search_step_mm * 100))


def _move_dwell_s(cfg: ResearchConfig) -> float:
    """不感帯探索の1刻みの移動（smooth_over_s）にかける時間 [s]。

    刻み幅 search_step_mm に比例させる（例: search_step_mm=0.1・step_mm=0.5・dwell_s=1.0 なら
    0.2s）。反応を測る待ち時間（dwell_s そのもの。SPEED_SAMPLE_INTERVAL_S 間隔でサンプルを
    取る時間）とは別で、動かす時間だけを短くする。刻みが小さいのに大きい刻み用の dwell_s を
    丸ごと移動にかけるのは無駄なため。下限は MOVE_DWELL_MIN_S。
    """
    s = cfg.pedal_search
    return max(MOVE_DWELL_MIN_S, s.dwell_s * s.search_step_mm / s.step_mm)


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


async def speed_samples(hw: ResearchHardware, duration_s: float) -> list[tuple[float, float]]:
    """duration_s の間 CAN 車速を SPEED_SAMPLE_INTERVAL_S 間隔で読み、
    (経過時間, 車速) の列を返す。
    """
    loop = asyncio.get_running_loop()
    start = loop.time()
    end = start + duration_s
    samples: list[tuple[float, float]] = []
    while True:
        try:
            speed = await hw.can.read_speed()
        except Exception as exc:
            raise DriveError(f"CAN 車速を読めません（{type(exc).__name__}: {exc}）") from exc
        samples.append((loop.time() - start, speed))
        if loop.time() >= end:
            return samples
        await asyncio.sleep(SPEED_SAMPLE_INTERVAL_S)


def speed_slope(samples: list[tuple[float, float]]) -> float:
    """(経過時間, 車速) の列の後半 SLOPE_TAIL_FRACTION から、最小二乗で傾き [km/h/s] を出す。

    後半だけを使うのは、踏んでから車速が動くまでの遅れ（むだ時間）が前半に乗るため
    （SLOPE_TAIL_FRACTION のコメント参照）。サンプルが2点未満なら 0.0 を返す。
    """
    tail_from = max(0, len(samples) - max(2, round(len(samples) * SLOPE_TAIL_FRACTION)))
    tail = samples[tail_from:]
    if len(tail) < 2:
        return 0.0
    n = len(tail)
    mean_t = sum(t for t, _ in tail) / n
    mean_v = sum(v for _, v in tail) / n
    num = sum((t - mean_t) * (v - mean_v) for t, v in tail)
    den = sum((t - mean_t) ** 2 for t, _ in tail)
    if den == 0.0:
        return 0.0
    return num / den


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


def _step_line(label: str, pos: int, speed: float, slope: float, base: float, mark: str) -> str:
    return (f"  {label} {pos:5d} pulse ({pulse_to_opening(pos):5.2f}%)  "
            f"平均車速 {speed:6.2f} km/h（基準比 {speed - base:+.2f}） "
            f"傾き {slope:+.2f} km/h/s {mark}")


async def _search_accel(
    hw: ResearchHardware, cfg: ResearchConfig, base: float, step: int
) -> int:
    s = cfg.pedal_search
    limit = opening_to_pulse(s.accel_max_pct)
    move_dwell = _move_dwell_s(cfg)
    say(f"アクセル探索: 車速の傾きが {s.onset_accel_kmhs:+.2f} km/h/s 以上になるまで "
        f"{s.confirm_count} 刻み連続で踏みます（上限 {s.accel_max_pct:g}%）")
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
        await hw.accel.move_to_position(pos, smooth_over_s=move_dwell)
        samples = await speed_samples(hw, s.dwell_s)
        speed = sum(v for _, v in samples) / len(samples)
        slope = speed_slope(samples)
        rising = slope >= s.onset_accel_kmhs
        streak = streak + 1 if rising else 0
        if streak == 1:
            first = pos
        say(_step_line("アクセル", pos, speed, slope, base, "↑ 反応" if rising else ""))
        if streak >= s.confirm_count:
            say(f"アクセル不感帯: {first} pulse = {pulse_to_opening(first):.2f}%")
            return first


async def _search_brake(
    hw: ResearchHardware, cfg: ResearchConfig, base: float, step: int
) -> tuple[int, int]:
    """ブレーキ不感帯の位置と、停止確認できた位置を返す。"""
    s = cfg.pedal_search
    deadband_limit = opening_to_pulse(s.deadband_max_pct)
    stop_limit = opening_to_pulse(s.brake_max_pct)
    move_dwell = _move_dwell_s(cfg)
    say(f"ブレーキ探索: 車速の傾きが {-s.onset_accel_kmhs:+.2f} km/h/s 以下になったら不感帯"
        f"（探索上限 {s.deadband_max_pct:g}%）、{VEHICLE_STOP_SPEED_KMH:g} km/h 未満で停止確認"
        f"（停止確認は {s.brake_max_pct:g}% まで続行）")
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
            limit = stop_limit if onset is not None else deadband_limit
            if pos + step > limit:
                if onset is None:
                    raise DriveError(
                        f"ブレーキを {s.deadband_max_pct:g}% まで踏んでも車速が下がりません"
                        f"（基準 {base:.2f} km/h）。"
                        "pedal_search.deadband_max_pct を見直してください"
                    )
                raise DriveError(
                    f"ブレーキを {s.brake_max_pct:g}% まで踏んでも停車しません"
                    f"（車速 {last_speed:.2f} km/h）。pedal_search.brake_max_pct を見直してください"
                )
            pos += step
            await hw.brake.move_to_position(pos, smooth_over_s=move_dwell)
        samples = await speed_samples(hw, s.dwell_s)
        speed = sum(v for _, v in samples) / len(samples)
        slope = speed_slope(samples)
        last_drop, last_speed = last_speed - speed, speed

        mark = "減速中のため保持" if holding else ""
        if onset is None:
            falling = slope <= -s.onset_accel_kmhs
            streak = streak + 1 if falling else 0
            if streak == 1:
                first = pos
            if falling:
                mark = "↓ 反応"
            if streak >= s.confirm_count:
                onset = first
                mark = f"↓ ブレーキ不感帯 {pulse_to_opening(onset):.2f}%"
        say(_step_line("ブレーキ", pos, speed, slope, base, mark))

        if speed < VEHICLE_STOP_SPEED_KMH:
            if onset is None:  # 確定前に止まった（効きが急）→ 最初に反応した位置を使う
                onset = first if streak > 0 else pos
            say(f"停止確認: {pos} pulse = {pulse_to_opening(pos):.2f}%")
            return onset, pos


async def run_pedal_search(
    hw: ResearchHardware,
    cfg: ResearchConfig,
    *,
    window_s: float | None = None,
    log: SessionLog | None = None,
) -> PedalSearchResult:
    """不感帯と停車保持開度を測り、停車保持の状態で返す。実機のときだけ YAML へ保存する。"""
    s = cfg.pedal_search
    # 不感帯探索の刻み（細かくできる）と、停車保持への刻み送りの刻み（据置）を分離
    search_step = deadband_search_step_pulse(cfg)
    hold_step = search_step_pulse(cfg)
    say(f"開度の定義: 原点 0 pulse = 0% / {STROKE_LIMIT_PULSE} pulse = 100%"
        f"（探索 1 刻み {search_step} pulse = {pulse_to_opening(search_step):.2f}%、"
        f"待ち {s.dwell_s:g}s）")
    mark(log, SECTION_PEDAL_SEARCH, PHASE_CREEP_WAIT)
    say("両ペダルを原点へ戻してクリープさせます …")
    await asyncio.gather(hw.accel.move_to_position(0), hw.brake.move_to_position(0))

    base = await wait_creep_stable(hw, cfg, window_s=window_s)
    mark(log, SECTION_PEDAL_SEARCH, PHASE_ACCEL_SEARCH)
    accel_pos = await _search_accel(hw, cfg, base, search_step)
    mark(log, SECTION_PEDAL_SEARCH, PHASE_CREEP_WAIT)
    say("アクセルを原点へ戻し、クリープが落ち着くのを待ちます …")
    await hw.accel.move_to_position(0)
    base = await wait_creep_stable(hw, cfg, window_s=window_s)
    mark(log, SECTION_PEDAL_SEARCH, PHASE_BRAKE_SEARCH)
    brake_pos, stop_pos = await _search_brake(hw, cfg, base, search_step)

    mark(log, SECTION_PEDAL_SEARCH, PHASE_STOP_HOLD)
    stop_pct = pulse_to_opening(stop_pos)
    hold_pct = round(min(stop_pct + s.stop_hold_margin_pct, cfg.vehicle.max_brake_opening_pct), 2)
    say(f"停車保持: 停止確認 {stop_pct:.2f}% + {s.stop_hold_margin_pct:g}% → "
        f"{hold_pct:.2f}% まで刻んで踏みます …")
    await step_to_position(
        hw.brake, stop_pos, opening_to_pulse(hold_pct),
        step_pulse=hold_step, dwell_s=HOLD_STEP_DWELL_S,
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
    # config_updates() / apply_to_profile() が書き戻すのは accel_deadband_pct・
    # brake_deadband_pct・stop_brake_opening_pct・creep_speed_kmh の4項目
    rows = (
        ("クリープ車速（基準）", f"{result.creep_speed_kmh:6.2f} km/h（yaml に保存。"
                                 f"手順2 末尾の推定値では上書きしない）"),
        ("アクセル不感帯", f"{result.accel_deadband_pct:6.2f} %"),
        ("ブレーキ不感帯", f"{result.brake_deadband_pct:6.2f} %"),
        ("停止確認開度", f"{result.stop_confirm_pct:6.2f} %"),
        ("停車保持開度", f"{result.stop_brake_opening_pct:6.2f} %（停止確認 + {margin_pct:g}%）"),
    )
    say("── ペダル探索の結果 ──")
    for label, value in rows:
        say(f"  {label:<12} {value}")
