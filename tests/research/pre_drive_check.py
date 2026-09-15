"""走行前チェック（手順 1 と手順 2 の間。本番の走行前チェックを引用）。

本番 `RobotController` の arm（学習運転・自動走行の開始前）と同じ 3 段で行う:
    1. 踏込前チェック … 本番 PreCheckRunner（車速確認を除く。まだ走っていてよい）
    2. 停車           … ブレーキを踏んで車速 0 km/h を確認する
    3. 踏込後チェック … 本番 PreCheckRunner（アクチュエータ位置を除く。ブレーキを踏んでいるため）

本番との違い:
    - 2. の踏み方は、次に実行する手順が 2（閉ループパターン走行 → FF モデル作成）かどうかで
      分岐する（`next_step` 引数）。
        - 次が手順 2: 停車ブレーキ位置（`stop_brake_opening_pct`）がまだ判明していない
          （手順 2 のペダル探索で初めて決まる）ため、`stop_brake_opening_pct` を一気に踏まない。
          クリープ中に一気に踏むのは危険なため、pedal_search.step_mm 刻みで踏み、刻むたびに
          dwell_s の平均車速を読み、本番 VEHICLE_STOP_SPEED_KMH（0.02 km/h）未満になったところで
          止めて停止確認とする。車速が下がっている間は踏み増さない（踏み過ぎ防止。ペダル探索と同じ）。
        - 次が手順 2 以外（3 以降）: 手順 2 で停車ブレーキ位置が判明済みのため、
          `stop_brake_opening_pct` まで 20mm/s で一気に踏んで停止を確認する
          （`DIRECT_PRESS_SPEED_MM_S`）。
    - キャリブレーション項目は除く（tests 環境ではキャリブレーションを使わない）。
"""

from __future__ import annotations

from src.domain.control.conversions import VEHICLE_STOP_SPEED_KMH
from src.domain.pre_check import (
    ITEM_ACTUATOR_POSITION,
    ITEM_CALIBRATION,
    ITEM_COMMUNICATION,
    ITEM_PROFILE,
    ITEM_SERVO,
    ITEM_UPS,
    ITEM_VEHICLE_STOPPED,
    PreCheckRunner,
)
from src.models.pre_check import PreCheckResult
from tests.research.config import ChecksSection, ResearchConfig
from tests.research.drive_log import SECTION_PRE_DRIVE_CHECK, SessionLog, mark
from tests.research.hardware import DriveError, PreDriveCheckError, ResearchHardware
from tests.research.pedal_search import mean_speed, search_step_pulse
from tests.research.term import display_width, say
from tests.research.vehicle import build_vehicle_profile, opening_to_pulse, pulse_to_opening

# 走行ログ（drive_log.SessionLog）の phase 列
PHASE_PRE_CHECK = "PRE_CHECK"
PHASE_BRAKE_STEP = "BRAKE_STEP"
PHASE_POST_CHECK = "POST_CHECK"

# 停車ブレーキ位置（stop_brake_opening_pct）判明済み（次が手順 2 以外）のときの踏み込み速度。
# 段階的に探る必要が無いため、一気に踏んでよい。
DIRECT_PRESS_SPEED_MM_S = 20.0
# 踏み込み後、停止確認のために平均車速を測り直して待つ上限 [s]
DIRECT_PRESS_MAX_WAIT_S = 5.0

# 1 pulse = 0.01mm（PCON-CB の PCMD 単位。search_step_pulse と同じ換算）
_PULSE_PER_MM = 100.0

# checks.pre_* キー → 本番 ITEM_* 定数の対応表（`_excluded_items` / SKIP 表示に使う）
_ITEM_KEY_MAP: dict[str, str] = {
    ITEM_COMMUNICATION: "pre_communication",
    ITEM_SERVO: "pre_servo_state",
    ITEM_PROFILE: "pre_profile",
    ITEM_UPS: "pre_ups",
    ITEM_ACTUATOR_POSITION: "pre_actuator_position",
    ITEM_VEHICLE_STOPPED: "pre_vehicle_stopped",
}
# 表示文言・SKIP 表示の並び順（本番 PreCheckRunner.run の実行順からキャリブレーションを除いたもの）
_ORDERED_ITEMS: tuple[str, ...] = (
    ITEM_COMMUNICATION, ITEM_SERVO, ITEM_PROFILE, ITEM_UPS, ITEM_ACTUATOR_POSITION,
    ITEM_VEHICLE_STOPPED,
)
_SHORT_LABEL: dict[str, str] = {
    ITEM_COMMUNICATION: "通信",
    ITEM_SERVO: "サーボ",
    ITEM_PROFILE: "プロファイル",
    ITEM_UPS: "UPS",
    ITEM_ACTUATOR_POSITION: "アクチュエータ位置",
    ITEM_VEHICLE_STOPPED: "車速",
}
# 各段が本番 PreCheckRunner に実施させうる項目（キャリブレーションは常に除外・対象外）
_PRE_PHASE_ITEMS = frozenset(
    {ITEM_COMMUNICATION, ITEM_SERVO, ITEM_PROFILE, ITEM_UPS, ITEM_ACTUATOR_POSITION}
)
_POST_PHASE_ITEMS = frozenset(
    {ITEM_COMMUNICATION, ITEM_SERVO, ITEM_PROFILE, ITEM_UPS, ITEM_VEHICLE_STOPPED}
)


def _excluded_items(checks: ChecksSection) -> frozenset[str]:
    """`checks.pre_*` が false の項目名（ITEM_* 定数）を集める。"""
    return frozenset(item for item, key in _ITEM_KEY_MAP.items() if not getattr(checks, key))


def _phase_title(phase_items: frozenset[str], excluded: frozenset[str]) -> str:
    """実施する項目名から表示文言を組み立てる（`checks.*` で外した項目は載せない）。"""
    labels = [
        _SHORT_LABEL[item]
        for item in _ORDERED_ITEMS
        if item in phase_items and item not in excluded
    ]
    return "・".join(labels) if labels else "（実施項目なし）"


async def run_pre_drive_check(
    hw: ResearchHardware, cfg: ResearchConfig, *, log: SessionLog | None = None, next_step: int = 2
) -> int:
    """走行前チェックを行い、停車を確認したブレーキ位置 [pulse] を返す（その位置で保持したまま）。

    1 項目でも NG なら `PreDriveCheckError` を投げて走行に進ませない。
    `cfg.checks.pre_*` が false の項目は判定せず [SKIP] にする（`checks.pre_brake_stop: false` なら
    ブレーキも踏まない）。

    Args:
        next_step: この走行前チェックの直後に実行する手順番号。
            2（閉ループパターン走行）のときだけ停車ブレーキ位置が未判明のため段階的に踏む。
            それ以外は `stop_brake_opening_pct` まで一気に踏む。
    """
    runner = PreCheckRunner(
        accel_driver=hw.accel,
        brake_driver=hw.brake,
        can_reader=hw.can,
        ups_monitor=hw.ups,
        profile=build_vehicle_profile(cfg),
    )
    checks = cfg.checks
    checks_excluded = _excluded_items(checks)
    pre_skip = checks_excluded & _PRE_PHASE_ITEMS
    post_skip = checks_excluded & _POST_PHASE_ITEMS

    pre_exclude = frozenset({ITEM_CALIBRATION, ITEM_VEHICLE_STOPPED}) | checks_excluded
    post_exclude = frozenset({ITEM_CALIBRATION, ITEM_ACTUATOR_POSITION}) | checks_excluded

    mark(log, SECTION_PRE_DRIVE_CHECK, PHASE_PRE_CHECK)
    say(f"1. 踏込前チェック（{_phase_title(_PRE_PHASE_ITEMS, checks_excluded)}）…")
    _require_passed(await runner.run(exclude=pre_exclude), "踏込前チェック", skipped=pre_skip)
    mark(log, SECTION_PRE_DRIVE_CHECK, PHASE_BRAKE_STEP)
    if not checks.pre_brake_stop:
        say("2. 停止確認をスキップ（checks.pre_brake_stop: false）。踏まず現在位置のまま続行")
        stop_pos = await hw.brake.read_position()
    else:
        try:
            if next_step == 2:
                say("2. ブレーキを小刻みに踏んで停止確認 …（停車ブレーキ位置は手順 2 で判明）")
                stop_pos = await brake_until_stopped(hw, cfg)
            else:
                say("2. 停車保持ブレーキ位置（stop_brake_opening_pct）まで一気に踏んで停止確認 …")
                stop_pos = await brake_to_target(hw, cfg)
        except DriveError as exc:
            raise PreDriveCheckError(str(exc)) from exc
    mark(log, SECTION_PRE_DRIVE_CHECK, PHASE_POST_CHECK)
    say(f"3. 踏込後チェック（{_phase_title(_POST_PHASE_ITEMS, checks_excluded)}）…")
    _require_passed(await runner.run(exclude=post_exclude), "踏込後チェック", skipped=post_skip)
    say(f"走行前チェック OK: ブレーキ {pulse_to_opening(stop_pos):.2f}% で停車保持中")
    return stop_pos


async def brake_until_stopped(hw: ResearchHardware, cfg: ResearchConfig) -> int:
    """ブレーキを 1 刻みずつ踏み、平均車速が停車しきい値未満になった位置 [pulse] を返す。"""
    s = cfg.pedal_search
    step = search_step_pulse(cfg)
    limit = opening_to_pulse(s.brake_max_pct)
    say(f"  {s.step_mm:g}mm（{pulse_to_opening(step):.2f}%）刻み・待ち {s.dwell_s:g}s、"
        f"平均車速 {VEHICLE_STOP_SPEED_KMH:g} km/h 未満で停止確認（上限 {s.brake_max_pct:g}%）")
    pos = await hw.brake.read_position()
    speed = await mean_speed(hw, s.dwell_s)
    say(_line(pos, speed, "現在"))
    drop = 0.0
    while speed >= VEHICLE_STOP_SPEED_KMH:
        # 直前の待ちで車速が下がっているうちは踏み増さない（踏み過ぎ防止）
        holding = drop >= s.onset_margin_kmh
        if not holding:
            if pos + step > limit:
                raise PreDriveCheckError(
                    f"ブレーキを {s.brake_max_pct:g}% まで踏んでも停車しません"
                    f"（車速 {speed:.2f} km/h）。pedal_search.brake_max_pct を見直してください"
                )
            pos += step
            await hw.brake.move_to_position(pos, smooth_over_s=s.dwell_s)
        prev = speed
        speed = await mean_speed(hw, s.dwell_s)
        drop = prev - speed
        say(_line(pos, speed, "減速中のため保持" if holding else ""))
    say(f"  停止確認: ブレーキ {pos} pulse = {pulse_to_opening(pos):.2f}%")
    return pos


async def brake_to_target(hw: ResearchHardware, cfg: ResearchConfig) -> int:
    """停車ブレーキ位置（stop_brake_opening_pct）まで一気に踏み、停止確認する [pulse] を返す。

    手順 2（ペダル探索）で判明済みの位置のため、小刻みに探る必要はない。
    `DIRECT_PRESS_SPEED_MM_S` で一気に踏んだ後、踏み込み直後はまだ車速が落ちきっていない分だけ
    平均車速を測り直して待つ（`DIRECT_PRESS_MAX_WAIT_S` まで。位置は動かさない）。
    """
    s = cfg.pedal_search
    target = opening_to_pulse(cfg.feedforward.stop_brake_opening_pct)
    current = await hw.brake.read_position()
    distance_mm = abs(target - current) / _PULSE_PER_MM
    duration_s = distance_mm / DIRECT_PRESS_SPEED_MM_S
    say(f"  停車保持開度 {cfg.feedforward.stop_brake_opening_pct:.2f}%（{target} pulse）まで "
        f"{DIRECT_PRESS_SPEED_MM_S:g}mm/s で踏み込み …")
    await hw.brake.move_to_position_timed(target, current, duration_s)
    speed = await mean_speed(hw, s.dwell_s)
    say(_line(target, speed, "現在"))
    waited_s = s.dwell_s
    while speed >= VEHICLE_STOP_SPEED_KMH:
        if waited_s >= DIRECT_PRESS_MAX_WAIT_S:
            raise PreDriveCheckError(
                f"stop_brake_opening_pct（{cfg.feedforward.stop_brake_opening_pct:.2f}%）まで"
                f"踏んでも停車しません（{DIRECT_PRESS_MAX_WAIT_S:g}s 待機・"
                f"車速 {speed:.2f} km/h）。手順 2（ペダル探索）からやり直してください"
            )
        speed = await mean_speed(hw, s.dwell_s)
        waited_s += s.dwell_s
        say(_line(target, speed, "停止待ち"))
    say(f"  停止確認: ブレーキ {target} pulse = {pulse_to_opening(target):.2f}%")
    return target


def _line(pos: int, speed: float, mark: str) -> str:
    return (f"  ブレーキ {pos:5d} pulse ({pulse_to_opening(pos):5.2f}%)  "
            f"平均車速 {speed:6.2f} km/h {mark}")


def _require_passed(
    result: PreCheckResult, title: str, *, skipped: frozenset[str] = frozenset()
) -> None:
    """実行結果を表示して、1 項目でも NG なら例外にする。

    `skipped`（checks.pre_* で外した項目）は [SKIP] として表示するだけで判定しない。
    全項目除外で `result.items` が空でも `max()` で落ちないよう既定値を持たせている。
    """
    names = [item.item_name for item in result.items] + list(skipped)
    width = max((display_width(name) for name in names), default=0)
    for item in result.items:
        pad = " " * (width - display_width(item.item_name))
        mark = "OK" if item.passed else "NG"
        detail = f"  … {item.error_message}" if item.error_message else ""
        say(f"  [{mark}] {item.item_name}{pad}{detail}")
    for name in _ORDERED_ITEMS:
        if name not in skipped:
            continue
        pad = " " * (width - display_width(name))
        say(f"  [SKIP] {name}{pad}  … checks.{_ITEM_KEY_MAP[name]}: false")
    if not result.passed:
        failed = ", ".join(
            f"{item.item_name}（{item.error_message}）" for item in result.failed_items
        )
        raise PreDriveCheckError(f"{title} NG: {failed}")
