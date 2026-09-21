"""手順 2: 閉ループパターン走行 → 2次多項式 FF モデル作成。

ProblemReport_20260910 手順 2:
    2-0. ペダル探索（pedal_search.py）… 不感帯と停車保持開度を実測し、停車保持まで行う
    2-1. 本番環境で採用している 2次多項式モデル作成用のパターン走行
    2-2. 走行結果を基にモデルを作成し、パラメータを config_testVehicle.yaml に保存

引用元（パターン生成・学習は本番の関数をそのまま呼ぶ。走行ループは pattern_loop.py が
アルゴリズムを移植した自前実装 — 遵守事項「/src の本番コードを実行しないこと」に対応）:
    src/domain/learning_drive.py        … LearningDriveManager.generate_patterns（パターン列。
                                          ペダルの固定開度だけ「不感帯 + offset」で指定する）
    tests/research/pattern_loop.py      … PatternLoop（100ms 周期の状態機械。開度は固定値を
                                          指令し、cap 到達・停車などの前進判定と上限G ガバナを
                                          実車速のフィードバックで回す。アルゴリズムは
                                          src/domain/control/learning_loop.py を移植）
    src/app/training_service.py         … train_inverse_model → estimate_dynamics_params の順
    src/infra/archive_manager.py        … CSV の列（drive_logs と同じ。drive_log.py）

本番との違い:
    - DB・セッション・学習サイクル・WebSocket は持たない。ログは走行前チェックから停車保持までを
      1 本の CSV/PNG（drive_log.SessionLog）に保存し、モデル作成はパターン走行の行だけを使う。
    - 走行ループは本番 LearningLoop のクラスを実行するのではなく、pattern_loop.py に
      アルゴリズムを移植した自前実装（PatternLoop）を使う。本番の CycleLoopBase が持つ
      絶対時刻グリッドスケジューリング・ウォッチドッグ・DBログバックログは、研究用の単発走行
      には不要なため持たない。安全チェックも SafetyMonitor クラスではなく過電流しきい値の
      直接比較で行う。
    - FOPDT 同定・SIMC の PID 初期ゲインは計算しない（Kp/Ki/Kd は手順 4/6/8 で適合する）。
    - 走行前の「停車保持ブレーキを踏む → 停車待ち」は 2-0 のペダル探索に置き換えた
      （いきなり踏まない）。
    - 走行後の緩減速は本番 _decelerate_to_stop（0.1s ごとに ±1%）ではなく stop_decel.py
      （一方向に刻んで踏み、行き過ぎそうなら待つ）で行う。
    - 不感帯・停車保持開度・クリープ平衡車速は 2-0 の実測を使う。estimate_dynamics_params の
      推定値は表示のみ。
    - アクセル不感帯プローブ・定常ブレーキ（BRAKE_HOLD）・高速巡航トリムの開度は、本番の絶対値
      ではなく「2-0 の不感帯 + learning.*_offsets_pct」（原点から測ると本番の値は遊びの中）。
    - 本番のパターン列に研究側で段を足す（A2・A5）: ACCEL_SWEEP を「不感帯 +
      learning.accel_sweep_add_offsets_pct」で、BRAKE_HOLD を「learning.brake_hold_low_start_kmh
      まで上げてから不感帯 + learning.brake_hold_low_offsets_pct を保持」で。
    - さらに A3・A4 を足す: トリム階段（learning.trim_stair_start_kmh の各車速まで上げてから
      不感帯 + learning.trim_stair_offsets_pct を順に保持）と、低速 × 高ブレーキ
      （learning.brake_hold_hard_start_kmh から不感帯 + learning.brake_hold_hard_offsets_pct を
      停車まで）。
    - 2026-09-14 定速階段（段2）: トリム階段の後に `learning.cruise_hold_speeds_kmh`
      （空なら足さない）を弱い PI で保持する `CruiseStairPattern` を 1 本足す（50 km/h 以上に
      定速保持の状態が無かったことへの対策。docs/memo.md 参照）。
    - ペダルゲインは不感帯 + learning.*_gain_min_offset_pct 以上のサンプルで推定する
      （pedal_gain.py。本番は +5% で、この車両のブレーキでは ≈0.38G になり定常サンプルが採れない）。
    - スタブ走行の結果は config_testVehicle.yaml に書き戻さない（実機の値を模擬値で壊さない）。
    - 2026-09-17 クリープ発進・クリープ域ブレーキ保持（段1。ProblemReport_20260916 課題#2）:
      定速階段の後、パターン列の末尾に `CreepLaunchPattern` を足す（`build_patterns` docstring
      参照）。モデル作成では `estimate_dynamics_params` の後に `creep_curve.
      estimate_creep_accel_curve` でクリープ加速カーブを推定し、それを基準にブレーキ側の
      低速ペダルゲインを同定する（`pedal_gain.py`）。クリープ加速カーブは `FeedforwardParams`
      ではなく研究側の `ff_params.ResearchFFParams` に持たせる（本番コード不変更）。
    - 2026-09-18 惰行カーブ低速端の再同定（段2.5。ProblemReport_20260916）:
      `estimate_dynamics_params` 直後に `coast_curve.estimate_coast_decel_curve` で
      惰行カーブの低速端（creep_speed_kmh〜learning.coast_curve_low_max_kmh）を細ビンで
      同定し直す（本番の COAST_CURVE_BIN_KMH=10.0 幅は 5〜10km/h を 1 ビンに潰し、
      `ff_params.free_accel_at` がクリープ平衡速度で段差になっていた）。推定順は
      惰行 → クリープ → ペダルゲイン が必須（クリープ・ゲインの `free_accel_at` 基準が
      惰行カーブを使うため）。クリープ加速カーブ・ペダルゲインの基準パラメータは
      `reference`（`before` の実測不感帯・停車保持・creep_speed_kmh を保ったまま、惰行
      カーブだけ今回同定したものに差し替えたもの）にする。`estimate_dynamics_params` の
      生の `after` をそのまま基準にしてはいけない（after の不感帯は表示専用の推定値で
      2-0 実測よりかなり粗く、「不感帯以下＝ペダルオフ」判定とゲインの分母が壊れる。
      `_estimate_research_coast_curve` / `_estimate_research_creep_curve` /
      `_estimate_research_pedal_gains` docstring 参照）。
    - 2026-09-19 クリープ域ブレーキの下限（段4改訂。ProblemReport_20260916）: 停止境界カーブは
      廃止し、定数 1 個（不感帯からの超過 [%]）に置き換えた。クリープ加速カーブ推定の直後に
      `stop_brake_floor.estimate_stop_brake_floor` で「クリープ域でブレーキを保持して実際に
      停車できた最小の不感帯超過」を同定する（基準は他と同じ `reference`）。
      `ff_candidate._apply_brake_trim` が低速のブレーキ開度をこの下限より浅くしない
      （`feedforward.brake_trim_max_kmh` が人が決める値として有効な場合のみ）。
"""

from __future__ import annotations

import asyncio
import math
import time
from collections.abc import Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

from src.domain.learning_drive import (
    ACCEL_SWEEP_RESET_BRAKE_PCT,
    BRAKE_HOLD_ACCEL_PCT,
    HOLD_DURATION_S,
    LearningDriveConfig,
    LearningDriveManager,
)
from src.domain.model_training import (
    PEDAL_GAIN_MIN_OPENING_PCT,
    estimate_dynamics_params,
)
from src.infra.settings import SafetySettings
from src.models.drive_log import DriveLog, DriveLogData
from src.models.learning_drive import LearningPattern, PatternKind
from src.models.profile import FeedforwardParams, VehicleProfile
from tests.research.axis_monitor import AxisMonitor
from tests.research.coast_curve import estimate_coast_decel_curve
from tests.research.config import ResearchConfig
from tests.research.creep_curve import estimate_creep_accel_curve
from tests.research.drive_log import (
    SECTION_DECEL_TO_STOP,
    SECTION_PATTERN_DRIVE,
    DriveSample,
    SessionLog,
    read_drive_logs,
)
from tests.research.ff_candidate import train_inverse_model_effective
from tests.research.ff_params import ResearchFFParams, research_ff_params
from tests.research.hardware import HW_REAL, DriveError, ResearchHardware
from tests.research.pattern_loop import (
    CreepLaunchPattern,
    CruiseStairPattern,
    LowOpenStairPattern,
    PatternLoop,
    PatternLoopConfig,
    SpeedTargetPattern,
    TrimStairPattern,
)
from tests.research.pedal_gain import apply_pedal_gains, estimate_gain_curve
from tests.research.pedal_search import PedalSearchResult
from tests.research.stop_brake_floor import estimate_stop_brake_floor
from tests.research.stop_decel import PHASE_APPROACH, StopDecelResult, decelerate_to_stop
from tests.research.term import drive_status_line, say
from tests.research.vehicle import build_vehicle_profile

__all__ = ["DriveError"]  # main・テストが pattern_drive.DriveError として参照する

# estimate_dynamics_params が推定する項目（すべて表示する）。creep_accel_* は FeedforwardParams
# ではなく ResearchFFParams が持つ（RESEARCH_PARAM_KEYS。_print_param_changes / build_ff_model の
# 保存処理はキーごとにどちらの器から読むかをそこで判定する）
FF_PARAM_KEYS: tuple[str, ...] = (
    "creep_speed_kmh",
    "creep_rate_kmhs",
    "creep_accel_speeds_kmh",
    "creep_accel_kmhs",
    "stop_brake_opening_pct",
    "engine_brake_decel_kmhs",
    "coast_decel_speeds_kmh",
    "coast_decel_kmhs",
    "stop_brake_floor_offset_pct",
    "pedal_gain_speeds_kmh",
    "accel_gain_kmhs_per_pct",
    "brake_gain_kmhs_per_pct",
    "accel_deadband_pct",
    "brake_deadband_pct",
)
# 2-0 のペダル探索で実測する項目。推定値は表示のみで書き戻さない。creep_speed_kmh は
# 2026-09-21 追加: estimate_dynamics_params の中央値は手順2末尾のクリープ発進（打ち切り時点の
# 上昇途中サンプルが69%）混じりで平衡より 0.24 km/h 低く出るため（実機確認）、2-0 の
# wait_creep_stable 実測（プラトーまで待つ）を正とする
MEASURED_KEYS: frozenset[str] = frozenset(
    {"stop_brake_opening_pct", "accel_deadband_pct", "brake_deadband_pct", "creep_speed_kmh"}
)
# FeedforwardParams ではなく ResearchFFParams が持つ項目（ProblemReport_20260916 課題#2・段4。
# coast_band_kmhs・brake_trim_max_kmh・brake_trim_ref_kmh は人が決める値なので自動保存の
# 対象外＝ここにも FF_PARAM_KEYS にも入れない）
RESEARCH_PARAM_KEYS: frozenset[str] = frozenset(
    {
        "creep_accel_speeds_kmh",
        "creep_accel_kmhs",
        "stop_brake_floor_offset_pct",
    }
)


@dataclass
class PatternDriveResult:
    csv_path: Path  # 走行ログの CSV（SessionLog.close で保存。学習はパターン走行の行だけ）
    samples: list[DriveSample]  # パターン走行の行
    duration_s: float  # パターン走行の所要時間（緩減速を含まない）
    stop: StopDecelResult | None = None


@dataclass
class ModelBuildResult:
    model_path: str
    metrics: dict[str, dict[str, float]]
    params: FeedforwardParams
    changed: list[str]
    # ResearchFFParams（クリープ加速カーブ等。FeedforwardParams には無い研究段階のみのパラメータ）
    research_params: ResearchFFParams = field(default_factory=ResearchFFParams)


# ─────────────────────────────────────────────────────────────────────
# 2-1. パターン走行
# ─────────────────────────────────────────────────────────────────────


class _DriveMonitor:
    """ログ 1 行ごとに走行ログ（CSV・グラフ）へ記録し、ターミナルに表示する。"""

    def __init__(
        self, cfg: ResearchConfig, patterns: Sequence[LearningPattern], log: SessionLog
    ) -> None:
        self._cfg = cfg
        self._patterns = patterns
        self._log = log
        self.samples: list[DriveSample] = []
        self._last_print = -math.inf
        self._last_pattern = -1

    def on_sample(
        self,
        data: DriveLogData,
        pattern_index: int,
        phase: str,
        governor_active: bool = False,
        alarm_accel: bool | None = None,
        alarm_brake: bool | None = None,
        monitor_accel: AxisMonitor | None = None,
        monitor_brake: AxisMonitor | None = None,
        cycle_ms: float | None = None,
    ) -> None:
        total = len(self._patterns)
        kind = self._patterns[pattern_index].kind.name if pattern_index < total else "DONE"
        sample = self._log.record(
            data, section=SECTION_PATTERN_DRIVE, phase=phase, pattern=f"{pattern_index + 1}:{kind}",
            governor_active=governor_active, alarm_accel=alarm_accel, alarm_brake=alarm_brake,
            monitor_accel=monitor_accel, monitor_brake=monitor_brake, cycle_ms=cycle_ms,
        )
        self.samples.append(sample)
        elapsed = sample.elapsed_s

        if pattern_index != self._last_pattern and pattern_index < total:
            self._last_pattern = pattern_index
            say(f"── パターン {pattern_index + 1}/{total}: "
                f"{_describe(self._patterns[pattern_index])} ──")
        if elapsed - self._last_print >= self._cfg.output.print_interval_s:
            self._last_print = elapsed
            pid = self._cfg.pid
            say(drive_status_line(
                t_s=elapsed,
                ref_kmh=data.ref_speed_kmh,
                actual_kmh=data.actual_speed_kmh,
                kp=pid.kp,
                ki=pid.ki,
                kd=pid.kd,
                accel_pct=data.accel_opening,
                brake_pct=data.brake_opening,
                extra=f"{pattern_index + 1}/{total} {kind} {phase}"
                      + ("  Gガバナー作動" if governor_active else ""),
            ))


def _describe(pattern: LearningPattern) -> str:
    text = (f"{pattern.kind.name}  アクセル {pattern.accel_opening:.1f}%  "
            f"ブレーキ {pattern.brake_opening:.1f}%")
    if isinstance(pattern, LowOpenStairPattern):  # TrimStairPattern の継承なので先に判定
        steps = " → ".join(f"{p:.2f}" for p in pattern.trim_steps_pct)
        return (f"{text}  低開度階段 {steps}%"
                f"（{len(pattern.trim_steps_pct)}段 各{pattern.step_hold_s:g}s、"
                f"{pattern.accel_target_kmh:g} km/h から開始、"
                f"{pattern.min_speed_kmh:g} km/h 以下で終了）")
    if isinstance(pattern, TrimStairPattern):
        steps = " → ".join(f"{p:.1f}" for p in pattern.trim_steps_pct)
        return (f"{text}  トリム階段 {steps}% 各 {pattern.step_hold_s:g}s"
                f"（{pattern.accel_target_kmh:g} km/h まで加速）")
    if isinstance(pattern, CruiseStairPattern):
        speeds = " → ".join(f"{v:g}" for v in pattern.hold_speeds_kmh)
        return (f"{text}  定速階段 {speeds} km/h  各 settle{pattern.settle_s:g}s+"
                f"hold{pattern.hold_s:g}s（打切り{pattern.step_timeout_s:g}s、"
                f"PI kp={pattern.kp:g} ki={pattern.ki:g}）")
    if isinstance(pattern, CreepLaunchPattern):
        action = (
            f"ブレーキ保持 {pattern.brake_opening:.1f}%で停車" if pattern.hold_after else "停車復帰"
        )
        return (f"{text}  クリープ発進 → {pattern.target_kmh:g} km/h"
                f"（打切り{pattern.timeout_s:g}s）→ {action}")
    if pattern.kind is PatternKind.CRUISE_TRIM:
        text += f"  トリム {pattern.trim_opening:.1f}%"
    if isinstance(pattern, SpeedTargetPattern):
        text += f"  （{pattern.accel_target_kmh:g} km/h まで加速）"
    return text


def _print_patterns(patterns: Sequence[LearningPattern]) -> None:
    say("── パターン一覧（本番 LearningDriveManager.generate_patterns ＋研究側の追加、"
        f"{len(patterns)} 本） ──")
    for i, pattern in enumerate(patterns, start=1):
        say(f"  {i:>2}. {_describe(pattern)}")


def build_patterns(cfg: ResearchConfig, profile: VehicleProfile) -> list[LearningPattern]:
    """本番のパターン列。ペダルの固定開度だけ「不感帯 + learning.*_offsets_pct」にする。

    対象はアクセル不感帯プローブ・定常ブレーキ（BRAKE_HOLD）・高速巡航トリム。本番の開度は
    キャリブレーション前提の絶対値で、原点から測る研究ハーネスでは遊びの中に入る（実機でブレーキ
    1〜10%・巡航トリム 1.5/3.0% は惰行と同じだった）。不感帯は profile の値（run_pattern_drive では
    2-0 の実測）。最大開度でのクランプは本番 generate_patterns が行う。

    研究側で足す段（A2・A5。最大開度でクランプする）:
      - ACCEL_SWEEP「不感帯 + accel_sweep_add_offsets_pct」… 本番の段（上限の割合）の前
      - BRAKE_HOLD「brake_hold_low_start_kmh まで加速 → 不感帯 + brake_hold_low_offsets_pct を保持」
        （SpeedTargetPattern）… cap からの BRAKE_HOLD の後
    研究側で足す段（A3・A4）:
      - BRAKE_HOLD「不感帯 + brake_hold_hard_accel_offset_pct で brake_hold_hard_start_kmh
        まで加速 → ブレーキ不感帯 + brake_hold_hard_offsets_pct を停車まで保持」
        （SpeedTargetPattern）… A5 の段の後
      - トリム階段「trim_stair_start_kmh の各車速まで 70% で加速 → 不感帯 +
        trim_stair_offsets_pct を trim_stair_step_s ずつ順に保持」（TrimStairPattern）
        … トリム階段の前まで（後述の定速階段の前）
    研究側で足す段（2026-09-14 定速階段。段2。空なら足さない）:
      - 定速階段「cruise_hold_speeds_kmh を弱い PI で 1 本ずつ保持」（CruiseStairPattern）
        … 定速階段の位置（トリム階段の後）
    研究側で足す段（2026-09-17 クリープ発進・クリープ域ブレーキ保持。段1。ProblemReport_20260916
    課題#2。creep_launch_count が 0 かつ creep_brake_hold_offsets_pct が空なら足さない）:
      - クリープ発進「両ペダル解放でクリープ自走 → creep_launch_target_kmh 到達
        （または creep_launch_timeout_s で打ち切り）→ 通常の停車復帰」を creep_launch_count 本
      - クリープ域ブレーキ保持「同じくクリープ自走で target_kmh まで上げ、ブレーキ不感帯 +
        offset を保持して停車」を creep_brake_hold_offsets_pct の各値で
      … パターン列の**末尾**（定速階段の後）。手順3のモード走行と同じ暖機状態
      （長時間の走行で温まったアクチュエータ・タイヤ）でクリープを測るため、走行の最初ではなく
      最後に置く。
    研究側で足す段（2026-09-19 低開度階段。段3-1。ProblemReport_20260919 候補(c)。
    low_open_stair_offsets_pct が空なら足さない）:
      - 低開度階段「停車から low_open_stair_start_kmh まで（1 段目と同じ開度で）加速 →
        不感帯 + low_open_stair_offsets_pct を low_open_stair_step_s ずつ 1 段ずつ一定保持
        （low_open_stair_descend なら折り返して下りも測る）」（LowOpenStairPattern）… 定速階段の
        後・クリープ発進の前。手順3 の低開度サンプルが低速走行抵抗にさらされた状態のため、
        定速階段と同じ暖機状態（低速の走行抵抗にも影響する長時間走行後）で測る。
    """
    ff = profile.feedforward_params
    lr = cfg.learning

    def above(deadband_pct: float, offsets: Sequence[float]) -> tuple[float, ...]:
        return tuple(round(deadband_pct + offset, 2) for offset in offsets)

    drive_config = LearningDriveConfig(
        accel_deadband_probe_pcts=above(ff.accel_deadband_pct, lr.accel_deadband_probe_offsets_pct),
        cruise_trim_openings_pct=above(ff.accel_deadband_pct, lr.cruise_trim_offsets_pct),
        brake_hold_openings_pct=above(ff.brake_deadband_pct, lr.brake_hold_offsets_pct),
    )
    patterns = LearningDriveManager(drive_config).generate_patterns(profile)

    # A2: 低開度の ACCEL_SWEEP を本番の段（上限の割合）の前に足す
    reset_brake = min(ACCEL_SWEEP_RESET_BRAKE_PCT, profile.max_brake_opening)
    add_sweeps = [
        LearningPattern(
            kind=PatternKind.ACCEL_SWEEP,
            accel_opening=min(opening, profile.max_accel_opening),
            brake_opening=reset_brake,
            hold_duration_s=HOLD_DURATION_S,
        )
        for opening in above(ff.accel_deadband_pct, lr.accel_sweep_add_offsets_pct)
    ]
    # A5: brake_hold_low_start_kmh から保持する BRAKE_HOLD を cap からの段の後に足す
    low_holds = [
        SpeedTargetPattern(
            kind=PatternKind.BRAKE_HOLD,
            accel_opening=min(BRAKE_HOLD_ACCEL_PCT, profile.max_accel_opening),
            brake_opening=min(opening, profile.max_brake_opening),
            hold_duration_s=HOLD_DURATION_S,
            accel_target_kmh=lr.brake_hold_low_start_kmh,
        )
        for opening in above(ff.brake_deadband_pct, lr.brake_hold_low_offsets_pct)
    ]
    # A4: 低速から高ブレーキで停車まで保持する BRAKE_HOLD を A5 の段の後に足す
    hard_holds = [
        SpeedTargetPattern(
            kind=PatternKind.BRAKE_HOLD,
            accel_opening=min(
                round(ff.accel_deadband_pct + lr.brake_hold_hard_accel_offset_pct, 2),
                profile.max_accel_opening,
            ),
            brake_opening=min(opening, profile.max_brake_opening),
            hold_duration_s=HOLD_DURATION_S,
            accel_target_kmh=lr.brake_hold_hard_start_kmh,
        )
        for opening in above(ff.brake_deadband_pct, lr.brake_hold_hard_offsets_pct)
    ]
    # A3: トリム階段をパターン列の末尾に足す
    steps = tuple(
        min(opening, profile.max_accel_opening)
        for opening in above(ff.accel_deadband_pct, lr.trim_stair_offsets_pct)
    )
    stairs = [
        TrimStairPattern(
            kind=PatternKind.CRUISE_TRIM,
            accel_opening=min(BRAKE_HOLD_ACCEL_PCT, profile.max_accel_opening),
            brake_opening=0.0,
            hold_duration_s=lr.trim_stair_step_s * len(steps),
            trim_opening=steps[0],
            accel_target_kmh=start_kmh,
            trim_steps_pct=steps,
            step_hold_s=lr.trim_stair_step_s,
        )
        for start_kmh in (lr.trim_stair_start_kmh if steps else [])
    ]
    # 2026-09-14 定速階段（段2）: トリム階段の後、パターン列の末尾に 1 本足す（空なら足さない）。
    # DRIVE_ACCEL の加速用開度はトリム階段と同じ考え方（BRAKE_HOLD_ACCEL_PCT をクランプ）。
    # 最高速の階段でも目標は cfg.vehicle.max_speed_kmh 未満（validate_config で検証済み）なので
    # トリム階段ほどの行き過ぎリスクは無い（cap への先読み無しで加速を終える点は同じ）
    cruise_stairs = (
        [
            CruiseStairPattern(
                kind=PatternKind.CRUISE_TRIM,
                accel_opening=min(BRAKE_HOLD_ACCEL_PCT, profile.max_accel_opening),
                brake_opening=0.0,
                hold_duration_s=(
                    (lr.cruise_hold_settle_s + lr.cruise_hold_hold_s)
                    * len(lr.cruise_hold_speeds_kmh)
                ),
                hold_speeds_kmh=tuple(lr.cruise_hold_speeds_kmh),
                settle_tol_kmh=lr.cruise_hold_settle_tol_kmh,
                settle_s=lr.cruise_hold_settle_s,
                hold_s=lr.cruise_hold_hold_s,
                step_timeout_s=lr.cruise_hold_step_timeout_s,
                kp=lr.cruise_hold_kp,
                ki=lr.cruise_hold_ki,
                max_rate_pct_per_s=lr.cruise_hold_max_rate_pct_per_s,
                initial_offset_pct=lr.cruise_hold_initial_offset_pct,
            )
        ]
        if lr.cruise_hold_speeds_kmh
        else []
    )
    # 2026-09-19 低開度階段（段3-1）: 定速階段の後・クリープ発進の前に 1 本足す（空なら足さない）
    steps_up = above(ff.accel_deadband_pct, lr.low_open_stair_offsets_pct)
    low_steps = tuple(
        min(opening, profile.max_accel_opening)
        for opening in (steps_up + steps_up[-2::-1] if lr.low_open_stair_descend else steps_up)
    )
    low_open_stairs = (
        [
            LowOpenStairPattern(
                kind=PatternKind.CRUISE_TRIM,
                accel_opening=low_steps[0],   # DRIVE_ACCEL も 1 段目と同じ開度（段差を作らない）
                brake_opening=0.0,
                hold_duration_s=lr.low_open_stair_step_s * len(low_steps),
                trim_opening=low_steps[0],
                accel_target_kmh=lr.low_open_stair_start_kmh,
                trim_steps_pct=low_steps,
                step_hold_s=lr.low_open_stair_step_s,
                min_speed_kmh=lr.low_open_stair_min_speed_kmh,
            )
        ]
        if low_steps
        else []
    )
    # 2026-09-17 クリープ発進・クリープ域ブレーキ保持（段1）: パターン列の末尾に足す（build_patterns
    # docstring の理由参照）。クリープ発進は通常の停車復帰（hold_after=False）、クリープ域ブレーキ
    # 保持はブレーキ「不感帯 + offset」を保持して停車させる（hold_after=True）。
    creep_launches = [
        CreepLaunchPattern(
            kind=PatternKind.CREEP_SETTLE,
            accel_opening=0.0,
            brake_opening=0.0,
            hold_duration_s=lr.creep_launch_timeout_s,
            target_kmh=lr.creep_launch_target_kmh,
            timeout_s=lr.creep_launch_timeout_s,
            hold_after=False,
        )
        for _ in range(lr.creep_launch_count)
    ]
    creep_brake_holds = [
        CreepLaunchPattern(
            kind=PatternKind.CREEP_SETTLE,
            accel_opening=0.0,
            brake_opening=min(opening, profile.max_brake_opening),
            hold_duration_s=lr.creep_launch_timeout_s,
            target_kmh=lr.creep_launch_target_kmh,
            timeout_s=lr.creep_launch_timeout_s,
            hold_after=True,
        )
        for opening in above(ff.brake_deadband_pct, lr.creep_brake_hold_offsets_pct)
    ]
    kinds = [p.kind for p in patterns]
    first_sweep = kinds.index(PatternKind.ACCEL_SWEEP)
    patterns[first_sweep:first_sweep] = add_sweeps
    kinds = [p.kind for p in patterns]
    last_hold = len(kinds) - 1 - kinds[::-1].index(PatternKind.BRAKE_HOLD)
    patterns[last_hold + 1:last_hold + 1] = low_holds + hard_holds
    patterns.extend(stairs)
    patterns.extend(cruise_stairs)
    patterns.extend(low_open_stairs)
    patterns.extend(creep_launches)
    patterns.extend(creep_brake_holds)
    return patterns


def _overcurrent_limit_ma(hw: ResearchHardware) -> float:
    """過電流しきい値は本番と同じ config/settings.toml の [safety] を使う（スタブは既定値）。"""
    if hw.settings is not None:
        return hw.settings.safety.overcurrent_limit_ma
    return SafetySettings().overcurrent_limit_ma


async def _release_pedals(hw: ResearchHardware) -> None:
    say("異常終了: ペダルを離します（両軸原点復帰）…")
    try:
        await asyncio.gather(hw.accel.home_return(), hw.brake.home_return())
    except Exception as exc:
        say(f"  警告: 原点復帰に失敗しました（{type(exc).__name__}: {exc}）")


async def run_pattern_drive(
    hw: ResearchHardware,
    cfg: ResearchConfig,
    *,
    pedal: PedalSearchResult,
    log: SessionLog | None = None,
    patterns: Sequence[LearningPattern] | None = None,
    loop_config: PatternLoopConfig | None = None,
) -> PatternDriveResult:
    """本番の学習運転パターンを PatternLoop（本番 LearningLoop 相当）で走らせ、
    緩減速で停車保持まで行う。

    前提: 2-0 のペダル探索が済み、停車保持開度（pedal.stop_brake_opening_pct）で停車している。
    `log` は走行前チェックから続く走行ログ（main が作って保存する）。無ければここで作り、
    終わりに（異常終了でも）保存する。
    `patterns` / `loop_config` はテストで短縮するための差し込み口。既定は本番と同じ。
    """
    profile = pedal.apply_to_profile(build_vehicle_profile(cfg))
    if patterns is None:
        patterns = build_patterns(cfg, profile)
    if loop_config is None:
        loop_config = PatternLoopConfig(
            coast_timeout_s=cfg.learning.coast_timeout_s,
            creep_launch_settle_kmhs=cfg.learning.creep_launch_settle_kmhs,
            creep_launch_settle_s=cfg.learning.creep_launch_settle_s,
            creep_launch_min_speed_kmh=cfg.learning.creep_launch_min_speed_kmh,
            creep_launch_settle_min_s=cfg.learning.creep_launch_settle_min_s,
        )
    _print_patterns(patterns)
    if hw.is_real:
        cap = profile.max_speed * loop_config.accel_speed_cap_frac
        say(f"*** 実機モード: 車両が 0 → 約 {cap:.0f} km/h まで加減速します。"
            "シャシダイナモ上で実施してください ***")
    say(f"開始状態: 停車保持ブレーキ {pedal.stop_brake_opening_pct:.2f}%（2-0 で確定）")
    say(f"各運転パターンの後は DRIVE_BRAKE で停車してから次へ進みます"
        f"（{loop_config.brake_stop_timeout_s:g}s 以内に停車しなければ中断）。"
        f"加速の打ち切り {loop_config.accel_full_range_timeout_s:g}s")
    say(f"Gガバナー: {profile.max_decel_g:g}G × {loop_config.g_limit_frac:g} 以上で頭打ち"
        f"（{loop_config.gov_reduce_step_pct:g}%/周期で下げる）、"
        f"× {loop_config.gov_release_frac:g} 未満で "
        f"{loop_config.gov_raise_step_pct:g}%/周期ずつ戻す")
    say("パターン走行は開度指令のみで FF/PID は使いません（Kp/Ki/Kd は表示のみ）")

    own_log = log is None
    session = SessionLog(cfg, hw) if log is None else log
    if own_log:
        session.start(SECTION_PATTERN_DRIVE, "")
    monitor = _DriveMonitor(cfg, patterns, session)
    finished = asyncio.Event()
    emergency = False

    async def on_complete() -> None:
        finished.set()

    async def on_emergency() -> None:
        nonlocal emergency
        emergency = True
        finished.set()

    learning_loop = PatternLoop(
        on_sample=monitor.on_sample,
        accel_driver=hw.accel,
        brake_driver=hw.brake,
        can_reader=hw.can,
        profile=profile,
        patterns=list(patterns),
        overcurrent_limit_ma=_overcurrent_limit_ma(hw),
        on_complete=on_complete,
        on_emergency=on_emergency,
        config=loop_config,
    )

    try:
        # PatternLoop の 100ms ループと Modbus を取り合わないよう、走行中はサンプラーを止める
        # （記録は PatternLoop の on_sample コールバックから行う）
        await session.stop_sampler()
        session.mark(SECTION_PATTERN_DRIVE, "")
        say(f"パターン走行を開始します（{len(patterns)} 本、"
            f"打ち切り {cfg.learning.timeout_s:.0f}s）")
        started = time.monotonic()
        completed = False
        try:
            learning_loop.start()
            try:
                await asyncio.wait_for(finished.wait(), timeout=cfg.learning.timeout_s)
            except TimeoutError as exc:
                raise DriveError(
                    f"パターン走行が learning.timeout_s={cfg.learning.timeout_s:.0f}s 以内に"
                    "終わりませんでした"
                ) from exc
            if emergency:
                reason = learning_loop.abort_reason or "理由不明。直前のログを確認してください"
                raise DriveError(f"PatternLoop が非常停止しました（{reason}）")
            completed = True
        finally:
            await learning_loop.stop_and_join()
            if not completed:
                # 本番の非常停止と同じく、まずペダルを離す（CSV・グラフの保存より先に）
                await _release_pedals(hw)

        duration = time.monotonic() - started
        say(f"全パターン完了（{duration:.0f}s）。緩減速で停車させ、停車保持ブレーキをかけます …")
        session.mark(SECTION_DECEL_TO_STOP, PHASE_APPROACH)
        session.start_sampler()
        stop = await decelerate_to_stop(hw, cfg, profile, log=session)
        say(f"停車保持: ブレーキ {stop.hold_pct:.2f}%")
        return PatternDriveResult(
            csv_path=session.csv_path, samples=monitor.samples, duration_s=duration, stop=stop
        )
    finally:
        if own_log:
            await session.close()


# ─────────────────────────────────────────────────────────────────────
# 2-2. モデル作成
# ─────────────────────────────────────────────────────────────────────


def build_ff_model(
    cfg: ResearchConfig,
    csv_path: Path,
    *,
    hw_mode: str,
    pedal: PedalSearchResult | None = None,
    write_config: bool | None = None,
) -> ModelBuildResult:
    """走行 CSV から 2次多項式 Ridge 逆モデルと物理定数を作り、実機走行なら YAML へ保存する。

    本番 training_service.train_and_apply と同じ順序（逆モデル → 物理定数）で、学習時点の
    プロファイル値を使う。逆モデルだけは A1（そのペダルが効いている行だけで学習）を入れた
    研究用の `train_inverse_model_effective` を呼ぶ。`pedal` があれば 2-0 の実測値を
    プロファイルへ反映してから学習する。サンプル不足は本番の LearningDataError をそのまま送出する。

    `write_config` 省略時（None）は従来どおり `hw_mode == HW_REAL` で判定する。明示的に指定すると
    実機モードでも保存を止められる（`tests.research.relearn` の `--dry-run` 用。段2.5）。
    """
    logs = read_drive_logs(csv_path)
    say(f"学習データ: {csv_path}（{len(logs)} 行）")
    profile = build_vehicle_profile(cfg)
    if pedal is not None:
        profile = pedal.apply_to_profile(profile)
    if write_config is None:
        write_config = hw_mode == HW_REAL
    if hw_mode != HW_REAL:
        profile.id = f"{profile.id}_{hw_mode}"  # スタブのモデルは別名で保存する

    say("train_inverse_model_effective: 2次多項式＋標準化＋Ridge（アクセル/ブレーキの 2 モデル）"
        "を、そのペダルが効いている行だけで学習 …")
    models_dir = Path(cfg.feedforward.model_path).parent
    model_path, metrics = train_inverse_model_effective(logs, profile, output_dir=str(models_dir))
    say(f"モデル保存: {model_path}")
    _print_metrics(metrics)

    say("estimate_dynamics_params: クリープ・惰行減速カーブ・ペダルゲインを推定 …")
    before = profile.feedforward_params
    after = estimate_dynamics_params(logs, before)

    # 惰行カーブの低速端を先に直す（段2.5）。サンプル抽出条件（不感帯・creep_speed_kmh）は
    # before（2-0 実測）を基準にする。差し替え先は after（他の推定値はそのまま残す）
    after = _estimate_research_coast_curve(cfg, logs, before, after)

    # 以降の研究側推定（クリープ加速カーブ・ペダルゲイン）が使う基準 reference を作る:
    # before（2-0 実測の不感帯・停車保持・creep_speed_kmh）はそのまま保ち、惰行カーブだけ
    # 今回同定したものに差し替える。**after をそのまま基準に使ってはいけない**——after の
    # accel_deadband_pct/brake_deadband_pct は estimate_dynamics_params の推定値（表示専用・
    # MEASURED_KEYS で保存対象外の参考値）で、2-0 実測（8.53%/12.00%）よりかなり粗い
    # （実測 3.0%/3.0% 相当）。これを基準にすると「両ペダルが不感帯以下＝ペダルオフ」判定が
    # 崩れ、待機位置（6.53%/10.00%）の行が軒並み「踏んでいる」扱いになって惰行サンプルが
    # 995→361 点まで減り、ゲインの分母（開度−不感帯）も壊れる（accel_gain が 4.51→0.52 まで
    # 縮む）。クリープ加速カーブ・低速ブレーキゲインの順で推定する（クリープ加速カーブを
    # 基準に使うため、この順序は必須。ProblemReport_20260916 課題#2）
    research_before = research_ff_params(cfg)
    reference = replace(
        before,
        coast_decel_speeds_kmh=after.coast_decel_speeds_kmh,
        coast_decel_kmhs=after.coast_decel_kmhs,
    )
    research_after = _estimate_research_creep_curve(cfg, logs, reference, research_before)
    # 段4改訂（ProblemReport_20260916）: クリープ域ブレーキの下限も reference（before 基準）で
    # 同定する（_estimate_research_coast_curve の docstring と同じ理由。after の不感帯は
    # 表示専用の粗い推定値でサンプル抽出条件が壊れる）
    research_after = _estimate_research_stop_brake_floor(cfg, logs, reference, research_after)
    after = _estimate_research_pedal_gains(cfg, logs, reference, after, research_after)
    _print_param_changes(before, after, research_before, research_after)

    if not write_config:
        if hw_mode == HW_REAL:
            say(f"--dry-run のため {cfg.source_path} は更新しません（推定値の確認のみ）")
        else:
            say(f"スタブ走行のため {cfg.source_path} は更新しません"
                "（実機の値を模擬値で上書きしないため）")
        return ModelBuildResult(
            model_path=model_path, metrics=metrics, params=after, changed=[],
            research_params=research_after,
        )

    updates: dict[str, Any] = {"feedforward.model_path": model_path}
    for key in FF_PARAM_KEYS:
        if key in MEASURED_KEYS:
            continue
        value = getattr(research_after, key) if key in RESEARCH_PARAM_KEYS else getattr(after, key)
        updates[f"feedforward.{key}"] = list(value) if isinstance(value, tuple) else float(value)
    changed = cfg.save(updates)
    say(f"{cfg.source_path} に保存しました（{len(changed)} 行を更新）:")
    for line in changed:
        say(f"  {line}")
    return ModelBuildResult(
        model_path=model_path, metrics=metrics, params=after, changed=changed,
        research_params=research_after,
    )


def _estimate_research_coast_curve(
    cfg: ResearchConfig,
    logs: list[DriveLog],
    reference: FeedforwardParams,
    target: FeedforwardParams,
) -> FeedforwardParams:
    """惰行減速カーブの低速端を細ビンで再同定する（段2.5。ProblemReport_20260916）。

    estimate_dynamics_params の直後・クリープ加速カーブ推定（_estimate_research_creep_curve）の
    前に呼ぶこと（クリープ加速カーブ・ペダルゲインの両方が free_accel_at 経由でこのカーブを
    基準に使うため順序が必須）。

    `reference` はサンプル抽出条件（不感帯・creep_speed_kmh）に使う基準で、**必ず 2-0 実測を
    持つ `before` を渡すこと**。estimate_dynamics_params が返す after の
    accel_deadband_pct/brake_deadband_pct は表示専用の推定値（MEASURED_KEYS で保存されない
    参考値）で、2-0 実測（例 8.53%/12.00%）よりかなり粗い（例 3.0%/3.0%）。ここを基準にすると
    「両ペダルが不感帯以下」判定が崩れ、待機位置の行が全部「踏んでいる」扱いになって惰行
    サンプルが激減する（実測: 995→361 点）。
    `target` は差し替え先（coast_decel_speeds_kmh/coast_decel_kmhs だけを上書きして返す。
    他のフィールドは target のまま保持する）。FeedforwardParams が持つので、置き換えるだけで
    自動保存（FF_PARAM_KEYS）に乗る。
    """
    lr = cfg.learning
    say(f"coast_curve.estimate_coast_decel_curve: 惰行カーブの低速端を細ビンで再同定"
        f"（低速 {lr.coast_curve_low_bin_kmh:g} km/h 幅・{lr.coast_curve_low_max_kmh:g} km/h まで、"
        f"最少 {lr.coast_curve_low_min_bin_samples} 点/ビン）…")
    curve = estimate_coast_decel_curve(
        logs, reference,
        low_bin_kmh=lr.coast_curve_low_bin_kmh,
        low_max_kmh=lr.coast_curve_low_max_kmh,
        low_min_bin_samples=lr.coast_curve_low_min_bin_samples,
    )
    if curve.identified:
        say(f"  速度 {_format_param(curve.speeds_kmh)}  減速 {_format_param(curve.decels_kmh)}"
            f"（{curve.samples} 点）")
        return replace(
            target, coast_decel_speeds_kmh=curve.speeds_kmh, coast_decel_kmhs=curve.decels_kmh
        )
    say(f"  同定できず（条件を満たす {curve.samples} 点。既存値のまま）")
    return target


def _estimate_research_creep_curve(
    cfg: ResearchConfig,
    logs: list[DriveLog],
    params: FeedforwardParams,
    before: ResearchFFParams,
) -> ResearchFFParams:
    """クリープ加速カーブを推定する（ProblemReport_20260916 課題#2）。

    estimate_dynamics_params の直後・低速ブレーキゲイン推定（_estimate_research_pedal_gains）の
    前に呼ぶこと（低速ブレーキゲインはこのカーブを基準に使うため順序が必須）。`params` は
    2026-09-18（段2.5）から `build_ff_model` の `reference`（before の実測不感帯・
    creep_speed_kmh を保ったまま、惰行カーブだけ今回同定したものに差し替えたもの）を渡す。
    `estimate_dynamics_params` の生の `after` を渡してはいけない
    （`_estimate_research_coast_curve` の docstring 参照。after の不感帯は表示専用の粗い
    推定値で、サンプル抽出条件が壊れる）。
    """
    lr = cfg.learning
    say(f"creep_curve.estimate_creep_accel_curve: クリープ加速カーブ推定"
        f"（ビン幅 {lr.creep_curve_bin_kmh:g} km/h、"
        f"最少 {lr.creep_curve_min_bin_samples} 点/ビン）…")
    curve = estimate_creep_accel_curve(
        logs, params,
        bin_kmh=lr.creep_curve_bin_kmh, min_bin_samples=lr.creep_curve_min_bin_samples,
    )
    if curve.identified:
        say(f"  速度 {_format_param(curve.speeds_kmh)}  加速度 {_format_param(curve.accel_kmhs)}"
            f"（{curve.samples} 点）")
        return replace(
            before, creep_accel_speeds_kmh=curve.speeds_kmh, creep_accel_kmhs=curve.accel_kmhs
        )
    say(f"  同定できず（条件を満たす {curve.samples} 点。既存値のまま）")
    return before


def _estimate_research_stop_brake_floor(
    cfg: ResearchConfig,
    logs: list[DriveLog],
    params: FeedforwardParams,
    before: ResearchFFParams,
) -> ResearchFFParams:
    """クリープ域ブレーキの下限を推定する（段4改訂。ProblemReport_20260916）。

    `_estimate_research_coast_curve` と同じ流儀・同じ理由: サンプル抽出条件（不感帯・
    creep_speed_kmh）は必ず 2-0 実測を保つ `params`（`build_ff_model` の `reference`）を
    渡すこと。`estimate_dynamics_params` の生の `after` を渡してはいけない
    （`_estimate_research_coast_curve` の docstring 参照。after の不感帯は表示専用の粗い推定値で
    「両ペダルが不感帯以下」判定が崩れる）。

    候補開度はパターン生成（`build_patterns` の `creep_brake_hold_offsets_pct` と同じ式
    `above(ff.brake_deadband_pct, lr.creep_brake_hold_offsets_pct)`）で組む。手順2 が実際に
    指令した開度そのものなので、`stop_brake_floor.py` の `phase` 列が使えない制約に対応する
    （モジュール docstring 参照）。開始車速は `reference.creep_speed_kmh`（= before のクリープ
    平衡）。
    """
    lr = cfg.learning
    candidate_openings_pct = tuple(
        round(params.brake_deadband_pct + offset, 2)
        for offset in lr.creep_brake_hold_offsets_pct
    )
    say(f"stop_brake_floor.estimate_stop_brake_floor: クリープ域ブレーキの下限を推定"
        f"（候補開度 {_format_param(candidate_openings_pct)}、"
        f"平衡 {params.creep_speed_kmh:.2f}±{lr.stop_brake_floor_start_tol_kmh:g} km/h）…")
    floor = estimate_stop_brake_floor(
        logs, params,
        candidate_openings_pct=candidate_openings_pct,
        start_speed_kmh=params.creep_speed_kmh,
        start_tol_kmh=lr.stop_brake_floor_start_tol_kmh,
        min_float_s=lr.stop_brake_floor_min_float_s,
        opening_tol_pct=lr.stop_brake_floor_opening_tol_pct,
    )
    if floor.identified:
        say(f"  停止 {_format_param(floor.stopped_pct)}  浮く {_format_param(floor.floated_pct)}"
            f"  → offset_pct={floor.offset_pct:.2f}")
        return replace(before, stop_brake_floor_offset_pct=floor.offset_pct)
    say(f"  同定できず（停止 {_format_param(floor.stopped_pct)}  "
        f"浮く {_format_param(floor.floated_pct)}。既存値のまま）")
    return before


def _estimate_research_pedal_gains(
    cfg: ResearchConfig,
    logs: list[DriveLog],
    before: FeedforwardParams,
    after: FeedforwardParams,
    research: ResearchFFParams,
) -> FeedforwardParams:
    """ペダルゲインを「不感帯 + learning.*_gain_min_offset_pct 以上」で推定し直す。

    ブレーキ側はクリープ域（速度 < creep_speed_kmh）も対象になる（`research` を基準に使う。
    pedal_gain.estimate_gain_curve の docstring 参照）。

    `before` は free_accel_at の基準（惰行カーブ・不感帯・creep_speed_kmh）に使う
    FeedforwardParams。本番 estimate_dynamics_params はゲイン推定の基準線を意図的に
    `current`（前回同定値）に固定している（ビンの埋まり方でゲインが揺れるのを避けるため）が、
    2026-09-18（段2.5）からは `current` の惰行カーブが実測とずれていることが分かっているため、
    `build_ff_model` は `reference`（before の実測不感帯・creep_speed_kmh を保ったまま、惰行
    カーブだけ今回同定したものに差し替えたもの）を渡す（`brake_gain_kmhs_per_pct` の 5.0km/h
    落ち込み 0.132 がこれで直るかを確認する）。`estimate_dynamics_params` の生の `after` を
    渡してはいけない（`_estimate_research_coast_curve` の docstring 参照。after の不感帯は
    表示専用の粗い推定値で、「不感帯以下＝ペダルオフ」判定とゲインの分母（開度−不感帯）が
    壊れる）。
    """
    lr = cfg.learning
    say(f"ペダルゲイン: 研究側の推定（開度 ≥ 不感帯 + アクセル {lr.accel_gain_min_offset_pct:g}% / "
        f"ブレーキ {lr.brake_gain_min_offset_pct:g}%、開度 1s 定常。"
        f"本番は +{PEDAL_GAIN_MIN_OPENING_PCT:g}%）")
    say(f"  本番条件の推定: 速度 {_format_param(after.pedal_gain_speeds_kmh)}  "
        f"アクセル {_format_param(after.accel_gain_kmhs_per_pct)}  "
        f"ブレーキ {_format_param(after.brake_gain_kmhs_per_pct)}")
    curves = {}
    for label, is_accel, offset in (
        ("アクセル", True, lr.accel_gain_min_offset_pct),
        ("ブレーキ", False, lr.brake_gain_min_offset_pct),
    ):
        curve = estimate_gain_curve(
            logs, before, research, is_accel=is_accel, min_offset_pct=offset,
            creep_bin_kmh=lr.creep_curve_bin_kmh,
            creep_min_bin_samples=lr.creep_curve_min_bin_samples,
        )
        if curve.identified:
            say(f"  {label}: 使用 {curve.samples} 点・速度帯 {len(curve.speeds_kmh)} 個 → "
                f"速度 {_format_param(curve.speeds_kmh)}  ゲイン {_format_param(curve.gains)}")
        else:
            say(f"  {label}: 同定できず（条件を満たす {curve.samples} 点。本番推定の値のまま）")
        curves[is_accel] = curve
    return apply_pedal_gains(after, accel=curves[True], brake=curves[False])


def _print_metrics(metrics: dict[str, dict[str, float]]) -> None:
    say("── 逆モデルの当てはまり（学習データ上。開度 [%] の誤差） ──")
    for side, label in (("accel", "アクセル"), ("brake", "ブレーキ")):
        m = metrics[side]
        r2 = f"{m['r2']:.3f}" if "r2" in m else "---"
        # below_deadband: 予測が不感帯未満だった割合（B3 の切り上げ前。A1 が効いたかの指標）
        ratio = m.get("below_deadband")
        below = "" if ratio is None else f"  不感帯未満の予測={100.0 * ratio:.0f}%"
        say(f"  {label}: MAE={m['mae']:.2f}  RMSE={m['rmse']:.2f}  R²={r2}  n={int(m['n'])}{below}")


def _format_param(value: object) -> str:
    if isinstance(value, tuple):
        return "[" + ", ".join(f"{v:.3g}" for v in value) + "]"
    if isinstance(value, float):
        return f"{value:.3f}"
    return str(value)


def _print_param_changes(
    before: FeedforwardParams,
    after: FeedforwardParams,
    research_before: ResearchFFParams,
    research_after: ResearchFFParams,
) -> None:
    """FF_PARAM_KEYS を順に表示する。RESEARCH_PARAM_KEYS は FeedforwardParams ではなく
    ResearchFFParams から読む（getattr 先を出し分ける）。"""
    say("── 物理定数（観測が足りない項目は据え置き。[実測] は 2-0 の値を採用し推定値は参考） ──")
    for key in FF_PARAM_KEYS:
        if key in RESEARCH_PARAM_KEYS:
            old_value, new_value = getattr(research_before, key), getattr(research_after, key)
        else:
            old_value, new_value = getattr(before, key), getattr(after, key)
        old, new = _format_param(old_value), _format_param(new_value)
        if key in MEASURED_KEYS:
            say(f"  [実測] {key:<24} {old}（推定は {new}）")
            continue
        mark = "更新" if old != new else "据置"  # 表示桁で比べる（0.500→0.500 を更新扱いしない）
        say(f"  [{mark}] {key:<24} {old} → {new}")
