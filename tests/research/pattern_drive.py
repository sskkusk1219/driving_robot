"""手順 2: 閉ループパターン走行 → 2次多項式 FF モデル作成。

ProblemReport_20260910 手順 2:
    2-0. ペダル探索（pedal_search.py）… 不感帯と停車保持開度を実測し、停車保持まで行う
    2-1. 本番環境で採用している 2次多項式モデル作成用のパターン走行
    2-2. 走行結果を基にモデルを作成し、パラメータを config_testVehicle.yaml に保存

引用元（パターン生成・学習は本番の関数をそのまま呼ぶ。走行ループは pattern_loop.py が
アルゴリズムを移植した自前実装 — 遵守事項「/src の本番コードを実行しないこと」に対応）:
    （パターン列は 2026-09-25 段4 から本番 LearningDriveManager を使わず、`build_patterns` が
     格子ステップ走行（grid_patterns.py）などで組む）
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
    - 2026-09-25 段4（ProblemReport_20260925）: パターン列を「車両に依らない測定パターン」に
      置き換えた（`build_patterns` 参照）。コーストダウン ×2 → 格子ステップ走行（WLTP の車速 ×
      加速度の格子を狙い、開度は較正から車両ごとに自動で決め、測定区間は開度固定）→ クリープ
      発進 → クリープ域ブレーキ保持。旧パターン（不感帯 + 固定 % の ACCEL_SWEEP・BRAKE_HOLD・
      トリム階段・定速階段・低開度階段など）と、それ前提の解析ツール・config キーは削除した。
    - ペダルゲインは不感帯 + learning.*_gain_min_offset_pct 以上のサンプルで推定する
      （pedal_gain.py。本番は +5% で、この車両のブレーキでは ≈0.38G になり定常サンプルが採れない）。
    - スタブ走行の結果は config_testVehicle.yaml に書き戻さない（実機の値を模擬値で壊さない）。
    - 2026-09-17 クリープ発進・クリープ域ブレーキ保持（段1。ProblemReport_20260916 課題#2）:
      パターン列の末尾に `CreepLaunchPattern` を足す（`build_patterns` docstring
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
    - 2026-09-27 手順2 の計測効率化（段7。ProblemReport_20260925 段7）: 実測（065502、約3035s）で
      格子ステップの「惰行ステップ」と「ステップ後にステーション中心へ戻る待ち」が時間の大半を
      占めていた（惰行の実測 a はコーストダウンと ±0.1 km/h/s 以内）。
      - 段7a: コーストダウン ×`COAST_DOWN_COUNT`（2→1）。格子ステップの惰行ステップは
        `GridPlanner` の `coast_fn`（コーストダウンの実測 a を渡す）で置き換え、測れない車速帯
        だけ従来の惰行ステップにフォールバックする（`grid_planner.py` 参照）。
      - 段7b: ステップの後にステーション中心（または助走の目標）へ戻る区間を、専用フェーズ
        `_Phase.GRID_RETURN`（固定開度で速く戻る）→ 近づいたら `_Phase.CRUISE_HOLD`（PI で
        微調整・落ち着き判定）に分けた（`pattern_loop.py` 参照）。
"""

from __future__ import annotations

import asyncio
import math
import time
from collections.abc import Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

import numpy as np

from tests.research.app_settings import SafetySettings
from tests.research.axis_monitor import AxisMonitor
from tests.research.coast_curve import estimate_coast_decel_curve
from tests.research.config import ConfigError, FeaturesSection, ResearchConfig
from tests.research.creep_curve import estimate_creep_accel_curve
from tests.research.drive_log import (
    SECTION_DECEL_TO_STOP,
    SECTION_PATTERN_DRIVE,
    DriveSample,
    SessionLog,
    read_drive_logs,
)
from tests.research.dynamics_estimation import (
    PEDAL_GAIN_MIN_OPENING_PCT,
    estimate_dynamics_params,
)
from tests.research.ff_candidate import train_inverse_model_effective
from tests.research.ff_model import deviation_gain, export_model_coefficients
from tests.research.ff_params import ResearchFFParams, research_ff_params
from tests.research.grid_patterns import (
    WltpStatsLike,
    build_grid_patterns,
    grid_settings_from_config,
)
from tests.research.grid_planner import StepResult, status_line
from tests.research.grid_settle import SettleRecord, settle_report
from tests.research.hardware import HW_REAL, DriveError, ResearchHardware
from tests.research.horizon_search import (
    HorizonSearchSettings,
    format_result_table,
    read_pattern_groups,
    search_ff_horizons,
)
from tests.research.learning_patterns import (
    COAST_DOWN_ACCEL_PCT,
    COAST_DOWN_COUNT,
    HOLD_DURATION_S,
)
from tests.research.model_gain import _load_pkl
from tests.research.pattern_loop import (
    CreepLaunchPattern,
    GridLaunchPattern,
    GridStationPattern,
    GridSweepPattern,
    PatternLoop,
    PatternLoopConfig,
)
from tests.research.pedal_gain import apply_pedal_gains, estimate_gain_curve
from tests.research.pedal_lag import (
    KINDS as PEDAL_LAG_KINDS,
)
from tests.research.pedal_lag import (
    estimate_pedal_lag,
    read_step_rows,
)
from tests.research.pedal_search import PedalSearchResult
from tests.research.research_types import (
    DriveLog,
    DriveLogData,
    DrivingMode,
    FeedforwardParams,
    LearningPattern,
    PatternKind,
    VehicleProfile,
)
from tests.research.sample_weight import WltpWeighting
from tests.research.stop_brake_floor import creep_brake_hold_openings, estimate_stop_brake_floor
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
    # 格子ステップ走行（段3）のステップごとの結果（グリッドのパターンが無ければ空）
    grid_results: list[StepResult] = field(default_factory=list)
    # 落ち着き待ちの記録（段4b。許容幅・待ち時間を実機データで決め直す材料。grid_settle）
    grid_settles: list[SettleRecord] = field(default_factory=list)


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
    if isinstance(pattern, GridStationPattern) and pattern.plan is not None:
        plan = pattern.plan
        band = f"{plan.speed_lo_kmh:g}〜{plan.speed_hi_kmh:g}"
        return (f"格子ステップ {plan.speed_kmh:g} km/h（{band}）"
                f"  減速 {len(plan.decel)} 列・加速 {len(plan.accel)} 列"
                f"（WLTP 最大 {plan.a_max_kmhs:+.1f} / 最小 {plan.a_min_kmhs:+.1f} km/h/s）")
    if isinstance(pattern, GridLaunchPattern) and pattern.plan is not None:
        plan = pattern.plan
        return (f"格子 発進・停車 0〜{plan.end_kmh:g} km/h"
                f"  発進 {len(plan.accel)} 列・停車 {len(plan.decel)} 列")
    if isinstance(pattern, GridSweepPattern):
        return ("通し掃引  網羅の穴を走行中に数え、強い加減速のセルを開度を切り替えながら"
                f"通り抜けて測る（各 最大 {pattern.max_passes} 回。穴が無ければ何もしない）")
    if pattern.kind is PatternKind.G_CALIB:
        return (f"G 校正  加速（G 比例・頭打ち {pattern.accel_opening:.1f}%）→ cap から"
                f" 0.2G 狙いのブレーキ減速で低速まで（各車速の上限開度の予測用）")
    if isinstance(pattern, CreepLaunchPattern):
        action = (
            f"ブレーキ保持 {pattern.brake_opening:.1f}%で停車" if pattern.hold_after else "停車復帰"
        )
        return (f"{text}  クリープ発進 → {pattern.target_kmh:g} km/h"
                f"（打切り{pattern.timeout_s:g}s）→ {action}")
    return text


def _print_patterns(patterns: Sequence[LearningPattern]) -> None:
    say("── パターン一覧（コーストダウン → G 校正 → 格子ステップ走行 → クリープ、"
        f"{len(patterns)} 本） ──")
    for i, pattern in enumerate(patterns, start=1):
        say(f"  {i:>2}. {_describe(pattern)}")


def build_patterns(
    cfg: ResearchConfig, profile: VehicleProfile, wltp_stats: WltpStatsLike
) -> list[LearningPattern]:
    """手順2 のパターン列（2026-09-25 段4。ProblemReport_20260925）。

    「車速 × 加速度」の格子を WLTP に合わせて狙い、開度は車両ごとに自動で決める（車両に依らない
    測定パターン）。構成:
      1. コーストダウン ×`COAST_DOWN_COUNT`: 手順2 終了時の緩減速と同じ G を目標に、アクセルを
         刻み踏みで上げて cap まで加速し惰行（開度は COAST_DOWN_ACCEL_PCT で頭打ち）。車速で
         終わるので車両非依存（惰行カーブ用）。2026-09-27 段7a: この実測が格子ステップの惰行
         ステップの代わりになる（`GridPlanner` の `coast_fn`。測れない車速帯だけ従来どおり
         惰行ステップで測る）
      2. G 校正（段6a）: G 比例加速で cap まで → 0.2G 狙いのブレーキ減速で低速まで。各車速の
         「開度と減速」から、上限 G に届く開度を予測する（格子ステップの開度の上限）
      3. 格子ステップ走行: ステーション（車速）ごとの `GridStationPattern` → 発進・停車セルの
         `GridLaunchPattern`（`grid_patterns.build_grid_patterns`。開度は走行中に決まる）。
         2026-09-27 段7b: ステップの後・強い狙いの助走前は、まず `_Phase.GRID_RETURN`
         （固定開度）で目標へ速く戻り、近づいたら `_Phase.CRUISE_HOLD`（PI）で微調整する
      4. クリープ発進 ×`learning.creep_launch_count`: 両ペダル解放で自走 → 通常の停車復帰
         （クリープカーブ用。手順3 と同じ暖機状態で測るため末尾側に置く）
      5. クリープ域ブレーキ保持: 開度 = 不感帯 + frac × (停車保持開度 − 不感帯)
         （`learning.creep_brake_hold_fracs`。停止ブレーキの下限用。停車保持開度は 2-0 の実測）

    `wltp_stats` は `wltp_grid.wltp_cell_stats` の結果（WLTP の車速 × 加速度の集計）。
    `pattern_drive` が `wltp_grid` を import すると循環するため、集計は呼び出し側で行って渡す。
    """
    ff = profile.feedforward_params
    lr = cfg.learning

    coast_accel = min(COAST_DOWN_ACCEL_PCT, profile.max_accel_opening)
    coast_downs = [
        LearningPattern(
            kind=PatternKind.COAST_DOWN,
            accel_opening=coast_accel,
            brake_opening=0.0,
            hold_duration_s=HOLD_DURATION_S,
        )
        for _ in range(COAST_DOWN_COUNT if coast_accel > 0.0 else 0)
    ]
    # G 校正（段6a・門①）: コーストダウンの直後、格子ステップの前に 1 本。開度の上限の予測を作る
    g_calib = [
        LearningPattern(
            kind=PatternKind.G_CALIB,
            accel_opening=coast_accel,
            brake_opening=0.0,
            hold_duration_s=HOLD_DURATION_S,
        )
    ] if coast_accel > 0.0 else []
    grid = build_grid_patterns(cfg, wltp_stats)
    # 通し掃引（段6c）: 格子ステップ・発進停車の後。網羅の穴は走行中に数えて掃引を作る
    sweeps = [
        GridSweepPattern(
            kind=PatternKind.GRID_SWEEP,
            accel_opening=coast_accel,
            brake_opening=0.0,
            hold_duration_s=HOLD_DURATION_S,
            wltp_seconds=np.asarray(wltp_stats.seconds),
            wltp_mean=np.asarray(wltp_stats.mean_accel),
            speed_edges=tuple(lr.grid_speed_edges_kmh),
            accel_edges=tuple(lr.grid_accel_edges_kmhs),
            min_s=lr.grid_target_min_s,
            data_max_s=lr.grid_hole_data_max_s,
            max_passes=lr.grid_sweep_max_passes,
            settings=grid_settings_from_config(cfg),
        )
    ] if coast_accel > 0.0 and lr.grid_sweep_max_passes > 0 else []
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
        for opening in creep_brake_hold_openings(
            ff.brake_deadband_pct, ff.stop_brake_opening_pct, lr.creep_brake_hold_fracs
        )
    ]
    return [*coast_downs, *g_calib, *grid, *sweeps, *creep_launches, *creep_brake_holds]


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
    wltp_stats: WltpStatsLike | None = None,
    loop_config: PatternLoopConfig | None = None,
) -> PatternDriveResult:
    """本番の学習運転パターンを PatternLoop（本番 LearningLoop 相当）で走らせ、
    緩減速で停車保持まで行う。

    前提: 2-0 のペダル探索が済み、停車保持開度（pedal.stop_brake_opening_pct）で停車している。
    `log` は走行前チェックから続く走行ログ（main が作って保存する）。無ければここで作り、
    終わりに（異常終了でも）保存する。
    `patterns` を渡さないときは `wltp_stats`（`wltp_grid.wltp_cell_stats` の結果）から
    `build_patterns` で作る（無ければ ConfigError）。`patterns` / `loop_config` はテストや
    動作確認で短縮するための差し込み口。
    """
    profile = pedal.apply_to_profile(build_vehicle_profile(cfg))
    if patterns is None:
        if wltp_stats is None:
            raise ConfigError("run_pattern_drive には patterns か wltp_stats のどちらかが要ります")
        patterns = build_patterns(cfg, profile, wltp_stats)
    if loop_config is None:
        loop_config = PatternLoopConfig(
            coast_timeout_s=cfg.learning.coast_timeout_s,
            creep_launch_settle_kmhs=cfg.learning.creep_launch_settle_kmhs,
            creep_launch_settle_s=cfg.learning.creep_launch_settle_s,
            creep_launch_min_speed_kmh=cfg.learning.creep_launch_min_speed_kmh,
            creep_launch_settle_min_s=cfg.learning.creep_launch_settle_min_s,
            # コーストダウンの加速は、手順2 終了時の緩減速と同じ G・刻み・待ちで踏み進める
            coast_accel_target_g=cfg.decel_stop.target_decel_g,
            coast_accel_press_margin_g=cfg.decel_stop.press_margin_g,
            coast_accel_slope_window_s=cfg.decel_stop.slope_window_s,
            coast_accel_approach_margin_pct=cfg.decel_stop.approach_margin_pct,
            # G 校正のブレーキ減速も同じ刻み方（decel_stop と同じ値）
            calib_release_above_g=cfg.decel_stop.release_above_g,
            calib_step_mm=cfg.decel_stop.step_mm,
            calib_dwell_s=cfg.decel_stop.dwell_s,
            g_cap_g=cfg.learning.g_cap_g,
            # コーストダウンの終了判定（段7c）に使う「落ち着いた」許容幅
            grid_settle_tol_kmh=cfg.learning.grid_settle_tol_kmh,
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
    say(f"コーストダウン加速: 踏む速さ = {loop_config.coast_accel_rate_gain:g} ×"
        f"（{loop_config.coast_accel_target_g:g}G − 今の G）%/s、"
        f"±{loop_config.coast_accel_press_margin_g:g}G は保持、"
        f"上限 = パターンの加速開度（頭打ち）")
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
        on_grid_result=lambda result: say(status_line(result)),
        on_sweep_log=say,
        on_calib_done=lambda lines: say(
            f"── G 校正の結果: 上限 {cfg.learning.g_cap_g:g}G に届くと予測される開度 ──\n"
            + "\n".join(lines)
        ),
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
        grid_planner = learning_loop.grid_planner
        report = settle_report(
            learning_loop.grid_settles,
            grid_planner.results if grid_planner is not None else [],
            tol_kmh=cfg.learning.grid_settle_tol_kmh, settle_s=cfg.learning.grid_settle_s,
            duration_s=duration,
        )
        if report:
            say(report)
        say(
            f"── 走行後の上限開度の予測（{cfg.learning.g_cap_g:g}G。実測が増えた分を反映）──\n"
            + "\n".join(learning_loop.glimit_table())
        )
        session.mark(SECTION_DECEL_TO_STOP, PHASE_APPROACH)
        session.start_sampler()
        stop = await decelerate_to_stop(hw, cfg, profile, log=session)
        say(f"停車保持: ブレーキ {stop.hold_pct:.2f}%")
        planner = learning_loop.grid_planner
        return PatternDriveResult(
            csv_path=session.csv_path, samples=monitor.samples, duration_s=duration, stop=stop,
            grid_results=list(planner.results) if planner is not None else [],
            grid_settles=list(learning_loop.grid_settles),
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
    wltp_mode: DrivingMode | None = None,
    sample_weight_enabled: bool | None = None,
) -> ModelBuildResult:
    """走行 CSV から 2次多項式 Ridge 逆モデルと物理定数を作り、実機走行なら YAML へ保存する。

    本番 training_service.train_and_apply と同じ順序（逆モデル → 物理定数）で、学習時点の
    プロファイル値を使う。逆モデルだけは A1（そのペダルが効いている行だけで学習）を入れた
    研究用の `train_inverse_model_effective` を呼ぶ。`pedal` があれば 2-0 の実測値を
    プロファイルへ反映してから学習する。サンプル不足は本番の LearningDataError をそのまま送出する。

    `write_config` 省略時（None）は従来どおり `hw_mode == HW_REAL` で判定する。明示的に指定すると
    実機モードでも保存を止められる（`tests.research.relearn` の `--dry-run` 用。段2.5）。

    段2（ProblemReport_20260925。学習サンプルの WLTP 重み付け）:
        `wltp_mode` を渡すと、その基準車速から作った WLTP 格子（`wltp_grid.wltp_cells`）で
        `WltpWeighting` を組み、`train_inverse_model_effective` に渡す。`sample_weight_enabled`
        省略時（None）は `cfg.learning.sample_weight_enabled` に従う（明示すれば一時的に
        上書きできる。`tests.research.relearn` の `--weight` 用）。有効なのに `wltp_mode` が
        無い場合は `ConfigError`（重みの計算に WLTP の基準車速が要るため）。
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

    lr = cfg.learning
    enabled = lr.sample_weight_enabled if sample_weight_enabled is None else sample_weight_enabled
    if enabled and wltp_mode is None:
        raise ConfigError(
            "learning.sample_weight_enabled が有効ですが wltp_mode が渡されていません"
            "（WLTP 重み付けには基準車速が必要です）"
        )

    say("train_inverse_model_effective: 2次多項式＋標準化＋Ridge（アクセル/ブレーキの 2 モデル）"
        "を、そのペダルが効いている行だけで学習 …")
    models_dir = Path(cfg.feedforward.model_path).parent
    # spec は WLTP 重み付けの regime 情報（h1）を取り出すためだけに使う（手順6 でホライズン
    # 自動選択が有効でも、v0・要求加速度 dv_h1/h1 の値はどの spec で取り出しても同じ）
    spec = cfg.features.to_feature_spec()

    ft = cfg.features
    if ft.horizon_search:
        say("ホライズン自動選択（features.horizon_search: true）: "
            "アクセル・ブレーキ別に交差検証 MAE が最も下がる先読みを貪欲法で選択 …")
        settings = HorizonSearchSettings(
            grid=ft.search_grid(),
            max_horizons=ft.search_max_horizons,
            min_improvement=ft.search_min_improvement,
            cv_splits=ft.search_cv_splits,
            regime_horizon_s=ft.h1_s,
            past_horizons_s=ft.past_horizons_s(),
            past_as_delta=ft.past_as_delta,
            include_v0_sq=ft.use_v0_sq,
            include_dv_regime_x_v0=ft.use_dv1_x_v0,
            gain_check_speeds_kmh=tuple(ft.search_gain_check_speeds_kmh),
            min_deviation_gain=ft.search_min_deviation_gain,
        )
        patterns = read_pattern_groups(csv_path)
        accel_result, brake_result = search_ff_horizons(
            logs, patterns, profile.feedforward_params.accel_deadband_pct,
            profile.feedforward_params.brake_deadband_pct, settings,
        )
        for result in (accel_result, brake_result):
            say(f"  {result.pedal}:")
            for line in format_result_table(result).splitlines():
                say(f"    {line}")
        accel_spec, brake_spec = accel_result.spec, brake_result.spec
        say(f"  選んだ先読み: アクセル {list(accel_spec.lookahead_horizons_s)} / "
            f"ブレーキ {list(brake_spec.lookahead_horizons_s)}"
            f"（停車保持・ブレーキ下限は h0_s={ft.h0_s}s 先を見続ける）")
    else:
        accel_spec = brake_spec = spec
        say(f"特徴量（config features）: {spec.feature_names()}")

    weighting: WltpWeighting | None = None
    if wltp_mode is not None:
        # wltp_grid は mode_drive を import しており、mode_drive は pattern_drive を import して
        # いる（_overcurrent_limit_ma・_release_pedals）。モジュール先頭で import すると
        # pattern_drive → wltp_grid → mode_drive → pattern_drive の循環になるため、ここで
        # ローカル import する。
        from tests.research.wltp_grid import edges_from_config, wltp_cells

        edges = edges_from_config(lr.grid_speed_edges_kmh, lr.grid_accel_edges_kmhs)
        wltp_s = wltp_cells(wltp_mode, edges, spec)
        weighting = WltpWeighting(
            wltp_s=wltp_s, speed_edges_kmh=edges.speed_kmh, accel_edges_kmhs=edges.accel_kmhs,
            w_min=lr.sample_weight_min, w_max=lr.sample_weight_max, enabled=enabled,
        )
        say(f"WLTP 重み付け: {'有効' if enabled else '無効'}"
            f"（w_min={lr.sample_weight_min:g}, w_max={lr.sample_weight_max:g}）")
    else:
        say("WLTP 重み付け: 無効（wltp_mode 未指定）")

    model_path, metrics = train_inverse_model_effective(
        logs, profile, output_dir=str(models_dir), accel_spec=accel_spec, brake_spec=brake_spec,
        stop_horizon_s=ft.h0_s, weighting=weighting,
    )
    say(f"モデル保存: {model_path}")
    _print_metrics(metrics)

    say("実質Kp（deviation_gain）の符号確認 …")
    _verify_final_model_gain(model_path, ft)

    coef_path = export_model_coefficients(model_path)
    say(f"係数（元の単位に換算・参照用）: {coef_path}")

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
    pedal_lag_s = _estimate_research_pedal_lag(cfg, csv_path)

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

    updates: dict[str, Any] = {
        "feedforward.model_path": model_path,
        # 参照用（走行は pkl を読む）。ホライズン・特徴量・係数の場所を config から見えるようにする
        "feedforward.model_accel_horizons_s": list(accel_spec.lookahead_horizons_s),
        "feedforward.model_brake_horizons_s": list(brake_spec.lookahead_horizons_s),
        "feedforward.model_accel_features": accel_spec.feature_names(),
        "feedforward.model_brake_features": brake_spec.feature_names(),
        "feedforward.model_coef_path": str(coef_path),
    }
    for key in FF_PARAM_KEYS:
        if key in MEASURED_KEYS:
            continue
        value = getattr(research_after, key) if key in RESEARCH_PARAM_KEYS else getattr(after, key)
        updates[f"feedforward.{key}"] = list(value) if isinstance(value, tuple) else float(value)
    # L（ペダル選択窓の中心）。FF_PARAM_KEYS とは別扱いで、測れたときだけ書く
    if pedal_lag_s is not None:
        updates["feedforward.pedal_select_center_s"] = pedal_lag_s
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

    候補開度はパターン生成（`build_patterns`）と同じ関数 `creep_brake_hold_openings`
    （不感帯 + frac × (停車保持開度 − 不感帯)）で組む。手順2 が実際に
    指令した開度そのものなので、`stop_brake_floor.py` の `phase` 列が使えない制約に対応する
    （モジュール docstring 参照）。開始車速は `reference.creep_speed_kmh`（= before のクリープ
    平衡）。
    """
    lr = cfg.learning
    candidate_openings_pct = creep_brake_hold_openings(
        params.brake_deadband_pct, params.stop_brake_opening_pct, lr.creep_brake_hold_fracs
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


def _estimate_research_pedal_lag(cfg: ResearchConfig, csv_path: Path) -> float | None:
    """ペダル指令→加速度の遅れ L を HOLD_STEP から測って表示する（ProblemReport_20260929 段2）。

    測り方は `pedal_lag` のモジュール docstring 参照。戻り値は 0.01s に丸めた L
    （build_ff_model が write_config のときだけ `feedforward.pedal_select_center_s` に書く）。
    測れなかった（HOLD_STEP 無し・件数不足・範囲外）ときは None で、config の値は据え置き。
    """
    before = cfg.feedforward.pedal_select_center_s
    say("ペダル遅れ L: 格子ステップの HOLD_STEP から測定（指令 50% 点 → 加速度 50% 点の中央値） …")
    result = estimate_pedal_lag(read_step_rows(csv_path))
    if result.lag_s is None:
        say(f"  警告: L を測れませんでした（{result.reason}）。pedal_select_center_s は "
            f"{before:g}s のまま据え置きます")
        return None
    lag = round(result.lag_s, 2)
    iqr = f"（四分位 {result.iqr_s[0]:.2f}〜{result.iqr_s[1]:.2f}s）" if result.iqr_s else ""
    say(f"  pedal_select_center_s: {before:g}s → {lag:g}s  採用 {result.n_used}/"
        f"{result.n_steps_total} ステップ{iqr}")
    kinds = "  ".join(
        f"{k} {result.by_kind[k][0]}件 {result.by_kind[k][1]:.2f}s"
        for k in PEDAL_LAG_KINDS if k in result.by_kind
    )
    say(f"  種類別（参考。L は全部の中央値）: {kinds}")
    return lag


def _print_metrics(metrics: dict[str, dict[str, float]]) -> None:
    say("── 逆モデルの当てはまり（学習データ上。開度 [%] の誤差） ──")
    for side, label in (("accel", "アクセル"), ("brake", "ブレーキ")):
        m = metrics[side]
        r2 = f"{m['r2']:.3f}" if "r2" in m else "---"
        # below_deadband: 予測が不感帯未満だった割合（B3 の切り上げ前。A1 が効いたかの指標）
        ratio = m.get("below_deadband")
        below = "" if ratio is None else f"  不感帯未満の予測={100.0 * ratio:.0f}%"
        # mae_wltp: 段2（ProblemReport_20260925）。WLTP の分布で重み付けした MAE
        # （重みなし／ありのモデルを同じ物差しで比べるため。weighting 指定時のみ入る）
        mae_wltp = m.get("mae_wltp")
        wltp_text = "" if mae_wltp is None else f"  MAE_WLTP={mae_wltp:.2f}"
        say(f"  {label}: MAE={m['mae']:.2f}  RMSE={m['rmse']:.2f}  R²={r2}  n={int(m['n'])}"
            f"{below}{wltp_text}")
        if "weight_mean" in m:
            say(
                f"    重み: min={m['weight_min']:.2f} max={m['weight_max']:.2f} "
                f"mean={m['weight_mean']:.2f} "
                f"下限張付={100.0 * m['weight_at_min_ratio']:.0f}% "
                f"上限張付={100.0 * m['weight_at_max_ratio']:.0f}%"
            )


def _verify_final_model_gain(model_path: str, ft: FeaturesSection) -> None:
    """保存直後の pkl の両ペダルで実質Kp（deviation_gain）の符号を確認する。

    案a（ProblemReport_20260921 手順6 段2。2026-09-28）: `horizon_search.search_pedal` の
    候補選定で除外していても、WLTP 重み付けなど探索とは学習行・重みが違う条件で最終学習した
    結果は別物になりうる。`horizon_search: false`（固定ホライズン）のときも含め、保存した
    pkl そのものを最後にもう一度確かめる（実機破綻の再発防止の最後の網）。

    速度は `search_gain_check_speeds_kmh`（空なら確認しない）。推論時に実際にクリップされる
    上限（pkl の `speed_clip_max`）を超える速度は確認しない。

    Raises:
        ConfigError: いずれかのペダル・速度で実質Kpが `search_min_deviation_gain` を下回る場合
    """
    check_speeds = ft.search_gain_check_speeds_kmh
    if not check_speeds:
        return
    path = Path(model_path)
    for side in ("accel", "brake"):
        model, spec, meta = _load_pkl(path, side)
        clip = meta.get("speed_clip_max")
        speeds = [v for v in check_speeds if clip is None or v <= clip]
        gains = {v: deviation_gain(model, spec, v, pedal=side) for v in speeds}
        bad = {v: g for v, g in gains.items() if g < ft.search_min_deviation_gain}
        if bad:
            detail = ", ".join(f"{v:g}km/h={g:.2f}" for v, g in sorted(bad.items()))
            raise ConfigError(
                f"{side} モデル（{model_path}）の実質Kp（deviation_gain）が"
                f"下限 {ft.search_min_deviation_gain:g} を割っています（{detail}）。"
                "実車速のずれに逆向きに反応する pkl（ずれが自分で広がる）のため使用を止めます"
                "（ProblemReport_20260921 手順6 段2 の再発防止）。"
            )


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
