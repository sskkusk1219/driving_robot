"""手順 2: 閉ループパターン走行 → 2次多項式 FF モデル作成。

ProblemReport_20260910 手順 2:
    2-0. ペダル探索（pedal_search.py）… クリープ車速と停車保持開度を実測し、停車保持まで行う
         （2026-10-08 段2l-6: 不感帯の探索はやめた。待機位置・u の原点は手で測った min）
    2-1. 本番環境で採用している 2次多項式モデル作成用のパターン走行
    2-2. 走行結果を基にモデルを作成し、パラメータを config_testVehicle.yaml に保存

引用元（パターン生成・学習は本番の関数をそのまま呼ぶ。走行ループは pattern_loop.py が
アルゴリズムを移植した自前実装 — 遵守事項「/src の本番コードを実行しないこと」に対応）:
    （パターン列は 2026-09-25 段4 から本番 LearningDriveManager を使わず、`build_patterns` が
     組む。2026-10-06 段2k 段C から G ランプ走行（ramp_planner.py の段）で組む）
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
    - 2026-10-06 段2k 段C（ProblemReport_20261001）: パターン列を「コーストダウン ×1 → G ランプ
      （ramp_planner の段の実行順）→ クリープ域ブレーキ保持」に切り替えた（下の段4 以降の格子
      ステップ・G 校正・通し掃引の記述は旧列の経緯。コードは段E で削除済み）。
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
        コーストダウンの実測 a で置き換えた。
      - 段7b: ステップの後にステーション中心へ戻る区間を専用フェーズ（GRID_RETURN）に分けた。
      （どちらも格子ステップ走行ごと段2k 段E で削除。上の段2k 段C の記述が今の列）
"""

from __future__ import annotations

import asyncio
import copy
import math
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

import numpy as np

from tests.research import pedal_lag
from tests.research.app_settings import SafetySettings
from tests.research.axis_monitor import AxisMonitor
from tests.research.coast_curve import estimate_coast_decel_curve
from tests.research.config import ConfigError, FeaturesSection, ResearchConfig, load_config
from tests.research.creep_curve import estimate_creep_accel_curve
from tests.research.drive_log import (
    SECTION_DECEL_TO_STOP,
    SECTION_PATTERN_DRIVE,
    SECTION_PEDAL_SEARCH,
    SECTION_PRE_DRIVE_CHECK,
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
    RAMP_TI_RATIO,
    CreepLaunchPattern,
    GRampPattern,
    PatternLoop,
    PatternLoopConfig,
    RampResult,
    RampStatus,
)
from tests.research.pedal_gain import apply_pedal_gains, estimate_gain_curve
from tests.research.pedal_lag import PedalLagResult
from tests.research.pedal_search import PedalSearchResult
from tests.research.ramp_planner import CRUISE_MIN_MARGIN_KMH, estimate_s
from tests.research.research_types import (
    DriveLog,
    DriveLogData,
    DrivingMode,
    FeedforwardParams,
    LearningPattern,
    PatternKind,
    VehicleProfile,
)
from tests.research.sample_weight import (
    SAMPLE_WEIGHT_ENABLED,
    SAMPLE_WEIGHT_MAX,
    SAMPLE_WEIGHT_MIN,
    WltpWeighting,
)
from tests.research.stop_brake_floor import (
    CREEP_BRAKE_HOLD_FRACS,
    creep_brake_hold_openings,
    estimate_stop_brake_floor,
)
from tests.research.stop_decel import PHASE_APPROACH, StopDecelResult, decelerate_to_stop
from tests.research.term import drive_status_line, say
from tests.research.vehicle import build_vehicle_profile

__all__ = ["DriveError"]  # main・テストが pattern_drive.DriveError として参照する

# C8（順モデル）の学習の並列数（fwd_final.build_final_table の workers。既定 2）。学習時間の枠に
# 合わせて変える（build_ff_model の forward_workers でも上書きできる）。乱数は省略時
# fwd_final.DEFAULT_SEEDS
FORWARD_WORKERS = 2

# 2026-10-13 段2o-2（ユーザー決定）: 手順2 の C8 は relu（2 層 50 個・停車まぎわ除外あり・追加学習
# なし）。fwd_final の CLI の既定（1-1 絶対誤差: logistic 30×30）は変えず、`_build_forward_model` が
# `fwd_final.final_config` へ渡す引数だけ変える（段2n の実機 051308 の設定）
FORWARD_HIDDEN: tuple[int, ...] = (50, 50)
FORWARD_ACTIVATION = "relu"
FORWARD_DROP_STOP_APPROACH = True

# 2026-10-14 段2o-4（ユーザー決定）: C8 の表の u の刻み [%]。根拠: 低車速のブレーキ側の崖の幅が
# 約 1.5%（手順2 `193155` の実測）で、0.5% 刻みなら 3 点入る。C8 の表だけ細かくし、C7 の表は
# 2% のまま保存する（学習は変えず、表の格子だけ fwd_map.refine_u_grid で細かくする）
FORWARD_TABLE_U_STEP_PCT = 0.5

# 2026-10-14 段2o-4: 発進側の表（fwd_map.attach_launch）を作るときの加速度の窓 [s]。仮値: 発進は
# 約 1 秒で終わり、本体の 1.0 秒窓ではなまる。段2o-4 の確認で 1.0 と比べて決める
LAUNCH_ACCEL_WINDOW_S = 0.5

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
    "pedal_gain_speeds_kmh",
    "accel_gain_kmhs_per_pct",
    "brake_gain_kmhs_per_pct",
)
# 2-0 のペダル探索で実測する項目。推定値は表示のみで書き戻さない。creep_speed_kmh は
# 2026-09-21 追加: estimate_dynamics_params の中央値は手順2末尾のクリープ発進（打ち切り時点の
# 上昇途中サンプルが69%）混じりで平衡より 0.24 km/h 低く出るため（実機確認）、2-0 の
# wait_creep_stable 実測（プラトーまで待つ）を正とする。
# 2026-10-08 段2l-6: 不感帯（accel/brake_deadband_pct）は FF_PARAM_KEYS からも外した
# （u の原点は yaml の vehicle.min_*_opening_pct。estimate_dynamics_params の不感帯推定値は
# 表示にも yaml にも出さない）
MEASURED_KEYS: frozenset[str] = frozenset({"stop_brake_opening_pct", "creep_speed_kmh"})
# FeedforwardParams ではなく ResearchFFParams が持つ項目（ProblemReport_20260916 課題#2・段4。
# coast_band_kmhs は人が決める値なので自動保存の対象外＝ここにも FF_PARAM_KEYS にも入れない。
# 停止ブレーキの下限・ブレーキトリムの設定キーは 2026-10-09 に yaml から外した）
RESEARCH_PARAM_KEYS: frozenset[str] = frozenset(
    {
        "creep_accel_speeds_kmh",
        "creep_accel_kmhs",
    }
)


@dataclass
class PatternDriveResult:
    csv_path: Path  # 走行ログの CSV（SessionLog.close で保存。学習はパターン走行の行だけ）
    samples: list[DriveSample]  # パターン走行の行
    duration_s: float  # パターン走行の所要時間（緩減速を含まない）
    stop: StopDecelResult | None = None
    # G ランプ走行（段2k）の段ごとの結果（G ランプのパターンが無ければ空）
    ramp_results: list[RampResult] = field(default_factory=list)


@dataclass
class ModelBuildResult:
    model_path: str
    metrics: dict[str, dict[str, float]]
    params: FeedforwardParams
    changed: list[str]
    # ResearchFFParams（クリープ加速カーブ等。FeedforwardParams には無い研究段階のみのパラメータ）
    research_params: ResearchFFParams = field(default_factory=ResearchFFParams)
    # 手順2 の学習結果のチェック（step2_check.CheckItem の列。C7/C8 のとき）
    checks: list[Any] = field(default_factory=list)


def _comm_counts(hw: ResearchHardware) -> dict[str, tuple[dict[str, int], dict[str, int]]]:
    """軸ごとの（op 名ごとの再送した通信の回数, 実行回数）の写し。スタブは空。"""
    return {
        label: (dict(getattr(axis, "retry_counts", {})), dict(getattr(axis, "op_counts", {})))
        for label, axis in (("アクセル", hw.accel), ("ブレーキ", hw.brake))
    }


def _retry_summary(
    start: dict[str, tuple[dict[str, int], dict[str, int]]],
    end: dict[str, tuple[dict[str, int], dict[str, int]]],
) -> str:
    """走行中（開始〜終了の差）の通信の再送を 1 行にする。再送 0 回の op も出す。"""
    parts: list[str] = []
    for label, (retry_end, op_end) in end.items():
        retry_start, op_start = start.get(label, ({}, {}))
        items = [
            f"{op} {retry_end.get(op, 0) - retry_start.get(op, 0)}/"
            f"{op_end[op] - op_start.get(op, 0)} 回"
            for op in sorted(op_end)
            if op_end[op] - op_start.get(op, 0) > 0
        ]
        parts.append(f"{label} " + ("・".join(items) if items else "通信なし"))
    return "通信の再送（再送した回数/通信した回数）: " + " ／ ".join(parts)


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
        # G ランプ中の基準車速・Kp・Ki を読む口（PatternLoop.ramp_status。ループを作った後に渡す）
        self.ramp_status: Callable[[], RampStatus | None] = lambda: None

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
        # G ランプ中だけ基準（data.ref_speed_kmh ＝ v_ref）とペダル別の Kp・Ki がある
        rs = self.ramp_status()
        sample = self._log.record(
            data, section=SECTION_PATTERN_DRIVE, phase=phase, pattern=f"{pattern_index + 1}:{kind}",
            governor_active=governor_active, alarm_accel=alarm_accel, alarm_brake=alarm_brake,
            monitor_accel=monitor_accel, monitor_brake=monitor_brake, cycle_ms=cycle_ms,
            accel_kp=None if rs is None else rs.accel_kp,
            accel_ki=None if rs is None else rs.accel_ki,
            brake_kp=None if rs is None else rs.brake_kp,
            brake_ki=None if rs is None else rs.brake_ki,
        )
        self.samples.append(sample)
        elapsed = sample.elapsed_s

        if pattern_index != self._last_pattern and pattern_index < total:
            self._last_pattern = pattern_index
            lr = self._cfg.learning
            text = _describe(
                self._patterns[pattern_index],
                hold_s=lr.ramp_hold_time_s, turn_s=lr.ramp_turn_time_s,
            )
            say(f"── パターン {pattern_index + 1}/{total}: {text} ──")
        if elapsed - self._last_print >= self._cfg.output.print_interval_s:
            self._last_print = elapsed
            # それ以外のパターンは基準・偏差は ---、ゲインは 0。Kd は常に 0
            say(drive_status_line(
                t_s=elapsed,
                ref_kmh=data.ref_speed_kmh,
                actual_kmh=data.actual_speed_kmh,
                accel_kp=0.0 if rs is None else rs.accel_kp,
                accel_ki=0.0 if rs is None else rs.accel_ki,
                accel_kd=0.0,
                brake_kp=0.0 if rs is None else rs.brake_kp,
                brake_ki=0.0 if rs is None else rs.brake_ki,
                brake_kd=0.0,
                accel_pct=data.accel_opening,
                brake_pct=data.brake_opening,
                extra=f"{pattern_index + 1}/{total} {kind} {phase}"
                      + ("  Gガバナー作動" if governor_active else ""),
            ))


def _describe(pattern: LearningPattern, *, hold_s: float = 0.0, turn_s: float = 0.0) -> str:
    text = (f"{pattern.kind.name}  アクセル {pattern.accel_opening:.1f}%  "
            f"ブレーキ {pattern.brake_opening:.1f}%")
    if isinstance(pattern, GRampPattern):
        est = estimate_s(pattern.target_g, pattern.top_kmh, hold_s=hold_s, turn_s=turn_s)
        if pattern.lag_step:  # 2026-10-13 段2o-2: 遅れの測定の段
            return (
                f"遅れの測定  狙い {pattern.target_g:g}G で加速 → {pattern.top_kmh:g} km/h で"
                "定速になるまで保持 → 開度を固定でステップ（アクセル踏み増し・戻し・離す・"
                f"ブレーキ踏み増し・戻し）→ {pattern.target_g:g}G で減速"
            )
        extra = ""
        if pattern.stations_kmh:  # 2026-10-13 段2o-2
            ways = "上りだけ" if pattern.stations_up_only else "上り下りとも"  # 2026-10-14 段2o-5
            extra = (f"。途中で {', '.join(f'{v:g}' for v in pattern.stations_kmh)} km/h を"
                     f"{ways}定速保持")
        if pattern.release_stations:  # 2026-10-14 段2o-5
            extra += (f"。下りの {', '.join(f'{v:g}' for v in pattern.release_stations)} km/h で"
                      "ブレーキを山から離す戻し")
        return (f"G ランプ  狙い {pattern.target_g:g}G で加速 → 上端 {pattern.top_kmh:g} km/h で"
                f" {hold_s:g} 秒保持 → {pattern.target_g:g}G で減速"
                f"（見込み {est:.0f}s。"
                "開度は基準車速との偏差と、走行中に貯めた表・感度で決める" + extra + "）")
    if isinstance(pattern, CreepLaunchPattern):
        action = (
            f"ブレーキ保持 {pattern.brake_opening:.1f}%で停車" if pattern.hold_after else "停車復帰"
        )
        return (f"{text}  クリープ発進 → {pattern.target_kmh:g} km/h"
                f"（打切り{pattern.timeout_s:g}s）→ {action}")
    return text


def _print_patterns(
    patterns: Sequence[LearningPattern], *, hold_s: float = 0.0, turn_s: float = 0.0
) -> None:
    say(f"── パターン一覧（コーストダウン → G ランプ、{len(patterns)} 本） ──")
    for i, pattern in enumerate(patterns, start=1):
        say(f"  {i:>2}. {_describe(pattern, hold_s=hold_s, turn_s=turn_s)}")


def ramp_order_of(
    patterns: Sequence[LearningPattern],
) -> list[tuple[float, float, tuple[float, ...], bool]]:
    """パターン列の G ランプの (狙いの G, 上端の車速, 保持する車速, 遅れのステップか) を取り出す。

    2026-10-13 段2o-2: 保持する車速と lag_step を足した（結果の表と `run_pattern_drive` 用）。
    """
    return [
        (p.target_g, p.top_kmh, p.stations_kmh, p.lag_step)
        for p in patterns
        if isinstance(p, GRampPattern)
    ]


def _fmt_g(value: float | None) -> str:
    return "  -  " if value is None else f"{value:.3f}"


def _fmt_pct(value: float | None) -> str:
    return "  -  " if value is None else f"{value:.2f}%"


def result_table(
    order: Sequence[tuple[Any, ...]], results: Sequence[RampResult], total_s: float,
    *, hold_s: float = 0.0, turn_s: float = 0.0,
) -> str:
    """G ランプの段ごとの結果の表。results は走った順（order と同じ順で先頭から対応）。

    2026-10-10 段2l-10: 「保持秒」（最高速での保持。切り替え +G → 0 を含む）の列を足した。
    2026-10-14 段2o-5: 合計秒に戻し 秒を足し、「戻し 入った/飛ばした」の列を足した。
    合計秒 ＝ 加速秒 ＋ 保持秒 ＋ 減速秒 ＋ 途中保持 秒（2026-10-13 段2o-2。途中の定速保持の合計と、
    合計が ramp_cruise_hold_s に達した回数 / 入った回数を足した。上端に L が付く段は遅れの測定の
    段で、保持秒に HOLD_STEP の 12.5 秒前後を含む）。見込み秒には保持と切り替えを
    足す（`hold_s`・`turn_s`）。
    """
    head = (
        "段 | 狙いG | 上端km/h | 見込み秒 | 加速秒 | 保持秒 | 減速秒 | 合計秒 | "
        "加速G中央値 | 減速G中央値 | 最高km/h | 途中保持 秒 | 途中保持 達成/入った | "
        "戻し 秒 | 戻し 入った/飛ばした | 効き始め アクセル | 効き始め ブレーキ"
    )
    lines = [head, "-" * len(head)]
    for i, item in enumerate(order):
        g, top = item[0], item[1]
        lag = len(item) > 3 and bool(item[3])  # 遅れの測定の段（上端に L を付ける）
        top_text = f"{top:.0f}L" if lag else f"{top:.0f}"
        est = estimate_s(g, top, hold_s=hold_s, turn_s=turn_s)
        r = results[i] if i < len(results) else None
        if r is None:
            lines.append(f"{i + 1:>2} | {g:.3f} | {top_text:>6} | {est:>7.0f} | （走っていない）")
            continue
        lines.append(
            f"{i + 1:>2} | {g:.3f} | {top_text:>6} | {est:>7.0f} | {r.up_s:>6.1f} "
            f"| {r.hold_s:>6.1f} "
            f"| {r.down_s:>6.1f} | {r.up_s + r.hold_s + r.down_s + r.cruise_s + r.release_s:>6.1f} "
            f"| {_fmt_g(r.accel_g_median):>11} "
            f"| {_fmt_g(r.decel_g_median):>11} | {r.max_speed_kmh:>7.1f}"
            f" | {r.cruise_s:>6.1f} | {r.cruise_done}/{r.cruise_n}"
            f" | {r.release_s:>6.1f} | {r.release_n}/{r.release_skipped}"
            f" | {_fmt_pct(r.accel_onset_pct):>8} | {_fmt_pct(r.brake_onset_pct):>8}"
        )
    lines.append(f"全体の所要秒数: {total_s:.0f} s（{total_s / 60:.1f} 分）")
    return "\n".join(lines)


POLY_CANDIDATES: tuple[str, ...] = ("C1", "C2", "C3", "C4", "C5")


def require_forward_candidate(cfg: ResearchConfig) -> None:
    """手順2 の対象は C7・C8（順モデル）だけ。C1〜C5（多項式）なら ConfigError。

    2026-10-08 段2l-6（ProblemReport_20261001 ユーザー決定 #2）: C1〜C5 は今後使わない。
    走る前（ハードを動かす前）に呼ぶ。C1〜C5 のコード（`_build_poly_model`・ff_candidate.py）は
    実機確認の後の別の段で扱う。
    """
    candidate = cfg.feedforward.candidate
    if candidate in POLY_CANDIDATES:
        raise ConfigError(
            f"feedforward.candidate = {candidate} は手順 2 の対象外です"
            "（C1〜C5 の多項式モデルは今後使いません）。yaml の feedforward.candidate と "
            "model_path を C7 または C8 に変えてください"
        )


def build_patterns(
    cfg: ResearchConfig, profile: VehicleProfile, ramp_order: Sequence[tuple[Any, ...]],
    *, with_lag: bool = False,
) -> list[LearningPattern]:
    """手順2 のパターン列（2026-10-06 段2k 段C。ProblemReport_20261001）。

    格子ステップをやめ、「狙いの G で加速 → 同じ G で減速」を G の段ごとに繰り返す。構成:
      1. コーストダウン ×`COAST_DOWN_COUNT`: 手順2 終了時の緩減速と同じ G を目標に、アクセルを
         刻み踏みで上げて cap まで加速し惰行（開度は COAST_DOWN_ACCEL_PCT で頭打ち）。車速で
         終わるので車両非依存（惰行カーブ用）
      2. G ランプ: `ramp_order`（`RampPlan.execution_order()` の (狙いの G, 上端の車速)。
         弱い G から順、2 周目があれば後ろ）の 1 要素につき `GRampPattern` 1 本。開度は走行中に
         実測の G を見て決める。2026-10-13 段2o-2: 要素は (g, 上端) または (g, 上端, 保持する車速)。
         3 つ目は途中で定速を保持する車速（`GRampPattern.stations_kmh`。省略なら空）
         2026-10-14 段2o-4: 4 つ目（あれば）は保持を上りだけにするか
         （`stations_up_only`。省略は False）
         2026-10-14 段2o-5: 5 つ目（あれば）は下りで山からブレーキを離す戻しの車速
         （`release_stations`。省略は空）

    2026-10-13 段2o-2: `with_lag` が True のとき（手順2 の本番）、G ランプの後ろに遅れの測定の段を
      足す: `learning.ramp_lag_speeds_kmh` の各車速を上端とする、狙いの G が
      `ramp_lag_approach_g` の `GRampPattern`（lag_step=True）。保持の後に開度を固定でステップ。
      既定 False（既存のテストと短縮用の差し込みは今までどおりの列になる）。

    2026-10-08 段2l-6: クリープ域ブレーキ保持（CREEP_SETTLE の 8 本）とクリープ発進を列から外した
    （G ランプを 0 km/h まで測るため）。`CreepLaunchPattern`・stop_brake_floor.py・関連キーの
    コード削除は実機確認の後の別の段。

    G 校正・格子ステップ・発進停車セル・通し掃引はパターン列に入れない（段E でコードごと削除）。
    """
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
    ramps: list[LearningPattern] = [
        GRampPattern(
            kind=PatternKind.G_RAMP, accel_opening=0.0, brake_opening=0.0, hold_duration_s=0.0,
            target_g=item[0], top_kmh=item[1],
            stations_kmh=tuple(item[2]) if len(item) > 2 else (),
            stations_up_only=bool(item[3]) if len(item) > 3 else False,
            release_stations=tuple(item[4]) if len(item) > 4 else (),
        )
        for item in ramp_order
    ]
    if with_lag and ramps:
        lr = cfg.learning
        ramps += [
            GRampPattern(
                kind=PatternKind.G_RAMP, accel_opening=0.0, brake_opening=0.0,
                hold_duration_s=0.0, target_g=lr.ramp_lag_approach_g, top_kmh=float(v),
                lag_step=True,
            )
            for v in lr.ramp_lag_speeds_kmh
        ]
    return [*coast_downs, *ramps]


def loop_config_from_config(
    cfg: ResearchConfig, *, stop_confirm_pct: float | None = None
) -> PatternLoopConfig:
    """config から PatternLoopConfig を組む（`run_pattern_drive` と `ramp_stub_run` で共有）。

    `stop_confirm_pct` は 2-0 で測った止まった開度（G ランプの発進の開始位置。None なら停車保持
    開度から始める）。
    """
    lr = cfg.learning
    return PatternLoopConfig(
        coast_timeout_s=lr.coast_timeout_s,
        creep_launch_settle_kmhs=lr.creep_launch_settle_kmhs,
        creep_launch_settle_s=lr.creep_launch_settle_s,
        # コーストダウンの加速は、手順2 終了時の緩減速と同じ G・刻み・待ちで踏み進める
        coast_accel_target_g=cfg.decel_stop.target_decel_g,
        coast_accel_press_margin_g=cfg.decel_stop.press_margin_g,
        coast_accel_slope_window_s=cfg.decel_stop.slope_window_s,
        coast_accel_approach_margin_pct=cfg.decel_stop.approach_margin_pct,
        # 高い車速からの停車復帰の G 比例ブレーキも同じ刻み方（decel_stop と同じ値）
        calib_release_above_g=cfg.decel_stop.release_above_g,
        calib_step_mm=cfg.decel_stop.step_mm,
        calib_dwell_s=cfg.decel_stop.dwell_s,
        # コーストダウンの終了判定（段7c）に使う「落ち着いた」許容幅
        grid_settle_tol_kmh=lr.grid_settle_tol_kmh,
        # G ランプ走行（段2k 段B）の制御と、ガバナーの踏み増しを止める割合
        ramp_settle_time_s=lr.ramp_settle_time_s,
        ramp_launch_time_s=lr.ramp_launch_time_s,
        ramp_turn_time_s=lr.ramp_turn_time_s,
        ramp_stop_time_s=lr.ramp_stop_time_s,
        ramp_edge_g=lr.ramp_edge_g,
        # 2026-10-18 段2o-10: 発進のゆっくり離し（① 速く → ② ゆっくり。動き出す開度を測る）
        ramp_launch_fast_rate_pctps=lr.ramp_launch_fast_rate_pctps,
        ramp_launch_release_margin_pct=lr.ramp_launch_release_margin_pct,
        ramp_launch_slow_rates_pctps=tuple(lr.ramp_launch_slow_rates_pctps),
        ramp_launch_g_rate_gps=lr.ramp_launch_g_rate_gps,
        ramp_launch_blend_min_s=lr.ramp_launch_blend_min_s,
        ramp_one_sided=lr.ramp_one_sided,
        ramp_g_levels=tuple(lr.ramp_g_levels),
        ramp_hold_time_s=lr.ramp_hold_time_s,
        ramp_dev_deadband_kmh=lr.ramp_dev_deadband_kmh,
        ramp_actuator_lag_tol_pct=lr.ramp_actuator_lag_tol_pct,
        # 2026-10-13 段2o-2: いちばん弱い段の途中の定速保持
        ramp_cruise_band_g=lr.ramp_cruise_band_g,
        ramp_cruise_hold_s=lr.ramp_cruise_hold_s,
        ramp_cruise_timeout_s=lr.ramp_cruise_timeout_s,
        # 2026-10-13 段2o-2: 遅れの測定のステップ（開度を固定で持つ）
        ramp_lag_step_g=lr.ramp_lag_step_g,
        ramp_lag_step_hold_s=lr.ramp_lag_step_hold_s,
        # 2026-10-14 段2o-5: 下りの戻し。安全網の車速は planner と同じ クリープ ＋ 5 km/h
        ramp_release_fracs=tuple(lr.ramp_release_fracs),
        ramp_release_move_s=lr.ramp_release_move_s,
        ramp_release_hold_s=tuple(lr.ramp_release_hold_s),
        ramp_release_enabled=lr.ramp_release_enabled,
        ramp_release_min_kmh=cfg.feedforward.creep_speed_kmh + CRUISE_MIN_MARGIN_KMH,
        gov_soft_frac=lr.gov_soft_frac,
        # 待機位置 ＝ min（2026-10-08 段2l-6。待機しないなら掛けない）と、発進の開始位置
        pedal_standby=cfg.mode_drive.pedal_standby,
        stop_confirm_pct=stop_confirm_pct,
    )


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
    ramp_order: Sequence[tuple[float, float]] | None = None,
    loop_config: PatternLoopConfig | None = None,
    top_speed_cap_kmh: float | None = None,
) -> PatternDriveResult:
    """本番の学習運転パターンを PatternLoop（本番 LearningLoop 相当）で走らせ、
    緩減速で停車保持まで行う。

    前提: 2-0 のペダル探索が済み、停車保持開度（pedal.stop_brake_opening_pct）で停車している。
    `log` は走行前チェックから続く走行ログ（main が作って保存する）。無ければここで作り、
    終わりに（異常終了でも）保存する。
    `patterns` を渡さないときは `ramp_order`（`RampPlan.execution_order()`）から
    `build_patterns` で作る（無ければ ConfigError）。`patterns` / `loop_config` はテストや
    動作確認で短縮するための差し込み口。`top_speed_cap_kmh` はコーストダウン加速の上端
    （None なら車両の最高速 × accel_speed_cap_frac のまま。2026-10-07 段2l-4）。
    """
    profile = pedal.apply_to_profile(build_vehicle_profile(cfg))
    if patterns is None:
        if ramp_order is None:
            raise ConfigError("run_pattern_drive には patterns か ramp_order のどちらかが要ります")
        patterns = build_patterns(cfg, profile, ramp_order, with_lag=True)
    if loop_config is None:
        loop_config = loop_config_from_config(cfg, stop_confirm_pct=pedal.stop_confirm_pct)
    elif loop_config.stop_confirm_pct is None:
        loop_config = replace(loop_config, stop_confirm_pct=pedal.stop_confirm_pct)
    if top_speed_cap_kmh is not None:  # 2026-10-07 段2l-4: コーストダウンの上端（モード + 余裕）
        loop_config = replace(loop_config, top_speed_cap_kmh=top_speed_cap_kmh)
    _print_patterns(
        patterns, hold_s=loop_config.ramp_hold_time_s, turn_s=loop_config.ramp_turn_time_s
    )
    if hw.is_real:
        cap = profile.max_speed * loop_config.accel_speed_cap_frac
        if loop_config.top_speed_cap_kmh is not None:
            cap = min(cap, loop_config.top_speed_cap_kmh)
        say(f"*** 実機モード: 車両が 0 → 約 {cap:.0f} km/h まで加減速します。"
            "シャシダイナモ上で実施してください ***")
    say(f"開始状態: 停車保持ブレーキ {pedal.stop_brake_opening_pct:.2f}%（2-0 で確定）")
    say(f"各運転パターンの後は DRIVE_BRAKE で停車してから次へ進みます"
        f"（{loop_config.brake_stop_timeout_s:g}s 以内に停車しなければ中断）。"
        f"加速の打ち切り {loop_config.accel_full_range_timeout_s:g}s")
    say(f"コーストダウン加速: 踏む速さ = {loop_config.coast_accel_rate_gain:g} ×"
        f"（{loop_config.coast_accel_target_g:g}G − 今の G）%/s、"
        f"±{loop_config.coast_accel_press_margin_g:g}G は保持、"
        f"上限 = パターンの加速開度（頭打ち）、"
        f"指令は実開度 + {loop_config.ramp_actuator_lag_tol_pct:g}% まで（追いつくのを待つ）")
    say(f"Gガバナー: {profile.max_decel_g:g}G × {loop_config.g_limit_frac:g} 以上で頭打ち"
        f"（0.1 秒あたり {loop_config.gov_reduce_step_pct:g}% で下げる）、"
        f"× {loop_config.gov_release_frac:g} 未満で "
        f"0.1 秒あたり {loop_config.gov_raise_step_pct:g}% ずつ戻す")
    say(f"G ランプ: 基準車速（0 → 上端 → 保持 → 0 km/h。発進 "
        f"{loop_config.ramp_launch_time_s:g} 秒で {loop_config.ramp_edge_g:g}G → +G、"
        f"上端の手前で {loop_config.ramp_turn_time_s:g} 秒かけて +G → 0、"
        f"上端で {loop_config.ramp_hold_time_s:g} 秒保持、"
        f"{loop_config.ramp_turn_time_s:g} 秒かけて 0 → -G、"
        f"停止 {loop_config.ramp_stop_time_s:g} 秒で -G → -{loop_config.ramp_edge_g:g}G）との"
        f"偏差で開度を動かす（Kp = 1 ÷（走行中に測った感度 × "
        f"{loop_config.ramp_settle_time_s:g} 秒）、"
        f"Ki = Kp ÷ {RAMP_TI_RATIO * loop_config.ramp_settle_time_s:g} 秒。感度が無いペダルは"
        "最小刻み 1 個/周期。偏差は ±"
        f"{loop_config.ramp_dev_deadband_kmh:g} km/h の不感帯つき）。"
        "表の開度は毎周期は足さず、保持の頭・減速の頭・発進の1回だけ引いて寄せる。"
        "アクチュエータが指令に "
        f"{loop_config.ramp_actuator_lag_tol_pct:g}% 以上遅れている間は基準を待たせる。"
        "表示の acc/brk Kp・Ki は直近に計算した値（感度が無い間は 0）")
    if loop_config.ramp_one_sided:
        say("G ランプの片側調整: 加速は車が遅いときだけ・減速は車が速いときだけ動かし、"
            "行き過ぎは基準を実車速に合わせる。実測 G が段の上端（隣の段との中間）を超えたら戻す"
            "（保持と停止の直前は両方向）")
    else:
        say("G ランプの調整: 両方向（片側調整なし）")
    if any(isinstance(p, GRampPattern) and p.stations_kmh for p in patterns):
        limit_s = loop_config.ramp_turn_time_s + loop_config.ramp_cruise_timeout_s
        say(f"途中の定速保持（段2o-2）: 実測の加速度が ±{loop_config.ramp_cruise_band_g:g}G 以内の"
            f"時間が合計 {loop_config.ramp_cruise_hold_s:g} 秒で次へ（入ってから {limit_s:g} 秒で"
            "打ち切り）。保持中は偏差の項が両方向")
    if any(isinstance(p, GRampPattern) and p.release_stations for p in patterns):
        fracs = " → ".join(f"{f:g}" for f in loop_config.ramp_release_fracs)
        holds = "・".join(f"{h:g}" for h in loop_config.ramp_release_hold_s)
        say("下りの戻し（段2o-5）: 戻す車速に着いたら（ブレーキを踏んでいるときだけ。"
            "踏んでいなければ飛ばす）、山の開度に対して "
            f"{fracs} 倍まで {loop_config.ramp_release_move_s:g} 秒ずつ直線に離し、"
            f"それぞれ {holds} 秒保持（0 は待機位置）。出たら {loop_config.ramp_turn_time_s:g} "
            f"秒かけて 0 → -G。実車速が {loop_config.ramp_release_min_kmh:g} km/h 以下で打ち切り。"
            "戻し中は制御の項なし・表と中央値に貯めない")
    if any(isinstance(p, GRampPattern) and p.lag_step for p in patterns):
        say(f"遅れの測定: 上端で定速になったら、開度を固定で {loop_config.ramp_lag_step_hold_s:g} "
            f"秒ずつステップ（アクセル踏み増し {loop_config.ramp_lag_step_g:g}G 相当 → 戻し → "
            "離す → ブレーキ踏み増し → 戻し）。結果は 2-2 で表の遅れ L に入る")
    say("パターン走行は FF/PID を使いません（G ランプ以外は開度指令のみ。ゲインは表示のみ）")

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
        interval_s=cfg.control.loop_interval_s,  # yaml の周期（50 ms）。手順3・本番と同じ
    )
    monitor.ramp_status = learning_loop.ramp_status

    try:
        # PatternLoop の 100ms ループと Modbus を取り合わないよう、走行中はサンプラーを止める
        # （記録は PatternLoop の on_sample コールバックから行う）
        await session.stop_sampler()
        session.mark(SECTION_PATTERN_DRIVE, "")
        comm_start = _comm_counts(hw)  # 手順1 の分を混ぜないため、開始時点の値を控える
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
        say(_retry_summary(comm_start, _comm_counts(hw)))
        order = ramp_order_of(patterns)
        if order:
            say("── G ランプの結果 ──")
            say(result_table(
                order, learning_loop.ramp_results, duration,
                hold_s=loop_config.ramp_hold_time_s, turn_s=loop_config.ramp_turn_time_s,
            ))
        session.mark(SECTION_DECEL_TO_STOP, PHASE_APPROACH)
        session.start_sampler()
        stop = await decelerate_to_stop(hw, cfg, profile, log=session)
        say(f"停車保持: ブレーキ {stop.hold_pct:.2f}%")
        return PatternDriveResult(
            csv_path=session.csv_path, samples=monitor.samples, duration_s=duration, stop=stop,
            ramp_results=list(learning_loop.ramp_results),
        )
    finally:
        if own_log:
            await session.close()


# ─────────────────────────────────────────────────────────────────────
# 2-2. モデル作成
# ─────────────────────────────────────────────────────────────────────


@dataclass
class _BuiltModel:
    """② モデル作成の結果。updates は candidate 固有の参照用キー（config へ書く分）。"""

    model_path: str
    metrics: dict[str, dict[str, float]]
    updates: dict[str, Any] = field(default_factory=dict)
    # C8 のとき、学習の元にした C7 の表の pkl（チェック B1 で C8 と比べる）
    c7_path: str | None = None


def _forward_pkl_path(models_dir: Path, prefix: str, hw_mode: str) -> Path:
    """順モデルの pkl 名（UTC）。スタブは `_stub` を付け、実機の pkl と紛れないようにする。"""
    from datetime import UTC, datetime  # noqa: PLC0415

    stamp = datetime.now(UTC).strftime("%Y%m%d_%H%M%S")
    suffix = "" if hw_mode == HW_REAL else f"_{hw_mode}"
    return models_dir / f"{prefix}_{stamp}{suffix}.pkl"


@dataclass(frozen=True)
class MeasuredLags:
    """手順2 の遅れの測定（G ランプの後ろの HOLD_STEP。2026-10-13 段2o-2）で測れたペダル別の遅れ
    [s]。測れなかったペダルは None（表の既定値 0.05 秒のまま）。"""

    accel_s: float | None = None
    brake_s: float | None = None
    result: PedalLagResult | None = None  # 推定の中身（表示用）


def _usable_lag(entry: tuple[int, float] | None, min_n: int) -> float | None:
    """ペダル別の (件数, 中央値) が使えるなら中央値。件数が足りない・範囲外は None。"""
    if entry is None or entry[0] < min_n or not 0.0 < entry[1] <= pedal_lag.MAX_LAG_S:
        return None
    return float(entry[1])


def measure_pedal_lags(
    csv_path: Path, *, accel_min_pct: float | None = None, brake_min_pct: float | None = None,
) -> MeasuredLags:
    """走行 CSV の HOLD_STEP（手順2 の遅れの測定）から、ペダル別の遅れを測る。

    ペダル別の中央値（踏み増し・戻しをまとめる）を、件数が `pedal_lag.LAG_STEP_MIN_STEPS` 以上で
    0 < L ≤ `pedal_lag.MAX_LAG_S` のときだけ採る。HOLD_STEP の無い CSV（旧形式・遅れの測定を
    しなかった走行・スタブ）は両方 None。読めない CSV も None（遅れは既定値のまま）。

    2026-10-16 段2o-6: min（待機位置）を渡すと、min から出る・min へ戻るステップは使わない
    （遊びを通る時間が遅れに混ざるため。`pedal_lag.estimate_hold_step_lag`）。
    """
    try:
        res = pedal_lag.estimate_hold_step_lag(
            pedal_lag.read_step_rows(csv_path),
            accel_min_pct=accel_min_pct, brake_min_pct=brake_min_pct,
        )
    except (OSError, ValueError):
        return MeasuredLags()
    n = pedal_lag.LAG_STEP_MIN_STEPS
    return MeasuredLags(
        accel_s=_usable_lag(res.by_pedal.get(pedal_lag.PEDAL_ACCEL), n),
        brake_s=_usable_lag(res.by_pedal.get(pedal_lag.PEDAL_BRAKE), n),
        result=res,
    )


def _say_pedal_lags(lags: MeasuredLags, speeds_kmh: Sequence[float]) -> None:
    """遅れの測定の結果を表示する（測れたペダルは表に使う値、測れないものは既定値のまま）。"""
    from tests.research.fwd_tool import DEFAULT_ACCEL_LAG_S, DEFAULT_BRAKE_LAG_S  # noqa: PLC0415

    res = lags.result
    if res is None or res.n_steps_total == 0:
        say(f"遅れの測定: 走行ログに HOLD_STEP がありません → 表の遅れは既定値"
            f"（アクセル {DEFAULT_ACCEL_LAG_S:g}・ブレーキ {DEFAULT_BRAKE_LAG_S:g} 秒）のまま")
        return

    def cell(key: str, group: Sequence[Any]) -> str:
        pedal, _, kind = key.partition("_")
        sel = [st.lag_s for st in group if st.pedal == pedal and st.press == (kind == "press")]
        if not sel:
            return "-"
        med = sorted(sel)[len(sel) // 2] if len(sel) % 2 else sum(
            sorted(sel)[len(sel) // 2 - 1:len(sel) // 2 + 1]
        ) / 2.0
        return f"{med:.2f}({len(sel)})"

    keys = ("accel_press", "accel_release", "brake_press", "brake_release")
    say(f"遅れ（指令の 50% 点 → 加速度の 50% 点。HOLD_STEP {res.n_steps_total} 区間・採用 "
        f"{res.n_used} 件・min 発着で除外 {len(res.excluded_steps)} 件。表の値は 中央値(件数)）:")
    say("  条件           アクセル踏み増し アクセル戻し ブレーキ踏み増し ブレーキ戻し")
    say("  全体           " + " ".join(f"{cell(k, res.steps):>14}" for k in keys))
    if speeds_kmh and res.steps:
        for v in sorted(speeds_kmh):  # 各ステップを、いちばん近い測定車速に振り分ける
            group = [
                st for st in res.steps
                if min(speeds_kmh, key=lambda x: abs(x - st.speed_kmh)) == v
            ]
            say(f"  {v:>4g} km/h     " + " ".join(f"{cell(k, group):>14}" for k in keys))
    used_a = f"{lags.accel_s:.2f} 秒" if lags.accel_s is not None else (
        f"測れず既定値 {DEFAULT_ACCEL_LAG_S:g} 秒"
    )
    used_b = f"{lags.brake_s:.2f} 秒" if lags.brake_s is not None else (
        f"測れず既定値 {DEFAULT_BRAKE_LAG_S:g} 秒"
    )
    say(f"  → 表に使う遅れ: アクセル {used_a}・ブレーキ {used_b}（惰行はアクセルと同じ値）")


def _forward_settings(profile: VehicleProfile, lags: MeasuredLags | None = None) -> Any:
    """C7・C8 の u の原点は min（プロファイル。名前は不感帯のまま）。

    遅れ L は既定値。2026-10-13 段2o-2: `lags`（手順2 の遅れの測定）で測れたペダルは、その値を
    `accel_lag_s`・`brake_lag_s` に入れる（惰行の遅れ `coast_lag_s` は None のままアクセルと同じ
    値になる）。測れなかったペダルは既定値のまま。
    """
    from tests.research.fwd_tool import DeadbandLag  # noqa: PLC0415

    fp = profile.feedforward_params
    st = DeadbandLag(fp.accel_deadband_pct, fp.brake_deadband_pct)
    if lags is not None:
        if lags.accel_s is not None:
            st = replace(st, accel_lag_s=lags.accel_s)
        if lags.brake_s is not None:
            st = replace(st, brake_lag_s=lags.brake_s)
    return st


def _attach_onsets(table: Any, profile: VehicleProfile, csv_path: Path) -> None:
    """表の meta に、ペダルの効き始め（min からの量 u [%]）を入れる（2026-10-16 段2o-6）。

    手順2 の CSV の G ランプ行（HOLD_STEP 以外）から `play_onset` の折れ点で求める。惰行は表の
    `coast_accel`（u=0 の加速度。負）の符号を反転した減速量（モデルと同じ基準）。求まらなかった
    ペダルはキーを入れない（古い pkl もキーが無い。読む側は「効き始めなし」として扱う）。
    """
    from tests.research.play_onset import onsets_from_csv  # noqa: PLC0415

    fp = profile.feedforward_params
    amin, bmin = fp.accel_deadband_pct, fp.brake_deadband_pct

    def coast_decel(v: Any) -> Any:
        return -np.asarray(table.coast_accel(np.asarray(v, dtype=float)), dtype=float)

    acc, brk = onsets_from_csv(csv_path, accel_min=amin, brake_min=bmin, coast_fn=coast_decel)
    for label, key, res, pmin in (
        ("アクセル", "accel_onset_u_pct", acc, amin),
        ("ブレーキ", "brake_onset_u_pct", brk, bmin),
    ):
        if res.onset_pct is None:
            say(f"  効き始め[{label}]: 求まらず（{res.reason}）。meta に入れない")
            continue
        table.meta[key] = res.onset_pct - pmin
        say(f"  効き始め[{label}]: 絶対 {res.onset_pct:.2f}%（min {pmin:g}% から "
            f"u={res.onset_pct - pmin:.2f}%）、k={res.k:.3f}")


def _attach_returns(table: Any, csv_path: Path) -> None:
    """表の meta に、ペダルが min へ戻るまでの時間（遅れ・戻る速さ）を入れる（2026-10-17 段2o-9）。

    手順2 の CSV から `pedal_lag.estimate_pedal_return` で測る。
    戻る時間 = 遅れ + 戻す量 ÷ 戻る速さ。
    手順3 の開度の列（`fwd_plan`）が、切り替えのすき間をこの時間以上にするのに使う。測れなかった
    ペダルはキーを入れない（古い pkl もキーが無い。読む側は仮の既定値を使い、表示で知らせる）。
    """
    res = pedal_lag.estimate_pedal_return(csv_path)
    for label, prefix, fit in (
        ("アクセル", "accel", res.accel), ("ブレーキ", "brake", res.brake),
    ):
        if not fit.measured:
            say(f"  戻る時間[{label}]: 測れず（{fit.reason}）。meta に入れない")
            continue
        table.meta[f"{prefix}_return_delay_s"] = float(fit.delay_s)  # type: ignore[arg-type]
        table.meta[f"{prefix}_return_rate_pct_s"] = float(fit.rate_pct_s)  # type: ignore[arg-type]
        say(f"  戻る時間[{label}]: 遅れ {fit.delay_s:.3f}s・戻る速さ {fit.rate_pct_s:.0f}%/s"
            f"（場面 {fit.n} 件、当てはめの外れ最大 {fit.max_err_s:.3f}s）")


def _build_table_model(
    profile: VehicleProfile,
    csv_path: Path,
    models_dir: Path,
    hw_mode: str,
    *,
    announce: bool = True,
    lags: MeasuredLags | None = None,
) -> _BuiltModel:
    """C7: 走行 CSV から順モデルの表（車速 × 効く開度 u → 加速度）を作って保存する。

    `lags`（2026-10-13 段2o-2）: 手順2 の遅れの測定で測れたペダル別の遅れ。測れたペダルは表の
    遅れ L に使う（pkl の meta に残る）。None・測れなかったペダルは既定値。
    """
    from tests.research import fwd_tool  # noqa: PLC0415
    from tests.research.fwd_map import save_table  # noqa: PLC0415

    st = _forward_settings(profile, lags)
    if announce:
        say("順モデルの表（C7）: 車速 × 効く開度 u → 加速度 を作成 …")
    rows = fwd_tool.read_logs([csv_path], st)
    table = fwd_tool.table_from(rows, st)
    _attach_onsets(table, profile, csv_path)
    _attach_returns(table, csv_path)
    path = save_table(table, _forward_pkl_path(models_dir, "fwd_table", hw_mode))
    say(f"  u の原点（min）: アクセル {st.accel_deadband_pct:g}・"
        f"ブレーキ {st.brake_deadband_pct:g} %（vehicle.min_*_opening_pct）、"
        f"使えた秒 {sum(r.seconds for r in rows):.1f}")
    say(f"モデル保存: {path}")
    return _BuiltModel(model_path=str(path), metrics={})


def _attach_low_speed_tables(
    table: Any,
    csv_path: Path,
    st: Any,
    *,
    creep_speed_kmh: float,
    creep_launch_min_speed_kmh: float | None = None,
) -> Any:
    """C8 の表に、発進側（launch_accel）と停止側（stop_accel）を付けて返す（2026-10-18 段2o-10）。

    行の取り方（ニューラルネット本体の学習の行は変えない。表だけ）:
      発進側 … PATTERN_DRIVE の発進の区間（`launch_mask`）・PEDAL_SEARCH/CREEP_WAIT・
               PATTERN_DRIVE/RAMP_LAUNCH_RELEASE の停車中の行
      停止側 … PRE_DRIVE_CHECK/BRAKE_STEP・PEDAL_SEARCH/BRAKE_SEARCH・PATTERN_DRIVE の停車の手前の
               行（`stop_approach_rows`・停車まぎわの強いブレーキ `stop_approach_mask`）・
               CREEP_WAIT の u = 0 の行
    読み方は実開度・ずれ許容なし・停車の行を残す・窓 LAUNCH_ACCEL_WINDOW_S（従来の発進側と同じ）。
    「動き出す開度」は CSV の RAMP_LAUNCH_RELEASE（無ければ BRAKE_SEARCH）から求める。
    クリープ車速が 0 以下の車は両方とも作らない（None）。
    """
    from tests.research import fwd_tool  # noqa: PLC0415
    from tests.research.fwd_cv import stop_approach_mask  # noqa: PLC0415
    from tests.research.fwd_data import (  # noqa: PLC0415
        PHASE_BRAKE_SEARCH,
        PHASE_BRAKE_STEP,
        PHASE_CREEP_WAIT,
        PHASE_RAMP_LAUNCH_RELEASE,
        STOP_SPEED_KMH,
        concat_rows,
        launch_breakaway,
        launch_mask,
        stop_approach_rows,
        take_rows,
    )
    from tests.research.fwd_map import attach_launch, attach_stop  # noqa: PLC0415

    def read(sections: Sequence[str], phases: Sequence[str] | None = None) -> Any:
        extra: dict[str, Any] = {} if phases is None else {"phases": tuple(phases)}
        (rows,) = fwd_tool.read_logs(
            [csv_path], st, opening_source="actual", actuator_lag_tol_pct=None,
            keep_stopped=True, accel_window_s=LAUNCH_ACCEL_WINDOW_S, sections=tuple(sections),
            **extra,
        )
        return rows

    pattern = read((SECTION_PATTERN_DRIVE,))
    lm = launch_mask(pattern)
    creep = read((SECTION_PEDAL_SEARCH,), (PHASE_CREEP_WAIT,))
    release = read((SECTION_PATTERN_DRIVE,), (PHASE_RAMP_LAUNCH_RELEASE,))
    release = take_rows(release, release.speed_kmh < STOP_SPEED_KMH)
    launch_rows = concat_rows([take_rows(pattern, lm), creep, release], "launch")

    brake_step = read((SECTION_PRE_DRIVE_CHECK,), (PHASE_BRAKE_STEP,))
    brake_search = read((SECTION_PEDAL_SEARCH,), (PHASE_BRAKE_SEARCH,))
    near_stop = (
        (stop_approach_rows(pattern) | stop_approach_mask(pattern.speed_kmh, pattern.u_pct))
        & ~lm & (pattern.speed_kmh >= STOP_SPEED_KMH)
    )
    creep_zero = take_rows(
        creep, (creep.speed_kmh >= STOP_SPEED_KMH) & (np.abs(creep.u_pct) < 1e-9)
    )
    stop_rows = concat_rows(
        [brake_step, brake_search, take_rows(pattern, near_stop), creep_zero], "stop"
    )

    min_speed = STOP_SPEED_KMH if creep_launch_min_speed_kmh is None else creep_launch_min_speed_kmh
    breakaway = launch_breakaway(
        csv_path, brake_min_pct=st.brake_deadband_pct, brake_lag_s=st.brake_lag_s,
        min_speed_kmh=min_speed,
    )
    table = attach_launch(
        table, launch_rows, creep_speed_kmh=creep_speed_kmh, breakaway=breakaway,
        accel_window_s=LAUNCH_ACCEL_WINDOW_S,
    )
    table = attach_stop(
        table, stop_rows, creep_speed_kmh=creep_speed_kmh, accel_window_s=LAUNCH_ACCEL_WINDOW_S
    )
    if creep_speed_kmh <= 0.0:
        say("  発進側・停止側: クリープ車速が 0 なので作らない")
    if table.launch_accel is None:
        say(f"  発進側: 行が足りないので作らない（{table.meta['launch_rows_s']:.1f} 秒）")
    else:
        say(f"  発進側: 動いている行 {table.meta['launch_rows_s']:.1f} 秒、置き換えた車速の行 "
            f"{[round(x, 2) for x in table.meta['launch_speed_rows']]} km/h")
    if breakaway is None:
        say("  動き出す開度: 求まらず（車速 0 付近の深い側を 0 で埋めない）")
    else:
        say(f"  動き出す開度: u={breakaway.u_pct:.2f}%（{breakaway.source}、区間ごと "
            f"{[round(x, 2) for x in breakaway.per_segment_u_pct]}）")
    if table.stop_accel is None:
        say(f"  停止側: 行が足りないので作らない（{table.meta['stop_rows_s']:.1f} 秒）")
    else:
        say(f"  停止側: 動いている行 {table.meta['stop_rows_s']:.1f} 秒、置き換えた車速の行 "
            f"{[round(x, 2) for x in table.meta['stop_speed_rows']]} km/h"
            f"（引く車速の上限 {table.meta['stop_max_speed_kmh']:g} km/h）")
    return table


def _build_forward_model(
    profile: VehicleProfile,
    csv_path: Path,
    models_dir: Path,
    hw_mode: str,
    *,
    workers: int,
    seeds: Sequence[int] | None,
    max_iter: int | None,
    lags: MeasuredLags | None = None,
    creep_launch_min_speed_kmh: float | None = None,
) -> _BuiltModel:
    """C8: C7 の表を先に作って保存し（格子と設定の元）、順モデル（NN）を全ログで学習する。

    2026-10-13 段2o-2: 設定は relu（2 層 50 個・停車まぎわ除外あり・追加学習なし。ユーザー決定。
    `fwd_final` の CLI の既定（1-1）は変えず、ここで渡す引数だけ変える）。遅れ L は `lags` で
    測れたペダルだけ表に入れる（C7 の表を作るときと同じ）。
    """
    from tests.research import fwd_final, fwd_tool, fwd_torch  # noqa: PLC0415
    from tests.research.fwd_map import load_forward_map, save_table  # noqa: PLC0415

    say("順モデル（C8）: まず C7 の表を作り（格子と設定の元）、続けてニューラルネットを学習 …")
    like = _build_table_model(
        profile, csv_path, models_dir, hw_mode, announce=False, lags=lags
    )
    st = fwd_tool.settings_of_table(
        load_forward_map(Path(like.model_path)), _forward_settings(profile, lags)
    )
    rows = fwd_tool.read_logs([csv_path], st)
    use_seeds = tuple(fwd_final.DEFAULT_SEEDS if seeds is None else seeds)
    cfg8 = fwd_final.final_config(
        use_seeds, hidden=FORWARD_HIDDEN, activation=FORWARD_ACTIVATION,
        drop_stop_approach=FORWARD_DROP_STOP_APPROACH,
        table_u_step_pct=FORWARD_TABLE_U_STEP_PCT,
    )
    use_iter = fwd_torch.MAX_ITER if max_iter is None else max_iter
    say(f"  設定: {cfg8.name}（隠れ層 {cfg8.hidden}・{cfg8.activation}・損失 {cfg8.loss}）、"
        f"表の u の刻み {FORWARD_TABLE_U_STEP_PCT:g}%、乱数 {list(cfg8.seeds)}、"
        f"並列 {workers}、max_iter {use_iter}")
    started = time.perf_counter()
    table, secs = fwd_final.build_final_table(
        cfg8, rows, Path(like.model_path), workers=workers, max_iter=use_iter,
        seed_mae=True,  # 乱数ごとの MAE を meta に残す（チェック B1 の乱数の外れ用）
    )
    wall = time.perf_counter() - started
    for seed, sec in secs:
        say(f"    乱数{seed}: 学習 {sec:.1f} 秒")
    say(f"  学習の合計 {sum(s for _, s in secs):.1f} 秒・壁時計 {wall:.1f} 秒")
    table = _attach_low_speed_tables(
        table, csv_path, st, creep_speed_kmh=profile.feedforward_params.creep_speed_kmh,
        creep_launch_min_speed_kmh=creep_launch_min_speed_kmh,
    )
    # 2026-10-14 段2o-5: 戻しの表は build_final_table が付ける（final_config の既定で作る）
    rel_s = float(table.meta.get("release_rows_s", 0.0))
    if table.release_grid is None:
        say(f"  戻しの表: 戻しの行が足りないので作らない（{rel_s:.1f} 秒）")
    else:
        say(f"  戻しの表: 戻し量 0.3% 以上のブレーキの行 {rel_s:.1f} 秒、"
            f"戻し量の格子 {[float(x) for x in table.release_grid]} %、"
            f"頭打ち {table.meta['release_max_pct']:.2f} %")
    _attach_onsets(table, profile, csv_path)
    _attach_returns(table, csv_path)
    path = save_table(table, _forward_pkl_path(models_dir, "fwd_model", hw_mode))
    say(f"モデル保存: {path}")
    return _BuiltModel(model_path=str(path), metrics={}, c7_path=like.model_path)


def _apply_updates_to_cfg(cfg: ResearchConfig, updates: dict[str, Any]) -> None:
    """ドット区切りキー（"feedforward.creep_speed_kmh" 等）の値を cfg（メモリ上の写し）に入れる。"""
    for key, value in updates.items():
        section, name = key.split(".", 1)
        target = getattr(cfg, section, None)
        if target is not None and hasattr(target, name):
            setattr(target, name, value)


def _run_step2_checks(
    *,
    cfg: ResearchConfig,
    csv_path: Path,
    built: _BuiltModel,
    after: FeedforwardParams,
    research_after: ResearchFFParams,
    pedal: PedalSearchResult | None,
    ramp_results: Sequence[Any] | None,
    plan_mode: DrivingMode | None,
    real: bool,
) -> list[Any]:
    """手順2 の学習結果のチェック（C2 の「yaml が pkl を指すか」以外。それは yaml を書いた後）。

    手順3 が使う値で確かめるため、cfg の写しに「これから yaml へ書く値」（惰行・クリープ加速
    カーブ・model_path）と 2-0 の実測（停車保持・クリープ車速。cfg は 2-0 が yaml を書く前の
    ものなので）を入れて渡す。C1 は実機だけ、yaml を読み直した値と 2-0 の実測を比べる。
    """
    from tests.research import step2_check  # noqa: PLC0415

    check_cfg = copy.deepcopy(cfg)
    ff = check_cfg.feedforward
    ff.model_path = built.model_path
    ff.coast_decel_speeds_kmh = list(after.coast_decel_speeds_kmh)
    ff.coast_decel_kmhs = list(after.coast_decel_kmhs)
    ff.creep_accel_speeds_kmh = list(research_after.creep_accel_speeds_kmh)
    ff.creep_accel_kmhs = list(research_after.creep_accel_kmhs)
    if pedal is not None:
        _apply_updates_to_cfg(check_cfg, pedal.config_updates())
    yaml_cfg: ResearchConfig | None = None
    if real and pedal is not None:
        try:
            yaml_cfg = load_config(cfg.source_path)
        except (ConfigError, OSError):
            yaml_cfg = None
    return step2_check.run_checks_for_paths(
        cfg=check_cfg, csv_path=csv_path, model_path=built.model_path, c7_path=built.c7_path,
        plan_mode=plan_mode, ramp_results=ramp_results,
        pedal_result=pedal if real else None,  # スタブは 2-0 が yaml を書かないので比べない
        saved_path=None, yaml_cfg=yaml_cfg,
    )


def _outlier_seeds(model_path: str) -> list[int]:
    """保存した C8 の pkl の meta（seed_mae）から、外れた乱数を返す。読めなければ空。"""
    from tests.research import step2_check  # noqa: PLC0415
    from tests.research.fwd_map import load_forward_map  # noqa: PLC0415

    try:
        seed_mae = load_forward_map(Path(model_path)).meta.get("seed_mae")
    except Exception:  # noqa: BLE001 - 読めないときは学習し直さず、チェックの NG に任せる
        return []
    return step2_check.seed_outliers(seed_mae) if seed_mae else []


def _only_seed_outliers_ng(checks: Sequence[Any]) -> bool:
    """NG が B1 の 1 項目だけで、その原因に乱数の外れがあるか（= 学習し直す対象）。"""
    from tests.research import step2_check  # noqa: PLC0415

    ng = step2_check.ng_items(checks)
    return (
        len(ng) == 1 and ng[0].id == "B1" and ng[0].metrics.get("seed_outlier_count", 0.0) > 0.0
    )


def _replace_seeds(seeds: Sequence[int], outliers: Sequence[int]) -> tuple[int, ...]:
    """外れた乱数を別の乱数（+100。他と重なるなら更に +100）に替える。"""
    used = set(seeds)
    out: list[int] = []
    for sd in seeds:
        if sd in outliers:
            new = sd + 100
            while new in used:
                new += 100
            used.add(new)
            out.append(new)
        else:
            out.append(sd)
    return tuple(out)


def build_ff_model(
    cfg: ResearchConfig,
    csv_path: Path,
    *,
    hw_mode: str,
    pedal: PedalSearchResult | None = None,
    write_config: bool | None = None,
    wltp_mode: DrivingMode | None = None,
    sample_weight_enabled: bool | None = None,
    forward_workers: int = FORWARD_WORKERS,
    forward_seeds: Sequence[int] | None = None,
    forward_max_iter: int | None = None,
    ramp_results: Sequence[Any] | None = None,
) -> ModelBuildResult:
    """走行 CSV からモデル（candidate の種類）と物理定数を作り、実機走行なら YAML へ保存する。

    ① 物理定数の推定と保存（惰行カーブ・クリープ加速カーブ・停止ブレーキ下限・ペダルゲイン。
       candidate に依らない）と、② モデル作成（candidate で分岐）に分かれる。
    ②（段2k 段D。ProblemReport_20261001）:
        C1〜C5 … 2次多項式 Ridge 逆モデル（`_build_poly_model`。本番 training_service と同じ
                  順序で、学習時点のプロファイル値を使う。A1＝そのペダルが効いている行だけで学習する
                  研究用の `train_inverse_model_effective`）
        C7    … 順モデルの表（`_build_table_model`。車速 × 効く開度 u → 加速度）
        C8    … C7 の表を先に作って格子・設定の元にし、ニューラルネットの順モデルを全ログで
                  学習した表（`_build_forward_model`）
        どの candidate でも write_config のとき `feedforward.model_path` を作った pkl に書く
        （`feedforward.candidate` は書き換えない）。
    `pedal` があれば 2-0 の実測値（停車保持開度・クリープ平衡車速）をプロファイルへ反映してから
    学習する（C7・C8 の u の原点は yaml の vehicle.min_*_opening_pct）。サンプル不足は本番の
    LearningDataError をそのまま送出する。

    `write_config` 省略時（None）は従来どおり `hw_mode == HW_REAL` で判定する。明示的に指定すると
    実機モードでも保存を止められる（`tests.research.relearn` の `--dry-run` 用。段2.5）。

    段2（ProblemReport_20260925。学習サンプルの WLTP 重み付け。C1〜C5 のみ）:
        `wltp_mode` を渡すと、その基準車速から作った WLTP 格子（`wltp_grid.wltp_cells`）で
        `WltpWeighting` を組み、`train_inverse_model_effective` に渡す。`sample_weight_enabled`
        省略時（None）は `sample_weight.SAMPLE_WEIGHT_ENABLED` に従う（明示すれば一時的に
        上書きできる。`tests.research.relearn` の `--weight` 用）。有効なのに `wltp_mode` が
        無い場合は `ConfigError`（重みの計算に WLTP の基準車速が要るため）。

    `forward_workers`・`forward_seeds`・`forward_max_iter` は C8 の学習用（並列数・乱数・
    最大反復。省略時は FORWARD_WORKERS・fwd_final.DEFAULT_SEEDS・fwd_torch.MAX_ITER。
    学習時間の枠に合わせて変えられる。テストは max_iter を小さくする）。

    手順2 の学習結果のチェック（2026-10-16 段2o-7。`step2_check`）:
        順番は「モデル作成 → チェック（C2 以外）→ OK なら yaml 書き込み → C2（yaml を読み直す）」。
        `ramp_results`（G ランプの結果。B5① 用）・`pedal`（C1 用）・`wltp_mode`（D1・B3 用）を
        チェックの材料にする。無い材料の項目は SKIP。表は yaml を書く前に表示する。
        NG があれば、**yaml を書き換えず**（pkl は残す）`Step2CheckError` を出す。スタブ
        （hw_mode != real）は表示するだけで止めない。C8 で NG が B1 の乱数の外れだけなら、
        外れた乱数を別の乱数に替えて 1 回だけ学習し直して再チェックする。
    """
    require_forward_candidate(cfg)  # 2026-10-08 段2l-6: C1〜C5 は対象外（読み込みの前に止める）
    logs = read_drive_logs(csv_path)
    say(f"学習データ: {csv_path}（{len(logs)} 行）")
    profile = build_vehicle_profile(cfg)
    if pedal is not None:
        profile = pedal.apply_to_profile(profile)
    if write_config is None:
        write_config = hw_mode == HW_REAL
    if hw_mode != HW_REAL:
        profile.id = f"{profile.id}_{hw_mode}"  # スタブのモデルは別名で保存する

    candidate = cfg.feedforward.candidate
    models_dir = Path(cfg.feedforward.model_path).parent
    lags: MeasuredLags | None = None
    # 2026-10-13 段2o-2: 遅れの測定（HOLD_STEP）を表の遅れ L に入れる（C10 は C8 と同じ。段2p）
    if candidate in ("C7", "C8", "C10", "C11"):
        lags = measure_pedal_lags(
            csv_path,
            accel_min_pct=profile.feedforward_params.accel_deadband_pct,
            brake_min_pct=profile.feedforward_params.brake_deadband_pct,
        )
        _say_pedal_lags(lags, cfg.learning.ramp_lag_speeds_kmh)
    if candidate == "C7":
        built = _build_table_model(profile, csv_path, models_dir, hw_mode, lags=lags)
    elif candidate in ("C8", "C10", "C11"):  # C10・C11 は C8 と同じ pkl（段2p・段2q）
        built = _build_forward_model(
            profile, csv_path, models_dir, hw_mode,
            workers=forward_workers, seeds=forward_seeds, max_iter=forward_max_iter, lags=lags,
            creep_launch_min_speed_kmh=getattr(cfg.learning, "creep_launch_min_speed_kmh", None),
        )
    else:
        built = _build_poly_model(
            cfg, csv_path, logs, profile, models_dir,
            wltp_mode=wltp_mode, sample_weight_enabled=sample_weight_enabled,
        )

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
    # 2026-10-08 段2l-6: クリープ域ブレーキ保持をパターン列から外したので、クリープ域ブレーキの
    # 下限（stop_brake_floor_offset_pct）は推定しない（yaml の値を据え置く）。
    # _estimate_research_stop_brake_floor のコード削除は実機確認の後の別の段
    after = _estimate_research_pedal_gains(cfg, logs, reference, after, research_after)
    _print_param_changes(before, after, research_before, research_after)

    # 手順2 の学習結果のチェック（yaml を書く前）。NG なら止める（スタブは表示のみ）
    from tests.research import step2_check  # noqa: PLC0415

    real = hw_mode == HW_REAL

    def run_checks_now() -> list[Any]:
        return _run_step2_checks(
            cfg=cfg, csv_path=csv_path, built=built, after=after, research_after=research_after,
            pedal=pedal, ramp_results=ramp_results, plan_mode=wltp_mode, real=real,
        )

    say()
    say("手順2 の学習結果のチェック（手順3 に影響する範囲）:")
    checks = run_checks_now()
    if candidate in ("C8", "C10", "C11") and _only_seed_outliers_ng(checks):
        outliers = _outlier_seeds(built.model_path)
        from tests.research import fwd_final  # noqa: PLC0415

        base_seeds = tuple(fwd_final.DEFAULT_SEEDS if forward_seeds is None else forward_seeds)
        new_seeds = _replace_seeds(base_seeds, outliers)
        say(step2_check.format_checks(checks))
        say(f"乱数 {outliers} の学習データでの MAE が外れているので、{list(new_seeds)} に替えて"
            "1 回だけ学習し直します …")
        built = _build_forward_model(
            profile, csv_path, models_dir, hw_mode,
            workers=forward_workers, seeds=new_seeds, max_iter=forward_max_iter, lags=lags,
            creep_launch_min_speed_kmh=getattr(cfg.learning, "creep_launch_min_speed_kmh", None),
        )
        checks = run_checks_now()
    say(step2_check.format_checks(checks))
    ng = step2_check.ng_items(checks)
    model_path, metrics = built.model_path, built.metrics
    if ng and not real:
        say("チェックに NG がありますが、スタブのため止めません。")
    elif ng:
        say(f"yaml は書き換えません。作った pkl は残してあります: {model_path}")
        raise step2_check.Step2CheckError(checks)

    if not write_config:
        if hw_mode == HW_REAL:
            say(f"--dry-run のため {cfg.source_path} は更新しません（推定値の確認のみ）")
        else:
            say(f"スタブ走行のため {cfg.source_path} は更新しません"
                "（実機の値を模擬値で上書きしないため）")
        return ModelBuildResult(
            model_path=model_path, metrics=metrics, params=after, changed=[],
            research_params=research_after, checks=checks,
        )

    updates: dict[str, Any] = {"feedforward.model_path": model_path, **built.updates}
    for key in FF_PARAM_KEYS:
        if key in MEASURED_KEYS:
            continue
        value = getattr(research_after, key) if key in RESEARCH_PARAM_KEYS else getattr(after, key)
        updates[f"feedforward.{key}"] = list(value) if isinstance(value, tuple) else float(value)
    changed = cfg.save(updates)
    say(f"{cfg.source_path} に保存しました（{len(changed)} 行を更新）:")
    for line in changed:
        say(f"  {line}")
    # C2: 書いた yaml を読み直して、model_path が保存した pkl を指し、読めることを確かめる
    c2 = step2_check.check_model_path(load_config(cfg.source_path), model_path)
    checks = [c2 if c.id == "C2" else c for c in checks]
    say(f"C2 {c2.name}: {c2.status}（{c2.value} ← 保存した pkl {model_path}。{c2.reason}）")
    if c2.status == step2_check.STATUS_NG:
        raise step2_check.Step2CheckError([c2])
    return ModelBuildResult(
        model_path=model_path, metrics=metrics, params=after, changed=changed,
        research_params=research_after, checks=checks,
    )


def _build_poly_model(
    cfg: ResearchConfig,
    csv_path: Path,
    logs: list[DriveLog],
    profile: VehicleProfile,
    models_dir: Path,
    *,
    wltp_mode: DrivingMode | None,
    sample_weight_enabled: bool | None,
) -> _BuiltModel:
    """C1〜C5: 2次多項式 Ridge 逆モデル（ホライズン自動選択・実質Kp 符号確認・係数書き出し）。"""
    enabled = SAMPLE_WEIGHT_ENABLED if sample_weight_enabled is None else sample_weight_enabled
    if enabled and wltp_mode is None:
        raise ConfigError(
            "sample_weight_enabled が有効ですが wltp_mode が渡されていません"
            "（WLTP 重み付けには基準車速が必要です）"
        )

    say("train_inverse_model_effective: 2次多項式＋標準化＋Ridge（アクセル/ブレーキの 2 モデル）"
        "を、そのペダルが効いている行だけで学習 …")
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
        from tests.research.wltp_grid import (
            GRID_ACCEL_EDGES_KMHS,
            GRID_SPEED_EDGES_KMH,
            edges_from_config,
            wltp_cells,
        )

        edges = edges_from_config(GRID_SPEED_EDGES_KMH, GRID_ACCEL_EDGES_KMHS)
        wltp_s = wltp_cells(wltp_mode, edges, spec)
        weighting = WltpWeighting(
            wltp_s=wltp_s, speed_edges_kmh=edges.speed_kmh, accel_edges_kmhs=edges.accel_kmhs,
            w_min=SAMPLE_WEIGHT_MIN, w_max=SAMPLE_WEIGHT_MAX, enabled=enabled,
        )
        say(f"WLTP 重み付け: {'有効' if enabled else '無効'}"
            f"（w_min={SAMPLE_WEIGHT_MIN:g}, w_max={SAMPLE_WEIGHT_MAX:g}）")
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

    say("ペダル遅れ L: 格子ステップが無いので測りません（pedal_select_center_s は据え置き）")
    updates: dict[str, Any] = {
        # 参照用（走行は pkl を読む）。ホライズン・特徴量・係数の場所を config から見えるようにする
        "feedforward.model_accel_horizons_s": list(accel_spec.lookahead_horizons_s),
        "feedforward.model_brake_horizons_s": list(brake_spec.lookahead_horizons_s),
        "feedforward.model_accel_features": accel_spec.feature_names(),
        "feedforward.model_brake_features": brake_spec.feature_names(),
        "feedforward.model_coef_path": str(coef_path),
    }
    return _BuiltModel(model_path=model_path, metrics=metrics, updates=updates)


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
        params.brake_deadband_pct, params.stop_brake_opening_pct, CREEP_BRAKE_HOLD_FRACS
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
