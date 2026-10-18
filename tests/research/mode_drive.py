"""手順 3: 走行モード管理の WLTP を FF だけで走り、車速追従を記録する（手順 5/7/9 で PID を足す）。

    基準車速 → FF（手順 2 のモデル）──┐
                                      ＋ → effort → 調停（PedalArbiter）→ アクセル/ブレーキ → 車両
      PID（手順 5 以降。手順 3 は 0）──┘                                                      │
       ↑                                                                                      │
       └──────────────────────────── 車速（CAN） ───────────────────────────────────────────────┘

引用元（アルゴリズムを移植。ループ自体は本番クラスを実行しない — 遵守事項「/src を実行しない」）:
    src/domain/control/drive_loop.py   … 1 サイクルの順序（基準車速の線形補間 → 先読み/過去の
                                          基準車速 → CAN 車速 → FF → 調停 → 変化した軸だけ位置指令
                                          → 電流 → 過電流・逸脱判定 → 100ms ごとのログ）、
                                          軸指令の smooth_over_s
    src/domain/control/feedforward.py  … FeedforwardController.predict_effort（手順 2 のモデルの
                                          推論。model_training と同じくドメインの関数として呼ぶ）。
                                          レジーム合成だけ tests/research/ff_candidate.py の
                                          CandidateFeedforward（C1）で差し替えている
    tests/research/pattern_loop.py     … 減速G ガバナー（0.4s の傾きで判定・頭打ち → 2% ずつ下げる）
    src/infra/mode_repository.py       … driving_modes.reference_speed の読み方

本番の自動走行との違い（第1段階として極端に単純にしたところ）:
    - プラン・トリム・フェーズ権限・ゲインスケジューリング・最小実効ブレーキ・FB 用ローパスは
      持たない。
    - 調停は pedal_arbiter.PedalArbiter（本番の移植）。機能は arbiter.enable_* で 1 つずつ有効化する
      （ヒステリシス・不感帯補償・レートリミット・微小変化の保持・再踏込ディレイ・解放レート、
      研究専用の加速度帯の保持・向きのヒステリシス）。
      すべて false なら effort の符号でアクセル/ブレーキに振り分け、開度上限でクランプするだけ
      （split_effort と同一）。
    - 一時停止・WebSocket・DB ログは持たない。ログは SessionLog（CSV/PNG）に 0.1s 刻みで残す。
    - 周期は固定の asyncio ループ。1 周期以上遅れたら次の周期から数え直し、回数をレポートに出す。
    - 減速G ガバナー（安全網）を持つ。本番の自動走行には無いので、作動した時間をレポートに出す。
"""

from __future__ import annotations

import asyncio
import bisect
import json
import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any

from tests.research.axis_monitor import AxisMonitor
from tests.research.axis_safety import AxisSafetyNet
from tests.research.config import ConfigError, ResearchConfig
from tests.research.dob import PEDAL_ACCEL, PedalDisturbanceObserver
from tests.research.drive_log import (
    SECTION_DECEL_TO_STOP,
    SECTION_MODE_DRIVE,
    DriveSample,
    SessionLog,
)
from tests.research.ff_candidate import (
    CandidateFeedforward,
    CoastAssistFeedforward,
    DeviationForwardFeedforward,
    DisturbanceObserverFeedforward,
    ForwardMapFeedforward,
    make_candidate,
)
from tests.research.ff_params import research_ff_params
from tests.research.hardware import ActuatorProtocol, DriveError, ResearchHardware
from tests.research.pattern_drive import _overcurrent_limit_ma, _release_pedals
from tests.research.pedal_arbiter import (
    PedalArbiter,
    enabled_arbiter_features,
    plan_accel_kmhs,
)
from tests.research.pedal_select import window_slope_kmhs
from tests.research.research_types import (
    G_TO_KMHS,
    VEHICLE_STOP_SPEED_KMH,
    DriveLogData,
    DrivingMode,
    SpeedPoint,
)
from tests.research.stop_decel import PHASE_APPROACH, PHASE_STOP_HOLD, decelerate_to_stop
from tests.research.term import drive_status_line, say
from tests.research.vehicle import (
    build_vehicle_profile,
    feedforward_params,
    opening_to_pulse,
    pulse_to_opening,
)

# 本番 drive_loop と同じ: 1 周期のうち軸の移動にかける割合と、書き込み直後の読み取りまでの待ち
AXIS_SMOOTH_DUTY = 0.8
AXIS_PRE_READ_DELAY_S = 0.002
# 減速G ガバナー（pattern_loop.PatternLoopConfig と同じ値）
GOVERNOR_WINDOW_S = 0.4  # 減速度を出す車速の傾きの窓 [s]
GOVERNOR_LIMIT_FRAC = 0.98  # vehicle.max_decel_g のこの割合で作動


# CSV の phase 列（その周期に動かしたペダル）
PHASE_ACCEL = "ACCEL"
PHASE_BRAKE = "BRAKE"
PHASE_BRAKE_GOVERNED = "BRAKE_GOV"  # 減速G ガバナーでブレーキを頭打ちにした周期
PHASE_COAST = "COAST"


# ─────────────────────────────────────────────────────────────────────
# 準備（走行前チェックより前に行う。失敗しても HW には触らない）
# ─────────────────────────────────────────────────────────────────────


@dataclass
class ModeDriveSetup:
    mode: DrivingMode
    ff: CandidateFeedforward | ForwardMapFeedforward


async def load_mode(cfg: ResearchConfig, name: str) -> DrivingMode:
    """走行モード管理（PostgreSQL の driving_modes）からモードを名前で読む。"""
    import asyncpg  # noqa: PLC0415 - DB を使わないテスト・手順で import を要求しない

    try:
        conn = await asyncpg.connect(cfg.hardware.database_url)
    except Exception as exc:
        raise ConfigError(
            f"走行モードの DB に接続できません（{cfg.hardware.database_url}、"
            f"{type(exc).__name__}: {exc}）"
        ) from exc
    try:
        row = await conn.fetchrow(
            "SELECT id, name, description, reference_speed, total_duration, max_speed, "
            "created_at, is_system FROM driving_modes WHERE name = $1",
            name,
        )
    finally:
        await conn.close()
    if row is None:
        raise ConfigError(f"走行モード {name!r} が driving_modes にありません（modes セクション）")
    points = [
        SpeedPoint(time_s=float(p["time_s"]), speed_kmh=float(p["speed_kmh"]))
        for p in json.loads(row["reference_speed"])
    ]
    if len(points) < 2:
        raise ConfigError(f"走行モード {name!r} の基準車速が 2 点未満です")
    return DrivingMode(
        id=str(row["id"]),
        name=row["name"],
        description=row["description"],
        reference_speed=points,
        total_duration=float(row["total_duration"]),
        max_speed=float(row["max_speed"]),
        created_at=row["created_at"],
        is_system=bool(row["is_system"]),
    )


def load_feedforward(cfg: ResearchConfig) -> CandidateFeedforward | ForwardMapFeedforward:
    """手順 2 で作った FF モデルと物理定数（config_testVehicle.yaml）を読み込む。

    C7・C8 は多項式モデルではなく順モデルの pkl（表・モデル版）を読む（物理定数・研究パラメータ・
    先読みホライズンの検査は多項式モデル用なので通さない）。開度の列は prepare_mode_drive が作る。
    """
    ff_cfg = cfg.feedforward
    if not ff_cfg.is_model_trained:
        raise ConfigError(
            f"FF モデルがありません: {ff_cfg.model_path}（手順 2 で作成してください）"
        )
    ff = make_candidate(ff_cfg.candidate)  # V1 案スイッチ（C1〜C5・C7・C8。既定 C1）
    if isinstance(ff, ForwardMapFeedforward):
        try:
            ff.load_model(ff_cfg.model_path)
        except (OSError, ValueError) as exc:
            raise ConfigError(
                f"FF モデルを読み込めません: {ff_cfg.model_path}（{exc}）"
            ) from exc
        return ff
    ff.set_params(feedforward_params(cfg))
    ff.set_research_params(research_ff_params(cfg))
    try:
        ff.load_model(ff_cfg.model_path)
    except (OSError, ValueError) as exc:
        raise ConfigError(f"FF モデルを読み込めません: {ff_cfg.model_path}（{exc}）") from exc
    # 段3: reach_horizons_s はモデルの先読みホライズンの部分集合でなければならない
    # （predict_effort が future_speeds の添字を引くため）。走行前に落とす
    missing = [h for h in ff_cfg.reach_horizons_s if h not in ff.horizons]
    if missing:
        raise ConfigError(
            f"feedforward.reach_horizons_s の {missing} はモデルの先読みホライズン "
            f"{list(ff.horizons)} に含まれていません"
        )
    return ff


def _prepare_forward_map_plan(
    cfg: ResearchConfig, mode: DrivingMode, ff: ForwardMapFeedforward
) -> None:
    """C7・C8: 基準車速から開度の列を作り、中身を表示する（走行前チェックより前。HW 不使用）。"""
    ff_cfg, fm = cfg.feedforward, cfg.forward_map
    plan = ff.build_plan(
        ReferenceSpeed(mode).at,
        mode.total_duration,
        accel_deadband_pct=cfg.vehicle.min_accel_opening_pct,
        brake_deadband_pct=cfg.vehicle.min_brake_opening_pct,
        stop_brake_opening_pct=cfg.stop_brake_opening_pct,
        stop_horizon_s=cfg.features.h0_s,
        period_s=cfg.control.loop_interval_s,
        accel_lag_s=fm.accel_lag_s,
        brake_lag_s=fm.brake_lag_s,
        need_window_s=fm.need_window_s,
        coast_band_kmhs=fm.coast_band_kmhs,
        preposition_enabled=fm.preposition_enabled,
        preposition_margin_pct=fm.preposition_margin_pct,
        preposition_lead_s=fm.preposition_lead_s,
    )
    meta = ff.table.meta
    # 使った遅れ（None なら fwd_plan と同じく pkl の meta の値）
    lag_a = fm.accel_lag_s if fm.accel_lag_s is not None else meta.get("accel_lag_s")
    lag_b = fm.brake_lag_s if fm.brake_lag_s is not None else meta.get("brake_lag_s")
    say(f"FF 候補: {ff.candidate}（順モデルの表）　表: {ff_cfg.model_path}")
    say(f"  遅れ アクセル {lag_a}s / ブレーキ {lag_b}s・惰行の帯 ±{fm.coast_band_kmhs:g} km/h/s・"
        f"窓 {fm.need_window_s:g}s")
    if isinstance(ff, DeviationForwardFeedforward):
        # 2026-10-17 段2p: τ ＝ c × 上の遅れ（yaml の上書き、なければ pkl の meta）。
        # どちらにも無いペダルは None で、fwd_plan の既定値には落とさない（上乗せしない）
        ff.set_deviation(
            c=fm.deviation_c, row=fm.deviation_row,
            lag_accel_s=None if lag_a is None else float(lag_a),
            lag_brake_s=None if lag_b is None else float(lag_b),
        )
        say(f"  C10（偏差の上乗せ）: {c10_settings_text(ff)}")
    if isinstance(ff, DisturbanceObserverFeedforward):
        # 2026-10-18 段2q: Tq ＝ dob_tq_coef × 上の遅れ（C10 の τ と同じ元）。
        # どちらにも無いペダルは None で、そのペダルは DOB を使わない
        ff.set_dob(
            tq_coef=fm.dob_tq_coef,
            lag_accel_s=None if lag_a is None else float(lag_a),
            lag_brake_s=None if lag_b is None else float(lag_b),
        )
        say(f"  C11（DOB）: {dob_settings_text(ff)}")
    if isinstance(ff, CoastAssistFeedforward):
        # 2026-10-18 段2r: 惰行の補助。保持時間 ＝ 係数 × (遅れ ＋ 効き始めまでの戻り時間)
        ff.set_coast(
            on_kmh=fm.coast_on_kmh, off_kmh=fm.coast_off_kmh, hold_coef=fm.coast_hold_coef
        )
        say(f"  C12（惰行の補助）: {coast_settings_of(ff, cfg)}")
    sec = plan.pedal_seconds()
    say(f"  範囲外を引いた秒数 {plan.out_of_range_s():.1f}s・穴埋めを引いた秒数 "
        f"{plan.filled_s():.1f}s")
    say("  " + "・".join(f"{name} {s:.1f}s" for name, s in sec.items())
        + f"・ペダル切替 {plan.pedal_switches()} 回")
    if plan.preposition is not None:
        say(f"  先置き（効き始め−{fm.preposition_margin_pct:g}% を踏み出しの"
            f"{fm.preposition_lead_s:g}s 前から）: {plan.preposition.summary()}")
    else:
        say("  先置き: なし（forward_map.preposition_enabled: false）")
    if plan.return_gap is not None:
        say(f"  切り替えのすき間（戻る時間 T 以上に）: {plan.return_gap.summary()}")
        if plan.return_gap.defaults_used:
            say("  注: pkl に戻る時間の測定値が無いので仮値（遅れ 0.15s・50%/s）で待つ。"
                "手順2 の再学習（--retrain）で測定値が入る")
    mismatches = []
    # 2026-10-08 段2l-6: pkl の meta のキー名は accel_deadband_pct・brake_deadband_pct のまま、
    # 値は u の原点（vehicle.min_*_opening_pct）
    for label, key, cfg_val in (
        ("アクセル", "accel_deadband_pct", cfg.vehicle.min_accel_opening_pct),
        ("ブレーキ", "brake_deadband_pct", cfg.vehicle.min_brake_opening_pct),
    ):
        table_val = meta.get(key)
        if table_val is not None and abs(float(table_val) - cfg_val) > 0.01:
            mismatches.append(f"{label} pkl {float(table_val):.2f} / 設定 {cfg_val:.2f}")
    if mismatches:
        # 2026-10-08 段2l-7: 表は u（開度 − min）で持ち、走行時に設定の min ＋ u へ戻すので、
        # min が違っても警告ではなくお知らせにする（取り付け直し後も手順2 をやり直さない運用）
        say(f"  お知らせ: u の原点（min）が表を作ったときと違います（{'、'.join(mismatches)}）。"
            "表は u（開度 − min）で持つので、そのまま使います。"
            "動きの比率が変わる付け直しをした場合は手順2 をやり直してください")
    say(f"  待機位置（min） アクセル {cfg.vehicle.min_accel_opening_pct:.2f}% / "
        f"ブレーキ {cfg.vehicle.min_brake_opening_pct:.2f}%・"
        f"停車保持 {cfg.stop_brake_opening_pct:.2f}%（min ＋ {ff_cfg.stop_brake_offset_pct:.2f}%）")


async def prepare_mode_drive(cfg: ResearchConfig, mode_name: str) -> ModeDriveSetup:
    say(f"走行モードを読み込みます: {mode_name!r}（{cfg.hardware.database_url}）")
    mode = await load_mode(cfg, mode_name)
    say(f"  {len(mode.reference_speed)} 点・{mode.total_duration:.0f}s・"
        f"最高 {mode.max_speed:.1f} km/h")
    if mode.max_speed > cfg.vehicle.max_speed_kmh:
        raise ConfigError(
            f"モードの最高速 {mode.max_speed:.1f} km/h が vehicle.max_speed_kmh "
            f"{cfg.vehicle.max_speed_kmh:.1f} を超えています"
        )
    ff = load_feedforward(cfg)
    ff_cfg = cfg.feedforward
    ft = cfg.features
    if isinstance(ff, ForwardMapFeedforward):
        _prepare_forward_map_plan(cfg, mode, ff)
        return ModeDriveSetup(mode=mode, ff=ff)
    if ff.is_pedal_separated:
        # 手順6: アクセル・ブレーキが別々のホライズンを選んだ pkl（horizon_search）
        say(f"FF 候補: {ff.candidate}　モデル: {ff_cfg.model_path}")
        say(f"  先読み アクセル {list(ff.accel_spec.lookahead_horizons_s)}s / "
            f"ブレーキ {list(ff.brake_spec.lookahead_horizons_s)}s・"
            f"停車保持/ブレーキ下限 {ff.stop_horizon_s:g}s 先")
        say(f"  過去 アクセル {list(ff.accel_spec.past_horizons_s)}s / "
            f"ブレーキ {list(ff.brake_spec.past_horizons_s)}s")
        # horizon_search が選ぶのは先読み・過去だけなので、それ以外（レジーム・停車保持の
        # ホライズン・past_as_delta・v0_sq・交互作用項）だけ設定と食い違いが無いか見る
        mismatches = []
        if ff.spec.regime_horizon_s != ft.h1_s:
            mismatches.append(
                f"h1_s（レジーム）: pkl {ff.spec.regime_horizon_s:g} / 設定 {ft.h1_s:g}"
            )
        if ff.stop_horizon_s != ft.h0_s:
            mismatches.append(f"h0_s（停車保持）: pkl {ff.stop_horizon_s:g} / 設定 {ft.h0_s:g}")
        if ff.spec.past_as_delta != ft.past_as_delta:
            mismatches.append("past_as_delta")
        if ff.spec.include_v0_sq != ft.use_v0_sq:
            mismatches.append("use_v0_sq")
        if ff.spec.include_dv_regime_x_v0 != ft.use_dv1_x_v0:
            mismatches.append("use_dv1_x_v0")
        if mismatches:
            say(f"  ⚠ 設定 features と pkl が食い違っています（{'、'.join(mismatches)}）。"
                "設定を変えたなら手順2（学習）をやり直してください（走行は pkl のまま続けます）")
    else:
        say(f"FF 候補: {ff.candidate}　モデル: {ff_cfg.model_path}"
            f"（先読み {list(ff.horizons)}s・過去 {list(ff.past_horizons)}s）")
        say(f"  特徴量（pkl）: {ff.spec.feature_names()}")
        if ff.spec != ft.to_feature_spec():
            say("  ⚠ 設定 features の特徴量と pkl の特徴量が違います。設定を変えたなら"
                "手順2（学習）をやり直してください（走行は pkl の特徴量で続けます）")
    say(f"  待機位置（min） アクセル {cfg.vehicle.min_accel_opening_pct:.2f}% / "
        f"ブレーキ {cfg.vehicle.min_brake_opening_pct:.2f}%・"
        f"停車保持 {cfg.stop_brake_opening_pct:.2f}%（min ＋ {ff_cfg.stop_brake_offset_pct:.2f}%）")
    return ModeDriveSetup(mode=mode, ff=ff)


# ─────────────────────────────────────────────────────────────────────
# 部品
# ─────────────────────────────────────────────────────────────────────


class ReferenceSpeed:
    """基準車速の線形補間（本番 DriveLoop._ref_speed_at と同じ。範囲外は端点値）。"""

    def __init__(self, mode: DrivingMode) -> None:
        self._times = [p.time_s for p in mode.reference_speed]
        self._speeds = [p.speed_kmh for p in mode.reference_speed]

    def at(self, t_s: float) -> float:
        times, speeds = self._times, self._speeds
        if not times:
            return 0.0
        if t_s <= times[0]:
            return speeds[0]
        if t_s >= times[-1]:
            return speeds[-1]
        i = bisect.bisect_right(times, t_s) - 1
        dt = times[i + 1] - times[i]
        if dt <= 0.0:
            return speeds[i + 1]
        return speeds[i] + (t_s - times[i]) / dt * (speeds[i + 1] - speeds[i])


class ActualSpeedHistory:
    """実測車速の履歴（V2: C5 用）。`record` で追記し、`at` で線形補間する。

    過去ホライズンぶんだけ保持すればよいので、それより古い点は捨てる。履歴が浅い
    （走行開始直後）ときは直近値でフォールバックする（実測 ≈ 基準として実害は小さい）。
    """

    def __init__(self, max_span_s: float) -> None:
        self._max_span_s = max_span_s
        self._points: deque[tuple[float, float]] = deque()

    def record(self, t: float, speed: float) -> None:
        self._points.append((t, speed))
        while len(self._points) >= 2 and t - self._points[0][0] > self._max_span_s:
            self._points.popleft()

    def at(self, t: float) -> float:
        pts = self._points
        if not pts:
            return 0.0
        if t <= pts[0][0]:
            return pts[0][1]
        if t >= pts[-1][0]:
            return pts[-1][1]
        prev = pts[0]
        for point in pts:
            t1, v1 = point
            t0, v0 = prev
            if t0 <= t <= t1:
                return v1 if t1 <= t0 else v0 + (t - t0) / (t1 - t0) * (v1 - v0)
            prev = point
        return pts[-1][1]


def standby_openings(cfg: ResearchConfig) -> tuple[float, float]:
    """使っていないペダルの待機開度 (アクセル, ブレーキ) [%]。無効なら (0, 0)。

    2026-10-08 段2l-6（ProblemReport_20261001）: 「不感帯 − standby_margin_pct」をやめ、
    ペダルに触れた位置 vehicle.min_*_opening_pct そのものにした。
    """
    if not cfg.mode_drive.pedal_standby:
        return 0.0, 0.0
    v = cfg.vehicle
    return v.min_accel_opening_pct, v.min_brake_opening_pct


def standby_label(cfg: ResearchConfig) -> str:
    """表示・レポート用の待機位置の説明。"""
    if not cfg.mode_drive.pedal_standby:
        return "なし（0%）"
    accel, brake = standby_openings(cfg)
    return f"{accel:.2f}% / {brake:.2f}%（触れた位置 min）"


def split_effort(effort: float, max_accel_pct: float, max_brake_pct: float) -> tuple[float, float]:
    """調停（第1段階）: effort の符号でアクセル/ブレーキに振り分け、開度上限でクランプする。"""
    if effort > 0.0:
        return min(effort, max_accel_pct), 0.0
    if effort < 0.0:
        return 0.0, min(-effort, max_brake_pct)
    return 0.0, 0.0


@dataclass
class DecelGovernor:
    """減速G ガバナー（安全網）。pattern_loop.PatternLoop._update_governor と同じ下げ方の規則。

    ただし「減速が上限を十分下回ったら頭打ちを戻す」解除は pattern_loop だけに入れた
    （2026-09-13 A6）。こちらの「下限に張り付いて戻らない」穴 B は別課題 2 として残している。

    直近 GOVERNOR_WINDOW_S の車速の傾きで減速度を出し、max_decel_g × GOVERNOR_LIMIT_FRAC 以上なら
    ブレーキ開度を前周期の値で頭打ちにする。超えている間は周期ごとに reduce_step_pct ずつ下げる。
    指令がブレーキでなくなるか、頭打ちより浅くなったら解除する。
    """

    max_decel_g: float
    reduce_step_pct: float
    enabled: bool = True
    decel_kmhs: float = 0.0  # 直近の減速度（正が減速）
    _hist: deque[tuple[float, float]] = field(default_factory=deque)
    _cap: float | None = None
    _last_brake: float = 0.0

    @property
    def limit_kmhs(self) -> float:
        return self.max_decel_g * G_TO_KMHS * GOVERNOR_LIMIT_FRAC

    def apply(self, now: float, speed_kmh: float, brake_pct: float) -> tuple[float, bool]:
        """ブレーキ指令 [%] に頭打ちをかけ、(かけた後の開度, 頭打ち中か) を返す。"""
        hist = self._hist
        hist.append((now, speed_kmh))
        while len(hist) >= 2 and now - hist[0][0] > GOVERNOR_WINDOW_S:
            hist.popleft()
        (t0, v0), (t1, v1) = hist[0], hist[-1]
        self.decel_kmhs = (v0 - v1) / (t1 - t0) if t1 > t0 else 0.0

        if not self.enabled or brake_pct <= 0.0:
            self._cap = None
            self._last_brake = max(0.0, brake_pct)
            return brake_pct, False
        if self._cap is not None and brake_pct < self._cap:
            self._cap = None  # FF 自身が頭打ちより浅くした
        if self.decel_kmhs >= self.limit_kmhs:
            if self._cap is None:
                self._cap = self._last_brake
            else:
                self._cap = max(0.0, self._cap - self.reduce_step_pct)
        applied = brake_pct if self._cap is None else min(brake_pct, self._cap)
        self._last_brake = applied
        return applied, self._cap is not None


# ─────────────────────────────────────────────────────────────────────
# 走行
# ─────────────────────────────────────────────────────────────────────


# 実開度の安全網で「もう一方のペダルが min を超えている」とみなす幅 [%]（止まっているときの
# 読みの揺れ 4.19〜4.24% より少し大きい値。`pedal_lag.RETURN_SETTLE_TOL_PCT` と同じ）
INTERLOCK_TOL_PCT = 0.05


@dataclass
class InterlockStats:
    """実開度の安全網（pedal_interlock_enabled。2026-10-17 段2o-9）の集計。"""

    accel_events: int = 0  # アクセルを止め始めた回数（ブレーキの実開度が min を超えていた）
    accel_cycles: int = 0  # アクセルを止めた周期数
    brake_events: int = 0
    brake_cycles: int = 0
    period_s: float = 0.05

    @property
    def accel_s(self) -> float:
        return self.accel_cycles * self.period_s

    @property
    def brake_s(self) -> float:
        return self.brake_cycles * self.period_s

    def summary(self) -> str:
        return (
            f"アクセルを止めた {self.accel_events} 回・{self.accel_s:.2f}s／"
            f"ブレーキを止めた {self.brake_events} 回・{self.brake_s:.2f}s"
        )


@dataclass
class C10Stats:
    """C10（偏差の上乗せ。2026-10-17 段2p）の集計。ペダル別（行のペダル）に数える。"""

    accel_cycles: int = 0  # 上乗せで effort が変わった周期数
    brake_cycles: int = 0
    accel_abs_sum_pct: float = 0.0  # |上乗せ| の合計（平均の計算用）
    brake_abs_sum_pct: float = 0.0
    accel_max_pct: float = 0.0  # |上乗せ| の最大
    brake_max_pct: float = 0.0
    accel_clamp_events: int = 0  # 減らす向きで min に止めた回数（止め始めの数）
    accel_clamp_cycles: int = 0
    brake_clamp_events: int = 0
    brake_clamp_cycles: int = 0
    period_s: float = 0.05

    def add(self, *, is_accel: bool, delta_pct: float, clamped: bool, prev_clamped: bool) -> None:
        """1 周期ぶんを足す。delta_pct は上乗せ後 − 今の列（変わらなければ 0）。"""
        if is_accel:
            if delta_pct != 0.0:
                self.accel_cycles += 1
                self.accel_abs_sum_pct += abs(delta_pct)
                self.accel_max_pct = max(self.accel_max_pct, abs(delta_pct))
            if clamped:
                self.accel_clamp_cycles += 1
                self.accel_clamp_events += 0 if prev_clamped else 1
        else:
            if delta_pct != 0.0:
                self.brake_cycles += 1
                self.brake_abs_sum_pct += abs(delta_pct)
                self.brake_max_pct = max(self.brake_max_pct, abs(delta_pct))
            if clamped:
                self.brake_clamp_cycles += 1
                self.brake_clamp_events += 0 if prev_clamped else 1

    @property
    def accel_s(self) -> float:
        return self.accel_cycles * self.period_s

    @property
    def brake_s(self) -> float:
        return self.brake_cycles * self.period_s

    @property
    def accel_mean_pct(self) -> float:
        return self.accel_abs_sum_pct / self.accel_cycles if self.accel_cycles else 0.0

    @property
    def brake_mean_pct(self) -> float:
        return self.brake_abs_sum_pct / self.brake_cycles if self.brake_cycles else 0.0

    def summary(self) -> str:
        p = self.period_s
        return (
            f"アクセル {self.accel_s:.1f}s（上乗せ量の平均 {self.accel_mean_pct:.2f}%・"
            f"最大 {self.accel_max_pct:.2f}%・min で止めた {self.accel_clamp_events} 回/"
            f"{self.accel_clamp_cycles * p:.1f}s）／"
            f"ブレーキ {self.brake_s:.1f}s（平均 {self.brake_mean_pct:.2f}%・"
            f"最大 {self.brake_max_pct:.2f}%・min で止めた {self.brake_clamp_events} 回/"
            f"{self.brake_clamp_cycles * p:.1f}s）"
        )


def c10_settings_text(ff: object) -> str:
    """C10 の設定の表示（走行前とレポートで同じ文）。C10 でなければ「なし」。"""
    if not isinstance(ff, DeviationForwardFeedforward):
        return "なし"
    case = "基準車速（案A）" if ff.deviation_row == "reference" else "実車速（案B）"

    def tau(v: float | None) -> str:
        return f"{v:.1f}s" if v is not None else "上乗せなし（遅れが測れていない）"

    return (
        f"c={ff.deviation_c:g}・τ アクセル {tau(ff.tau_accel_s)} / ブレーキ {tau(ff.tau_brake_s)}"
        f"・表の行 {case}"
    )


@dataclass
class DobStats:
    """C11（DOB。2026-10-18 段2q）の集計。ペダル別。推定は更新した周期、上乗せ量は DOB の分が
    出た周期（押している行）で数える。"""

    accel_cycles: int = 0  # 推定を更新した周期数
    brake_cycles: int = 0
    accel_sum: float = 0.0  # 推定 d の合計（平均の計算用）
    brake_sum: float = 0.0
    accel_min: float = 0.0
    accel_max: float = 0.0
    brake_min: float = 0.0
    brake_max: float = 0.0
    accel_final: float = 0.0
    brake_final: float = 0.0
    accel_effort_cycles: int = 0  # DOB の上乗せが 0 でなかった周期数
    brake_effort_cycles: int = 0
    accel_effort_abs_sum: float = 0.0
    brake_effort_abs_sum: float = 0.0
    accel_effort_max: float = 0.0
    brake_effort_max: float = 0.0
    period_s: float = 0.05

    def add_estimate(self, *, is_accel: bool, d: float) -> None:
        """推定を更新した周期の d [km/h/s] を足す。"""
        if is_accel:
            self.accel_cycles += 1
            self.accel_sum += d
            self.accel_min = d if self.accel_cycles == 1 else min(self.accel_min, d)
            self.accel_max = d if self.accel_cycles == 1 else max(self.accel_max, d)
            self.accel_final = d
        else:
            self.brake_cycles += 1
            self.brake_sum += d
            self.brake_min = d if self.brake_cycles == 1 else min(self.brake_min, d)
            self.brake_max = d if self.brake_cycles == 1 else max(self.brake_max, d)
            self.brake_final = d

    def add_effort(self, *, is_accel: bool, effort_pct: float) -> None:
        """DOB の上乗せ量 [%]（P を除いた分）を足す。0 は数えない。"""
        if effort_pct == 0.0:
            return
        if is_accel:
            self.accel_effort_cycles += 1
            self.accel_effort_abs_sum += abs(effort_pct)
            self.accel_effort_max = max(self.accel_effort_max, abs(effort_pct))
        else:
            self.brake_effort_cycles += 1
            self.brake_effort_abs_sum += abs(effort_pct)
            self.brake_effort_max = max(self.brake_effort_max, abs(effort_pct))

    def _text(self, name: str, is_accel: bool) -> str:
        p = self.period_s
        n = self.accel_cycles if is_accel else self.brake_cycles
        ne = self.accel_effort_cycles if is_accel else self.brake_effort_cycles
        if is_accel:
            total, lo, hi, fin = self.accel_sum, self.accel_min, self.accel_max, self.accel_final
            esum, emax = self.accel_effort_abs_sum, self.accel_effort_max
        else:
            total, lo, hi, fin = self.brake_sum, self.brake_min, self.brake_max, self.brake_final
            esum, emax = self.brake_effort_abs_sum, self.brake_effort_max
        if n == 0:
            est = "更新なし"
        else:
            est = (
                f"推定 最終 {fin:+.3f}・平均 {total / n:+.3f}・"
                f"最小 {lo:+.3f}・最大 {hi:+.3f} km/h/s"
            )
        eff = f"上乗せ 平均 {esum / ne:.2f}%・最大 {emax:.2f}%" if ne else "上乗せなし"
        return f"{name} 更新 {n * p:.1f}s（{est}／{eff}）"

    def summary(self) -> str:
        return f"{self._text('アクセル', True)}／{self._text('ブレーキ', False)}"


def dob_settings_text(ff: object) -> str:
    """C11 の設定の表示（走行前とレポートで同じ文）。C11 でなければ「なし」。"""
    if not isinstance(ff, DisturbanceObserverFeedforward):
        return "なし"

    def tq(v: float | None) -> str:
        return f"{v:.1f}s" if v is not None else "DOB なし（遅れが測れていない）"

    return (
        f"Tq 係数 {ff.dob_tq_coef:g}・"
        f"Tq アクセル {tq(ff.tq_accel_s)} / ブレーキ {tq(ff.tq_brake_s)}"
    )


def coast_params(ff: CoastAssistFeedforward, cfg: ResearchConfig) -> dict[str, Any]:
    """C12 の CoastAssist に渡す値（保持時間・戻り時間・効き始めの開度）。
    保持時間 hold_p ＝ coast_hold_coef × (遅れ_p ＋ p の効き始めの開度が min へ戻る時間)。
    効き始めの開度は、アクセル ＝ min ＋ accel_onset_u_pct、
    ブレーキ ＝ −(min ＋ brake_onset_u_pct)。
    pkl の meta に効き始めが無ければ min を使う。遅れが測れていないペダルは補助しないので 0。"""
    # fwd_plan → ff_bias → mode_report → mode_drive の循環を避けるため、ここで読む
    from tests.research.fwd_plan import return_time_of  # noqa: PLC0415

    v, period = cfg.vehicle, cfg.control.loop_interval_s
    ret = return_time_of(ff.table, v.min_accel_opening_pct, v.min_brake_opening_pct)

    def onset(key: str) -> float:
        x = ff.table.meta.get(key)
        return float(x) if x is not None and math.isfinite(float(x)) else 0.0

    acc_on = v.min_accel_opening_pct + onset("accel_onset_u_pct")
    brk_on = -(v.min_brake_opening_pct + onset("brake_onset_u_pct"))

    def hold(lag: float | None, effort: float) -> float:
        if lag is None:
            return 0.0
        return ff.coast_hold_coef * (lag + ret.seconds(effort, period))

    return {
        "on_kmh": ff.coast_on_kmh, "off_kmh": ff.coast_off_kmh,
        "hold_accel_s": hold(ff.dob_lag_accel_s, acc_on),
        "hold_brake_s": hold(ff.dob_lag_brake_s, brk_on),
        "ret": ret, "period_s": period,
        "accel_onset_effort": acc_on, "brake_onset_effort": brk_on,
        "one_pedal_per_section": cfg.forward_map.coast_one_pedal_per_section,
    }


def coast_settings_of(ff: object, cfg: ResearchConfig) -> str:
    """C12 の設定の表示（走行前とレポートで同じ文）。C12 でなければ「なし」。"""
    if not isinstance(ff, CoastAssistFeedforward):
        return "なし"
    from tests.research.coast_assist import coast_settings_text  # noqa: PLC0415

    p = coast_params(ff, cfg)
    return coast_settings_text(
        ff.coast_on_kmh, ff.coast_off_kmh, ff.coast_hold_coef, p["hold_accel_s"], p["hold_brake_s"],
        floor_onset=cfg.forward_map.coast_floor_onset,
        one_pedal_per_section=cfg.forward_map.coast_one_pedal_per_section,
    )


@dataclass
class ModeDriveResult:
    mode_name: str
    mode_duration_s: float  # 走らせる予定だった長さ（--limit-s で縮めたらその値）
    run_duration_s: float  # 実際にモードを走った長さ
    completed: bool
    abort_reason: str = ""
    cycles: int = 0
    overruns: int = 0  # 1 周期以上遅れた回数
    governor_cycles: int = 0
    # 2026-10-17 段2o-9: 実開度の安全網（pedal_interlock_enabled）が出そうとしたペダルを止めた回数
    # （止め始めの数）と周期数。None は安全網が切れていた
    interlock: InterlockStats | None = None
    # 2026-10-17 段2p: C10（偏差の上乗せ）の設定の表示と集計。C10 でなければ None
    c10_settings: str | None = None
    c10: C10Stats | None = None
    # 2026-10-18 段2q: C11（DOB）の設定の表示と集計。C11 でなければ None
    dob_settings: str | None = None
    dob: DobStats | None = None
    # 2026-10-18 段2r: C12（惰行の補助）の設定の表示と集計。C12 でなければ None
    coast_settings: str | None = None
    coast_stats: Any = None  # coast_assist.CoastStats（循環 import を避けるため型は書かない）
    samples: list[DriveSample] = field(default_factory=list)  # MODE_DRIVE の行（0.1s 刻み）


class _ModeRun:
    """1 本のモード走行の状態。run() が 50ms 周期でモード終わりまで回す。"""

    def __init__(
        self,
        hw: ResearchHardware,
        cfg: ResearchConfig,
        setup: ModeDriveSetup,
        log: SessionLog,
        duration_s: float,
    ) -> None:
        self.hw = hw
        self.cfg = cfg
        self.ff = setup.ff
        self.mode = setup.mode
        self.ref = ReferenceSpeed(setup.mode)
        self.log = log
        self.duration_s = duration_s
        self.interval_s = cfg.control.loop_interval_s
        self.overcurrent_ma = _overcurrent_limit_ma(hw)
        self.governor = DecelGovernor(
            max_decel_g=cfg.vehicle.max_decel_g,
            reduce_step_pct=cfg.mode_drive.governor_reduce_step_pct,
            enabled=cfg.mode_drive.decel_governor,
        )
        self.accel_standby, self.brake_standby = standby_openings(cfg)
        self.arbiter = PedalArbiter(
            cfg.arbiter,
            accel_deadband_pct=cfg.vehicle.min_accel_opening_pct,
            brake_deadband_pct=cfg.vehicle.min_brake_opening_pct,
            max_accel_opening=cfg.vehicle.max_accel_opening_pct,
            max_brake_opening=cfg.vehicle.max_brake_opening_pct,
            nominal_dt_s=cfg.control.loop_interval_s,
        )
        self.arbiter.reset()
        self._prev_now: float | None = None  # 調停の dt 用（前サイクルの時刻）
        # V2: C5 だけが使う実測車速の履歴（過去ホライズンの最大値ぶんだけ保持すればよい）
        self.actual_history = ActualSpeedHistory(max(self.ff.past_horizons, default=1.0) + 0.5)
        self.cycles = 0
        self.overruns = 0
        self.governor_cycles = 0
        # 実開度の安全網: 前の周期のまとめ読みの実開度（None は未取得 = 止めない）
        self.interlock: InterlockStats | None = (
            InterlockStats(period_s=self.interval_s) if cfg.mode_drive.pedal_interlock_enabled
            else None
        )
        # C10（偏差の上乗せ）: C10 のときだけ集計を持つ
        self.c10: C10Stats | None = (
            C10Stats(period_s=self.interval_s)
            if isinstance(self.ff, DeviationForwardFeedforward) else None
        )
        self._c10_prev_clamped = False
        # C11（DOB）: C11 のときだけ推定器と集計を持つ
        self.dob: PedalDisturbanceObserver | None = None
        self.dob_stats: DobStats | None = None
        self._dob_effort: float | None = None  # 今の周期の上乗せのうち DOB の分 [%]
        if isinstance(self.ff, DisturbanceObserverFeedforward):
            # fwd_data → ff_bias → mode_report → mode_drive の循環を避けるため、ここで読む
            from tests.research.fwd_data import RELEASE_WINDOW_S  # noqa: PLC0415

            self.dob = PedalDisturbanceObserver(
                self.ff.table,
                tq_accel_s=self.ff.tq_accel_s, tq_brake_s=self.ff.tq_brake_s,
                lag_accel_s=self.ff.dob_lag_accel_s, lag_brake_s=self.ff.dob_lag_brake_s,
                release_window_s=RELEASE_WINDOW_S,
            )
            self.dob_stats = DobStats(period_s=self.interval_s)
        # C12（惰行の補助）: C12 のときだけ状態機械と集計を持つ
        self.coast: Any = None
        self.coast_settings: str | None = None
        self._coast_effort = 0.0  # 今の周期の惰行の補助の開度 [%]（補助しない周期は 0）
        if isinstance(self.ff, CoastAssistFeedforward):
            from tests.research.coast_assist import CoastAssist  # noqa: PLC0415

            plan = self.ff.plan
            self.coast = CoastAssist(
                plan.effort_pct, plan.flag, plan.t_s, **coast_params(self.ff, cfg)
            )
            self.coast_settings = coast_settings_of(self.ff, cfg)
        self._last_accel_actual: float | None = None
        self._last_brake_actual: float | None = None
        self._accel_blocked = False
        self._brake_blocked = False
        self.cycle_ms: list[float] = []  # 全周期の処理時間（A7 の関門: 周期を超えないか）
        self.samples: list[DriveSample] = []
        self._accel_cmd: int | None = None
        self._brake_cmd: int | None = None
        self._deviation_since: float | None = None
        self.max_abs_dev = 0.0
        self._next_print = 0.0
        self._segment = ""
        # アクチュエータ脱落の安全網（アラーム確認・電流ゼロ継続。axis_safety.py）
        self.safety = AxisSafetyNet(self.interval_s)

    async def run(self) -> float:
        """モード終わり（または duration_s）まで回し、走った秒数を返す。異常は DriveError。"""
        loop = asyncio.get_running_loop()
        started = loop.time()
        next_tick = started
        while True:
            t = loop.time() - started
            if t >= self.duration_s:
                return t
            await self._cycle(t, loop.time())
            self.cycles += 1
            next_tick += self.interval_s
            now = loop.time()
            if now - next_tick > self.interval_s:
                self.overruns += 1  # 1 周期以上遅れた: まとめて取り返さず次の周期から数え直す
                next_tick = now
            await asyncio.sleep(max(0.0, next_tick - now))

    def _ff_inputs(
        self, t: float, ref: float, speed: float
    ) -> tuple[float, list[float], list[float]]:
        """FF へ渡す v0/future/past（V2）。

        C1〜C3 は今まで通り基準車速だけ。C4 は動作点だけ実車速にし、先読み・過去の変化量は
        基準車速のまま（偏差そのものは入れない）。C5 は過去を実測履歴、先読みを基準の絶対値にする
        （`dv = ref(t+h) − v_実測(t)` が自然に出る。KAIZEN 報告書 3 章 結論 8〜10）。
        ただし基準が停車レジームのときは C4・C5 も基準車速だけを返す（下記参照）。
        """
        ff = self.ff
        future = [self.ref.at(t + h) for h in ff.horizons]
        # 停車判定は候補によらず基準車速で行う（C1 に倣う）。v0 を実測にする C4・C5 は
        # 実車速が VEHICLE_STOP_SPEED_KMH を下回らない限り停車保持に入らないため
        # （A8 報告 5 章 #2）。判定の形は predict_effort の停車レジーム（ff_candidate.py）と
        # 同じく最短ホライズンのみ（3 秒先まで見ると発進の 3 秒前に保持ブレーキが解除され、
        # クリープで動き出す。src/domain/control/feedforward.py:211-223 のレビュー指摘 #5）。
        ref_is_stopped = (
            ref <= VEHICLE_STOP_SPEED_KMH and future and future[0] <= VEHICLE_STOP_SPEED_KMH
        )
        if not ff.uses_actual_speed or ref_is_stopped:
            past = [self.ref.at(t - h) for h in ff.past_horizons]
            return ref, future, past
        if ff.candidate == "C5":
            past = [self.actual_history.at(t - h) for h in ff.past_horizons]
        else:  # C4
            future = [speed + (self.ref.at(t + h) - ref) for h in ff.horizons]
            past = [speed + (self.ref.at(t - h) - ref) for h in ff.past_horizons]
        return speed, future, past

    def _select_kwargs(self, t: float, v0: float) -> dict[str, float]:
        """ペダル選択の差し替え引数（ProblemReport_20260929 段3）。

        `pedal_select_mode` が window のときは、窓の傾き G と ref(t+L) を渡す
        （L_b 指定時はブレーキ用に L_b の同じ 2 つも）。
        point のときは、`reach_horizons_s` が空なら `pedal_select_point_s`（hp）先の傾き
        (ref(t+hp) − v0) / hp だけを渡す（惰行加速度は従来どおり v0 で評価するので速度は渡さない。
        モデルの h1 とは独立に先読みを変えられる）。C4 は v0 が実車速で、FF の入力上の偏差が
        ref(t+h) − ref(t) なので、その場合だけ基準を ref(t) にする。到達判定
        （`reach_horizons_s` が空でない）がペダル選択を担うときは何も渡さない。
        """
        ff_cfg = self.cfg.feedforward
        if ff_cfg.pedal_select_mode != "window":
            if ff_cfg.reach_horizons_s:
                return {}
            hp = ff_cfg.pedal_select_point_s
            base = self.ref.at(t) if getattr(self.ff, "candidate", "") == "C4" else v0
            return {"select_accel_kmhs": (self.ref.at(t + hp) - base) / hp}
        center, width = ff_cfg.pedal_select_center_s, ff_cfg.pedal_select_width_s
        kwargs = {
            "select_accel_kmhs": window_slope_kmhs(self.ref.at, t, center, width),
            "select_speed_kmh": self.ref.at(t + center),
        }
        # L_b（ブレーキの判定だけの窓の中心。段2-2）。None なら L と同じ＝何も足さない
        center_b = ff_cfg.pedal_select_brake_center_s
        if center_b is not None:
            kwargs["select_brake_accel_kmhs"] = window_slope_kmhs(self.ref.at, t, center_b, width)
            kwargs["select_brake_speed_kmh"] = self.ref.at(t + center_b)
        return kwargs

    async def _cycle(self, t: float, now: float) -> None:
        cfg, hw = self.cfg, self.hw
        cycle_started = time.perf_counter()
        try:
            speed = await hw.can.read_speed()
        except Exception as exc:
            raise DriveError(f"CAN 車速を読めません（{type(exc).__name__}: {exc}）") from exc
        self.actual_history.record(t, speed)

        ref = self.ref.at(t)
        if isinstance(self.ff, ForwardMapFeedforward):
            # C7・C8: 開度の列を時刻で引く（走る前に作り終えている。実測車速は見ない）
            ff_effort = self.ff.effort_at(t)
        else:
            v0, future, past = self._ff_inputs(t, ref, speed)
            ff_effort = self.ff.predict_effort(v0, future, past, **self._select_kwargs(t, v0))
        c10_effort: float | None = None
        if self.c10 is not None:
            c10_effort = self._c10_offset(t, ref, speed, ff_effort)
        coast_active = False
        if self.coast is not None:
            # C12: 惰行の行（計画が 0）で偏差が積み上がったら、浅く踏んで詰める
            self._coast_effort = 0.0
            coast_effort = self._coast_assist(t, ref, speed, ff_effort)
            if coast_effort != 0.0:
                c10_effort, coast_active = coast_effort, True
        pid_effort = 0.0  # 手順 5 以降でここに PID の出力を足す
        # C10 の上乗せは FF の列（accel_ff・brake_ff・plan_effort）には入れない（作ったずれが
        # 今までと同じ意味で比べられるように）。実際に出す effort にだけ足す
        effort = ff_effort + (c10_effort or 0.0) + pid_effort
        # ログの「指示開度 FF」列（FF の出力をペダル別に分けた値。ガバナー・待機位置の前）
        accel_ff, brake_ff = split_effort(
            ff_effort, cfg.vehicle.max_accel_opening_pct, cfg.vehicle.max_brake_opening_pct
        )
        # 調停の dt は前サイクルからの経過。初回は公称周期（arbitrate が dt<=0 で置き換える）
        arb_dt = 0.0 if self._prev_now is None else now - self._prev_now
        self._prev_now = now
        if cfg.arbiter.enable_accel_band:
            # 加速度帯の保持（段3c）: 計画加速度（基準 0〜H 秒先の傾き）と偏差を渡す
            arb = self.arbiter.arbitrate(
                effort,
                arb_dt,
                plan_accel_kmhs=plan_accel_kmhs(
                    self.ref.at, t, cfg.arbiter.accel_band_horizon_s
                ),
                deviation_kmh=speed - ref,
            )
        else:
            arb = self.arbiter.arbitrate(effort, arb_dt)
        accel, brake = arb.accel_opening, arb.brake_opening
        brake, governed = self.governor.apply(now, speed, brake)
        if governed:
            self.governor_cycles += 1
        accel, brake = self._apply_interlock(accel, brake)
        if self.dob is not None:
            self._dob_update(t, ref, speed, accel, brake, governed, coast_active)
        phase = (
            PHASE_ACCEL if accel > 0.0
            else PHASE_BRAKE_GOVERNED if governed
            else PHASE_BRAKE if brake > 0.0
            else PHASE_COAST
        )
        # 使っていないペダルも待機位置（min）で待たせる（phase は待機位置を掛ける前の選択で決める）
        accel, brake = max(accel, self.accel_standby), max(brake, self.brake_standby)
        accel_pos, brake_pos = opening_to_pulse(accel), opening_to_pulse(brake)

        try:
            monitor_accel, monitor_brake = await asyncio.gather(
                self.drive_axis(hw.accel, accel_pos, is_accel=True),
                self.drive_axis(hw.brake, brake_pos, is_accel=False),
            )
        except Exception as exc:
            raise DriveError(
                f"アクチュエータ通信に失敗しました（{type(exc).__name__}: {exc}）"
            ) from exc
        self._last_accel_actual = pulse_to_opening(monitor_accel.position_pulse)
        self._last_brake_actual = pulse_to_opening(monitor_brake.position_pulse)
        cycle_ms = 1000.0 * (time.perf_counter() - cycle_started)
        self.cycle_ms.append(cycle_ms)
        accel_current, brake_current = monitor_accel.current_ma, monitor_brake.current_ma
        alarm_accel, alarm_brake = await self.safety.poll_alarms(self.cycles, hw.accel, hw.brake)
        self._check_safety(
            t, now, ref, speed, accel_current, brake_current,
            accel_pos, brake_pos, alarm_accel, alarm_brake,
        )

        deviation = speed - ref
        self.max_abs_dev = max(self.max_abs_dev, abs(deviation))
        segment = cfg.modes.segment_at(t)
        if self.cycles % cfg.control.log_every_n_cycles == 0:
            data = DriveLogData(
                ref_speed_kmh=ref,
                actual_speed_kmh=speed,
                accel_opening=accel,
                brake_opening=brake,
                accel_pos=accel_pos,
                brake_pos=brake_pos,
                accel_current=accel_current,
                brake_current=brake_current,
                plan_effort_pct=ff_effort,
                trim_effort_pct=pid_effort,
                applied_effort_pct=effort,
                phase=phase,
            )
            self.samples.append(
                self.log.record(
                    data, section=SECTION_MODE_DRIVE, phase=phase, pattern=segment,
                    mode_time_s=t, governor_active=governed,
                    alarm_accel=alarm_accel, alarm_brake=alarm_brake,
                    accel_ff_pct=accel_ff, brake_ff_pct=brake_ff,
                    monitor_accel=monitor_accel, monitor_brake=monitor_brake, cycle_ms=cycle_ms,
                    candidate=self.ff.candidate, c10_effort_pct=c10_effort,
                    dob_accel_kmhs=None if self.dob is None else self.dob.d_accel,
                    dob_brake_kmhs=None if self.dob is None else self.dob.d_brake,
                    dob_effort_pct=self._dob_effort,
                    coast_effort_pct=None if self.coast is None else self._coast_effort,
                )
            )
        if segment != self._segment:
            self._segment = segment
            say(f"── 区間 {segment} ──")
        if t >= self._next_print:
            interval = cfg.output.print_interval_s  # 周期の端数で表示間隔が伸びないよう格子で数える
            self._next_print = (math.floor(t / interval) + 1) * interval
            gov = "  減速Gガバナー作動" if governed else ""
            say(drive_status_line(
                t_s=t, ref_kmh=ref, actual_kmh=speed,
                accel_kp=0.0, accel_ki=0.0, accel_kd=0.0,
                brake_kp=0.0, brake_ki=0.0, brake_kd=0.0,
                accel_pct=accel, brake_pct=brake,
                extra=(f"FF={ff_effort:+6.1f}% {segment} "
                       f"max|偏差|={self.max_abs_dev:.2f} "
                       f"{100.0 * t / self.duration_s:4.1f}%{gov}"),
            ))

    def _c10_offset(self, t: float, ref: float, speed: float, ff_effort: float) -> float:
        """C10: 今の列の effort に偏差の分を上乗せしたときの差（上乗せ後 − 今の列）[%]。
        偏差は今の周期の CAN 車速から（dev ＝ 基準 − 実車速）。表を引く車速の行は案A が基準、
        案B が実車速。惰行・停車保持・先置き・τ なしは 0。"""
        # fwd_plan → ff_bias → mode_report → mode_drive の循環を避けるため、ここで読む
        from tests.research.fwd_plan import deviation_offset_detail  # noqa: PLC0415

        ff = self.ff
        assert isinstance(ff, DeviationForwardFeedforward)
        assert self.c10 is not None
        plan = ff.plan
        i = plan.index_at(t)
        v = self.cfg.vehicle
        kwargs: dict[str, Any] = {
            "effort": ff_effort,
            "flag": int(plan.flag[i]),
            "need_kmhs": float(plan.need_accel_kmhs[i]),
            "ref_kmh": ref,
            "row_kmh": ref if ff.deviation_row == "reference" else speed,
            "dev_kmh": ref - speed,
            "tau_accel_s": ff.tau_accel_s,
            "tau_brake_s": ff.tau_brake_s,
            "accel_min_pct": v.min_accel_opening_pct,
            "brake_min_pct": v.min_brake_opening_pct,
            "stops_ahead": bool(plan.stops_ahead[i]) if plan.stops_ahead is not None else False,
        }
        dob = self.dob
        # C11: 押しているペダルの行にだけ、そのペダルの外乱の推定を引いて表を引く。
        # DOB の分は「P ＋ DOB の上乗せ − P だけの上乗せ」で別に出す（ログ・集計用）
        d_row = 0.0 if dob is None or ff_effort == 0.0 else dob.estimate(is_accel=ff_effort > 0.0)
        # 2026-10-18 段2q ③ `135125` の修正: 推定を更新しない低速の行（発進の前のブレーキを離す所
        # など）では使わない。そこで持ち越したブレーキの推定（−0.05〜−0.35）がブレーキを浅くし、
        # 発進で早く動き出した（アクセルを踏み始めた瞬間の偏差 +0.04〜+0.37 → +0.32〜+0.90）
        if dob is not None and self._dob_low_speed(speed, int(plan.flag[i])):
            d_row = 0.0
        new, clamped = deviation_offset_detail(ff.table, dob_kmhs=d_row, **kwargs)
        delta = new - ff_effort
        if dob is not None:
            p_only, _ = deviation_offset_detail(ff.table, **kwargs)
            self._dob_effort = (new - p_only) if ff_effort != 0.0 else 0.0
            if self.dob_stats is not None and ff_effort != 0.0:
                self.dob_stats.add_effort(is_accel=ff_effort > 0.0, effort_pct=self._dob_effort)
        if ff_effort != 0.0:
            self.c10.add(
                is_accel=ff_effort > 0.0, delta_pct=delta, clamped=clamped,
                prev_clamped=self._c10_prev_clamped,
            )
        self._c10_prev_clamped = clamped
        return delta

    def _coast_assist(self, t: float, ref: float, speed: float, ff_effort: float) -> float:
        """C12: 惰行の行の補助の開度 [%]（正＝アクセル・負＝ブレーキ。補助しなければ 0）。
        入る・離すは `CoastAssist.step`（毎周期呼ぶ）、踏む量は `fwd_plan.coast_assist_effort`。
        τ が None のペダル（遅れが測れない）は踏まない。DOB の分は
        「推定を引いた開度 − 引かない開度」を `_dob_effort` に入れる
        （低速の行では推定を使わない。C11 と同じ）。"""
        # fwd_plan → ff_bias → mode_report → mode_drive の循環を避けるため、ここで読む
        from tests.research.fwd_plan import coast_assist_effort  # noqa: PLC0415

        ff, coast, dob = self.ff, self.coast, self.dob
        assert isinstance(ff, CoastAssistFeedforward) and coast is not None and dob is not None
        plan = ff.plan
        i = plan.index_at(t)
        pedal = coast.step(t=t, i=i, dev_kmh=ref - speed, speed_kmh=speed, ref_kmh=ref)
        if pedal is None or ff_effort != 0.0:
            return 0.0
        is_accel = pedal == PEDAL_ACCEL
        tau = ff.tau_accel_s if is_accel else ff.tau_brake_s
        if tau is None:
            return 0.0
        v = self.cfg.vehicle
        kwargs: dict[str, Any] = {
            "is_accel": is_accel,
            "need_kmhs": float(plan.need_accel_kmhs[i]),
            "ref_kmh": ref,
            "row_kmh": ref if ff.deviation_row == "reference" else speed,
            "dev_kmh": ref - speed,
            "tau_s": tau,
            "accel_min_pct": v.min_accel_opening_pct,
            "brake_min_pct": v.min_brake_opening_pct,
            "stops_ahead": bool(plan.stops_ahead[i]) if plan.stops_ahead is not None else False,
        }
        if self.cfg.forward_map.coast_floor_onset:
            # 段2r ②: 補助中のそのペダルは手順2 の効き始め以上に置く（meta に無ければ下限なし）
            on = ff.table.meta.get("accel_onset_u_pct" if is_accel else "brake_onset_u_pct")
            kwargs["onset_u_pct"] = None if on is None else float(on)
        low = self._dob_low_speed(speed, int(plan.flag[i]))
        d = 0.0 if low else dob.estimate(is_accel=is_accel)
        with_dob = coast_assist_effort(ff.table, dob_kmhs=d, **kwargs)
        without = coast_assist_effort(ff.table, **kwargs) if d != 0.0 else with_dob
        self._dob_effort = with_dob - without
        self._coast_effort = with_dob
        coast.note_effort(with_dob)
        return with_dob

    def _dob_launch_top(self) -> float:
        """発進側の表が置き換えた車速の行の上の境界 [km/h]（無ければ 0）"""
        table = self.ff.table  # type: ignore[attr-defined]
        rows = table.meta.get("launch_speed_rows") or []
        if not rows:
            return 0.0
        return min((float(x) for x in table.speed_edges if x > max(rows)), default=0.0)

    def _dob_low_speed(self, speed: float, flag: int) -> bool:
        """C11 の推定を更新せず、使いもしない行か（停車保持の行・停車速度以下・発進側／停止側の表が
        置き換えた車速（launch_top と meta の stop_max_speed_kmh の大きいほう）未満）。"""
        from tests.research.ff_model import STOP_SPEED_KMH  # noqa: PLC0415
        from tests.research.fwd_plan import FLAG_STOP  # noqa: PLC0415

        meta = self.ff.table.meta  # type: ignore[attr-defined]
        low_top = max(self._dob_launch_top(), float(meta.get("stop_max_speed_kmh", 0.0) or 0.0))
        return flag == FLAG_STOP or speed <= STOP_SPEED_KMH or speed < low_top

    def _dob_update(
        self,
        t: float,
        ref: float,
        speed: float,
        accel: float,
        brake: float,
        governed: bool,
        coast_active: bool = False,
    ) -> None:
        """C11: 今の周期で出した開度（調停・ガバナー・安全網の後、待機位置の前）で推定器を進める。
        u ＝ 開度 − min（アクセル正・ブレーキ負・min 以下は 0）。発進側・停止側の行では更新しない
        （今の計画の行で `low_speed_sides` を引く。C10 の上乗せと同じ条件）。"""
        # fwd_plan → ff_bias → mode_report → mode_drive の循環を避けるため、ここで読む
        from tests.research.fwd_plan import low_speed_sides  # noqa: PLC0415

        ff, dob, v = self.ff, self.dob, self.cfg.vehicle
        assert isinstance(ff, DisturbanceObserverFeedforward)
        assert dob is not None and self.dob_stats is not None
        plan = ff.plan
        i = plan.index_at(t)
        ahead = bool(plan.stops_ahead[i]) if plan.stops_ahead is not None else False
        launch, stop = low_speed_sides(ff.table, float(plan.need_accel_kmhs[i]), ref, ahead)
        # 発進側は need > 0 の行すべてで True になる（表を引く車速の範囲は表の中で決まる）ので、
        # 止めるのは発進側の表が置き換えた車速の行（meta の launch_speed_rows）の中だけにする
        launch = bool(launch) and speed < self._dob_launch_top()
        # 2026-10-18 段2q ① `121218` の修正: 停車中（停車保持の深いブレーキ）は表が強い減速を
        # 予測し、実測は 0 なので、ブレーキの推定が +9 km/h/s まで積み上がって次の減速で踏み過ぎた。
        # 停車保持の行・停車速度以下・低速の表の車速の行では更新しない（`_dob_low_speed`）
        held = self._dob_low_speed(speed, int(plan.flag[i]))
        # 2026-10-18 段2r: 惰行の補助をしている周期も更新しない。浅い踏み込みは表がいちばん不確かな
        # 所なので、推定を FF の行から作ったままにする
        held = held or coast_active
        if accel > v.min_accel_opening_pct:
            u = accel - v.min_accel_opening_pct
        elif brake > v.min_brake_opening_pct:
            u = -(brake - v.min_brake_opening_pct)
        else:
            u = 0.0
        pedal = dob.update(
            t=t, speed_kmh=speed, u_now=u, governed=governed,
            at_limit_now=(
                accel >= v.max_accel_opening_pct or brake >= v.max_brake_opening_pct
            ),
            launch=bool(launch), stop=bool(stop) or held,
        )
        if pedal is not None:
            is_accel = pedal == PEDAL_ACCEL
            self.dob_stats.add_estimate(is_accel=is_accel, d=dob.estimate(is_accel=is_accel))

    def _apply_interlock(self, accel: float, brake: float) -> tuple[float, float]:
        """実開度の安全網（2026-10-17 段2o-9）: もう一方のペダルの実開度（前の周期のまとめ読み）が
        min ＋ INTERLOCK_TOL_PCT を超えている間は、出そうとしているペダルを 0（待機位置のまま）に
        する。アクセルでもブレーキでも同じ。止める上限時間は持たない（実位置が戻らないのは
        アクチュエータの故障で、axis_safety・偏差の停止が拾う）。切っているとき・実開度が未取得の
        ときは何もしない。"""
        stats = self.interlock
        if stats is None:
            return accel, brake
        v = self.cfg.vehicle
        block_accel = (
            accel > 0.0 and self._last_brake_actual is not None
            and self._last_brake_actual > v.min_brake_opening_pct + INTERLOCK_TOL_PCT
        )
        block_brake = (
            brake > 0.0 and self._last_accel_actual is not None
            and self._last_accel_actual > v.min_accel_opening_pct + INTERLOCK_TOL_PCT
        )
        if block_accel:
            stats.accel_cycles += 1
            stats.accel_events += 0 if self._accel_blocked else 1
            accel = 0.0
        if block_brake:
            stats.brake_cycles += 1
            stats.brake_events += 0 if self._brake_blocked else 1
            brake = 0.0
        self._accel_blocked, self._brake_blocked = block_accel, block_brake
        return accel, brake

    async def drive_axis(
        self, axis: ActuatorProtocol, pos: int, *, is_accel: bool
    ) -> AxisMonitor:
        """位置が変わった軸だけ指令し（本番 _drive_accel_axis と同じ）、まとめ読みする。

        A7: 以前は電流（CNOW）だけ読んでいた。0x9000 から 14 レジスタのまとめ読みで
        実位置・ステータスも同じ 1 回で取る。
        """
        last = self._accel_cmd if is_accel else self._brake_cmd
        if pos != last:
            await axis.move_to_position(pos, smooth_over_s=self.interval_s * AXIS_SMOOTH_DUTY)
            if is_accel:
                self._accel_cmd = pos
            else:
                self._brake_cmd = pos
            await asyncio.sleep(AXIS_PRE_READ_DELAY_S)
        return await axis.read_monitor()

    def _check_safety(
        self,
        t: float,
        now: float,
        ref: float,
        speed: float,
        accel_current: float,
        brake_current: float,
        accel_pos: int,
        brake_pos: int,
        alarm_accel: bool | None,
        alarm_brake: bool | None,
    ) -> None:
        v = self.cfg.vehicle
        for label, current in (("アクセル", accel_current), ("ブレーキ", brake_current)):
            if current > self.overcurrent_ma:
                raise DriveError(
                    f"{label}軸が過電流です"
                    f"（{current:.0f} mA > {self.overcurrent_ma:.0f} mA、t={t:.1f}s）"
                )
        # アラーム・電流ゼロの継続（アラームが出ないサーボ脱落を拾う安全網。axis_safety.py）
        reason = self.safety.check(
            t=t, accel_pos=accel_pos, brake_pos=brake_pos,
            accel_current=accel_current, brake_current=brake_current,
            alarm_accel=alarm_accel, alarm_brake=alarm_brake,
        )
        if reason is not None:
            raise DriveError(reason)
        if speed > v.max_speed_kmh:
            raise DriveError(
                f"車速 {speed:.1f} km/h が最高速 {v.max_speed_kmh:.1f} km/h を超えました"
            )
        if abs(speed - ref) > v.stop_deviation_threshold_kmh:
            if self._deviation_since is None:
                self._deviation_since = now
            if now - self._deviation_since >= v.stop_deviation_duration_s:
                raise DriveError(
                    f"偏差 {speed - ref:+.2f} km/h が {v.stop_deviation_threshold_kmh:g} km/h を"
                    f" {v.stop_deviation_duration_s:g}s 超え続けました（t={t:.1f}s）"
                )
        else:
            self._deviation_since = None


def cycle_time_text(cycle_ms: list[float], interval_s: float) -> str:
    """周期の処理時間 [ms] の平均 / p95 / 最大と、周期を超えた回数（A7 の関門）。"""
    ordered = sorted(cycle_ms)
    p95 = ordered[min(len(ordered) - 1, math.ceil(0.95 * len(ordered)) - 1)]
    period_ms = 1000.0 * interval_s
    over = sum(1 for ms in cycle_ms if ms > period_ms)
    return (f"平均 {sum(cycle_ms) / len(cycle_ms):.1f} / p95 {p95:.1f} / 最大 {ordered[-1]:.1f} ms"
            f"（周期 {period_ms:.0f}ms を超えた {over} 周期）")


async def run_mode_drive(
    hw: ResearchHardware,
    cfg: ResearchConfig,
    setup: ModeDriveSetup,
    *,
    log: SessionLog,
    limit_s: float | None = None,
) -> ModeDriveResult:
    """停車保持の状態からモードを走り、停車保持で終える。

    異常（CAN・通信・過電流・最高速超え・逸脱）は例外にせず、ペダルを離してから
    abort_reason 付きの結果を返す（途中までのレポートを作れるように）。
    `limit_s` はモードの先頭だけ走る動作確認用。
    """
    mode = setup.mode
    duration = mode.total_duration if limit_s is None else min(limit_s, mode.total_duration)
    limited = duration < mode.total_duration
    say(f"モード走行: {mode.name}（{duration:.0f}s"
        f"{f' ／ 動作確認のため先頭 {duration:.0f}s で打ち切り' if limited else ''}）")
    arb_features = enabled_arbiter_features(cfg.arbiter)
    say("制御構成: FF のみ（Kp=Ki=Kd=0）→ 調停は "
        + (" + ".join(arb_features) if arb_features else "符号で振り分けのみ"))
    gov = cfg.mode_drive
    say(f"減速G ガバナー（安全網）: {'有効' if gov.decel_governor else '無効'}"
        f"（{cfg.vehicle.max_decel_g:g}G × {GOVERNOR_LIMIT_FRAC:g} 以上で頭打ち）")
    say(f"ペダルの待機位置: {standby_label(cfg)}")
    if hw.is_real:
        say(f"*** 実機モード: 車両が最高 {mode.max_speed:.0f} km/h まで走ります。"
            "シャシダイナモ上で実施してください ***")

    run = _ModeRun(hw, cfg, setup, log, duration)
    await log.stop_sampler()  # 50ms の制御ループと Modbus を取り合わない（記録はループから）
    log.mark(SECTION_MODE_DRIVE, "")
    loop = asyncio.get_running_loop()
    started = loop.time()
    abort_reason = ""
    try:
        run_s = await run.run()
    except DriveError as exc:
        run_s = loop.time() - started
        abort_reason = str(exc)
        say(f"モード走行を中断しました: {exc}")
        await _release_pedals(hw)
    result = ModeDriveResult(
        mode_name=mode.name,
        mode_duration_s=duration,
        run_duration_s=run_s,
        completed=not abort_reason,
        abort_reason=abort_reason,
        cycles=run.cycles,
        overruns=run.overruns,
        governor_cycles=run.governor_cycles,
        interlock=run.interlock,
        c10_settings=None if run.c10 is None else c10_settings_text(run.ff),
        c10=run.c10,
        dob_settings=None if run.dob is None else dob_settings_text(run.ff),
        dob=run.dob_stats,
        coast_settings=run.coast_settings,
        coast_stats=None if run.coast is None else run.coast.stats,
        samples=run.samples,
    )
    say(f"モード走行 {'完了' if result.completed else '中断'}: {run_s:.1f}s・{run.cycles} 周期"
        f"（1 周期以上の遅れ {run.overruns} 回・減速G ガバナー作動 {run.governor_cycles} 周期）"
        f"・最大 |偏差| {run.max_abs_dev:.2f} km/h")
    if run.interlock is not None:
        say(f"  実開度の安全網（両踏みを防ぐために次のペダルを止めた）: {run.interlock.summary()}")
    else:
        say("  実開度の安全網: 無効（mode_drive.pedal_interlock_enabled: false）")
    if run.c10 is not None:
        say(f"  C10（偏差の上乗せ。{c10_settings_text(run.ff)}）: {run.c10.summary()}")
    if run.dob_stats is not None:
        say(f"  C11（DOB。{dob_settings_text(run.ff)}）: {run.dob_stats.summary()}")
    if run.coast is not None:
        say(f"  C12（惰行の補助。{run.coast_settings}）: {run.coast.stats.summary()}")
    if run.cycle_ms:
        say(f"  1 周期の処理時間: {cycle_time_text(run.cycle_ms, run.interval_s)}")
    if result.completed:
        await _finish_stopped(hw, cfg, log, run)
    return result


async def _finish_stopped(
    hw: ResearchHardware, cfg: ResearchConfig, log: SessionLog, run: _ModeRun
) -> None:
    """モード終わりで停車保持にする。

    止まっていなければ緩減速で止める（--limit-s で打ち切ったとき）。
    """
    try:
        speed = await hw.can.read_speed()
    except Exception as exc:
        await _release_pedals(hw)
        raise DriveError(
            f"モード終了時に CAN 車速を読めません（{type(exc).__name__}: {exc}）"
        ) from exc
    profile = build_vehicle_profile(cfg)
    if speed >= VEHICLE_STOP_SPEED_KMH:
        say(f"モード終了時の車速 {speed:.2f} km/h: 緩減速で停車させます …")
        log.mark(SECTION_DECEL_TO_STOP, PHASE_APPROACH)
        log.start_sampler()
        stop = await decelerate_to_stop(hw, cfg, profile, log=log)
        say(f"停車保持: ブレーキ {stop.hold_pct:.2f}%")
        return
    hold_pct = cfg.stop_brake_opening_pct  # min ＋ offset（max で頭打ち。段2l-7）
    log.mark(SECTION_DECEL_TO_STOP, PHASE_STOP_HOLD)
    await run.drive_axis(hw.accel, 0, is_accel=True)
    await run.drive_axis(hw.brake, opening_to_pulse(hold_pct), is_accel=False)
    log.start_sampler()
    say(f"停車しています（{speed:.2f} km/h）。停車保持: ブレーキ {hold_pct:.2f}%")
