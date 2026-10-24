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

import numpy as np

from tests.research.axis_monitor import AxisMonitor
from tests.research.axis_safety import AxisSafetyNet
from tests.research.config import ConfigError, ResearchConfig
from tests.research.creep_floor import CreepFloor, StopArrivals, creep_floor_opening_pct
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
    CreepFloorFeedforward,
    DeviationForwardFeedforward,
    DisturbanceObserverFeedforward,
    ForwardMapFeedforward,
    PredictorFeedforward,
    StopEndFeedforward,
    make_candidate,
)
from tests.research.ff_params import research_ff_params
from tests.research.hardware import ActuatorProtocol, DriveError, ResearchHardware
from tests.research.launch_hold import LaunchMoveHold, launch_hold_gain
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
from tests.research.speed_predictor import (
    PredictorStats,
    SpeedFilterPredictor,
    accel_error_var_from_meta,
)
from tests.research.stop_decel import PHASE_APPROACH, PHASE_STOP_HOLD, decelerate_to_stop
from tests.research.stop_end import StopEndP, StopEndStats, StopLeadStats, cap_stop_end_tau
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
    # 2026-10-19 段3: 開度の列を作ったときの「目標の基準」（ぼかした基準。σ=0 なら元の基準）。
    # 走行中の目標側（C10 の P など）が同じものを使えるよう持ち回る。表の候補以外は None
    target_ref: Any = None


async def load_mode(cfg: ResearchConfig, name: str) -> DrivingMode:
    """走行モードを名前で読む。modes.composed_modes にある名前なら、DB の元のモードを読んで
    区間を切り出してつないだ合成モードを返す（2026-10-19 段3b。DB は変えない）。無ければ DB から。
    """
    composed = next((c for c in cfg.modes.composed_modes if c.get("name") == name), None)
    if composed is None:
        return await _load_mode_from_db(cfg, name)
    source = await _load_mode_from_db(cfg, str(composed["source"]))
    return compose_mode(source, composed["windows_s"], name)


async def _load_mode_from_db(cfg: ResearchConfig, name: str) -> DrivingMode:
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


def compose_mode(
    source: DrivingMode,
    windows_s: list[list[float]],
    name: str,
    *,
    stop_kmh: float = 0.5,
) -> DrivingMode:
    """元のモードの基準車速を区間 [開始秒, 終了秒] ごとに切り出し、時刻をつないで 1 本にする。

    2026-10-19 段3b: モード別の表の外れの偏りを見るための短縮版（例 12_AMA → 12_AMA_short）。
    区間の端の基準は線形補間で作る。つなぎ目で車速が飛ばないよう、各区間の両端の基準が
    stop_kmh 以下（停車の途中）でなければ ConfigError にする。つなぎ目の 2 点は時刻が重なるので
    後のほうを捨てる。時刻は 0 から始め、区間の幅の合計が総時間になる。
    """
    if not windows_s:
        raise ConfigError(f"合成モード {name!r}: windows_s が空です")
    ref = ReferenceSpeed(source)
    points: list[SpeedPoint] = []
    offset = 0.0
    prev_end = -1.0
    for a, b in ((float(w[0]), float(w[1])) for w in windows_s):
        if not (0.0 <= a < b <= source.total_duration + 1e-9) or a < prev_end:
            raise ConfigError(
                f"合成モード {name!r}: 区間 [{a:g}, {b:g}] が不正です（開始 < 終了・昇順・"
                f"元のモード {source.name!r} の長さ {source.total_duration:g}s 以内）"
            )
        for edge in (a, b):
            if ref.at(edge) > stop_kmh:
                raise ConfigError(
                    f"合成モード {name!r}: 区間の端 {edge:g}s の基準が {ref.at(edge):.1f} km/h です"
                    f"（停車の途中 {stop_kmh:g} km/h 以下で切ってください）"
                )
        inner = [(p.time_s, p.speed_kmh) for p in source.reference_speed if a < p.time_s < b]
        for t, v in [(a, ref.at(a)), *inner, (b, ref.at(b))]:
            t_out = t - a + offset
            if points and t_out <= points[-1].time_s + 1e-9:
                continue  # つなぎ目の重なった点（どちらも停車）
            points.append(SpeedPoint(time_s=t_out, speed_kmh=v))
        offset += b - a
        prev_end = b
    return DrivingMode(
        id=f"composed:{name}",
        name=name,
        description=f"{source.name} から {len(windows_s)} 区間を切り出してつないだ合成モード"
                    "（研究コード側。DB は変えない）",
        reference_speed=points,
        total_duration=points[-1].time_s,
        max_speed=max(p.speed_kmh for p in points),
        created_at=source.created_at,
        is_system=False,
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
        # 2026-10-19 段2s-2: FEL（補正表 c）。有効で pkl に補正表があるときだけ f + c の表にする
        if cfg.fel.enabled:
            ff.apply_fel(cfg.fel.method)
        # 段5-6 追加 A2: 発進側の表の動き出す開度の床（既定 false ＝ 今と同じ）
        ff.apply_launch_breakaway_floor(cfg.forward_map.launch_table_breakaway_floor)
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


def fel_text(cfg: ResearchConfig, table: Any) -> str:
    """prepare の表示 1 行: FEL の有効無効・方法・補正のあるマス数・学習 CSV（段2s-2）"""
    info = table.meta.get("fel") or {}
    src = info.get("source_csv", "なし")
    if not cfg.fel.enabled:
        has = f"あり {src}" if info else "なし"
        return f"  FEL: 無効（fel.enabled=false。pkl の補正表: {has}）"
    if table.meta.get("fel_applied") != cfg.fel.method:
        return f"  FEL: 有効だが pkl に補正表なし（補正しない。方法 {cfg.fel.method}）"
    cells = (info.get("cells_with_correction") or {}).get(cfg.fel.method, "?")
    return f"  FEL: 有効・方法 {cfg.fel.method}・補正のあるマス {cells}・学習 CSV {src}"


def forward_map_plan_args(
    cfg: ResearchConfig, mode: DrivingMode, *, stop_end: bool
) -> tuple[Any, float, dict[str, Any]]:
    """手順3 の開度の列（fwd_plan.build_plan）に渡す引数: (基準 ref_at, 長さ [s], キーワード引数)。

    2026-10-23: 手順2 のチェック D1（step2_check.check_plan）と同じ作り方にするため切り出した。
    stop_end は FF 候補が C15（StopEndFeedforward）か（True のときだけ stop_lead_enabled を掛ける）
    """
    ff_cfg, fm = cfg.feedforward, cfg.forward_map
    # 2026-10-19 段3: 開度の列は「ぼかした基準」で作る（σ=0 なら元の基準そのもの）。停車の判定
    # （停車保持・停止側の表）だけは元の基準のまま（stop_ref_at）
    orig_ref = ReferenceSpeed(mode)
    target_ref = make_target_reference(mode, ff_cfg.ref_smooth_sigma_s)
    kwargs: dict[str, Any] = dict(
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
        stop_ref_at=None if target_ref is orig_ref else orig_ref.at,
        # 2026-10-20 段4-2 停A: C15 で stop_lead_enabled のとき、停車保持へ遅れ分早く入る
        stop_lead=stop_end and fm.stop_lead_enabled,
        low_speed_side_rule=fm.low_speed_side_rule,  # 段5-2
        release_table_min_kmh=fm.release_table_min_kmh,  # 段5-3
        switch_lead=fm.switch_lead_enabled,  # 段5-5c
        launch_midway_brake_cap=fm.launch_midway_brake_cap,  # 段5-6 追加 A1
        launch_release_ramp=fm.launch_release_ramp,  # 段7-b
        launch_release_lead_s=fm.launch_release_lead_s,
        launch_release_pre_pct=fm.launch_release_pre_pct,
        launch_short_stop_s=fm.launch_short_stop_s,
        switch_gap_compensate=fm.switch_gap_compensate,  # 段7-c
        switch_gap_min_loss=fm.switch_gap_min_loss,  # 段9-b
        accel_release_in_range_only=fm.accel_release_in_range_only,  # 段9-c
    )
    return target_ref.at, mode.total_duration, kwargs


def candidate_is_stop_end(candidate: str) -> bool:
    """FF 候補の名前が StopEndFeedforward（C15）か"""
    return isinstance(make_candidate(candidate), StopEndFeedforward)


def apply_table_options(cfg: ResearchConfig, table: Any) -> Any:
    """load_model の後に mode_drive が表へ掛ける設定（FEL の補正・A2 の床）を、素の表にも掛ける。
    D1 が手順3 と同じ表で計画を作るために使う。"""
    if cfg.fel.enabled and table.has_fel(cfg.fel.method):
        table = table.with_fel(cfg.fel.method)
    if cfg.forward_map.launch_table_breakaway_floor != table.launch_breakaway_floor:
        table = table.with_launch_breakaway_floor(cfg.forward_map.launch_table_breakaway_floor)
    return table


def make_launch_hold(
    cfg: ResearchConfig, ff: ForwardMapFeedforward, plan: Any
) -> tuple[LaunchMoveHold | None, str]:
    """段8-a: 発進で動き出したら離しを止めて待つ仕組み（forward_map.launch_move_hold）と、設定の
    表示文。働かないとき（off・launch_prep なし・k ≦ 0 など）は (None, 理由)。
    τ ＝ deviation_c × brake_lag_s（P と同じ。yaml の brake_lag_s、なければ pkl の meta）。"""
    fm = cfg.forward_map
    if not fm.launch_move_hold:
        return None, "なし"
    if plan.launch_prep is None:
        return None, "あり（設定）だが働かない: 計画に launch_prep が無い（ramp が off）"
    meta = ff.table.meta
    lag_b = fm.brake_lag_s if fm.brake_lag_s is not None else meta.get("brake_lag_s")
    tau = None if lag_b is None else fm.deviation_c * float(lag_b)
    ki, _k, note = launch_hold_gain(ff.table, tau_brake_s=tau)
    if ki is None:
        return None, f"あり（設定）だが働かない: {note}"
    bk = cfg.vehicle.min_brake_opening_pct + abs(float(meta["launch_breakaway_u_pct"]))
    hold = LaunchMoveHold(
        ki=ki, bk_pct=bk, brake_min_pct=cfg.vehicle.min_brake_opening_pct,
        dt_s=cfg.control.loop_interval_s,
    )
    return hold, f"あり・{note}・bk ＝ {bk:.2f}%"


def _prepare_forward_map_plan(
    cfg: ResearchConfig, mode: DrivingMode, ff: ForwardMapFeedforward
) -> Any:
    """C7・C8: 基準車速から開度の列を作り、中身を表示する（走行前チェックより前。HW 不使用）。"""
    ff_cfg, fm = cfg.feedforward, cfg.forward_map
    ref_at, duration_s, plan_kwargs = forward_map_plan_args(
        cfg, mode, stop_end=isinstance(ff, StopEndFeedforward)
    )
    plan = ff.build_plan(ref_at, duration_s, **plan_kwargs)
    target_ref = make_target_reference(mode, ff_cfg.ref_smooth_sigma_s)
    meta = ff.table.meta
    # 使った遅れ（None なら fwd_plan と同じく pkl の meta の値）
    lag_a = fm.accel_lag_s if fm.accel_lag_s is not None else meta.get("accel_lag_s")
    lag_b = fm.brake_lag_s if fm.brake_lag_s is not None else meta.get("brake_lag_s")
    say(f"FF 候補: {ff.candidate}（順モデルの表）　表: {ff_cfg.model_path}")
    say(f"  {ref_smooth_text(ff_cfg.ref_smooth_sigma_s)}")
    say(fel_text(cfg, ff.table))
    say(f"  遅れ アクセル {lag_a}s / ブレーキ {lag_b}s・惰行の帯 ±{fm.coast_band_kmhs:g} km/h/s・"
        f"窓 {fm.need_window_s:g}s")
    say(f"  低速の表の引き方（段5-2）: 設定 {fm.low_speed_side_rule}・計画に使った規則 "
        f"{plan.side_rule}（pedal・dir でも表が発進側・停止側の両方を持たなければ legacy）")
    say("  発進の途中は FB もブレーキを深くしない（段6 C）: "
        f"{'あり' if fm.launch_midway_fb_no_deeper else 'なし'}")
    say(f"  切り替えの先読み（段5-5c）: {'あり' if fm.switch_lead_enabled else 'なし'}")
    say(f"  発進のブレーキを基準の動き出しに合わせて離す（段7-b）: "
        f"{'あり' if fm.launch_release_ramp else 'なし'}")
    if isinstance(ff, ForwardMapFeedforward):
        say(f"  動き出したら離しを止めて待つ（段8-a）: {make_launch_hold(cfg, ff, plan)[1]}")
    say("  P・惰行の補助の表から A2 を外す（段8-b）: "
        f"{'あり' if fm.fb_no_breakaway_floor else 'なし'}")
    say(f"  すき間で失う分の補償（段7-c）: {'あり' if fm.switch_gap_compensate else 'なし'}")
    say("  すき間を前後の損の合計が最小で取る（段9-b）: "
        f"{'あり' if fm.switch_gap_min_loss else 'なし'}")
    say(
        "  アクセルの戻しの層を学習の範囲の中だけで使う（段9-c）: "
        f"{'あり' if fm.accel_release_in_range_only else 'なし'}"
    )
    say(f"  戻しの表を使う基準車速の下限（段5-3）: {fm.release_table_min_kmh:g} km/h"
        "（0 ＝ 全部の車速で使う）")
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
            fast_enabled=fm.dob_fast_enabled, fast_tq_coef=fm.dob_fast_tq_coef,
            fast_need_kmhs=fm.dob_fast_need_kmhs,
        )
        say(f"  C11（DOB）: {dob_settings_text(ff)}")
        say(f"  低速の DOB・停C の τ（段6-2）: {dob_low_speed_text(fm)}")
    if isinstance(ff, CoastAssistFeedforward):
        # 2026-10-18 段2r: 惰行の補助。保持時間 ＝ 係数 × (遅れ ＋ 効き始めまでの戻り時間)
        ff.set_coast(
            on_kmh=fm.coast_on_kmh, off_kmh=fm.coast_off_kmh, hold_coef=fm.coast_hold_coef
        )
        say(f"  C12（惰行の補助）: {coast_settings_of(ff, cfg)}")
    if isinstance(ff, PredictorFeedforward):
        # 2026-10-19 段2u: 遅れの予測器 ＋ 車速のフィルタ。Q は pkl の meta、R は最初の停車保持で
        # 決まる（走り出してから）ので、ここでは yaml の値だけ表示する
        ff.set_predictor(
            enabled=fm.predictor_enabled, q=fm.speed_filter_q, r=fm.speed_filter_r,
            r_floor=fm.speed_filter_r_floor, accel_sigma=fm.speed_filter_accel_sigma,
        )
        say(f"  C13（予測器・車速のフィルタ）: {predictor_config_text(ff)}")
    if isinstance(ff, CreepFloorFeedforward):
        # 2026-10-19 段3a: 停車の最後のブレーキの下限。値は pkl の meta（手順2 の測定）から
        ff.set_creep_floor(lead_s=fm.creep_floor_lead_s)
        cf_text = make_creep_floor(ff, cfg, mode).settings_text()
        say(f"  C14（停車の最後のブレーキの下限）: {cf_text}")
    if isinstance(ff, StopEndFeedforward):
        # 2026-10-20 段4-2: 停A（停車保持を早める）・停C（停車まぎわの P）。値は pkl の meta から
        ff.set_stop_end(
            lead_enabled=fm.stop_lead_enabled, p_enabled=fm.stop_end_p_enabled,
            lead_s=fm.creep_floor_lead_s,
        )
        lead_rows = 0 if plan.stop_lead is None else int(plan.stop_lead.sum())
        say(f"  C15 停A（停車保持をブレーキの遅れ {float(lag_b or 0.0):.3f}s 早める）: "
            f"{'有効' if fm.stop_lead_enabled else '無効'}・早めた行 {lead_rows} 行"
            f"（{lead_rows * plan.period_s:.1f}s）")
        se_text = make_stop_end(ff, cfg, mode).settings_text()
        say(f"  C15 停C（停車まぎわの P）: "
            f"{'有効' if fm.stop_end_p_enabled else '無効'}・{se_text}")
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
    return target_ref


async def prepare_mode_drive(cfg: ResearchConfig, mode_name: str) -> ModeDriveSetup:
    say(f"走行モードを読み込みます: {mode_name!r}（{cfg.hardware.database_url}）")
    mode = await load_mode(cfg, mode_name)
    return prepare_mode_drive_from(cfg, mode)


def prepare_mode_drive_from(cfg: ResearchConfig, mode: DrivingMode) -> ModeDriveSetup:
    """DB を引かずに、渡された DrivingMode で走行の準備（FF モデルの読み込みと開度の列）をする。

    2026-10-19 段2s-2b: FEL の学習用パターン（fel_pattern.py が作る DrivingMode）を同じ流れで
    走るために `prepare_mode_drive` から切り出した。
    """
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
    if ff_cfg.ref_smooth_sigma_s > 0.0 and not isinstance(ff, ForwardMapFeedforward):
        say(f"  注: feedforward.ref_smooth_sigma_s={ff_cfg.ref_smooth_sigma_s:g} は表の候補"
            "（C7〜C15）だけに効きます。この候補では基準をぼかしません")
    if isinstance(ff, ForwardMapFeedforward):
        target_ref = _prepare_forward_map_plan(cfg, mode, ff)
        return ModeDriveSetup(mode=mode, ff=ff, target_ref=target_ref)
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


def ref_smooth_text(sigma_s: float) -> str:
    """基準のぼかし（feedforward.ref_smooth_sigma_s）の 1 行表示（走行前の表示・レポート 5 章）。"""
    if sigma_s <= 0.0:
        return "基準のぼかし: なし（σ=0。目標も元の基準）"
    return (f"基準のぼかし: σ={sigma_s:g}s のガウス（開度の列・C10 の P・DOB の表の行・"
            "C12 の惰行の補助・C13 の予測の偏差の目標だけ。KPI・偏差の記録・停車の判定・"
            "ガバナー・安全網・調停は元の基準）")


class SmoothedReference:
    """基準車速をガウス（標準偏差 σ 秒）で畳み込んだ「目標の基準」（2026-10-19 段3・G6）。

    左右対称なので遅れを足さない。走行前に 0.01s 刻みで全体を作り、`at` は線形補間で引く。
    停車の扱い: 元の基準が 0 の区間（停車）では、ぼかした基準も元の基準（0）に戻す。そうしないと
    停車の手前で基準が 0 より大きいまま長く残り、停車の始まり・終わりがずれて見える。
    戻した端では、ぼかした値との間に最大 約 0.4σ × 傾き の段差が出る（σ 0.5s・傾き 1.5 km/h/s
    で 0.3 km/h 程度。停車の中は停車保持の行なので目標としては使わない）。
    範囲外は元の基準と同じく端点値。
    """

    GRID_S = 0.01  # 畳み込みの刻み [s]
    KERNEL_SPAN_SIGMA = 4.0  # カーネルの半幅 [σ]
    ZERO_KMH = 1e-6  # 元の基準がこれ以下の時刻は停車（0）として戻す

    def __init__(self, mode: DrivingMode, sigma_s: float) -> None:
        if sigma_s <= 0.0:
            raise ValueError("sigma_s は正値（0 ならぼかさない = ReferenceSpeed を使う）")
        self.sigma_s = sigma_s
        times = np.array([p.time_s for p in mode.reference_speed], dtype=float)
        speeds = np.array([p.speed_kmh for p in mode.reference_speed], dtype=float)
        half = int(math.ceil(self.KERNEL_SPAN_SIGMA * sigma_s / self.GRID_S))
        n = int(round((times[-1] - times[0]) / self.GRID_S)) + 1
        grid = times[0] + (np.arange(n + 2 * half) - half) * self.GRID_S
        orig = np.interp(grid, times, speeds)  # 範囲外は端点値（ReferenceSpeed と同じ）
        x = np.arange(-half, half + 1) * self.GRID_S
        kernel = np.exp(-0.5 * (x / sigma_s) ** 2)
        kernel /= kernel.sum()
        blurred = np.convolve(orig, kernel, mode="valid")  # 長さ n（grid[half:-half] の位置）
        inner = grid[half:half + n]
        orig_inner = orig[half:half + n]
        self._grid = inner
        self._vals = np.where(orig_inner <= self.ZERO_KMH, orig_inner, blurred)

    def at(self, t_s: float) -> float:
        return float(np.interp(t_s, self._grid, self._vals))


def make_target_reference(mode: DrivingMode, sigma_s: float) -> ReferenceSpeed | SmoothedReference:
    """目標側の基準。σ が 0 以下なら元の基準（ReferenceSpeed）をそのまま返す（今と完全に同じ）。"""
    return ReferenceSpeed(mode) if sigma_s <= 0.0 else SmoothedReference(mode, sigma_s)


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


DOB_FADE_START_KMH = 0.5  # 段6-2 (a): 薄める重みが 0 になる車速 [km/h]（fwd_data の停車速度と同値）


def dob_fade_weight(v_kmh: float, stop_kmh: float, top_kmh: float) -> float:
    """DOB の上乗せの重み（2026-10-23 段6-2 (a)）: clip((v − 0.5) / (境目 − 0.5), 0, 1)。
    境目が停車速度以下なら、停車速度を超えれば 1。"""
    if top_kmh <= stop_kmh:
        return 1.0 if v_kmh > stop_kmh else 0.0
    return min(1.0, max(0.0, (v_kmh - stop_kmh) / (top_kmh - stop_kmh)))


def dob_low_speed_text(fm: Any) -> str:
    """段6-2 の設定の表示（走行前）。"""
    return (
        f"薄める {'あり' if fm.dob_low_speed_fade else 'なし'}・"
        f"境目と表を基準車速でそろえる {'あり' if fm.dob_low_speed_align else 'なし'}・"
        f"停C の τ を普段より弱くしない {'あり' if fm.stop_end_tau_cap_normal else 'なし'}"
    )


def dob_settings_text(ff: object) -> str:
    """C11 の設定の表示（走行前とレポートで同じ文）。C11 でなければ「なし」。"""
    if not isinstance(ff, DisturbanceObserverFeedforward):
        return "なし"

    def tq(v: float | None) -> str:
        return f"{v:.1f}s" if v is not None else "DOB なし（遅れが測れていない）"

    return (
        f"Tq 係数 {ff.dob_tq_coef:g}・"
        f"Tq アクセル {tq(ff.tq_accel_s)} / ブレーキ {tq(ff.tq_brake_s)}・"
        f"強い加速・減速の間だけ速くする（段9-d）: "
        + (
            f"あり（Tq 係数 {ff.dob_fast_tq_coef:g}・要る加速度 {ff.dob_fast_need_kmhs:g} "
            "km/h/s 以上）"
            if ff.dob_fast_enabled else "なし"
        )
    )


def predictor_config_text(ff: object) -> str:
    """C13 の yaml の設定の表示（走行前）。C13 でなければ「なし」。Q・R の実値は走行後に出る。"""
    if not isinstance(ff, PredictorFeedforward):
        return "なし"
    if not ff.predictor_enabled:
        return "無効（forward_map.predictor_enabled: false。C12 と同じ指令）"
    q = "pkl の meta から" if ff.filter_q is None else f"yaml の上書き σ_a² {ff.filter_q:g}"
    r = "最初の停車保持で測る" if ff.filter_r is None else f"yaml の上書き {ff.filter_r:g}"
    return (
        f"有効・Q {q}（meta に無ければ σ_a {ff.filter_accel_sigma:g} km/h/s）"
        f"・R {r}（下限 {ff.filter_r_floor:g} (km/h)²）"
    )


def make_creep_floor(
    ff: CreepFloorFeedforward, cfg: ResearchConfig, mode: DrivingMode
) -> CreepFloor:
    """C14 の下限を作る。下限 ＝ ブレーキ min ＋ |meta の動き出す開度|（停車保持より深くしない）。
    しきい値 ＝ cfg.feedforward.creep_speed_kmh（手順2 の測定値）＋ meta の low_speed_margin_kmh
    （無ければ 0）。meta に動き出す開度が無ければ下限なし。"""
    meta = ff.table.meta
    margin = meta.get("low_speed_margin_kmh")
    margin = float(margin) if margin is not None and math.isfinite(float(margin)) else 0.0
    return CreepFloor(
        floor_pct=creep_floor_opening_pct(
            meta, cfg.vehicle.min_brake_opening_pct, cfg.stop_brake_opening_pct
        ),
        lead_s=ff.creep_floor_lead_s,
        speed_limit_kmh=cfg.feedforward.creep_speed_kmh + margin,
        arrivals=StopArrivals(
            [p.time_s for p in mode.reference_speed], [p.speed_kmh for p in mode.reference_speed]
        ),
        accel_min_pct=cfg.vehicle.min_accel_opening_pct,
        period_s=cfg.control.loop_interval_s,
    )


def make_stop_end(ff: StopEndFeedforward, cfg: ResearchConfig, mode: DrivingMode) -> StopEndP:
    """C15 の停C を作る。区間の上の車速 ＝ meta の stop_max_speed_kmh、τ の下限 ＝ ブレーキの遅れ
    （yaml の上書き、なければ meta の brake_lag_s）、開度の範囲 ＝ ブレーキ min 〜 停車保持。
    停C を使わない設定（stop_end_p_enabled が false）や meta に値が無いときは、区間なし
    （C12 と同じ指令）。"""
    meta = ff.table.meta
    lag = cfg.forward_map.brake_lag_s
    if lag is None:
        lag = meta.get("brake_lag_s")
    top = meta.get("stop_max_speed_kmh")
    return StopEndP(
        top_kmh=float(top) if ff.stop_end_p_enabled and top is not None else None,
        lead_s=ff.stop_end_lead_s,
        brake_lag_s=None if lag is None else float(lag),
        hold_pct=cfg.stop_brake_opening_pct,
        brake_min_pct=cfg.vehicle.min_brake_opening_pct,
        arrivals=StopArrivals(
            [p.time_s for p in mode.reference_speed], [p.speed_kmh for p in mode.reference_speed]
        ),
        period_s=cfg.control.loop_interval_s,
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
    # 2026-10-19 段2u: C13（予測器・車速のフィルタ）の設定（走行後の Q・R を含む）と集計。C13 で
    # ない・predictor_enabled が false なら None
    predictor_settings: str | None = None
    predictor: PredictorStats | None = None
    # 2026-10-19 段3a: C14（停車の最後のブレーキの下限）の設定と集計。C14 でなければ None
    creep_floor_settings: str | None = None
    creep_floor_stats: Any = None  # creep_floor.CreepFloorStats
    # 2026-10-20 段4-2: C15（停A・停C）の設定と集計。C15 でなければ None
    stop_end_settings: str | None = None
    stop_lead_stats: StopLeadStats | None = None
    stop_end_stats: StopEndStats | None = None
    samples: list[DriveSample] = field(default_factory=list)  # MODE_DRIVE の行（0.1s 刻み）


def cut_deeper_brake_fb(ff_effort: float, fb_effort: float) -> float:
    """段6 C: FB の上乗せがブレーキを FF の開度より深くする向き（FF ≤ 0 で FB < 0）なら 0。
    浅くする向き（FB > 0）・アクセル側（FF > 0）はそのまま返す。"""
    return 0.0 if ff_effort <= 0.0 and fb_effort < 0.0 else fb_effort


class _ModeRun:
    """1 本のモード走行の状態。run() が 50ms 周期でモード終わりまで回す。"""

    # C13（段2u）の今の周期の値の既定。C13 以外・予測器が無効のときは動かさない（None のまま）
    _speed_est: float | None = None
    _pred_dev: float | None = None
    # C15（段4-2）の既定。C15 以外は持たない（None のまま）
    stop_end: StopEndP | None = None
    stop_lead_stats: StopLeadStats | None = None
    _stop_lead_now: bool | None = None
    _stop_end_tau_s: float | None = None
    # 段8-a・段8-b の既定（__init__ を通らない最小の _ModeRun でも読めるように）
    launch_hold: LaunchMoveHold | None = None
    _launch_hold_pct: float | None = None
    fb_no_floor: bool = False

    def __init__(
        self,
        hw: ResearchHardware,
        cfg: ResearchConfig,
        setup: ModeDriveSetup,
        log: SessionLog,
        duration_s: float,
        section: str = SECTION_MODE_DRIVE,
    ) -> None:
        self.hw = hw
        self.cfg = cfg
        self.section = section  # CSV の section 列（FEL の学習用パターンは別の名前）
        self.ff = setup.ff
        self.mode = setup.mode
        self.ref = ReferenceSpeed(setup.mode)  # 元の基準（KPI・偏差の記録・停車の判定・安全網）
        # 目標の基準（段3。開度の列と同じもの。σ=0 なら self.ref と同じ値）
        self.ref_t = (
            setup.target_ref if setup.target_ref is not None
            else make_target_reference(setup.mode, cfg.feedforward.ref_smooth_sigma_s)
        )
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
                tq_fast_accel_s=self.ff.tq_fast_accel_s, tq_fast_brake_s=self.ff.tq_fast_brake_s,
                fast_need_kmhs=self.ff.dob_fast_need_kmhs,
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
        # C13（予測器・車速のフィルタ。2026-10-19 段2u）: C13 で predictor_enabled のときだけ
        self.pred: SpeedFilterPredictor | None = None
        self._speed_est: float | None = None  # 今の周期のフィルタの推定車速（DOB の実加速度に使う）
        self._speed_pred: float | None = None  # 今の周期の L 秒先の予測（使わない周期は None）
        # 今の周期の予測の偏差（基準の L 秒先 − 予測。使わなければ None）
        self._pred_dev: float | None = None
        self._pred_row_dev: float | None = None  # ログ用: P の上乗せに使った偏差
        self._first_hold = True  # 走行の最初の停車保持の間か（フィルタの R を測る）
        if isinstance(self.ff, PredictorFeedforward) and self.ff.predictor_enabled:
            assert self.dob is not None
            meta = self.ff.table.meta
            if self.ff.filter_q is not None:
                accel_var, note = self.ff.filter_q, f"yaml の上書き σ_a² {self.ff.filter_q:g}"
            else:
                accel_var, note = accel_error_var_from_meta(meta, self.ff.filter_accel_sigma)
            coast_lag = meta.get("coast_lag_s")
            self.pred = SpeedFilterPredictor(
                self.dob, period_s=self.interval_s, accel_var=accel_var,
                r_override=self.ff.filter_r, r_floor=self.ff.filter_r_floor,
                lag_accel_s=self.ff.dob_lag_accel_s, lag_brake_s=self.ff.dob_lag_brake_s,
                coast_lag_s=self.ff.dob_lag_accel_s if coast_lag is None else float(coast_lag),
                accel_var_note=note,
            )
        # C14（停車の最後のブレーキの下限。2026-10-19 段3a）: C14 のときだけ
        self.cfloor: CreepFloor | None = (
            make_creep_floor(self.ff, cfg, setup.mode)
            if isinstance(self.ff, CreepFloorFeedforward) else None
        )
        # C15（停A・停C。2026-10-20 段4-2）: C15 のときだけ
        self.stop_end: StopEndP | None = None
        self.stop_lead_stats: StopLeadStats | None = None
        self._stop_lead_now: bool | None = None  # 今の周期が停A の行か（C15 以外は None）
        self._stop_end_tau_s: float | None = None  # 今の周期の停C の τ（区間外は None）
        if isinstance(self.ff, StopEndFeedforward):
            self.stop_end = make_stop_end(self.ff, cfg, setup.mode)
            self.stop_lead_stats = StopLeadStats(period_s=self.interval_s)
        # 段6 C: 発進の途中は FB もブレーキを深くしない（設定が true のときだけ働く）
        self.midway_fb_no_deeper = bool(cfg.forward_map.launch_midway_fb_no_deeper)
        self.midway_fb_cut_cycles = 0  # 抑えた周期数
        self.midway_fb_cut_pct_sum = 0.0  # 抑えた量（ブレーキの深さ [%]）の合計
        # 段8-a: 基準 0 で動き出したら離しを止めて待つ（forward_map.launch_move_hold）
        self.launch_hold = None
        self._launch_hold_pct = None
        if isinstance(self.ff, ForwardMapFeedforward):
            self.launch_hold, _text = make_launch_hold(cfg, self.ff, self.ff.plan)
            if self.launch_hold is not None:
                self._launch_hold_pct = 0.0
        # 段8-b: P・惰行の補助の表の引き直しだけ A2 を外す（forward_map.fb_no_breakaway_floor）
        self.fb_no_floor = bool(cfg.forward_map.fb_no_breakaway_floor)
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
        ref_t = self.ref_t.at(t)  # 目標側の基準（ぼかし。σ=0 なら ref と同じ）
        if isinstance(self.ff, ForwardMapFeedforward):
            # C7・C8: 開度の列を時刻で引く（走る前に作り終えている。実測車速は見ない）
            ff_effort = self.ff.effort_at(t)
        else:
            v0, future, past = self._ff_inputs(t, ref, speed)
            ff_effort = self.ff.predict_effort(v0, future, past, **self._select_kwargs(t, v0))
        if self.pred is not None:
            # C13: 車速のフィルタを進め、L 秒先の予測から P の上乗せに使う偏差を決める
            self._predictor_step(t, ref_t, speed, ff_effort)
        c10_effort: float | None = None
        self._stop_end_tau_s = None
        if self.stop_lead_stats is not None:
            # C15 停A: 計画が、停車保持へ遅れ分早く入れた行か
            lead = self.ff.plan.stop_lead
            self._stop_lead_now = bool(lead is not None and lead[self.ff.plan.index_at(t)])
            self.stop_lead_stats.add(self._stop_lead_now)
        if self.c10 is not None:
            c10_effort = self._c10_offset(t, ref_t, speed, ff_effort)
        coast_active = False
        if self.coast is not None:
            # C12: 惰行の行（計画が 0）で偏差が積み上がったら、浅く踏んで詰める
            self._coast_effort = 0.0
            coast_effort = self._coast_assist(t, ref, ref_t, speed, ff_effort)
            if coast_effort != 0.0:
                c10_effort, coast_active = coast_effort, True
        if self.midway_fb_no_deeper and c10_effort and isinstance(self.ff, ForwardMapFeedforward):
            plan = self.ff.plan
            if plan.is_launch_midway(plan.index_at(t)):
                cut = cut_deeper_brake_fb(ff_effort, c10_effort)
                if cut != c10_effort:
                    self.midway_fb_cut_cycles += 1
                    self.midway_fb_cut_pct_sum += -c10_effort
                    c10_effort = cut
        pid_effort = 0.0  # 手順 5 以降でここに PID の出力を足す
        hold_delta = self._launch_hold_step(t, speed, ff_effort)  # 段8-a（off なら 0）
        # C10 の上乗せは FF の列（accel_ff・brake_ff・plan_effort）には入れない（作ったずれが
        # 今までと同じ意味で比べられるように）。実際に出す effort にだけ足す
        effort = ff_effort + (c10_effort or 0.0) + pid_effort + hold_delta
        ff_shown = ff_effort  # ログの FF 列に出す値（C14 の下限を掛けたもの）
        floor_raise: float | None = None
        if self.cfloor is not None:
            # C14: 最終の指令（FB の上乗せ・惰行の補助を足した後、調停・安全網の前）に下限を掛ける。
            # FF 列にも同じ条件で掛ける（plan_effort_pct は掛ける前の FF のまま）
            args = (t, ref, speed)
            ff_shown = self.cfloor.apply(
                *args, ff_effort, self._last_accel_actual, record=False
            )
            floored = self.cfloor.apply(*args, effort, self._last_accel_actual)
            floor_raise = effort - floored
            effort = floored
        # ログの「指示開度 FF」列（FF の出力をペダル別に分けた値。ガバナー・待機位置の前）
        accel_ff, brake_ff = split_effort(
            ff_shown, cfg.vehicle.max_accel_opening_pct, cfg.vehicle.max_brake_opening_pct
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
            self._dob_update(t, ref_t, speed, accel, brake, governed, coast_active)
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
                    data, section=self.section, phase=phase, pattern=segment,
                    mode_time_s=t, governor_active=governed,
                    alarm_accel=alarm_accel, alarm_brake=alarm_brake,
                    accel_ff_pct=accel_ff, brake_ff_pct=brake_ff,
                    monitor_accel=monitor_accel, monitor_brake=monitor_brake, cycle_ms=cycle_ms,
                    candidate=self.ff.candidate, c10_effort_pct=c10_effort,
                    dob_accel_kmhs=None if self.dob is None else self.dob.d_accel,
                    dob_brake_kmhs=None if self.dob is None else self.dob.d_brake,
                    dob_effort_pct=self._dob_effort,
                    coast_effort_pct=None if self.coast is None else self._coast_effort,
                    launch_hold_pct=self._launch_hold_pct,
                    speed_est_kmh=self._speed_est,
                    speed_pred_kmh=self._speed_pred,
                    pred_dev_kmh=self._pred_row_dev,
                    ref_smooth_kmh=ref_t,
                    creep_floor_pct=floor_raise,
                    stop_lead_active=self._stop_lead_now,
                    stop_end_tau_s=self._stop_end_tau_s,
                    ff_side_accel=self._ff_side_now(t, True),
                    ff_side_brake=self._ff_side_now(t, False),
                    ff_switch_lead_s=self._ff_switch_lead_now(t),
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

    def _launch_hold_step(self, t: float, speed: float, ff_effort: float) -> float:
        """段8-a: 発進のブレーキに足す effort [%]（≦ 0。ブレーキを深くする向き）。off は 0。
        `launch_hold_pct`（CSV）にはその深さを入れる。FB の P・DOB はこの行では有効にしない
        （launch_prep の行は FLAG_STOP で P は何もしない）。ブレーキの上限を超えない。"""
        hold = self.launch_hold
        if hold is None:
            return 0.0
        plan = self.ff.plan  # type: ignore[attr-defined]
        i = plan.index_at(t)
        midway = plan.is_launch_midway(i) if plan.launch_midway is not None else True
        added = hold.step(
            prep=bool(plan.launch_prep[i]), speed_kmh=speed, ff_effort=ff_effort, midway=midway
        )
        room = max(self.cfg.vehicle.max_brake_opening_pct + ff_effort, 0.0)
        added = min(added, room)  # ff_effort は負: 深さ上限 − |ff|
        self._launch_hold_pct = added
        return -added

    def _fb_table(self) -> Any:
        """段8-b: P・惰行の補助が表を引き直すときの表（A2 を外した表）。"""
        table = self.ff.table  # type: ignore[attr-defined]
        return table.with_launch_breakaway_floor(False) if self.fb_no_floor else table

    def _ff_side_now(self, t: float, is_accel: bool) -> int | None:
        """CSV 用（段5-2）: 今の計画の行が、そのペダル側で引いた表 SIDE_*（計画なしは None）。"""
        ff = self.ff
        plan = ff._plan if isinstance(ff, ForwardMapFeedforward) else None  # noqa: SLF001
        arr = None if plan is None else (plan.side_accel if is_accel else plan.side_brake)
        return None if arr is None else int(arr[plan.index_at(t)])

    def _ff_switch_lead_now(self, t: float) -> float | None:
        """CSV 用（段5-5c）: 今の計画の行で要る加速度を見る時刻を早めた秒数（なしは None）。"""
        ff = self.ff
        plan = ff._plan if isinstance(ff, ForwardMapFeedforward) else None  # noqa: SLF001
        arr = None if plan is None else plan.switch_lead_s
        return None if arr is None else float(arr[plan.index_at(t)])

    def _row_sides(
        self, plan: Any, i: int, ref: float, is_accel: bool
    ) -> tuple[bool, bool]:
        """今の計画の行 i で、そのペダルが引く表の (発進側か, 停止側か)。段5-2: 規則が
        "pedal"・"dir" の計画は、計画が行ごとに決めた表（plan.side_accel / side_brake）。
        "legacy" は今までどおり
        `low_speed_sides`（need ＞ 0 → 発進側、停車へ向かう減速 → 停止側）。"""
        # fwd_plan → ff_bias → mode_report → mode_drive の循環を避けるため、ここで読む
        from tests.research.fwd_plan import SIDE_LAUNCH, SIDE_STOP, low_speed_sides  # noqa: PLC0415

        side = plan.side_for(i, is_accel)
        if side is not None:
            return side == SIDE_LAUNCH, side == SIDE_STOP
        ahead = bool(plan.stops_ahead[i]) if plan.stops_ahead is not None else False
        launch, stop = low_speed_sides(
            self.ff.table, float(plan.need_accel_kmhs[i]), ref, ahead  # type: ignore[attr-defined]
        )
        return bool(launch), bool(stop)

    def _low_flags(
        self, t: float, ref: float, speed: float, ff_effort: float = 0.0
    ) -> tuple[bool, bool, bool]:
        """今の計画の行の (発進側か, 停止側か, 低速の行か)。`_dob_update` と同じ条件
        （発進側は発進側の表が置き換えた車速の行の中だけ。低速の行は `_dob_low_speed`）。"""
        ff = self.ff
        assert isinstance(ff, ForwardMapFeedforward)
        plan = ff.plan
        i = plan.index_at(t)
        launch, stop = self._row_sides(plan, i, ref, is_accel=ff_effort >= 0.0)
        launch = launch and speed < self._dob_launch_top()
        return launch, stop, self._dob_low_speed(speed, int(plan.flag[i]))

    def _predictor_step(self, t: float, ref: float, speed: float, ff_effort: float) -> None:
        """C13（段2u）: 車速のフィルタを 1 周期進め、L 秒先の予測と P の上乗せの偏差を決める。

        `ref` は目標の基準（ぼかした基準。2026-10-19 段3。σ=0 なら元の基準）。
        どこを予測に変えたか: C10 の P の上乗せの偏差だけ（dev ＝ 基準の L 秒先 − 予測）。
        低速の行（停車保持・停車速度以下・発進側／停止側の表の車速）は C11 の DOB を使わない行と同じ
        なので、予測も使わず今の偏差（基準 − CAN 車速）のまま。ペダルの遅れが測れない行も同じ。
        DOB の実加速度（`_dob_update`）にはここで出した推定車速を使う。惰行の補助・DOB の
        上乗せの量・安全網・ガバナー・調停は CAN 車速のまま変えない。"""
        # fwd_plan → ff_bias → mode_report → mode_drive の循環を避けるため、ここで読む
        from tests.research.ff_model import STOP_SPEED_KMH  # noqa: PLC0415
        from tests.research.fwd_plan import FLAG_STOP  # noqa: PLC0415

        pred, ff = self.pred, self.ff
        assert pred is not None and isinstance(ff, ForwardMapFeedforward)
        launch, stop, held = self._low_flags(t, ref, speed, ff_effort)
        idle = (
            self._first_hold and speed <= STOP_SPEED_KMH
            and int(ff.plan.flag[ff.plan.index_at(t)]) == FLAG_STOP
        )
        if not idle:
            self._first_hold = False
        self._speed_est = pred.step(
            t=t, can_kmh=speed, launch=launch, stop=stop, held=held, idle=idle
        )
        self._speed_pred = None
        self._pred_dev = None
        lag = pred.lag_for(ff_effort)
        if not held and lag is not None and lag > 0.0:
            v_hat = pred.predict_ahead(t=t, lag_s=lag, launch=launch, stop=stop, held=held)
            if v_hat is not None:
                self._speed_pred = v_hat
                self._pred_dev = self.ref_t.at(t + lag) - v_hat  # 目標はぼかした基準（段3）
                pred.stats.add_prediction(t=t, lag_s=lag, pred_kmh=v_hat, can_kmh=speed)
        self._pred_row_dev = ref - speed if self._pred_dev is None else self._pred_dev

    def _c10_offset(self, t: float, ref: float, speed: float, ff_effort: float) -> float:
        """C10: 今の列の effort に偏差の分を上乗せしたときの差（上乗せ後 − 今の列）[%]。
        `ref` は目標の基準（ぼかした基準。2026-10-19 段3。開度の列を作った基準と同じ）。
        偏差は今の周期の CAN 車速から（dev ＝ 基準 − 実車速）。表を引く車速の行は案A が基準、
        案B が実車速。惰行・停車保持・先置き・τ なしは 0。"""
        # fwd_plan → ff_bias → mode_report → mode_drive の循環を避けるため、ここで読む
        from tests.research.fwd_plan import (  # noqa: PLC0415
            FLAG_COAST,
            FLAG_PREPOS,
            FLAG_STOP,
            deviation_offset_detail,
        )

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
            # C13: 予測を使う周期は「基準の L 秒先 − 予測した L 秒先の車速」。それ以外は今の偏差
            "dev_kmh": ref - speed if self._pred_dev is None else self._pred_dev,
            "tau_accel_s": ff.tau_accel_s,
            "tau_brake_s": ff.tau_brake_s,
            "accel_min_pct": v.min_accel_opening_pct,
            "brake_min_pct": v.min_brake_opening_pct,
            "stops_ahead": bool(plan.stops_ahead[i]) if plan.stops_ahead is not None else False,
            # 段5-2: 計画が行ごとに決めた表（規則が legacy なら None ＝ 今までと同じ）
            "side": plan.side_for(i, ff_effort > 0.0),
        }
        fb_table = self._fb_table()  # 段8-b
        dob = self.dob
        # C11: 押しているペダルの行にだけ、そのペダルの外乱の推定を引いて表を引く。
        # DOB の分は「P ＋ DOB の上乗せ − P だけの上乗せ」で別に出す（ログ・集計用）
        d_row = 0.0 if dob is None or ff_effort == 0.0 else dob.estimate(is_accel=ff_effort > 0.0)
        # 2026-10-18 段2q ③ `135125` の修正: 推定を更新しない低速の行（発進の前のブレーキを離す所
        # など）では使わない。そこで持ち越したブレーキの推定（−0.05〜−0.35）がブレーキを浅くし、
        # 発進で早く動き出した（アクセルを踏み始めた瞬間の偏差 +0.04〜+0.37 → +0.32〜+0.90）
        # 2026-10-23 段6-2: 設定が両方 false なら今までと同じ（低速の行は 0）。
        # dob_low_speed_fade なら 1 → 0 へ連続に薄める（`_dob_weight`）
        if dob is not None:
            d_row *= self._dob_weight(speed, int(plan.flag[i]), ref)
        new, clamped = deviation_offset_detail(fb_table, dob_kmhs=d_row, **kwargs)
        if self.stop_end is not None:
            # C15 停C: 停車まぎわのブレーキの行は、P の τ を「基準が 0 になるまでの残り時間
            # （ブレーキの遅れより短くしない）」にして引き直し、開度の範囲に収める
            tau_end = (
                self.stop_end.tau_s(t, self.ref.at(t), int(plan.flag[i]) == FLAG_STOP)
                if ff_effort < 0.0 and int(plan.flag[i]) not in (FLAG_COAST, FLAG_PREPOS)
                else None
            )
            # 2026-10-23 段6-2 (c): 普段の τ より弱くしない（残り時間が長いときは普段のまま）
            tau_end = cap_stop_end_tau(
                tau_end, ff.tau_brake_s, self.cfg.forward_map.stop_end_tau_cap_normal
            )
            if tau_end is None:
                self.stop_end.stats.add(False)
            else:
                normal = new
                kwargs["tau_brake_s"] = tau_end  # p_only（DOB の分けるとき）も同じ τ で出す
                new, clamped = deviation_offset_detail(fb_table, dob_kmhs=d_row, **kwargs)
                new = self.stop_end.clamp(new, ff_effort)
                self._stop_end_tau_s = tau_end
                self.stop_end.stats.add(True, normal - new)  # 深い向きが正
        delta = new - ff_effort
        if dob is not None:
            p_only, _ = deviation_offset_detail(fb_table, **kwargs)
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

    def _coast_assist(
        self, t: float, ref: float, ref_t: float, speed: float, ff_effort: float
    ) -> float:
        """C12: 惰行の行の補助の開度 [%]（正＝アクセル・負＝ブレーキ。補助しなければ 0）。
        入る・離すは `CoastAssist.step`（毎周期呼ぶ）、踏む量は `fwd_plan.coast_assist_effort`。
        τ が None のペダル（遅れが測れない）は踏まない。DOB の分は
        「推定を引いた開度 − 引かない開度」を `_dob_effort` に入れる
        （低速の行では推定を使わない。C11 と同じ）。
        2026-10-19 段3: 目標（偏差 dev・表の行 ref_kmh）は `ref_t`（ぼかした基準）。入る・離すの
        停車の判定（元の基準が停車速度を超えているか）は `ref`（元の基準）のまま。"""
        # fwd_plan → ff_bias → mode_report → mode_drive の循環を避けるため、ここで読む
        from tests.research.fwd_plan import coast_assist_effort  # noqa: PLC0415

        ff, coast, dob = self.ff, self.coast, self.dob
        assert isinstance(ff, CoastAssistFeedforward) and coast is not None and dob is not None
        plan = ff.plan
        i = plan.index_at(t)
        pedal = coast.step(t=t, i=i, dev_kmh=ref_t - speed, speed_kmh=speed, ref_kmh=ref)
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
            "ref_kmh": ref_t,
            "row_kmh": ref_t if ff.deviation_row == "reference" else speed,
            "dev_kmh": ref_t - speed,
            "tau_s": tau,
            "accel_min_pct": v.min_accel_opening_pct,
            "brake_min_pct": v.min_brake_opening_pct,
            "stops_ahead": bool(plan.stops_ahead[i]) if plan.stops_ahead is not None else False,
            "side": plan.side_for(i, is_accel),  # 段5-2（規則が legacy なら None）
        }
        if self.cfg.forward_map.coast_floor_onset:
            # 段2r ②: 補助中のそのペダルは手順2 の効き始め以上に置く（meta に無ければ下限なし）
            on = ff.table.meta.get("accel_onset_u_pct" if is_accel else "brake_onset_u_pct")
            kwargs["onset_u_pct"] = None if on is None else float(on)
        d = dob.estimate(is_accel=is_accel) * self._dob_weight(speed, int(plan.flag[i]), ref)
        fb_table = self._fb_table()  # 段8-b
        with_dob = coast_assist_effort(fb_table, dob_kmhs=d, **kwargs)
        without = coast_assist_effort(fb_table, **kwargs) if d != 0.0 else with_dob
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

    def _dob_low_top(self) -> float:
        """DOB を止める／薄める境目の車速 [km/h]（launch_top と stop_max_speed_kmh の大きい方）"""
        meta = self.ff.table.meta  # type: ignore[attr-defined]
        return max(self._dob_launch_top(), float(meta.get("stop_max_speed_kmh", 0.0) or 0.0))

    def _dob_boundary_speed(self, speed: float, ref: float | None) -> float:
        """境目と比べる車速。dob_low_speed_align なら基準車速（表の切り替えが見るもの）、
        そうでなければ実車速（今までと同じ。ref が None のときも実車速）"""
        if ref is not None and self.cfg.forward_map.dob_low_speed_align:
            return ref
        return speed

    def _dob_low_speed(self, speed: float, flag: int, ref: float | None = None) -> bool:
        """C11 の推定を更新せず、使いもしない行か（停車保持の行・停車速度以下・発進側／停止側の表が
        置き換えた車速（launch_top と meta の stop_max_speed_kmh の大きいほう）未満）。
        2026-10-23 段6-2 (f): dob_low_speed_align で、境目は基準車速（`ref`）を見る
        （停車保持・停車速度以下の判定は実車速のまま）。"""
        from tests.research.ff_model import STOP_SPEED_KMH  # noqa: PLC0415
        from tests.research.fwd_plan import FLAG_STOP  # noqa: PLC0415

        v_b = self._dob_boundary_speed(speed, ref)
        return flag == FLAG_STOP or speed <= STOP_SPEED_KMH or v_b < self._dob_low_top()

    def _dob_weight(self, speed: float, flag: int, ref: float | None = None) -> float:
        """DOB の上乗せに掛ける重み w（2026-10-23 段6-2 (a)）。dob_low_speed_fade が false なら
        今までと同じ（低速の行 0・それ以外 1）。true なら、停車保持の行・停車速度以下は 0、
        それ以外は w ＝ clip((v − 0.5) / (境目 − 0.5), 0, 1)（v は `_dob_boundary_speed`）。"""
        if not self.cfg.forward_map.dob_low_speed_fade:
            return 0.0 if self._dob_low_speed(speed, flag, ref) else 1.0
        from tests.research.ff_model import STOP_SPEED_KMH  # noqa: PLC0415
        from tests.research.fwd_plan import FLAG_STOP  # noqa: PLC0415

        if flag == FLAG_STOP or speed <= STOP_SPEED_KMH:
            return 0.0
        return dob_fade_weight(
            self._dob_boundary_speed(speed, ref), DOB_FADE_START_KMH, self._dob_low_top()
        )

    def _dob_follow_plan_tables(self) -> bool:
        """低速でも推定を更新し、計画が引いた表で予測するか（(a) と (f) を両方 true にしたとき）"""
        fm = self.cfg.forward_map
        return bool(fm.dob_low_speed_fade and fm.dob_low_speed_align)

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
        `ref` は目標の基準（ぼかした基準。表の側を選ぶ low_speed_sides が開度の列と同じ基準で
        判定するため。2026-10-19 段3）。停車の判定は計画の stops_ahead（元の基準）。
        u ＝ 開度 − min（アクセル正・ブレーキ負・min 以下は 0）。発進側・停止側の行では更新しない
        （今の計画の行で `low_speed_sides` を引く。C10 の上乗せと同じ条件）。"""
        ff, dob, v = self.ff, self.dob, self.cfg.vehicle
        assert isinstance(ff, DisturbanceObserverFeedforward)
        assert dob is not None and self.dob_stats is not None
        plan = ff.plan
        i = plan.index_at(t)
        # 段5-2: 引く表は計画が行ごとに決めたもの（規則が legacy なら low_speed_sides）。ペダルは
        # 実際に出したほう（どちらも min 以下なら計画の行の向き）
        if accel > v.min_accel_opening_pct:
            row_accel = True
        elif brake > v.min_brake_opening_pct:
            row_accel = False
        else:
            row_accel = float(plan.effort_pct[i]) >= 0.0
        launch, stop = self._row_sides(plan, i, ref, is_accel=row_accel)
        # 発進側は need > 0 の行すべてで True になる（表を引く車速の範囲は表の中で決まる）ので、
        # 止めるのは発進側の表が置き換えた車速の行（meta の launch_speed_rows）の中だけにする
        follow = self._dob_follow_plan_tables()
        launch = launch and self._dob_boundary_speed(speed, ref) < self._dob_launch_top()
        # 2026-10-18 段2q ① `121218` の修正: 停車中（停車保持の深いブレーキ）は表が強い減速を
        # 予測し、実測は 0 なので、ブレーキの推定が +9 km/h/s まで積み上がって次の減速で踏み過ぎた。
        # 停車保持の行・停車速度以下・低速の表の車速の行では更新しない（`_dob_low_speed`）
        # 2026-10-23 段6-2: (a)(f) が両方 true のときは、停車保持の行・停車速度以下だけ止め、
        # 低速の行は計画が引いた発進側・停止側の表で予測して更新を続ける（薄めた上乗せの推定を
        # 使う表と同じ表の残差にそろえる）
        if follow:
            from tests.research.ff_model import STOP_SPEED_KMH  # noqa: PLC0415
            from tests.research.fwd_plan import FLAG_STOP  # noqa: PLC0415

            held = int(plan.flag[i]) == FLAG_STOP or speed <= STOP_SPEED_KMH
        else:
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
        # C13: DOB の実加速度は、車速のフィルタの推定車速の差分から（C11・C12 は CAN 車速の差分）
        pedal = dob.update(
            t=t, speed_kmh=speed if self._speed_est is None else self._speed_est,
            u_now=u, governed=governed,
            at_limit_now=(
                accel >= v.max_accel_opening_pct or brake >= v.max_brake_opening_pct
            ),
            launch=bool(launch), stop=bool(stop) or held,
            side_tables=follow and not held,
            need_kmhs=float(plan.need_accel_kmhs[i]),
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
    section: str = SECTION_MODE_DRIVE,
) -> ModeDriveResult:
    """停車保持の状態からモードを走り、停車保持で終える。

    異常（CAN・通信・過電流・最高速超え・逸脱）は例外にせず、ペダルを離してから
    abort_reason 付きの結果を返す（途中までのレポートを作れるように）。
    `limit_s` はモードの先頭だけ走る動作確認用。`section` は CSV の section 列（既定 MODE_DRIVE。
    FEL の学習用パターンは FEL_DRIVE。2026-10-19 段2s-2b）。
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

    run = _ModeRun(hw, cfg, setup, log, duration, section=section)
    await log.stop_sampler()  # 50ms の制御ループと Modbus を取り合わない（記録はループから）
    log.mark(section, "")
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
        predictor_settings=None if run.pred is None else run.pred.settings_text(),
        predictor=None if run.pred is None else run.pred.stats,
        creep_floor_settings=None if run.cfloor is None else run.cfloor.settings_text(),
        creep_floor_stats=None if run.cfloor is None else run.cfloor.stats,
        stop_end_settings=None if run.stop_end is None else run.stop_end.settings_text(),
        stop_lead_stats=run.stop_lead_stats,
        stop_end_stats=None if run.stop_end is None else run.stop_end.stats,
        samples=run.samples,
    )
    say(f"モード走行 {'完了' if result.completed else '中断'}: {run_s:.1f}s・{run.cycles} 周期"
        f"（1 周期以上の遅れ {run.overruns} 回・減速G ガバナー作動 {run.governor_cycles} 周期）"
        f"・最大 |偏差| {run.max_abs_dev:.2f} km/h")
    if run.interlock is not None:
        say(f"  実開度の安全網（両踏みを防ぐために次のペダルを止めた）: {run.interlock.summary()}")
    else:
        say("  実開度の安全網: 無効（mode_drive.pedal_interlock_enabled: false）")
    if run.midway_fb_no_deeper:
        say("  発進の途中の FB のブレーキ踏み増しの抑え（段6 C）: "
            f"{run.midway_fb_cut_cycles} 周期（{run.midway_fb_cut_cycles * run.interval_s:.1f}s）・"
            f"抑えた量の合計 {run.midway_fb_cut_pct_sum:.1f}%")
    if run.c10 is not None:
        say(f"  C10（偏差の上乗せ。{c10_settings_text(run.ff)}）: {run.c10.summary()}")
    if run.dob_stats is not None:
        say(f"  C11（DOB。{dob_settings_text(run.ff)}）: {run.dob_stats.summary()}")
        if run.dob is not None and run.dob.fast_need_kmhs > 0.0 and (
            run.dob.tq_fast_accel_s is not None or run.dob.tq_fast_brake_s is not None
        ):
            p = run.interval_s
            say("  C11（DOB）速い Tq で更新した周期（段9-d）: "
                f"アクセル {run.dob.n_fast_accel} 周期（{run.dob.n_fast_accel * p:.1f}s）・"
                f"ブレーキ {run.dob.n_fast_brake} 周期（{run.dob.n_fast_brake * p:.1f}s）")
    if run.coast is not None:
        say(f"  C12（惰行の補助。{run.coast_settings}）: {run.coast.stats.summary()}")
    if run.pred is not None:
        say(f"  C13（予測器・車速のフィルタ。{run.pred.settings_text()}）: "
            f"{run.pred.stats.summary()}")
    if run.cfloor is not None:
        say(f"  C14（停車の最後のブレーキの下限。{run.cfloor.settings_text()}）: "
            f"{run.cfloor.stats.summary()}")
    if run.stop_end is not None and run.stop_lead_stats is not None:
        say(f"  C15 停A（停車保持を早める）: {run.stop_lead_stats.summary()}")
        say(f"  C15 停C（停車まぎわの P。{run.stop_end.settings_text()}）: "
            f"{run.stop_end.stats.summary()}")
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
