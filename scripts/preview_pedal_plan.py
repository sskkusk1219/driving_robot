"""登録モードのペダルプランを机上生成し、フェーズ内訳・切替回数を検証するスクリプト。

新アーキテクチャ（ペダルプラン＋トリム）の実機前ゲート:
  1. 各登録モードでプランを生成し、フェーズ時間内訳・プラン切替回数 vs 軌跡要求切替を出力。
     プランの micro-phase マージが「軌跡が要求しない不要切替」を消せているかを確認する。
  2. 検証専用パターン（build_verification_trajectory）を生成し、登録全モードの速度域・
     ランプ率を包絡しているかを確認する（--verify）。

FF モデルは指定しない（プランのフェーズ構造＝切替回数の検証が目的で、名目 effort は
本スクリプトでは評価しない）。DATABASE_URL 環境変数（既定 postgresql://localhost/driving_robot）。

使い方:
    python -m scripts.preview_pedal_plan            # 全モードのフェーズ内訳・切替
    python -m scripts.preview_pedal_plan --verify   # 検証専用パターンの包絡も出力
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
from datetime import UTC, datetime

import asyncpg
import numpy as np

from src.domain.control.feedforward import FeedforwardController
from src.domain.control.pedal_plan import PedalPlan, PedalPlanner, PlanPhase
from src.models.driving_mode import DrivingMode, SpeedPoint
from src.models.profile import FeedforwardParams

# 実機プロファイル 01fb8130（sample_002）の同定済み車両定数。プランのフェーズ分類は
# これらの定数（クリープ・エンジンブレーキ・停車保持）に依存する。
_PARAMS = FeedforwardParams(
    creep_speed_kmh=4.94,
    creep_rate_kmhs=0.065,
    engine_brake_decel_kmhs=1.60,
    stop_brake_opening_pct=19.2,
    brake_deadband_pct=1.5,
)


async def _fetch_modes(dsn: str) -> list[DrivingMode]:
    conn = await asyncpg.connect(dsn)
    try:
        rows = await conn.fetch(
            "SELECT id, name, reference_speed, total_duration, max_speed FROM driving_modes "
            "ORDER BY name"
        )
    finally:
        await conn.close()
    modes: list[DrivingMode] = []
    for r in rows:
        raw = r["reference_speed"]
        pts = json.loads(raw) if isinstance(raw, str) else raw
        modes.append(
            DrivingMode(
                id=str(r["id"]),
                name=r["name"],
                description="",
                reference_speed=[
                    SpeedPoint(time_s=p["time_s"], speed_kmh=p["speed_kmh"]) for p in pts
                ],
                total_duration=r["total_duration"],
                max_speed=r["max_speed"],
                created_at=datetime.now(tz=UTC),
            )
        )
    return modes


def _switch_counts(plan: PedalPlan) -> tuple[int, int]:
    """プランのペダル切替（DRIVE⇄BRAKE）回数と、うち2秒以内の交互踏み回数。"""
    switches = 0
    fast_alt = 0
    last_pedal = 0
    last_t: float | None = None
    for i, ph in enumerate(plan.phases):
        s = 1 if ph == PlanPhase.DRIVE else (-1 if ph == PlanPhase.BRAKE else 0)
        t = i * plan.dt_s
        if s in (1, -1) and s != last_pedal:
            if last_pedal in (1, -1):
                switches += 1
                if last_t is not None and (t - last_t) < 2.0:
                    fast_alt += 1
            last_pedal, last_t = s, t
        elif s in (1, -1):
            last_t = t
    return switches, fast_alt


def _print_plan_summary(mode: DrivingMode, plan: PedalPlan) -> None:
    dur = plan.duration_s
    n = len(plan.phases)
    counts = {ph: plan.phases.count(ph) for ph in PlanPhase}
    switches, fast_alt = _switch_counts(plan)
    per_min = switches / dur * 60 if dur > 0 else 0.0
    print(
        f"{mode.name:<26} {dur:6.0f}s  "
        f"DRIVE {counts[PlanPhase.DRIVE] / n * 100:4.0f}%  "
        f"COAST {counts[PlanPhase.COAST] / n * 100:4.0f}%  "
        f"BRAKE {counts[PlanPhase.BRAKE] / n * 100:4.0f}%  "
        f"STOP {counts[PlanPhase.STOP_HOLD] / n * 100:3.0f}%  "
        f"切替 {switches:3d}({per_min:4.1f}/min) 2s以内 {fast_alt:3d}"
    )


async def _main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dsn", default=os.environ.get("DATABASE_URL", "postgresql://localhost/driving_robot")
    )
    parser.add_argument("--verify", action="store_true", help="検証専用パターンの包絡も出力")
    args = parser.parse_args()

    modes = await _fetch_modes(args.dsn)
    ff = FeedforwardController()  # モデル未ロード（フェーズ構造の検証が目的）
    ff.set_params(_PARAMS)

    print("=== 登録モードのペダルプラン（フェーズ内訳・切替回数） ===")
    for mode in modes:
        plan = PedalPlanner.build(mode, ff, _PARAMS)
        _print_plan_summary(mode, plan)

    if args.verify:
        from src.domain.pid_tuning import build_verification_trajectory

        # 検証専用パターン（cap=登録モード最高速度と各プロファイル上限の min）。
        # ここではプロファイル上限を最大の 140 と仮定して包絡を確認する。
        from src.models.profile import PIDGains, StopConfig, VehicleProfile

        profile = VehicleProfile(
            id="preview",
            name="preview",
            max_accel_opening=100.0,
            max_brake_opening=100.0,
            max_speed=140.0,
            max_decel_g=0.4,
            pid_gains=PIDGains(kp=1.0, ki=1.0, kd=0.0),
            stop_config=StopConfig(deviation_threshold_kmh=5.0, deviation_duration_s=2.0),
            calibration=None,
            model_path=None,
            created_at=datetime.now(tz=UTC),
            updated_at=datetime.now(tz=UTC),
            feedforward_params=_PARAMS,
        )
        verify_mode = build_verification_trajectory(modes, profile)
        vt = np.array([p.time_s for p in verify_mode.reference_speed])
        vv = np.array([p.speed_kmh for p in verify_mode.reference_speed])
        print("\n=== 検証専用パターン ===")
        print(f"総時間 {vt[-1]:.1f}s  最高速度 {vv.max():.1f} km/h  点数 {len(vt)}")
        plan = PedalPlanner.build(verify_mode, ff, _PARAMS)
        _print_plan_summary(verify_mode, plan)
        grid_t = np.arange(vt[0], vt[-1], 0.1)
        grid_v = np.interp(grid_t, vt, vv)
        print("速度帯カバレッジ [s]:")
        bands = [(0, 5), (5, 30), (30, 50), (50, 70), (70, 90), (90, 110), (110, 125), (125, 140)]
        for lo, hi in bands:
            m = (grid_v >= lo) & (grid_v < hi)
            print(f"  {lo:3d}-{hi:3d}: {m.sum() * 0.1:6.1f}")


if __name__ == "__main__":
    asyncio.run(_main())
