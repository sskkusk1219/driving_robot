"""制御スタックの閉ループ机上模擬（むだ時間＋実 PedalArbiter 入り）。

実機を動かさずに「プラン整形・フィードバック権限・前倒し量」の変更を**順位付け**するための
ツール。プロファイルの同定値（惰行カーブ・ペダルゲイン・むだ時間 θ）と登録モードの基準軌跡を
DB から読み、実際の PedalPlanner / TrimController / PIDController / PedalArbiter を通して
車速を積分する。

**車両モデル**（`_accel_of`）:
    a(v) = a_coast(v) + (開度 − 不感帯) × pedal_gain(v)
実機ログ（9eee549b run1）に対し瞬時加速度が corr 0.906 / bias +0.003 km/h/s で一致する。
ただし開ループ積分は誤差が蓄積するため、**絶対値は実機より悪く出る**（実機 p95 1.93 の
構成で模擬は 2.6 前後）。**構成間の優劣を見る用途にのみ使うこと。**

未モデル化: モデル化されていない外乱・勾配・変速機の挙動・アクチュエータの動特性。

使い方:
    python -m scripts.simulate_control --profile-id <UUID> --mode-id <UUID>
    python -m scripts.simulate_control --sweep-lead      # 前倒し量の掃引
    python -m scripts.simulate_control --sweep-lowpass   # プラン平滑の掃引
"""

from __future__ import annotations

import argparse
import asyncio
import bisect
import collections
import dataclasses
import json
import os
from datetime import UTC, datetime

import asyncpg
import numpy as np

import src.domain.control.pedal_plan as pedal_plan
from src.domain.control.drive_loop import _GAIN_SIDE_HYSTERESIS_PCT
from src.domain.control.feedforward import FeedforwardController
from src.domain.control.pedal_arbiter import PedalArbiter
from src.domain.control.pedal_plan import PedalPlan, PedalPlanner, fold_times
from src.domain.control.pedal_safety import enforce_pedal_exclusion
from src.domain.control.pid import PIDController
from src.domain.control.trim import SIMC_TAU_C_FACTOR, TrimController
from src.models.driving_mode import DrivingMode, SpeedPoint
from src.models.profile import (
    SIMC_NOMINAL_SPEED_KMH,
    FeedforwardParams,
    pedal_gain_at,
    robust_kp_at,
)

_DSN = os.environ.get("DATABASE_URL", "postgresql://localhost/driving_robot")
_DT_S = 0.05  # 制御周期（CONTROL_LOOP_INTERVAL_S と同じ）
_GAIN_SCALE_MIN = 0.5
_GAIN_SCALE_MAX = 1.5


@dataclasses.dataclass(frozen=True)
class SimResult:
    p95_kmh: float
    max_kmh: float
    over_limit_s: float
    opening_rate_rms_pct_s: float

    def row(self, label: str) -> str:
        return (
            f"{label:44s} {self.p95_kmh:6.2f} {self.max_kmh:6.2f} "
            f"{self.over_limit_s:7.1f} {self.opening_rate_rms_pct_s:9.1f}"
        )


async def _load(
    profile_id: str, mode_id: str
) -> tuple[FeedforwardParams, str | None, DrivingMode, float, float, float]:
    conn = await asyncpg.connect(_DSN)
    try:
        prow = await conn.fetchrow(
            "SELECT feedforward_params, dynamics_params, model_path, pid_gains, "
            "max_accel_opening, max_brake_opening FROM vehicle_profiles WHERE id = $1",
            profile_id,
        )
        mrow = await conn.fetchrow(
            "SELECT id, name, reference_speed, total_duration, max_speed "
            "FROM driving_modes WHERE id = $1",
            mode_id,
        )
    finally:
        await conn.close()
    if prow is None or mrow is None:
        raise SystemExit("プロファイルまたはモードが見つかりません")

    def _j(v: object) -> dict:
        return json.loads(v) if isinstance(v, str) else dict(v)  # type: ignore[arg-type]

    ffp_raw = _j(prow["feedforward_params"])
    names = {f.name for f in dataclasses.fields(FeedforwardParams)}
    params = FeedforwardParams(
        **{
            k: (tuple(v) if isinstance(v, list) else v)
            for k, v in ffp_raw.items()
            if k in names
        }
    )
    dyn = _j(prow["dynamics_params"]) if prow["dynamics_params"] else {}
    theta = float(dyn.get("fopdt_theta") or 0.0)
    pts_raw = mrow["reference_speed"]
    pts_raw = json.loads(pts_raw) if isinstance(pts_raw, str) else pts_raw
    mode = DrivingMode(
        id=str(mrow["id"]),
        name=mrow["name"],
        description="",
        reference_speed=[
            SpeedPoint(time_s=float(x["time_s"]), speed_kmh=float(x["speed_kmh"]))
            for x in pts_raw
        ],
        total_duration=float(mrow["total_duration"]),
        max_speed=float(mrow["max_speed"]),
        created_at=datetime.now(UTC),
    )
    return (
        params,
        prow["model_path"],
        mode,
        theta,
        float(prow["max_accel_opening"]),
        float(prow["max_brake_opening"]),
    )


def _accel_of(v: float, accel_open: float, brake_open: float, p: FeedforwardParams) -> float:
    """開度から加速度 [km/h/s] を返す（惰行カーブ＋同定済みペダルゲイン）。"""
    a = pedal_plan.coast_accel(v, p)
    if accel_open > 0.0:
        g = pedal_gain_at(p, v, is_accel=True)
        if g:
            a += max(0.0, accel_open - max(0.0, p.accel_deadband_pct)) * g
    if brake_open > 0.0:
        g = pedal_gain_at(p, v, is_accel=False)
        if g:
            a -= max(0.0, brake_open - max(0.0, p.brake_deadband_pct)) * g
    return a


def _plan_sample(
    plan: PedalPlan,
    now_s: float,
    lead_s: float,
    folds: list[float],
    *,
    directional: bool,
) -> tuple[float, object]:
    """本番 DriveLoop._plan_at / _plan_lead_at と同じ規則でプランを読む。

    本番の判定を変えたらここも合わせること（模擬の順位付けが実機と食い違うため）。
    """
    if lead_s <= 0.0:
        return plan.effort_at(now_s), plan.phase_at(now_s)
    if not directional:
        return plan.effort_at(now_s + lead_s), plan.phase_at(now_s + lead_s)

    lead = lead_s
    if folds:
        idx = bisect.bisect_left(folds, now_s)
        distance = float("inf")
        if idx < len(folds):
            distance = folds[idx] - now_s
        if idx > 0:
            distance = min(distance, now_s - folds[idx - 1])
        lead = lead_s * min(1.0, max(0.0, distance) / lead_s)
    if lead <= 0.0:
        return plan.effort_at(now_s), plan.phase_at(now_s)

    effort_now = plan.effort_at(now_s)
    plan_t = now_s + lead
    effort_lead = plan.effort_at(plan_t)
    side = effort_now if effort_now != 0.0 else effort_lead
    if side > 0.0:
        use_lead = effort_lead > effort_now
    elif side < 0.0:
        use_lead = effort_lead < effort_now
    else:
        use_lead = False
    if not use_lead:
        return effort_now, plan.phase_at(now_s)
    return effort_lead, plan.phase_at(plan_t)


def simulate(
    plan: PedalPlan,
    mode: DrivingMode,
    params: FeedforwardParams,
    ff: FeedforwardController,
    *,
    kp: float,
    ki: float,
    theta_s: float,
    lead_s: float,
    directional_lead: bool = True,
    pedal_side: bool = True,
    g_nominal: float | None,
    max_accel_opening: float,
    max_brake_opening: float,
) -> SimResult:
    """1 走行ぶんを閉ループで積分して KPI 相当を返す。"""
    src_t = np.array([p.time_s for p in mode.reference_speed], dtype=float)
    src_v = np.array([p.speed_kmh for p in mode.reference_speed], dtype=float)
    n = int(float(src_t[-1]) / _DT_S) + 1
    grid_t = np.arange(n, dtype=float) * _DT_S
    ref = np.interp(grid_t, src_t, src_v)

    pid = PIDController(kp, ki, 0.0, dt=_DT_S, output_limit=params.pid_output_limit_pct)
    trim = TrimController(pid)
    arbiter = PedalArbiter(params, max_accel_opening, max_brake_opening, nominal_dt_s=_DT_S)
    delay = max(1, int(round(theta_s / _DT_S)))
    pipe: collections.deque[tuple[float, float]] = collections.deque(
        [(0.0, 0.0)] * delay, maxlen=delay
    )
    schedule = ff.gain_schedule
    folds = fold_times(plan.efforts, plan.phases, plan.dt_s)

    v = float(ref[0])
    actual = np.zeros(n, dtype=float)
    signed_opening = np.zeros(n, dtype=float)
    prev_phase = None
    side_accel: bool | None = None
    last_effort = 0.0
    for i in range(n):
        actual[i] = v
        # ゲインの向き判定（本番 DriveLoop._is_accel_side と同じ規則）。
        # pedal_side=True は「今動いているペダル」（直前サイクルの合成 effort）で決める。
        if pedal_side:
            if last_effort > _GAIN_SIDE_HYSTERESIS_PCT:
                side_accel = True
            elif last_effort < -_GAIN_SIDE_HYSTERESIS_PCT:
                side_accel = False
            elif side_accel is None:
                ahead = float(np.interp(grid_t[i] + 0.5, src_t, src_v))
                side_accel = (ahead - float(ref[i])) >= -0.1
            is_accel = bool(side_accel)
        else:
            ahead = float(np.interp(grid_t[i] + 0.5, src_t, src_v))
            is_accel = (ahead - float(ref[i])) >= -0.1
        scale = 1.0
        if schedule is not None and g_nominal:
            g = schedule.accel_gain_at(v) if is_accel else schedule.brake_gain_at(v)
            scale = max(_GAIN_SCALE_MIN, min(_GAIN_SCALE_MAX, g / g_nominal))
        robust = robust_kp_at(
            params, v, theta_s, is_accel=is_accel, tau_c_factor=SIMC_TAU_C_FACTOR
        )
        if robust is not None and kp > 0.0:
            scale = min(scale, robust / kp)
        base, phase = _plan_sample(
            plan, grid_t[i], lead_s, folds, directional=directional_lead
        )
        if prev_phase is not None and phase != prev_phase:
            trim.notify_phase_change()
        prev_phase = phase
        trim_u = trim.update(
            float(ref[i]),
            v,
            _DT_S,
            phase=phase,
            gain_scale=scale,
            fast_kp_effective=kp * scale,
        )
        last_effort = base + trim_u
        out = arbiter.arbitrate(last_effort, _DT_S)
        a_open, b_open = enforce_pedal_exclusion(out.accel_opening, out.brake_opening)
        pipe.append((a_open, b_open))
        d_accel, d_brake = pipe[0]
        signed_opening[i] = d_accel - d_brake
        v = max(0.0, v + _DT_S * _accel_of(v, d_accel, d_brake, params))

    dev = np.abs(actual - ref)
    rate = np.abs(np.diff(signed_opening)) / _DT_S
    return SimResult(
        p95_kmh=float(np.percentile(dev, 95)),
        max_kmh=float(dev.max()),
        over_limit_s=float((dev > 1.0).sum() * _DT_S),
        opening_rate_rms_pct_s=float(np.sqrt(np.mean(rate**2))),
    )


def _build_plan(
    mode: DrivingMode,
    ff: FeedforwardController,
    params: FeedforwardParams,
    smooth_s: float,
    lowpass_hz: float,
    center_ratio: float = 1.0,
) -> PedalPlan:
    orig = (
        pedal_plan.PLAN_ACCEL_SMOOTH_S,
        pedal_plan.PLAN_LOWPASS_HZ,
        pedal_plan.PLAN_ACCEL_SMOOTH_CENTER,
    )
    pedal_plan.PLAN_ACCEL_SMOOTH_S = smooth_s
    pedal_plan.PLAN_LOWPASS_HZ = lowpass_hz
    pedal_plan.PLAN_ACCEL_SMOOTH_CENTER = center_ratio
    try:
        return PedalPlanner.build(mode, ff, params)
    finally:
        (
            pedal_plan.PLAN_ACCEL_SMOOTH_S,
            pedal_plan.PLAN_LOWPASS_HZ,
            pedal_plan.PLAN_ACCEL_SMOOTH_CENTER,
        ) = orig


async def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile-id", required=True)
    parser.add_argument("--mode-id", required=True)
    parser.add_argument(
        "--kp", type=float, default=None, help="速い層 kp（既定: 積分系 SIMC で算出）"
    )
    parser.add_argument("--ki", type=float, default=None)
    parser.add_argument("--lead", type=float, default=None, help="前倒し量 [s] を明示する")
    parser.add_argument("--sweep-lead", action="store_true", help="前倒し量を掃引する")
    parser.add_argument("--sweep-lowpass", action="store_true", help="プラン平滑を掃引する")
    parser.add_argument(
        "--sweep-lead-mode",
        action="store_true",
        help="向き別前倒し（優先A）と a_req 窓中心 center_ratio を掃引する",
    )
    args = parser.parse_args()

    params, model_path, mode, theta, max_a, max_b = await _load(
        args.profile_id, args.mode_id
    )
    if theta <= 0.0:
        raise SystemExit("fopdt_theta が未同定のため模擬できません")
    ff = FeedforwardController()
    ff.set_params(params)
    if model_path:
        ff.load_model(model_path)
        ff.rebuild_gain_schedule(mode.max_speed)

    nominal_gain = pedal_gain_at(params, SIMC_NOMINAL_SPEED_KMH, is_accel=True)
    if nominal_gain is None:
        raise SystemExit("ペダルゲイン曲線が未同定のため模擬できません")
    g_nominal = 1.0 / nominal_gain
    tau_sum = theta * (1.0 + SIMC_TAU_C_FACTOR)
    kp = args.kp if args.kp is not None else 1.0 / (nominal_gain * tau_sum)
    ki = args.ki if args.ki is not None else kp / (4.0 * tau_sum)

    from src.domain.control.drive_loop import PLAN_LEAD_MAX_S, PLAN_LEAD_THETA_FACTOR

    lead = (
        args.lead
        if args.lead is not None
        else min(theta * PLAN_LEAD_THETA_FACTOR, PLAN_LEAD_MAX_S)
    )
    print(f"profile={args.profile_id} mode={mode.name} θ={theta:.3f}s")
    print(f"kp={kp:.3f} ki={ki:.3f} g_nominal={g_nominal:.3f} lead={lead:.2f}s")
    print(
        f"\n{'構成':44s} {'p95':>6} {'max':>6} {'違反s':>7} {'開度変化RMS':>9}"
    )

    def run(
        label: str,
        smooth: float,
        hz: float,
        lead_s: float,
        *,
        center_ratio: float = 1.0,
        directional: bool = True,
        pedal_side: bool = True,
    ) -> None:
        plan = _build_plan(mode, ff, params, smooth, hz, center_ratio)
        res = simulate(
            plan,
            mode,
            params,
            ff,
            kp=kp,
            ki=ki,
            theta_s=theta,
            lead_s=lead_s,
            directional_lead=directional,
            pedal_side=pedal_side,
            g_nominal=g_nominal,
            max_accel_opening=max_a,
            max_brake_opening=max_b,
        )
        print(res.row(label))

    cur_smooth = pedal_plan.PLAN_ACCEL_SMOOTH_S
    cur_hz = pedal_plan.PLAN_LOWPASS_HZ
    if args.sweep_lead_mode:
        run(
            "旧: 一律前倒し・ref トレンド向き",
            cur_smooth,
            cur_hz,
            lead,
            directional=False,
            pedal_side=False,
        )
        run(
            "ペダル側で向き判定のみ",
            cur_smooth,
            cur_hz,
            lead,
            directional=False,
        )
        # 優先A の 2 つのノブ: 前倒しの向き別化（A-1）と a_req 窓の中心位置（A-2）。
        # 旧構成（一律前倒し・センタリング窓）を先頭に置いて比較基準にする。
        run("向き別前倒しのみ（ref トレンド向き）", cur_smooth, cur_hz, lead, pedal_side=False)
        for cr in (1.0, 0.75, 0.5, 0.25, 0.0):
            run(
                f"向き別前倒し・center={cr:.2f}",
                cur_smooth,
                cur_hz,
                lead,
                center_ratio=cr,
            )
        for cr in (1.0, 0.5, 0.0):
            run(
                f"一律前倒し・center={cr:.2f}",
                cur_smooth,
                cur_hz,
                lead,
                center_ratio=cr,
                directional=False,
            )
    elif args.sweep_lead:
        for candidate in (0.0, 0.1, 0.2, 0.3, 0.4, 0.5):
            run(f"lead={candidate:.1f}s", cur_smooth, cur_hz, candidate)
    elif args.sweep_lowpass:
        for smooth in (1.0, 0.5, 0.3):
            for hz in (0.25, 0.5, 1.0, 2.0):
                run(f"平滑{smooth}s LPF{hz}Hz", smooth, hz, lead)
    else:
        run("旧設定 (平滑1.0s LPF0.25Hz lead=0)", 1.0, 0.25, 0.0)
        run(
            f"現行設定 (平滑{cur_smooth}s LPF{cur_hz}Hz lead={lead:.2f}s)",
            cur_smooth,
            cur_hz,
            lead,
        )
    print(
        "\n※ 模擬は誤差を過大評価する（実機 p95 1.93 の構成で 2.6 前後）。"
        "構成間の優劣の判断にのみ使うこと。"
    )


if __name__ == "__main__":
    asyncio.run(main())
