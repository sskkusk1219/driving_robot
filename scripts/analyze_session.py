"""走行セッションの車速追従 KPI を drive_logs から解析するスクリプト。

Stage A/B/C の実機検証ゲート判定に使う。プライマリー KPI（p95≤0.2 / max≤1.0 /
符号反転≤1回/5s窓）に加え、Stage A の系統ラグ（実車速の基準に対する進み/遅れ）を
相互相関のラグ走査で確認する。

使い方:
    python -m scripts.analyze_session <session_id> [--latest] [--dsn ...]
    python -m scripts.analyze_session --latest auto      # 直近の auto セッション

DATABASE_URL 環境変数（既定 postgresql://localhost/driving_robot）を参照する。
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os

import asyncpg
import numpy as np

_HARD_LIMIT_KMH = 1.0
_P95_LIMIT_KMH = 0.2
_REVERSAL_WINDOW_S = 5.0
_SIGN_NOISE_FLOOR_KMH = 0.05
# ペダル「踏んでいる」しきい値 [%]（kpi_monitor._PEDAL_ON_THRESHOLD_PCT と揃える）。
_PEDAL_ON_THRESHOLD_PCT = 0.5
# B-7' ゲート目安: 110-135km/h 帯の accel ON 立ち上がり回数/min（現状 36、目標 ≤10）。
_PEDAL_ON_GATE_PER_MIN = 10.0


async def _fetch_logs(dsn: str, session_id: str) -> list[asyncpg.Record]:
    conn = await asyncpg.connect(dsn)
    try:
        return await conn.fetch(
            """
            SELECT timestamp, ref_speed_kmh, actual_speed_kmh, accel_opening, brake_opening
            FROM drive_logs WHERE session_id = $1 ORDER BY timestamp
            """,
            session_id,
        )
    finally:
        await conn.close()


async def _fetch_profile_params(dsn: str, session_id: str) -> dict | None:
    """セッションのプロファイルの車両定数（クリープ・エンジンブレーキ）を取得する。

    不要切替の「軌跡要求切替」を pedal_plan の分類器で計算するのに使う。取れなければ None。
    """
    conn = await asyncpg.connect(dsn)
    try:
        row = await conn.fetchrow(
            "SELECT p.feedforward_params FROM drive_sessions s "
            "JOIN vehicle_profiles p ON p.id = s.profile_id WHERE s.id = $1",
            session_id,
        )
    finally:
        await conn.close()
    if row is None or row["feedforward_params"] is None:
        return None
    raw = row["feedforward_params"]
    return json.loads(raw) if isinstance(raw, str) else dict(raw)


async def _latest_session(dsn: str, run_type: str | None) -> str | None:
    conn = await asyncpg.connect(dsn)
    try:
        if run_type:
            row = await conn.fetchrow(
                "SELECT id FROM drive_sessions WHERE run_type = $1 "
                "ORDER BY started_at DESC LIMIT 1",
                run_type,
            )
        else:
            row = await conn.fetchrow(
                "SELECT id FROM drive_sessions ORDER BY started_at DESC LIMIT 1"
            )
        return str(row["id"]) if row else None
    finally:
        await conn.close()


def _switch_metrics(
    records: list[asyncpg.Record], t: np.ndarray, ref: np.ndarray, dt: float, params: dict | None
) -> dict[str, float]:
    """アクセル⇔ブレーキ交互踏み（実測）と軌跡要求切替、その差＝不要切替を計算する。

    実測: 調停後のアクセル/ブレーキ開度から、異種ペダルが 2s 以内に踏まれた回数。
    要求: 基準軌跡を pedal_plan の分類器でフェーズ分けした DRIVE⇄BRAKE 切替（マージ後）。
    不要切替 = 実測 − 要求（回/min）。人間的な操作なら ≈0。params が無ければ要求は NaN。
    """
    acc = np.array([r["accel_opening"] for r in records], dtype=float)
    brk = np.array([r["brake_opening"] for r in records], dtype=float)
    dur_min = (t[-1] / 60.0) if t[-1] > 0 else 0.0

    def count_switches(pedal_state: np.ndarray) -> int:
        sw = 0
        last = 0
        last_t: float | None = None
        for i, s in enumerate(pedal_state):
            if s != 0 and s != last:
                if last != 0 and last_t is not None and (t[i] - last_t) < 2.0:
                    sw += 1
                last, last_t = int(s), float(t[i])
            elif s != 0:
                last_t = float(t[i])
        return sw

    measured_state = np.where(
        acc > _PEDAL_ON_THRESHOLD_PCT, 1, np.where(brk > _PEDAL_ON_THRESHOLD_PCT, -1, 0)
    )
    measured = count_switches(measured_state)
    measured_per_min = measured / dur_min if dur_min > 0 else 0.0

    required_per_min = float("nan")
    if params is not None:
        from src.domain.control.pedal_plan import (
            classify_phases,
            merge_micro_phases,
            required_accel,
        )
        from src.models.profile import FeedforwardParams

        ffp = FeedforwardParams(
            creep_speed_kmh=params.get("creep_speed_kmh", 7.0),
            creep_rate_kmhs=params.get("creep_rate_kmhs", 0.5),
            engine_brake_decel_kmhs=params.get("engine_brake_decel_kmhs", 1.0),
        )
        a_req = required_accel(ref, dt)
        phases = merge_micro_phases(classify_phases(ref, a_req, ffp), dt)
        from src.domain.control.pedal_plan import PlanPhase

        req_state = np.array(
            [1 if p == PlanPhase.DRIVE else (-1 if p == PlanPhase.BRAKE else 0) for p in phases]
        )
        required = count_switches(req_state)
        required_per_min = required / dur_min if dur_min > 0 else 0.0

    excess = (measured_per_min - required_per_min) if params is not None else float("nan")
    return {
        "seesaw_measured_per_min": measured_per_min,
        "seesaw_required_per_min": required_per_min,
        "seesaw_excess_per_min": excess,
    }


def _analyze(records: list[asyncpg.Record], params: dict | None = None) -> dict[str, float]:
    ts = np.array([r["timestamp"].timestamp() for r in records], dtype=float)
    t = ts - ts[0]
    ref = np.array([r["ref_speed_kmh"] for r in records], dtype=float)
    act = np.array([r["actual_speed_kmh"] for r in records], dtype=float)
    dev = act - ref
    abs_dev = np.abs(dev)
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.1

    # 符号反転（ノイズフロア超）の 5s 窓最大回数
    signs = np.where(dev > _SIGN_NOISE_FLOOR_KMH, 1, np.where(dev < -_SIGN_NOISE_FLOOR_KMH, -1, 0))
    rev_times = []
    last = 0
    for i, s in enumerate(signs):
        if s != 0:
            if last != 0 and s != last:
                rev_times.append(t[i])
            last = s
    rev = np.array(rev_times)
    max_rev_5s = 0
    if len(rev):
        max_rev_5s = int(max(((rev >= x) & (rev < x + _REVERSAL_WINDOW_S)).sum()
                             for x in np.arange(0.0, t[-1], 1.0)))

    # 系統ラグ: 実車速を ±N サンプルずらしたときの相関が最大になるラグ（負=実車速が先行）
    best_lag = 0
    best_corr = -2.0
    for lag in range(-20, 21):
        if lag >= 0:
            a, r = (act[lag:], ref[: len(ref) - lag]) if lag else (act, ref)
        else:
            a, r = act[:lag], ref[-lag:]
        if len(a) < 10:
            continue
        c = float(np.corrcoef(a, r)[0, 1])
        if c > best_corr:
            best_corr, best_lag = c, lag

    pedal = _pedal_activity(records, t, act, dt)
    switches = _switch_metrics(records, t, ref, dt, params)

    return {
        "n": float(len(t)),
        "dt_s": dt,
        "duration_s": float(t[-1]),
        "ref_max_kmh": float(ref.max()),
        "dev_p50": float(np.percentile(abs_dev, 50)),
        "dev_p95": float(np.percentile(abs_dev, 95)),
        "dev_max": float(abs_dev.max()),
        "viol_frac": float((abs_dev > _HARD_LIMIT_KMH).mean()),
        "viol_time_s": float((abs_dev > _HARD_LIMIT_KMH).sum() * dt),
        "reversal_max_per_5s": float(max_rev_5s),
        "best_lag_s": best_lag * dt,
        "best_lag_corr": best_corr,
        **pedal,
        **switches,
    }


def _pedal_activity(
    records: list[asyncpg.Record], t: np.ndarray, act: np.ndarray, dt: float
) -> dict[str, float]:
    """ペダルハンチング指標（B-7-4/B-7'）: アクセル ON-OFF 立ち上がり回数・速度帯別・OFF滞在。

    accel ON = 開度 > _PEDAL_ON_THRESHOLD_PCT。立ち上がり（0→非0）を全体と 110-135km/h 帯で
    数え、走行時間で正規化する（回/min）。OFF 滞在（ON→OFF→ON の谷）の中央値も出す。
    """
    acc = np.array([r["accel_opening"] for r in records], dtype=float)
    on = acc > _PEDAL_ON_THRESHOLD_PCT
    rising = np.flatnonzero((~on[:-1]) & on[1:]) + 1  # 立ち上がりサンプル位置
    dur_min = (t[-1] / 60.0) if t[-1] > 0 else 0.0
    total_on = int(rising.size)
    on_per_min = total_on / dur_min if dur_min > 0 else 0.0
    # 高速帯 110-135km/h に限定した立ち上がり密度（滞在時間で正規化）
    hs = (act >= 110.0) & (act < 135.0)
    hs_rise = int(((~on[:-1]) & on[1:] & hs[1:]).sum())
    hs_dwell_min = (float(hs.sum()) * dt) / 60.0
    hs_on_per_min = hs_rise / hs_dwell_min if hs_dwell_min > 0 else 0.0
    # OFF 滞在（立ち下がり→次の立ち上がりまでの時間）の中央値
    falling = np.flatnonzero(on[:-1] & (~on[1:])) + 1
    off_dwells = []
    ri = 0
    for f in falling:
        while ri < rising.size and rising[ri] <= f:
            ri += 1
        if ri < rising.size:
            off_dwells.append(float(t[rising[ri]] - t[f]))
    off_median = float(np.median(off_dwells)) if off_dwells else 0.0
    return {
        "accel_on_count": float(total_on),
        "accel_on_per_min": on_per_min,
        "accel_on_per_min_110_135": hs_on_per_min,
        "accel_off_dwell_median_s": off_median,
    }


def _print_report(session_id: str, m: dict[str, float]) -> None:
    def ok(passed: bool) -> str:
        return "OK " if passed else "NG "

    print(f"session {session_id}")
    print(f"  samples={m['n']:.0f} dt={m['dt_s']:.3f}s dur={m['duration_s']:.1f}s "
          f"ref_max={m['ref_max_kmh']:.1f}km/h")
    print("  --- プライマリー KPI ---")
    print(f"  {ok(m['dev_p95'] <= _P95_LIMIT_KMH)}|dev| p95 = {m['dev_p95']:.3f} km/h (≤0.2)")
    print(f"  {ok(m['dev_max'] <= _HARD_LIMIT_KMH)}|dev| max = {m['dev_max']:.3f} km/h (≤1.0)")
    print(f"  {ok(m['reversal_max_per_5s'] <= 1.0)}符号反転 max/5s = "
          f"{m['reversal_max_per_5s']:.0f} (≤1)")
    print(f"     |dev| p50 = {m['dev_p50']:.3f} km/h")
    print(f"     違反時間 = {m['viol_time_s']:.1f}s ({100 * m['viol_frac']:.1f}%)")
    print("  --- Stage A 系統ラグ ---")
    lag_ok = abs(m["best_lag_s"]) <= 0.1
    print(f"  {ok(lag_ok)}最良ラグ = {m['best_lag_s']:+.2f}s "
          f"(corr={m['best_lag_corr']:.4f})  負=実車速が先行")
    print("  --- B-7 ペダルハンチング ---")
    hs_ok = m["accel_on_per_min_110_135"] <= _PEDAL_ON_GATE_PER_MIN
    print(f"  {ok(hs_ok)}accel ON 110-135km/h = {m['accel_on_per_min_110_135']:.1f} 回/min (≤10)")
    print(f"     accel ON 全体 = {m['accel_on_count']:.0f} 回 "
          f"({m['accel_on_per_min']:.1f} 回/min)")
    print(f"     OFF 滞在中央値 = {m['accel_off_dwell_median_s']:.2f}s")
    print("  --- 不要切替（アクセル⇔ブレーキ交互踏み） ---")
    excess = m.get("seesaw_excess_per_min", float("nan"))
    req = m.get("seesaw_required_per_min", float("nan"))
    meas = m.get("seesaw_measured_per_min", float("nan"))
    if excess == excess:  # NaN でない（プロファイル定数が取得できた）
        print(f"  {ok(excess <= 2.0)}不要切替 = {excess:.1f} 回/min "
              f"（実測 {meas:.1f} − 軌跡要求 {req:.1f}、目安 ≤2）")
    else:
        print(f"     交互踏み（実測）= {meas:.1f} 回/min"
              "（軌跡要求は算出不可＝プロファイル定数なし）")


async def _main() -> None:
    ap = argparse.ArgumentParser(description="走行セッションの車速追従 KPI を解析する")
    ap.add_argument("session_id", nargs="?", help="解析対象の session_id (UUID)")
    ap.add_argument("--latest", metavar="RUN_TYPE", nargs="?", const="",
                    help="直近セッションを解析（RUN_TYPE 省略で全種別から最新）")
    ap.add_argument("--dsn", default=os.environ.get("DATABASE_URL",
                                                    "postgresql://localhost/driving_robot"))
    args = ap.parse_args()

    session_id = args.session_id
    if args.latest is not None:
        session_id = await _latest_session(args.dsn, args.latest or None)
        if session_id is None:
            print("該当セッションがありません")
            return
    if not session_id:
        ap.error("session_id か --latest を指定してください")

    records = await _fetch_logs(args.dsn, session_id)
    if not records:
        print(f"session {session_id} にログがありません")
        return
    params = await _fetch_profile_params(args.dsn, session_id)
    _print_report(session_id, _analyze(records, params))


if __name__ == "__main__":
    asyncio.run(_main())
