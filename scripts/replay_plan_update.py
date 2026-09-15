"""ILC(プラン更新)のむだ時間シフト Δ が正しく位相補償できているかを、
既存の学習サイクルのログだけを使ってオフラインで検証するツール（読み取り専用・実機不要）。

学習サイクル内の連続する走行ペア (session_n → session_n+1) について、
実際にプランがどう更新されたか（Δplan = plan_effort_pct の差分）と、
session_n の追従誤差 e(t) = actual - ref の相互相関を lag を振って計算する。

ILC が正しく位相補償できていれば、Δplan は -e(t+Δ) に追従するはずなので、
相互相関のピークは lag=0 付近（Δ 自体は update_plan 内で織り込み済みのため）に来る。
ピークが lag>0 にずれている場合、そのズレが「実際に必要だったリード時間」であり、
学習サイクルの FOPDT 同定値（theta・tau）と比較することで
Δ = theta（旧実装）と Δ = theta + response_s（新実装）のどちらが実測ズレに近いかを示す。

使い方:
    .venv/bin/python scripts/replay_plan_update.py --cycle-id <cycle_id>
"""

from __future__ import annotations

import argparse
import asyncio
import os
from dataclasses import dataclass

import asyncpg
import numpy as np

from src.domain.control.plan_update import PLAN_LEAD_RESPONSE_S, lead_time_from_fopdt

DEFAULT_DATABASE_URL = "postgresql://localhost/driving_robot"
_GRID_DT_S = 0.1
_LAG_RANGE_S = 3.0


@dataclass
class _SessionSeries:
    session_id: str
    run_type: str
    started_at: object
    t: np.ndarray
    ref: np.ndarray
    actual: np.ndarray
    plan_effort: np.ndarray


async def _fetch_cycle_sessions(conn: asyncpg.Connection, cycle_id: str) -> list[_SessionSeries]:
    rows = await conn.fetch(
        """
        SELECT id, run_type, started_at
        FROM drive_sessions
        WHERE cycle_id = $1 AND run_type IN ('tuning', 'auto')
        ORDER BY started_at ASC
        """,
        cycle_id,
    )
    out: list[_SessionSeries] = []
    for row in rows:
        logs = await conn.fetch(
            """
            SELECT timestamp, ref_speed_kmh, actual_speed_kmh, plan_effort_pct
            FROM drive_logs
            WHERE session_id = $1
            ORDER BY timestamp
            """,
            row["id"],
        )
        if len(logs) < 20:
            continue
        t0 = logs[0]["timestamp"]
        t = np.array([(log_["timestamp"] - t0).total_seconds() for log_ in logs])
        ref = np.array([float(log_["ref_speed_kmh"] or 0.0) for log_ in logs])
        actual = np.array([float(log_["actual_speed_kmh"] or 0.0) for log_ in logs])
        plan = np.array([float(log_["plan_effort_pct"] or 0.0) for log_ in logs])
        out.append(
            _SessionSeries(
                session_id=str(row["id"]),
                run_type=row["run_type"],
                started_at=row["started_at"],
                t=t,
                ref=ref,
                actual=actual,
                plan_effort=plan,
            )
        )
    return out


async def _fetch_fopdt(
    conn: asyncpg.Connection, cycle_id: str
) -> tuple[float | None, float | None]:
    row = await conn.fetchrow(
        "SELECT detail FROM learning_cycles WHERE id = $1",
        cycle_id,
    )
    if row is None or row["detail"] is None:
        return None, None
    import json

    detail = row["detail"] if isinstance(row["detail"], dict) else json.loads(row["detail"])
    fopdt = detail.get("fopdt") or {}
    return fopdt.get("fopdt_theta"), fopdt.get("fopdt_tau")


def _best_lag(
    delta_plan: np.ndarray, err: np.ndarray, dt: float, lag_range_s: float
) -> tuple[float, float]:
    """delta_plan と err の相互相関が最大になる lag [s] と相関係数を返す。"""
    n_lag = int(round(lag_range_s / dt))
    edge = max(n_lag, 1)
    best_lag, best_corr = 0.0, 0.0
    for k in range(-n_lag, n_lag + 1):
        shifted = np.roll(err, k)
        a = delta_plan[edge:-edge]
        b = shifted[edge:-edge]
        if a.std() == 0.0 or b.std() == 0.0:
            continue
        corr = float(np.corrcoef(a, b)[0, 1])
        if abs(corr) > abs(best_corr):
            best_lag, best_corr = k * dt, corr
    return best_lag, best_corr


def _corr_at_lag(delta_plan: np.ndarray, err: np.ndarray, dt: float, lag_s: float) -> float:
    k = int(round(lag_s / dt))
    shifted = np.roll(err, k)
    edge = max(abs(k), 1)
    a = delta_plan[edge:-edge]
    b = shifted[edge:-edge]
    if a.std() == 0.0 or b.std() == 0.0:
        return 0.0
    return float(np.corrcoef(a, b)[0, 1])


async def _cycle_pairs(
    conn: asyncpg.Connection, cycle_id: str, response_s: float
) -> tuple[float | None, float, float, list[tuple[str, float, float, float, float]]]:
    """1 サイクル分のペア比較結果を返す。

    Returns:
        (theta, Δ旧, Δ新, [(ラベル, best_lag, corr@best, corr@旧Δ, corr@新Δ), ...])
    """
    sessions = await _fetch_cycle_sessions(conn, cycle_id)
    theta, tau = await _fetch_fopdt(conn, cycle_id)
    delta_old = theta if theta is not None else 0.0
    delta_new = lead_time_from_fopdt(theta, tau, response_s=response_s)
    if len(sessions) < 2:
        return theta, delta_old, delta_new, []

    # 走行長はログ実長（ペア共通の最短）から取る。旧実装は 190s 決め打ちで、
    # 検証パターン以外のモードでは末尾が外挿（np.interp の端点クランプ）になっていた。
    duration_s = min(float(s.t[-1]) for s in sessions)
    grid = np.arange(0.0, duration_s, _GRID_DT_S)
    rows: list[tuple[str, float, float, float, float]] = []
    for i in range(len(sessions) - 1):
        cur, nxt = sessions[i], sessions[i + 1]
        plan_cur = np.interp(grid, cur.t, cur.plan_effort)
        plan_nxt = np.interp(grid, nxt.t, nxt.plan_effort)
        ref_cur = np.interp(grid, cur.t, cur.ref)
        act_cur = np.interp(grid, cur.t, cur.actual)
        err = act_cur - ref_cur
        delta_plan = plan_nxt - plan_cur

        best_lag, best_corr = _best_lag(delta_plan, err, _GRID_DT_S, _LAG_RANGE_S)
        rows.append(
            (
                f"{cur.session_id[:8]}->{nxt.session_id[:8]}",
                best_lag,
                best_corr,
                _corr_at_lag(delta_plan, err, _GRID_DT_S, delta_old),
                _corr_at_lag(delta_plan, err, _GRID_DT_S, delta_new),
            )
        )
    return theta, delta_old, delta_new, rows


_LEGEND = (
    "符号規約: 本スクリプトの err = actual − ref で、plan_update の err_arr = ref − actual とは\n"
    "          **逆符号**。正しい ILC 挙動（速度が足りない所で次回プランを踏み増す）は\n"
    "          ここでは**負の相関**として出る。**corr がより負であるほど良い**。\n"
    "見方: best_lag は |corr| が最大になる lag（符号は問わない）。0 に近いほど\n"
    "      「Δplan は現在の誤差に正しく追従」= 発散しにくい。"
)


async def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cycle-id", help="learning_cycles.id（--all-cycles 指定時は不要）")
    parser.add_argument(
        "--all-cycles",
        action="store_true",
        help="全サイクルを走査し、Δ=theta と Δ=theta+response_s の勝敗を集計する",
    )
    parser.add_argument(
        "--response-s",
        type=float,
        default=PLAN_LEAD_RESPONSE_S,
        help=(
            "新実装の Δ=theta+response_s の response_s [s]"
            "（既定: plan_update.PLAN_LEAD_RESPONSE_S）"
        ),
    )
    args = parser.parse_args()
    if not args.cycle_id and not args.all_cycles:
        parser.error("--cycle-id か --all-cycles を指定してください")

    database_url = os.environ.get("DATABASE_URL", DEFAULT_DATABASE_URL)
    conn = await asyncpg.connect(database_url)
    try:
        if args.all_cycles:
            cycle_ids = [
                str(r["id"])
                for r in await conn.fetch(
                    "SELECT id FROM learning_cycles ORDER BY started_at"
                )
            ]
        else:
            cycle_ids = [args.cycle_id]
        results = [
            (cid, await _cycle_pairs(conn, cid, args.response_s)) for cid in cycle_ids
        ]
    finally:
        await conn.close()

    wins_old = wins_new = ties = 0
    for cycle_id, (theta, delta_old, delta_new, rows) in results:
        if not rows:
            print(f"サイクル {cycle_id}: 比較可能なセッションペアがありません")
            continue
        print(f"サイクル {cycle_id}")
        print(f"  theta={theta}  旧 Δ={delta_old:.3f}s  新 Δ(+{args.response_s}s)={delta_new:.3f}s")
        print(
            f"{'pair':22s} {'best_lag[s]':>11s} {'corr@best':>10s} "
            f"{'corr@旧Δ':>10s} {'corr@新Δ':>10s}"
        )
        for label, best_lag, best_corr, corr_old, corr_new in rows:
            print(
                f"{label:22s} {best_lag:11.2f} {best_corr:10.2f} "
                f"{corr_old:10.2f} {corr_new:10.2f}"
            )
            # より負＝正しい向きの相関が強い。
            if corr_old < corr_new:
                wins_old += 1
            elif corr_new < corr_old:
                wins_new += 1
            else:
                ties += 1
        print()

    if args.all_cycles:
        total = wins_old + wins_new + ties
        print(f"=== 集計（全 {total} ペア）===")
        print(f"  Δ=theta            が優位: {wins_old} ペア")
        print(f"  Δ=theta+{args.response_s}s が優位: {wins_new} ペア")
        print(f"  引き分け                  : {ties} ペア")
        print()
    print(_LEGEND)


if __name__ == "__main__":
    asyncio.run(main())
