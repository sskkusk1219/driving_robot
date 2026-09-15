"""むだ時間 θ の同定方式をオフライン比較するスクリプト（実機不要・DB 読み取りのみ）。

旧方式（`_segment_fopdt` のオンセット検出＝「区間開始から実車速が 0.3km/h 上昇するまで」）と
新方式（`_segment_theta_xcorr` のアクセル開度×加速度応答の相互相関ピーク）を、既存の
学習サイクルすべてに当てて θ のばらつきを比べる。

θ は FB ロバスト上限 Kc・プラン参照の前倒し・ILC リード Δ の 3 か所を同時に決めるため、
サイクルごとに 0.30〜1.05s（3.5 倍）とばらつくと「毎回違う制御器で学習している」状態になる
（docs/Problem/引き継ぎ20260909.md 4-②、docs/Problem/制御フロー.md 8-④）。
本スクリプトの目的は「新方式でばらつきが縮むか」をゲート（±20% 以内）で判定すること。

θ を決めるのは学習サイクルの TRAINING_1（`run_type='learning'` の 1 セッション、
`update_pid_gains=True`）なので、既定ではそのセッションだけを見る。

使い方:
    .venv/bin/python -m scripts.compare_theta_id
    .venv/bin/python -m scripts.compare_theta_id --cycle-id <UUID> --run-types learning,tuning

DATABASE_URL 環境変数（既定 postgresql://localhost/driving_robot）を参照する。
"""

from __future__ import annotations

import argparse
import asyncio
import os
from dataclasses import dataclass
from zoneinfo import ZoneInfo

import asyncpg
import numpy as np

from src.domain.model_training import DEFAULT_DT_S
from src.domain.pid_tuning import (
    MIN_SEGMENTS_FOR_ID,
    _find_accel_segments,
    _segment_fopdt,
    _segment_theta_xcorr,
    smooth_theta,
)

# 判定ゲート: サイクル間の θ ばらつき（中央値まわりの最大偏差率）。
_SPREAD_GATE = 0.20
# DB の時刻は UTC 保存。報告は JST で出す（ユーザーが照合できるように）。
_JST = ZoneInfo("Asia/Tokyo")


@dataclass
class CycleTheta:
    """1 学習サイクル分の θ 同定結果（方式ごと）。"""

    cycle_id: str
    started_jst: str
    onset: list[float]
    xcorr: list[float]

    @staticmethod
    def _med(values: list[float]) -> float:
        return float(np.median(values)) if values else float("nan")

    @property
    def theta_onset(self) -> float:
        return self._med(self.onset)

    @property
    def theta_xcorr(self) -> float:
        return self._med(self.xcorr)


async def _fetch_cycles(conn: asyncpg.Connection, cycle_id: str | None) -> list[asyncpg.Record]:
    if cycle_id:
        return await conn.fetch(
            "SELECT id, started_at FROM learning_cycles WHERE id = $1", cycle_id
        )
    return await conn.fetch("SELECT id, started_at FROM learning_cycles ORDER BY started_at")


async def _fetch_cycle_logs(
    conn: asyncpg.Connection, cycle_id: str, run_types: list[str]
) -> list[list[asyncpg.Record]]:
    """サイクル配下のセッションごとに、時刻昇順のログを返す。

    セッション境界をまたいで区間を切らないよう、セッション単位で分けたまま返す
    （`model_training._group_by_session` と同じ理由）。
    """
    rows = await conn.fetch(
        "SELECT id FROM drive_sessions WHERE cycle_id = $1 AND run_type = ANY($2::text[]) "
        "ORDER BY started_at",
        cycle_id,
        run_types,
    )
    out: list[list[asyncpg.Record]] = []
    for row in rows:
        logs = await conn.fetch(
            "SELECT timestamp, actual_speed_kmh, accel_opening, brake_opening "
            "FROM drive_logs WHERE session_id = $1 ORDER BY timestamp",
            row["id"],
        )
        if logs:
            out.append(logs)
    return out


async def _fetch_brake_deadband(conn: asyncpg.Connection, cycle_id: str) -> float:
    """サイクルのプロファイルのブレーキ不感帯 [%]。区間抽出の「ブレーキオフ」判定に使う。"""
    row = await conn.fetchrow(
        "SELECT p.feedforward_params->>'brake_deadband_pct' AS db "
        "FROM learning_cycles c JOIN vehicle_profiles p ON p.id = c.profile_id "
        "WHERE c.id = $1",
        cycle_id,
    )
    if row is None or row["db"] is None:
        return 1.0
    return float(row["db"])


def _thetas_for_session(
    logs: list[asyncpg.Record], brake_db: float
) -> tuple[list[float], list[float]]:
    """1 セッションのログから、区間ごとの θ（旧方式・新方式）を返す。"""
    speed = np.clip(np.array([r["actual_speed_kmh"] for r in logs], dtype=float), 0.0, None)
    accel = np.array([r["accel_opening"] for r in logs], dtype=float)
    brake = np.array([r["brake_opening"] for r in logs], dtype=float)

    epochs = np.array([r["timestamp"].timestamp() for r in logs], dtype=float)
    d = np.diff(epochs)
    d = d[d > 0.0]
    dt = float(np.median(d)) if len(d) > 0 else DEFAULT_DT_S

    onset: list[float] = []
    xcorr: list[float] = []
    for s, e in _find_accel_segments(speed, accel, brake, brake_db):
        fit = _segment_fopdt(speed[s:e], accel[s:e], dt)
        if fit is not None:
            onset.append(fit[2])
        theta_x = _segment_theta_xcorr(speed, accel, dt, s, e)
        if theta_x is not None:
            xcorr.append(theta_x)
    return onset, xcorr


def _spread(values: list[float]) -> float:
    """中央値に対する最大の相対偏差（`max|v−med|/med`）。ばらつきの判定に使う。"""
    vals = [v for v in values if v == v]
    if len(vals) < 2:
        return float("nan")
    med = float(np.median(vals))
    if med <= 0.0:
        return float("nan")
    return max(abs(v - med) for v in vals) / med


def _print_report(results: list[CycleTheta]) -> None:
    print(f"{'cycle':10s} {'started(JST)':>13s} "
          f"{'θ_onset':>9s} {'n':>3s} {'θ_xcorr':>9s} {'n':>3s}")
    for r in results:
        n_on = len(r.onset)
        n_xc = len(r.xcorr)
        print(
            f"{r.cycle_id[:8]:10s} {r.started_jst:>13s} "
            f"{r.theta_onset:9.2f} {n_on:3d} {r.theta_xcorr:9.2f} {n_xc:3d}"
        )

    print()
    for label, values in (
        ("旧: オンセット", [r.theta_onset for r in results]),
        ("新: 相互相関  ", [r.theta_xcorr for r in results]),
    ):
        vals = [v for v in values if v == v]
        if not vals:
            print(f"{label}: 有効なサイクルなし")
            continue
        spread = _spread(vals)
        ratio = max(vals) / min(vals) if min(vals) > 0 else float("nan")
        gate = "OK " if spread <= _SPREAD_GATE else "NG "
        print(
            f"{label}: 中央値 {np.median(vals):.2f}s  範囲 {min(vals):.2f}-{max(vals):.2f}s "
            f"({ratio:.1f} 倍)  ばらつき {100 * spread:.0f}%  {gate}"
            f"(ゲート ≤{100 * _SPREAD_GATE:.0f}%)"
        )

    # 実際に制御へ渡る θ は smooth_theta（前回値との EMA）を通した値。生の同定値が
    # 走行間で跳ねても、FB ロバスト上限・プラン前倒し・ILC リードが 2〜3.5 倍動かない
    # ことをここで確認する。
    applied: list[float] = []
    theta: float | None = None
    for r in results:
        value = r.theta_xcorr
        if value != value:
            continue
        theta = smooth_theta(theta, value)
        applied.append(theta)
    if len(applied) >= 3:
        # 初回はシード（同定値そのまま）なので 2 本目以降の連続比で見る。
        ratios = [
            max(a, b) / min(a, b)
            for a, b in zip(applied[1:], applied[2:], strict=False)
        ]
        worst = max(ratios)
        gate = "OK " if worst <= 1.2 else "NG "
        print(
            "制御へ渡る θ（EMA 後）: "
            + " → ".join(f"{v:.2f}" for v in applied)
            + f"  連続サイクル比 最大 {worst:.2f} 倍  {gate}(ゲート ≤1.20)"
        )

    print()
    print(
        "見方: 同定値そのもののばらつきが ±20% 以内なら θ をそのまま使える。収まらない場合は\n"
        "      EMA 後の「連続サイクル比 ≤1.20」が満たされていれば、FB 権限・前倒し量が\n"
        "      走行ごとに大きく変わらない＝ILC の学習対象が安定する、と判断してよい。"
    )


async def _main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--cycle-id", help="特定の learning_cycles.id だけを見る（既定: 全件）")
    ap.add_argument(
        "--run-types",
        default="learning",
        help="対象の run_type（カンマ区切り）。既定 learning＝θ を決める TRAINING_1 と同じ",
    )
    ap.add_argument(
        "--dsn",
        default=os.environ.get("DATABASE_URL", "postgresql://localhost/driving_robot"),
    )
    args = ap.parse_args()
    run_types = [x.strip() for x in args.run_types.split(",") if x.strip()]

    conn = await asyncpg.connect(args.dsn)
    try:
        cycles = await _fetch_cycles(conn, args.cycle_id)
        results: list[CycleTheta] = []
        for c in cycles:
            cycle_id = str(c["id"])
            brake_db = await _fetch_brake_deadband(conn, cycle_id)
            sessions = await _fetch_cycle_logs(conn, cycle_id, run_types)
            onset: list[float] = []
            xcorr: list[float] = []
            for logs in sessions:
                o, x = _thetas_for_session(logs, brake_db)
                onset.extend(o)
                xcorr.extend(x)
            # identify_fopdt と同じ足切り（有効区間が足りないサイクルは同定不能）
            results.append(
                CycleTheta(
                    cycle_id=cycle_id,
                    started_jst=c["started_at"].astimezone(_JST).strftime("%m-%d %H:%M"),
                    onset=onset if len(onset) >= MIN_SEGMENTS_FOR_ID else [],
                    xcorr=xcorr if len(xcorr) >= MIN_SEGMENTS_FOR_ID else [],
                )
            )
    finally:
        await conn.close()

    if not results:
        print("学習サイクルが見つかりません")
        return
    _print_report(results)


if __name__ == "__main__":
    asyncio.run(_main())
