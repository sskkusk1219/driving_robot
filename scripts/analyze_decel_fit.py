"""減速 R² の原因調査スクリプト（読み取り専用・機能1）。

学習走行ログの減速側を coast(brake<deadband) と brake>=deadband に分離し、
それぞれ単独で Ridge を fit して R² を比較する。さらに brake_opening と実減速率(dv)の
相関・むだ時間ラグを数値化し、「定常ブレーキデータの追加だけで R²>=0.8 に届くか」を
判断する材料を標準出力に示す。DB / モデルは一切書き換えない。

使い方:
    .venv/bin/python -m scripts.analyze_decel_fit --profile-id <UUID> [--session-id <UUID> ...]
                                                  [--since 2026-06-30T07:00] [--until ...]
"""

from __future__ import annotations

import argparse
import asyncio
from datetime import datetime

import numpy as np
from sklearn.linear_model import Ridge
from sklearn.metrics import mean_absolute_error, mean_squared_error, r2_score

from src.domain.model_training import (
    _REGIME_COL,
    FEATURE_NAMES,
    _build_feature_matrix,
    _estimate_offsets,
    _group_by_session,
)
from src.infra.db import create_pool
from src.infra.profile_repository import ProfileRepository
from src.infra.session_repository import SessionRepository
from src.infra.settings import load_settings
from src.models.drive_log import DriveLog


def _fit_and_score(x: np.ndarray, y: np.ndarray) -> dict[str, float]:
    """与えられたサンプルで model_training と同条件の Ridge を fit して指標を返す。"""
    result: dict[str, float] = {"n": float(len(y))}
    if len(y) < 2:
        return result
    model = Ridge(alpha=1.0, fit_intercept=False)
    model.fit(x, y)
    pred = model.predict(x)
    result["mae"] = float(mean_absolute_error(y, pred))
    result["rmse"] = float(mean_squared_error(y, pred) ** 0.5)
    if float(np.var(y)) > 1e-9:
        result["r2"] = float(r2_score(y, pred))
    return result


def _fmt(metrics: dict[str, float]) -> str:
    n = int(metrics.get("n", 0))
    if "mae" not in metrics:
        return f"n={n} (サンプル不足で fit 不可)"
    r2 = metrics.get("r2")
    r2s = f"{r2:.3f}" if r2 is not None else "N/A(分散≈0)"
    return f"n={n} MAE={metrics['mae']:.3f} RMSE={metrics['rmse']:.3f} R²={r2s}"


def _build_decel_samples(
    grouped: list[list[DriveLog]], brake_db: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """全セッションの減速レジーム(dv_1.0<0)サンプルの (X, brake_label, brake_raw) を返す。"""
    x_parts: list[np.ndarray] = []
    y_parts: list[np.ndarray] = []
    raw_parts: list[np.ndarray] = []
    for session_logs in grouped:
        if len(session_logs) < 2:
            continue
        speed = np.clip(
            np.array([lg.actual_speed_kmh for lg in session_logs], dtype=float), 0.0, None
        )
        brake_raw = np.array([lg.brake_opening for lg in session_logs], dtype=float)
        brake_label = np.where(brake_raw >= brake_db, brake_raw, 0.0)
        offsets = _estimate_offsets([lg.timestamp for lg in session_logs])
        x, idx = _build_feature_matrix(speed, offsets)
        if len(idx) == 0:
            continue
        decel_mask = x[:, _REGIME_COL] < 0.0
        x_parts.append(x[decel_mask])
        y_parts.append(brake_label[idx][decel_mask])
        raw_parts.append(brake_raw[idx][decel_mask])
    if not x_parts:
        return np.empty((0, len(FEATURE_NAMES))), np.empty(0), np.empty(0)
    return np.vstack(x_parts), np.concatenate(y_parts), np.concatenate(raw_parts)


def _lag_analysis(grouped: list[list[DriveLog]], brake_db: float) -> None:
    """brake_opening と実減速率(dv: i→i+1 の負方向)のむだ時間ラグを相互相関で推定する。"""
    best_lags: list[int] = []
    dts: list[float] = []
    for session_logs in grouped:
        if len(session_logs) < 10:
            continue
        speed = np.clip(
            np.array([lg.actual_speed_kmh for lg in session_logs], dtype=float), 0.0, None
        )
        brake = np.array([lg.brake_opening for lg in session_logs], dtype=float)
        epochs = np.array([lg.timestamp.timestamp() for lg in session_logs])
        d = np.diff(epochs)
        d = d[d > 0.0]
        dt = float(np.median(d)) if len(d) > 0 else 0.1
        dts.append(dt)
        decel_rate = -np.diff(speed) / dt  # 正で減速が強い、長さ n-1
        b = brake[:-1]
        if np.std(b) < 1e-6 or np.std(decel_rate) < 1e-6:
            continue
        # brake を 0..N サンプル先行させたときに減速率と最も相関する遅れを探す
        max_lag = min(20, len(b) // 4)
        best_corr = -2.0
        best_lag = 0
        for lag in range(max_lag + 1):
            if lag == 0:
                bb, rr = b, decel_rate
            else:
                bb, rr = b[:-lag], decel_rate[lag:]
            if len(bb) < 3 or np.std(bb) < 1e-6 or np.std(rr) < 1e-6:
                continue
            corr = float(np.corrcoef(bb, rr)[0, 1])
            if corr > best_corr:
                best_corr = corr
                best_lag = lag
        best_lags.append(best_lag)
        print(
            f"  session {session_logs[0].session_id[:8]}: "
            f"最良ラグ={best_lag}サンプル(~{best_lag * dt:.2f}s) 相関={best_corr:.3f} dt={dt:.3f}s"
        )
    if best_lags:
        med_dt = float(np.median(dts))
        med_lag = float(np.median(best_lags))
        print(
            f"  → むだ時間ラグ中央値 {med_lag:.0f}サンプル (~{med_lag * med_dt:.2f}s)"
        )


async def _load_logs(
    profile_id: str,
    session_ids: list[str] | None,
    since: datetime | None,
    until: datetime | None,
) -> tuple[list[DriveLog], float]:
    settings = load_settings()
    pool = await create_pool(settings.database.dsn)
    try:
        profile_repo = ProfileRepository(pool)
        session_repo = SessionRepository(pool)
        profile = await profile_repo.get_by_id(profile_id)
        if profile is None:
            raise SystemExit(f"プロファイルが見つかりません: {profile_id}")
        brake_db = profile.feedforward_params.brake_deadband_pct
        logs = await session_repo.list_logs_for_training(profile_id, session_ids or None)
    finally:
        await pool.close()
    if since is not None:
        logs = [lg for lg in logs if lg.timestamp >= since]
    if until is not None:
        logs = [lg for lg in logs if lg.timestamp <= until]
    return logs, brake_db


def main() -> None:
    parser = argparse.ArgumentParser(description="減速 R² の原因調査（読み取り専用）")
    parser.add_argument("--profile-id", required=True, help="対象プロファイル UUID")
    parser.add_argument(
        "--session-id", action="append", dest="session_ids", help="対象セッション（複数可）"
    )
    parser.add_argument("--since", help="ISO8601 開始日時でフィルタ（例 2026-06-30T07:00）")
    parser.add_argument("--until", help="ISO8601 終了日時でフィルタ")
    args = parser.parse_args()

    since = datetime.fromisoformat(args.since) if args.since else None
    until = datetime.fromisoformat(args.until) if args.until else None
    logs, brake_db = asyncio.run(
        _load_logs(args.profile_id, args.session_ids, since, until)
    )

    grouped = _group_by_session(logs)
    print("=== 減速 R² 原因調査 ===")
    print(f"総ログ {len(logs)} 点 / {len(grouped)} セッション / brake_deadband={brake_db}%\n")

    x, y, raw = _build_decel_samples(grouped, brake_db)
    if len(y) == 0:
        raise SystemExit("減速サンプルが 0 件です。対象セッション/期間を確認してください。")

    # 全減速サンプル（現行 model_training と同じ集団）
    print("[1] 全減速サンプル（現行モデルと同条件）")
    print(f"  {_fmt(_fit_and_score(x, y))}\n")

    # coast / brake 分離
    coast_mask = raw < brake_db
    brake_mask = ~coast_mask
    print("[2] coast(brake<deadband) と brake>=deadband に分離して個別 fit")
    print(f"  coast : {_fmt(_fit_and_score(x[coast_mask], y[coast_mask]))}")
    print(f"  brake : {_fmt(_fit_and_score(x[brake_mask], y[brake_mask]))}")
    coast_frac = float(np.mean(coast_mask)) * 100.0
    print(
        f"  → 減速サンプルの {coast_frac:.1f}% が coast(ブレーキ実質0)。"
        f"ラベルの大半が 0 に張り付く構造\n"
    )

    # brake_opening ↔ 実減速率の関係（brake>0 サンプルのみ）
    print("[3] brake_opening ↔ 実減速率(dv_1.0 を負反転)の相関（brake>=deadband のみ）")
    if int(np.sum(brake_mask)) >= 3:
        bx = raw[brake_mask]
        # dv_1.0 列 = _REGIME_COL。減速なので負。減速率は -dv。
        bdv = -x[brake_mask, _REGIME_COL]
        if np.std(bx) > 1e-6 and np.std(bdv) > 1e-6:
            corr = float(np.corrcoef(bx, bdv)[0, 1])
            print(f"  Pearson 相関(brake_opening, 減速率) = {corr:.3f}")
        print(
            f"  brake_opening 統計: 中央値 {np.median(bx):.1f}% "
            f"[{np.min(bx):.1f}, {np.max(bx):.1f}]"
        )
    else:
        print("  brake>=deadband サンプルが不足（定常ブレーキ計測が未採取の可能性）")
    print()

    # むだ時間ラグ
    print("[4] brake→減速 のむだ時間ラグ（相互相関のピーク）")
    _lag_analysis(grouped, brake_db)


if __name__ == "__main__":
    main()
