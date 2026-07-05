"""FeatureSpec 候補セットのオフライン評価スクリプト（読み取り専用）。

既存の走行ログ（学習運転 = train / PID適合走行 = holdout）で候補特徴量セットを
in-sample・holdout の両面から比較し、本番の特徴量セット変更（9特徴からの変更）を
判断する材料を標準出力・JSON に出力する。DB / モデルファイルは一切書き換えない。

holdout は「実速度(actual_speed_kmh)から特徴構築」(holdout-A) と
「基準速度(ref_speed_kmh)から特徴構築」(holdout-B) の両方で評価する。学習は
ノイジーな実速度、推論（配備時）は滑らかな基準速度を使うため、この2つの差
（A-Bギャップ）が大きい特徴（特に短期ホライズン）は配備条件で性能劣化しやすい。
採否判断は配備条件に近い **holdout-B MAE** を優先すること。

使い方:
    # 学習サイクル指定（train=サイクルの学習セッション、holdout=サイクルの適合走行セッション）
    .venv/bin/python -m scripts.evaluate_feature_sets --profile-id <UUID> --cycle-id <UUID>

    # セッション明示指定
    .venv/bin/python -m scripts.evaluate_feature_sets --profile-id <UUID> \
        --train-session-id <UUID> --eval-session-id <UUID> [--eval-session-id <UUID> ...]

    # 候補セット・出力を指定
    .venv/bin/python -m scripts.evaluate_feature_sets --profile-id <UUID> --cycle-id <UUID> \
        --specs baseline short_lookahead --json-out result.json
"""

from __future__ import annotations

import argparse
import asyncio
import json
from dataclasses import asdict
from typing import Any

import numpy as np

from src.domain.model_training import (
    DEFAULT_FEATURE_SPEC,
    FeatureSpec,
    _build_feature_matrix,
    _estimate_offsets,
    _group_by_session,
    _make_estimator,
    _metrics,
)
from src.infra.db import create_pool
from src.infra.profile_repository import ProfileRepository
from src.infra.session_repository import SessionRepository
from src.infra.settings import load_settings
from src.models.drive_log import DriveLog

# 組み込み候補セット。baseline は本番デフォルトと完全一致（比較基準）。
BUILTIN_SPECS: dict[str, FeatureSpec] = {
    "baseline": DEFAULT_FEATURE_SPEC,
    "short_lookahead": FeatureSpec(
        lookahead_horizons_s=(0.1, 0.2, 0.3, 0.5, 1.0, 2.0, 3.0),
        past_horizons_s=(0.1, 0.2, 0.5, 1.0),
        regime_horizon_s=1.0,
    ),
    "short_plus_accel": FeatureSpec(
        lookahead_horizons_s=(0.1, 0.2, 0.3, 0.5, 1.0, 2.0, 3.0),
        past_horizons_s=(0.1, 0.2, 0.5, 1.0),
        regime_horizon_s=1.0,
        accel_horizons_s=(0.2, 0.5),
    ),
}


def _speed_array(session_logs: list[DriveLog], speed_key: str) -> np.ndarray | None:
    """指定キーの速度系列を返す。ref_speed_kmh 欠落セッションは None（スキップ対象）。"""
    raw = [getattr(lg, speed_key) for lg in session_logs]
    if any(v is None for v in raw):
        return None
    result: np.ndarray = np.clip(np.array(raw, dtype=float), 0.0, None)
    return result


def _combined_feature_matrix(
    logs: list[DriveLog], spec: FeatureSpec, speed_key: str
) -> np.ndarray:
    """レジーム分割前の結合特徴行列を返す（特徴std比の算出用）。"""
    parts: list[np.ndarray] = []
    for session_logs in _group_by_session(logs):
        if len(session_logs) < 2:
            continue
        speed = _speed_array(session_logs, speed_key)
        if speed is None:
            continue
        timestamps = [lg.timestamp for lg in session_logs]
        offsets = _estimate_offsets(timestamps, spec.lookahead_horizons_s)
        past_offsets = _estimate_offsets(timestamps, spec.past_horizons_s)
        x, idx = _build_feature_matrix(speed, offsets, past_offsets, spec)
        if len(idx) > 0:
            parts.append(x)
    return np.vstack(parts) if parts else np.empty((0, len(spec.feature_names())))


def _build_regime_samples(
    logs: list[DriveLog], spec: FeatureSpec, brake_db: float, speed_key: str
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """ログから (x_accel, y_accel, x_brake, y_brake) を構築する（train_inverse_model と同条件）。

    speed_key: "actual_speed_kmh"（実速度）または "ref_speed_kmh"（基準速度）。
    ref_speed_kmh が欠落するセッションはスキップする（呼び出し側で警告を出す）。
    """
    x_accel_parts: list[np.ndarray] = []
    y_accel_parts: list[np.ndarray] = []
    x_brake_parts: list[np.ndarray] = []
    y_brake_parts: list[np.ndarray] = []
    regime_col = spec.regime_col()

    for session_logs in _group_by_session(logs):
        if len(session_logs) < 2:
            continue
        speed = _speed_array(session_logs, speed_key)
        if speed is None:
            continue
        accel_open = np.array([lg.accel_opening for lg in session_logs], dtype=float)
        brake_raw = np.array([lg.brake_opening for lg in session_logs], dtype=float)
        brake_open = np.where(brake_raw >= brake_db, brake_raw, 0.0)

        timestamps = [lg.timestamp for lg in session_logs]
        offsets = _estimate_offsets(timestamps, spec.lookahead_horizons_s)
        past_offsets = _estimate_offsets(timestamps, spec.past_horizons_s)
        x, idx = _build_feature_matrix(speed, offsets, past_offsets, spec)
        if len(idx) == 0:
            continue

        accel_mask = x[:, regime_col] >= 0.0
        x_accel_parts.append(x[accel_mask])
        y_accel_parts.append(accel_open[idx][accel_mask])
        x_brake_parts.append(x[~accel_mask])
        y_brake_parts.append(brake_open[idx][~accel_mask])

    n_features = len(spec.feature_names())
    x_accel = np.vstack(x_accel_parts) if x_accel_parts else np.empty((0, n_features))
    y_accel = np.concatenate(y_accel_parts) if y_accel_parts else np.empty(0)
    x_brake = np.vstack(x_brake_parts) if x_brake_parts else np.empty((0, n_features))
    y_brake = np.concatenate(y_brake_parts) if y_brake_parts else np.empty(0)
    return x_accel, y_accel, x_brake, y_brake


def _feature_std_ratio(
    x_actual: np.ndarray, x_ref: np.ndarray, feature_names: list[str]
) -> dict[str, float]:
    """特徴ごとの std(actual) / std(ref) を返す（ノイズ増幅の可視化）。"""
    if len(x_actual) == 0 or len(x_ref) == 0:
        return {}
    std_actual = np.std(x_actual, axis=0)
    std_ref = np.std(x_ref, axis=0)
    return {
        name: (float(sa / sr) if sr > 1e-9 else float("inf"))
        for name, sa, sr in zip(feature_names, std_actual, std_ref, strict=True)
    }


def _safe_metrics(model: Any, x: np.ndarray, y: np.ndarray) -> dict[str, float]:
    if len(y) == 0:
        return {"n": 0.0}
    return _metrics(model, x, y)


def _weighted_mae(accel: dict[str, float], brake: dict[str, float]) -> float | None:
    """accel/brake の MAE をサンプル数で加重平均する（順位付け用の単一指標）。"""
    na, nb = accel.get("n", 0.0), brake.get("n", 0.0)
    if na + nb <= 0.0:
        return None
    ma, mb = accel.get("mae"), brake.get("mae")
    if ma is None or mb is None:
        return None
    return (ma * na + mb * nb) / (na + nb)


def evaluate_spec(
    name: str,
    spec: FeatureSpec,
    train_logs: list[DriveLog],
    holdout_logs: list[DriveLog],
    brake_db: float,
) -> dict[str, Any]:
    """1候補セットの in-sample / holdout-A / holdout-B 指標を算出する。"""
    result: dict[str, Any] = {"name": name, "feature_names": spec.feature_names()}

    x_accel, y_accel, x_brake, y_brake = _build_regime_samples(
        train_logs, spec, brake_db, "actual_speed_kmh"
    )
    if len(y_accel) < 2 or len(y_brake) < 2:
        result["error"] = (
            f"学習サンプル不足（accel={len(y_accel)}点 brake={len(y_brake)}点）。"
            "評価をスキップします。"
        )
        return result

    accel_model = _make_estimator()
    accel_model.fit(x_accel, y_accel)
    brake_model = _make_estimator()
    brake_model.fit(x_brake, y_brake)
    result["in_sample"] = {
        "accel": _metrics(accel_model, x_accel, y_accel),
        "brake": _metrics(brake_model, x_brake, y_brake),
    }

    # holdout-A: 実速度(actual_speed_kmh)から特徴構築
    xa_accel, ya_accel, xa_brake, ya_brake = _build_regime_samples(
        holdout_logs, spec, brake_db, "actual_speed_kmh"
    )
    holdout_a = {
        "accel": _safe_metrics(accel_model, xa_accel, ya_accel),
        "brake": _safe_metrics(brake_model, xa_brake, ya_brake),
    }
    result["holdout_a"] = holdout_a
    result["holdout_a_mae"] = _weighted_mae(holdout_a["accel"], holdout_a["brake"])

    # holdout-B: 基準速度(ref_speed_kmh、配備条件に近い)から特徴構築
    ref_available = any(lg.ref_speed_kmh is not None for lg in holdout_logs)
    if ref_available:
        xb_accel, yb_accel, xb_brake, yb_brake = _build_regime_samples(
            holdout_logs, spec, brake_db, "ref_speed_kmh"
        )
        holdout_b = {
            "accel": _safe_metrics(accel_model, xb_accel, yb_accel),
            "brake": _safe_metrics(brake_model, xb_brake, yb_brake),
        }
        result["holdout_b"] = holdout_b
        result["holdout_b_mae"] = _weighted_mae(holdout_b["accel"], holdout_b["brake"])
        result["ab_gap_mae"] = {
            regime: (
                holdout_a[regime].get("mae", 0.0) - holdout_b[regime].get("mae", 0.0)
                if "mae" in holdout_a[regime] and "mae" in holdout_b[regime]
                else None
            )
            for regime in ("accel", "brake")
        }
        result["feature_std_ratio_actual_over_ref"] = _feature_std_ratio(
            _combined_feature_matrix(holdout_logs, spec, "actual_speed_kmh"),
            _combined_feature_matrix(holdout_logs, spec, "ref_speed_kmh"),
            spec.feature_names(),
        )
    else:
        result["holdout_b"] = None
        result["holdout_b_mae"] = None
        result["warning"] = "holdoutログに ref_speed_kmh が無いため holdout-B をスキップしました"

    return result


def _resolve_specs(
    spec_names: list[str], spec_json_path: str | None
) -> dict[str, FeatureSpec]:
    """組み込み名 + `--spec-json` のカスタム定義から評価対象 spec 辞書を解決する。"""
    custom: dict[str, FeatureSpec] = {}
    if spec_json_path:
        with open(spec_json_path, encoding="utf-8") as f:
            raw = json.load(f)
        for name, fields in raw.items():
            kwargs = dict(fields)
            for key in ("lookahead_horizons_s", "past_horizons_s", "accel_horizons_s"):
                if key in kwargs:
                    kwargs[key] = tuple(kwargs[key])
            custom[name] = FeatureSpec(**kwargs)

    all_specs = {**BUILTIN_SPECS, **custom}
    resolved: dict[str, FeatureSpec] = {}
    for name in spec_names:
        if name not in all_specs:
            raise SystemExit(
                f"未知の spec 名です: {name!r}（組み込み: {list(BUILTIN_SPECS)}、"
                f"--spec-json で定義: {list(custom)}）"
            )
        resolved[name] = all_specs[name]
    return resolved


def _fmt_metrics(m: dict[str, float]) -> str:
    n = int(m.get("n", 0))
    if n == 0 or "mae" not in m:
        return f"n={n}"
    r2 = m.get("r2")
    r2s = f"{r2:.3f}" if r2 is not None else "N/A"
    return f"n={n} MAE={m['mae']:.3f} RMSE={m.get('rmse', float('nan')):.3f} R2={r2s}"


def _print_report(results: list[dict[str, Any]]) -> None:
    print("=== FeatureSpec オフライン評価 ===\n")
    for r in results:
        print(f"[{r['name']}] 特徴数={len(r['feature_names'])}")
        if "error" in r:
            print(f"  エラー: {r['error']}\n")
            continue
        print(f"  in-sample : accel {_fmt_metrics(r['in_sample']['accel'])}")
        print(f"              brake {_fmt_metrics(r['in_sample']['brake'])}")
        print(f"  holdout-A : accel {_fmt_metrics(r['holdout_a']['accel'])}")
        print(f"              brake {_fmt_metrics(r['holdout_a']['brake'])}")
        if r.get("holdout_b") is not None:
            print(f"  holdout-B : accel {_fmt_metrics(r['holdout_b']['accel'])}")
            print(f"              brake {_fmt_metrics(r['holdout_b']['brake'])}")
            gap = r.get("ab_gap_mae", {})
            print(
                f"  A-Bギャップ(MAE): accel={gap.get('accel')} brake={gap.get('brake')}"
            )
        else:
            print(f"  警告: {r.get('warning')}")
        print()

    # holdout-B MAE 順のランキング（None は末尾）
    ranked = sorted(
        (r for r in results if "error" not in r),
        key=lambda r: (r["holdout_b_mae"] is None, r["holdout_b_mae"] or float("inf")),
    )
    print("=== holdout-B MAE 順ランキング（配備条件に近い指標。小さいほど良い）===")
    for i, r in enumerate(ranked, start=1):
        mae = r["holdout_b_mae"]
        mae_s = f"{mae:.3f}" if mae is not None else "N/A（holdout-Bなし）"
        print(f"  {i}. {r['name']}: holdout-B加重MAE={mae_s}")


async def _load_logs(
    profile_id: str,
    cycle_id: str | None,
    train_session_ids: list[str] | None,
    eval_session_ids: list[str] | None,
) -> tuple[list[DriveLog], list[DriveLog], float]:
    settings = load_settings()
    pool = await create_pool(settings.database.dsn)
    try:
        profile_repo = ProfileRepository(pool)
        session_repo = SessionRepository(pool)
        profile = await profile_repo.get_by_id(profile_id)
        if profile is None:
            raise SystemExit(f"プロファイルが見つかりません: {profile_id}")
        brake_db = profile.feedforward_params.brake_deadband_pct

        if cycle_id:
            ids = await session_repo.list_session_ids_for_cycle(cycle_id)
            if not ids:
                raise SystemExit(f"学習サイクルにセッションがありません: {cycle_id}")
            train_ids: list[str] = []
            eval_ids: list[str] = []
            for sid in ids:
                session = await session_repo.get_by_id(sid)
                if session is None:
                    continue
                if session.run_type == "learning":
                    train_ids.append(sid)
                elif session.run_type == "tuning":
                    eval_ids.append(sid)
        elif train_session_ids and eval_session_ids:
            train_ids = train_session_ids
            eval_ids = eval_session_ids
        else:
            raise SystemExit(
                "データ選択には --cycle-id か、--train-session-id + --eval-session-id "
                "(複数可)のいずれかを指定してください。"
            )

        if not train_ids:
            raise SystemExit("学習用セッション(train)が見つかりません。")
        if not eval_ids:
            raise SystemExit("評価用セッション(holdout)が見つかりません。")

        train_logs = await session_repo.list_logs_for_training(profile_id, train_ids)
        holdout_logs = await session_repo.list_logs_for_training(profile_id, eval_ids)
    finally:
        await pool.close()
    return train_logs, holdout_logs, brake_db


async def _run(args: argparse.Namespace) -> None:
    train_logs, holdout_logs, brake_db = await _load_logs(
        args.profile_id, args.cycle_id, args.train_session_ids, args.eval_session_ids
    )
    print(f"train ログ {len(train_logs)}点 / holdout ログ {len(holdout_logs)}点\n")

    specs = _resolve_specs(args.specs, args.spec_json)
    results = [
        evaluate_spec(name, spec, train_logs, holdout_logs, brake_db)
        for name, spec in specs.items()
    ]

    _print_report(results)

    if args.json_out:
        payload = [{**r, "feature_spec": asdict(specs[r["name"]])} for r in results]
        with open(args.json_out, "w", encoding="utf-8") as f:
            json.dump(payload, f, ensure_ascii=False, indent=2)
        print(f"\n結果を {args.json_out} へ保存しました。")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="FeatureSpec 候補セットのオフライン評価（読み取り専用）"
    )
    parser.add_argument("--profile-id", required=True, help="対象プロファイル UUID")
    parser.add_argument(
        "--cycle-id",
        help="学習サイクル UUID（train=サイクルの学習セッション、holdout=サイクルの適合走行）",
    )
    parser.add_argument(
        "--train-session-id",
        action="append",
        dest="train_session_ids",
        help="学習用セッション ID（--cycle-id の代わりに明示指定する場合。複数可）",
    )
    parser.add_argument(
        "--eval-session-id",
        action="append",
        dest="eval_session_ids",
        help="評価用（holdout）セッション ID（複数可）",
    )
    parser.add_argument(
        "--specs",
        nargs="+",
        default=list(BUILTIN_SPECS),
        help=f"評価する候補名（組み込み: {list(BUILTIN_SPECS)}）",
    )
    parser.add_argument(
        "--spec-json", help="追加/上書きするカスタム spec 定義（JSON ファイルパス）"
    )
    parser.add_argument(
        "--json-out", help="結果を保存する JSON ファイルパス（省略時は標準出力のみ）"
    )
    args = parser.parse_args()

    asyncio.run(_run(args))


if __name__ == "__main__":
    main()
