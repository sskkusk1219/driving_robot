"""D1・V4: 手順 2 の CSV から、ラベル（指令/実開度）と先読み窓（通常/0.5s ずらし）を選んで
逆モデルを学習し pkl 保存する。表 5-5 順7（C1〜C5 の実機比較）の準備。C6（骨格を定速階段の
実測テーブルにする案）の pkl もここで作る。

車両・アクチュエータには触らない。CSV を読んで学習するだけ。

    .venv/bin/python -m tests.research.train_candidate_model \
        tests/research/results/drive_log_real_20260913_214423.csv --label actual

    .venv/bin/python -m tests.research.train_candidate_model \
        tests/research/results/drive_log_real_20260913_214423.csv --label cmd --shift-lookahead 0.5

    .venv/bin/python -m tests.research.train_candidate_model \
        tests/research/results/drive_log_real_20260915_052432.csv --label actual \
        --cruise-curve-from tests/research/results/drive_log_real_20260915_052432.csv

学習は `ff_candidate.train_inverse_model_effective`（A1: そのペダルが効いている行だけ）と
まったく同じロジック。差し替えているのは:
    --label             "cmd"（既定、今までの指令ラベル）／ "actual"（PNOW 実開度ラベル。
                        A7 の新形式 CSV だけ。読めなかった行は除外）
    --shift-lookahead   先読みホライズンと regime_horizon に一律 +N 秒する
                        （C3: 実測の遅れぶんずらして学習・推論する用。既定は 0 = ずらさない）
    --cruise-curve-from 定速階段（CRUISE_HOLD）を含む CSV から実測テーブルを作り、C6 用の
                        pkl（アクセル側は骨格 + 残差 ML）を保存する（既定は指定なし = C1）。

出すもの:
    1. 学習データ上の当てはまり（pattern_drive._print_metrics と同じ表示）
    2. パターン単位 GroupKFold 分割検証の MAE/RMSE（model_analysis の仕組みを再利用）。
       速度帯別（[0,40)/[40,80)/[80,120)/[120,∞) km/h）の内訳も出す（参考値）
    3. 保存した pkl のパス
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from src.domain.model_training import (
    DEFAULT_FEATURE_SPEC,
    FeatureSpec,
    _build_feature_matrix,  # noqa: PLC2701 - 学習の特徴量は本番と同一にする
    _estimate_offsets,  # noqa: PLC2701
)
from src.models.profile import FeedforwardParams
from tests.research.config import DEFAULT_CONFIG_PATH, ResearchConfig, load_config
from tests.research.cruise_curve import CruiseCurve, build_cruise_curve
from tests.research.debug_process23 import md_table
from tests.research.ff_candidate import cruise_skeleton, train_inverse_model_effective
from tests.research.model_analysis import (
    RegimeData,
    load_training_rows,
    predict_out_of_pattern,
    score,
)
from tests.research.pattern_drive import _print_metrics  # noqa: PLC2701 - 表示を揃える
from tests.research.term import say
from tests.research.vehicle import build_vehicle_profile

#: 速度帯別 CV の帯（v0 [km/h]）。参考値（帯ごとのサンプル数は均一でない）
V0_SPEED_BANDS: tuple[tuple[float, float], ...] = (
    (0.0, 40.0),
    (40.0, 80.0),
    (80.0, 120.0),
    (120.0, float("inf")),
)


def shifted_spec(base: FeatureSpec, shift_s: float) -> FeatureSpec:
    """先読みホライズン・regime_horizon に一律 +shift_s（C3 用。過去ホライズンは変えない）。"""
    if shift_s == 0.0:
        return base
    return replace(
        base,
        lookahead_horizons_s=tuple(h + shift_s for h in base.lookahead_horizons_s),
        regime_horizon_s=base.regime_horizon_s + shift_s,
    )


def effective_regime_data(
    logs: list,
    patterns: np.ndarray,
    phases: np.ndarray,
    *,
    accel_deadband_pct: float,
    brake_deadband_pct: float,
    spec: FeatureSpec,
    cruise_curve: CruiseCurve | None = None,
    params: FeedforwardParams | None = None,
) -> tuple[RegimeData, RegimeData]:
    """A1（そのペダルが効いている行だけ）と同じ選び方で (アクセル側, ブレーキ側) を作る。

    `train_inverse_model_effective` のセッション内ロジックと同じ（1 CSV = 1 セッションなので
    セッション分けは不要）。パターン単位 CV に使うため、pattern/phase 列を付けて返す。

    Args:
        cruise_curve: 指定すると C6 として、アクセル側の `y` を「ラベル − 骨格」（残差）にする
            （骨格は `ff_candidate.cruise_skeleton` を学習と同じ関数で計算し、定義がずれない
            ようにする）。指定するときは `params` も必要。
            骨格は行ごとに一意に決まる（v0・a_req だけの関数）ため、パターン単位 CV の
            MAE/RMSE は「残差同士の差」= 「(ラベル−骨格) と (骨格+残差予測−骨格)」の差
            = 「ラベルと骨格+残差予測の差」と一致する（骨格が両辺で打ち消し合う）。つまり
            この関数が返す残差ベースの CV は、C6 の実際の開度誤差とそのまま比べられる。
        params: `cruise_curve` 指定時の骨格計算に使う車両物理定数（`pedal_gain_at` 参照）。
    """
    speed = np.clip(np.array([lg.actual_speed_kmh for lg in logs], dtype=float), 0.0, None)
    accel = np.array([lg.accel_opening for lg in logs], dtype=float)
    brake = np.array([lg.brake_opening for lg in logs], dtype=float)
    timestamps = [lg.timestamp for lg in logs]
    x, idx = _build_feature_matrix(
        speed,
        _estimate_offsets(timestamps, spec.lookahead_horizons_s),
        _estimate_offsets(timestamps, spec.past_horizons_s),
        spec,
    )
    a_mask = accel[idx] >= accel_deadband_pct
    b_mask = brake[idx] >= brake_deadband_pct
    x_accel = x[a_mask]
    y_accel = accel[idx][a_mask]
    if cruise_curve is not None:
        if params is None:
            raise ValueError("cruise_curve を指定するときは params も指定してください")
        regime_col = spec.regime_col()
        v0_accel = x_accel[:, 0]
        a_req_accel = x_accel[:, regime_col] / spec.regime_horizon_s
        skeleton_accel = np.array(
            [
                cruise_skeleton(cruise_curve, params, float(v), float(a))
                for v, a in zip(v0_accel, a_req_accel, strict=True)
            ]
        )
        y_accel = y_accel - skeleton_accel
    accel_data = RegimeData(
        name="accel", label="アクセル", x=x_accel, y=y_accel,
        pattern=patterns[idx][a_mask], phase=phases[idx][a_mask], deadband_pct=accel_deadband_pct,
    )
    brake_data = RegimeData(
        name="brake", label="ブレーキ", x=x[b_mask], y=brake[idx][b_mask],
        pattern=patterns[idx][b_mask], phase=phases[idx][b_mask], deadband_pct=brake_deadband_pct,
    )
    return accel_data, brake_data


@dataclass
class CvResult:
    mae: float
    rmse: float
    n: int
    n_patterns: int


@dataclass
class BandCvResult:
    """速度帯別（v0 [km/h]）の分割検証結果。"""

    lo: float
    hi: float
    mae: float
    rmse: float
    n: int


def _pattern_oof(data: RegimeData) -> np.ndarray | None:
    """パターン単位 GroupKFold の out-of-fold 予測。パターンが 2 本未満なら None。"""
    if len(np.unique(data.pattern)) < 2:
        return None
    return predict_out_of_pattern(data)


def pattern_cv(data: RegimeData) -> CvResult | None:
    """パターン単位 GroupKFold の分割検証 MAE/RMSE。パターンが 2 本未満なら None。"""
    oof = _pattern_oof(data)
    if oof is None:
        return None
    valid = ~np.isnan(oof)
    m = score(data.y[valid], oof[valid])
    return CvResult(
        mae=m["mae"], rmse=m["rmse"], n=int(m["n"]), n_patterns=len(np.unique(data.pattern))
    )


def speed_band_cv(data: RegimeData) -> list[BandCvResult]:
    """速度帯別（v0）の分割検証 MAE/RMSE（参考値）。空の帯は返さない。

    `data.y` が cruise_curve 指定時の残差でも、骨格は帯分けに使う v0 と 1 対 1 で決まるため
    帯ごとの残差 MAE は帯ごとの開度 MAE とそのまま一致する（`effective_regime_data` の注記）。
    """
    oof = _pattern_oof(data)
    if oof is None:
        return []
    valid = ~np.isnan(oof)
    v0 = data.x[:, 0]
    results: list[BandCvResult] = []
    for lo, hi in V0_SPEED_BANDS:
        band_mask = valid & (v0 >= lo) & (v0 < hi)
        n = int(np.count_nonzero(band_mask))
        if n == 0:
            continue
        m = score(data.y[band_mask], oof[band_mask])
        results.append(BandCvResult(lo=lo, hi=hi, mae=m["mae"], rmse=m["rmse"], n=n))
    return results


def _print_band_cv(bands: list[BandCvResult]) -> None:
    for b in bands:
        hi_label = "∞" if b.hi == float("inf") else f"{b.hi:.0f}"
        say(f"    [{b.lo:.0f},{hi_label}) km/h: MAE={b.mae:.2f}  RMSE={b.rmse:.2f}  n={b.n}")


def _load_cruise_curve(csv_path: Path, cfg: ResearchConfig) -> CruiseCurve:
    """`--cruise-curve-from` の CSV から実測テーブルを作る（表は cruise_curve.py と同じ）。"""
    with csv_path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    curve = build_cruise_curve(rows, cfg.learning)
    say(f"実測テーブル（{csv_path}）:")
    say(md_table(
        ["車速中央値 [km/h]", "開度中央値 [%]", "保持窓の行数"],
        [
            [f"{s:.1f}", f"{o:.2f}", str(n)]
            for s, o, n in zip(curve.speeds_kmh, curve.openings_pct, curve.n_rows, strict=True)
        ],
    ))
    return curve


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("csv", type=Path, help="手順 2 の走行ログ CSV（PATTERN_DRIVE を含む）")
    ap.add_argument("--label", choices=("cmd", "actual"), default="cmd",
                     help="学習ラベル（既定: cmd = 指令開度）")
    ap.add_argument("--shift-lookahead", type=float, default=0.0,
                     help="先読みホライズンに +N 秒（C3 用。既定 0）")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    ap.add_argument("--out-dir", type=Path, default=None,
                     help="pkl の保存先（既定: config の feedforward.model_path と同じフォルダ）")
    ap.add_argument("--cruise-curve-from", type=Path, default=None, metavar="CSV",
                     help="定速階段（CRUISE_HOLD）を含む CSV から実測テーブルを作り、"
                          "C6 用の pkl を保存する（既定: 指定なし = C1）")
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    profile = build_vehicle_profile(cfg)
    out_dir = args.out_dir or Path(cfg.feedforward.model_path).parent
    spec = shifted_spec(DEFAULT_FEATURE_SPEC, args.shift_lookahead)

    cruise_curve: CruiseCurve | None = None
    if args.cruise_curve_from is not None:
        cruise_curve = _load_cruise_curve(args.cruise_curve_from, cfg)
        say("feedforward.candidate: C6（アクセル側は骨格 + 残差 ML。ブレーキ側は C1 と同じ）")

    say(f"学習データ: {args.csv}（label={args.label}"
        f"{f'、先読み +{args.shift_lookahead:g}s' if args.shift_lookahead else ''}）")
    logs, patterns, phases = load_training_rows(args.csv, label=args.label)
    say(f"  {len(logs)} 行・{len(np.unique(patterns))} パターン")

    model_path, metrics = train_inverse_model_effective(
        logs, profile, output_dir=str(out_dir), feature_spec=spec, cruise_curve=cruise_curve
    )
    _print_metrics(metrics)

    say("── パターン単位 GroupKFold 分割検証（そのパターンを学習に使っていないモデルで予測） ──")
    accel_data, brake_data = effective_regime_data(
        logs, patterns, phases,
        accel_deadband_pct=cfg.feedforward.accel_deadband_pct,
        brake_deadband_pct=cfg.feedforward.brake_deadband_pct,
        spec=spec,
        cruise_curve=cruise_curve,
        params=profile.feedforward_params if cruise_curve is not None else None,
    )
    for data in (accel_data, brake_data):
        cv = pattern_cv(data)
        if cv is None:
            say(f"  {data.label}: パターンが 2 本未満のため分割検証できません")
            continue
        say(f"  {data.label}: MAE={cv.mae:.2f}  RMSE={cv.rmse:.2f}  n={cv.n}"
            f"（{cv.n_patterns} パターンで分割）")
        _print_band_cv(speed_band_cv(data))

    if cruise_curve is not None:
        say("参考値: 骨格は定速階段パターンの保持区間から作っているため、そのパターンが検証側に"
            "入るときも骨格は学習データを見ています。採否は実機比較で決めます。")

    say(f"モデル保存: {model_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
