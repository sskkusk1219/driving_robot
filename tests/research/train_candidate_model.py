"""D1・V4: 手順 2 の CSV から、ラベル（指令/実開度）と先読み窓（通常/0.5s ずらし）を選んで
逆モデルを学習し pkl 保存する。表 5-5 順7（C1〜C5 の実機比較）の準備。
（C6 と `--cruise-curve-from` は 2026-09-25 段4 で、定速階段とともに削除した。）

車両・アクチュエータには触らない。CSV を読んで学習するだけ。

    .venv/bin/python -m tests.research.train_candidate_model \
        tests/research/results/drive_log_real_20260913_214423.csv --label actual

    .venv/bin/python -m tests.research.train_candidate_model \
        tests/research/results/drive_log_real_20260913_214423.csv --label cmd --shift-lookahead 0.5

学習は `ff_candidate.train_inverse_model_effective`（A1: そのペダルが効いている行だけ）と
まったく同じロジック。差し替えているのは:
    --label             "cmd"（既定、今までの指令ラベル）／ "actual"（PNOW 実開度ラベル。
                        A7 の新形式 CSV だけ。読めなかった行は除外）
    --shift-lookahead   先読みホライズンと regime_horizon に一律 +N 秒する
                        （C3: 実測の遅れぶんずらして学習・推論する用。既定は 0 = ずらさない）

出すもの:
    1. 学習データ上の当てはまり（pattern_drive._print_metrics と同じ表示）
    2. パターン単位 GroupKFold 分割検証の MAE/RMSE（model_analysis の仕組みを再利用）。
       速度帯別（[0,40)/[40,80)/[80,120)/[120,∞) km/h）の内訳も出す（参考値）
    3. 保存した pkl のパス
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from tests.research.config import DEFAULT_CONFIG_PATH, load_config
from tests.research.ff_candidate import train_inverse_model_effective
from tests.research.ff_model import (
    DEFAULT_FEATURE_SPEC,
    FeatureSpec,
    build_feature_matrix,
    estimate_offsets,
)
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
) -> tuple[RegimeData, RegimeData]:
    """A1（そのペダルが効いている行だけ）と同じ選び方で (アクセル側, ブレーキ側) を作る。

    `train_inverse_model_effective` のセッション内ロジックと同じ（1 CSV = 1 セッションなので
    セッション分けは不要）。パターン単位 CV に使うため、pattern/phase 列を付けて返す。
    """
    speed = np.clip(np.array([lg.actual_speed_kmh for lg in logs], dtype=float), 0.0, None)
    accel = np.array([lg.accel_opening for lg in logs], dtype=float)
    brake = np.array([lg.brake_opening for lg in logs], dtype=float)
    timestamps = [lg.timestamp for lg in logs]
    x, idx = build_feature_matrix(
        speed,
        estimate_offsets(timestamps, spec.lookahead_horizons_s),
        estimate_offsets(timestamps, spec.past_horizons_s),
        spec,
    )
    a_mask = accel[idx] >= accel_deadband_pct
    b_mask = brake[idx] >= brake_deadband_pct
    x_accel = x[a_mask]
    y_accel = accel[idx][a_mask]
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
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    profile = build_vehicle_profile(cfg)
    out_dir = args.out_dir or Path(cfg.feedforward.model_path).parent
    spec = shifted_spec(DEFAULT_FEATURE_SPEC, args.shift_lookahead)

    say(f"学習データ: {args.csv}（label={args.label}"
        f"{f'、先読み +{args.shift_lookahead:g}s' if args.shift_lookahead else ''}）")
    logs, patterns, phases = load_training_rows(args.csv, label=args.label)
    say(f"  {len(logs)} 行・{len(np.unique(patterns))} パターン")

    model_path, metrics = train_inverse_model_effective(
        logs, profile, output_dir=str(out_dir), accel_spec=spec, brake_spec=spec
    )
    _print_metrics(metrics)

    say("── パターン単位 GroupKFold 分割検証（そのパターンを学習に使っていないモデルで予測） ──")
    accel_data, brake_data = effective_regime_data(
        logs, patterns, phases,
        accel_deadband_pct=cfg.feedforward.accel_deadband_pct,
        brake_deadband_pct=cfg.feedforward.brake_deadband_pct,
        spec=spec,
    )
    for data in (accel_data, brake_data):
        cv = pattern_cv(data)
        if cv is None:
            say(f"  {data.label}: パターンが 2 本未満のため分割検証できません")
            continue
        say(f"  {data.label}: MAE={cv.mae:.2f}  RMSE={cv.rmse:.2f}  n={cv.n}"
            f"（{cv.n_patterns} パターンで分割）")
        _print_band_cv(speed_band_cv(data))

    say(f"モデル保存: {model_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
