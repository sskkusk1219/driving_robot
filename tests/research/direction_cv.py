"""交差検証の誤差を「踏み込みの行」と「戻しの行」に分けて出す。

C1 の式（逆モデル）は、同じ開度でも「踏み込み中」と「戻し中」で車の応答が違うのを 1 本の式で
まとめて当てている。戻しの行だけ誤差が大きい／符号が偏るなら、向きで式を分ける意味がある。
それを見るため、パターン単位の GroupKFold（`predict_out_of_pattern` と同じ分け方）で
out-of-fold 予測を 3 通り作り、向き別の誤差を出す。

    全部の行で学習     → 踏み込みの行・戻しの行それぞれの誤差
    踏み込みの行だけ   → 踏み込みの行の誤差
    戻しの行だけ       → 戻しの行の誤差

「片方だけで学習」でも、テストの行のパターンは学習から外す（全行で GroupKFold を切り、
各 fold の学習側をその向きの行だけに絞る）。

行の選び方・特徴量は `train_candidate_model.main` が C1 用に使うものと同じ
（そのペダルの開度 ≥ 不感帯、`DEFAULT_FEATURE_SPEC`、シフト 0、ラベルは指令開度 cmd）。
向きは `wltp_grid.pedal_release` を「セッション全行の開度の時系列」に掛けてから行を絞る。

    .venv/bin/python -m tests.research.direction_cv <手順2 の CSV>

車両・アクチュエータには触らない。CSV を読むだけ。
"""

from __future__ import annotations

import argparse
import pickle
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from sklearn.model_selection import GroupKFold

from tests.research.config import DEFAULT_CONFIG_PATH, ConfigError, load_config
from tests.research.ff_model import (
    DEFAULT_FEATURE_SPEC,
    FeatureSpec,
    build_feature_matrix,
    estimate_offsets,
    make_estimator,
)
from tests.research.model_analysis import CV_SPLITS, RegimeData, load_training_rows
from tests.research.train_candidate_model import V0_SPEED_BANDS
from tests.research.wltp_grid import (
    DIRECTION_THRESHOLD_ACCEL_PCT,
    DIRECTION_THRESHOLD_PCT,
    pedal_release,
)

#: 学習側の行がこれより少ない fold は式を作らず、その fold のテスト行は nan（数えない）
MIN_TRAIN_ROWS = 30

DIR_PRESS = "踏み込み"
DIR_RELEASE = "戻し"
TRAIN_ALL = "全部の行"
TRAIN_SAME = "同じ向きだけ"

# 結果の行の (向き, 学習の仕方)。「同じ向きだけ」= その向きの行だけで学習
CASES: tuple[tuple[str, str], ...] = (
    (DIR_PRESS, TRAIN_ALL),
    (DIR_RELEASE, TRAIN_ALL),
    (DIR_PRESS, TRAIN_SAME),
    (DIR_RELEASE, TRAIN_SAME),
)


@dataclass
class DirectionCvRow:
    """向き × 学習の仕方 ×（速度帯）の誤差。band が None のものは全体。"""

    direction: str
    training: str
    band: tuple[float, float] | None
    n: int
    seconds: float
    mae: float  # nan = 数えた行が無い
    mean_error: float  # 予測 − 実測 [%]（符号つき平均）


def directed_regime_data(
    logs: Sequence,
    patterns: np.ndarray,
    phases: np.ndarray,
    *,
    accel_deadband_pct: float,
    brake_deadband_pct: float,
    accel_threshold_pct: float,
    brake_threshold_pct: float,
    accel_spec: FeatureSpec = DEFAULT_FEATURE_SPEC,
    brake_spec: FeatureSpec = DEFAULT_FEATURE_SPEC,
) -> tuple[tuple[RegimeData, np.ndarray], tuple[RegimeData, np.ndarray]]:
    """`effective_regime_data` と同じ行・特徴で、行ごとの「戻し」フラグを付けて返す。

    特徴量 spec はペダルごと（`accel_spec`/`brake_spec`。pkl のペダル別 spec に合わせる）。
    返り値は ((アクセル側データ, 戻しフラグ), (ブレーキ側データ, 戻しフラグ))。
    戻しフラグは `pedal_release` を全行の開度に掛けてから、同じ idx・マスクで絞ったもの。
    """
    speed = np.clip(np.array([lg.actual_speed_kmh for lg in logs], dtype=float), 0.0, None)
    accel = np.array([lg.accel_opening for lg in logs], dtype=float)
    brake = np.array([lg.brake_opening for lg in logs], dtype=float)
    timestamps = [lg.timestamp for lg in logs]
    out = []
    for name, label, opening, deadband, threshold, spec in (
        ("accel", "アクセル", accel, accel_deadband_pct, accel_threshold_pct, accel_spec),
        ("brake", "ブレーキ", brake, brake_deadband_pct, brake_threshold_pct, brake_spec),
    ):
        # 特徴量行列・有効行 idx はペダルごとの spec で作る（ホライズンがペダルで違うため）
        x, idx = build_feature_matrix(
            speed,
            estimate_offsets(timestamps, spec.lookahead_horizons_s),
            estimate_offsets(timestamps, spec.past_horizons_s),
            spec,
        )
        release_all = pedal_release(opening, deadband, threshold)  # 全行で判定してから絞る
        mask = opening[idx] >= deadband
        data = RegimeData(
            name=name, label=label, x=x[mask], y=opening[idx][mask],
            pattern=patterns[idx][mask], phase=phases[idx][mask], deadband_pct=deadband,
        )
        out.append((data, np.asarray(release_all[idx][mask], dtype=bool)))
    return out[0], out[1]


def load_pkl_specs(model_path: Path) -> tuple[FeatureSpec, FeatureSpec, dict, list[str]]:
    """pkl からペダル別の FeatureSpec と sample_weight 設定を読む。

    `accel_feature_spec`/`brake_feature_spec` が無い古い pkl は `feature_spec`、
    それも無ければ DEFAULT_FEATURE_SPEC に落とす（落とした旨を notes に入れる）。
    """
    with model_path.open("rb") as f:
        data = pickle.load(f)  # noqa: S301 - 手順 2 で作った信頼済みファイル
    notes: list[str] = []
    specs = []
    for side in ("accel", "brake"):
        d = data.get(f"{side}_feature_spec")
        if d is None and data.get("feature_spec") is not None:
            d = data["feature_spec"]
            notes.append(f"{side}_feature_spec が無いため feature_spec を使用")
        if d is None:
            specs.append(DEFAULT_FEATURE_SPEC)
            notes.append(f"{side} の spec が pkl に無いため DEFAULT_FEATURE_SPEC を使用")
        else:
            specs.append(FeatureSpec(**d))
    return specs[0], specs[1], data.get("sample_weight") or {}, notes


def oof_predictions(
    data: RegimeData, release: np.ndarray, *, n_splits: int = CV_SPLITS
) -> dict[str, np.ndarray]:
    """パターン単位 GroupKFold の out-of-fold 予測を 3 通り返す。

    キーは "all"（全部の行で学習）・"press"（踏み込みの行だけで学習）・"release"（戻しの行だけ）。
    学習側が `MIN_TRAIN_ROWS` 未満の fold のテスト行は nan。
    """
    n = len(data.y)
    preds = {k: np.full(n, np.nan) for k in ("all", "press", "release")}
    splits = min(n_splits, len(np.unique(data.pattern)))
    if splits < 2:
        return preds
    for train, test in GroupKFold(n_splits=splits).split(data.x, data.y, groups=data.pattern):
        train_sets = {
            "all": train,
            "press": train[~release[train]],
            "release": train[release[train]],
        }
        for key, rows in train_sets.items():
            if len(rows) < MIN_TRAIN_ROWS:
                continue
            model = make_estimator().fit(data.x[rows], data.y[rows])
            preds[key][test] = model.predict(data.x[test])
    return preds


def _row(
    direction: str, training: str, band: tuple[float, float] | None,
    sel: np.ndarray, pred: np.ndarray, y: np.ndarray, period_s: float,
) -> DirectionCvRow:
    ok = sel & ~np.isnan(pred)
    n = int(np.count_nonzero(ok))
    err = pred[ok] - y[ok]
    return DirectionCvRow(
        direction=direction, training=training, band=band, n=n, seconds=n * period_s,
        mae=float(np.mean(np.abs(err))) if n else float("nan"),
        mean_error=float(np.mean(err)) if n else float("nan"),
    )


def direction_cv_table(
    data: RegimeData,
    release: np.ndarray,
    *,
    period_s: float = 1.0,
    n_splits: int = CV_SPLITS,
    bands: Sequence[tuple[float, float]] = V0_SPEED_BANDS,
) -> list[DirectionCvRow]:
    """全体（band=None）と車速帯別（v0 = x[:, 0]）の、向き × 学習の仕方ごとの誤差。"""
    release = np.asarray(release, dtype=bool)
    preds = oof_predictions(data, release, n_splits=n_splits)
    v0 = data.x[:, 0]
    sources = {
        (DIR_PRESS, TRAIN_ALL): (~release, preds["all"]),
        (DIR_RELEASE, TRAIN_ALL): (release, preds["all"]),
        (DIR_PRESS, TRAIN_SAME): (~release, preds["press"]),
        (DIR_RELEASE, TRAIN_SAME): (release, preds["release"]),
    }
    rows: list[DirectionCvRow] = []
    for band in (None, *bands):
        in_band = np.ones(len(v0), dtype=bool) if band is None else (v0 >= band[0]) & (v0 < band[1])
        for direction, training in CASES:
            sel, pred = sources[(direction, training)]
            rows.append(_row(direction, training, band, sel & in_band, pred, data.y, period_s))
    return rows


def _band_label(band: tuple[float, float] | None) -> str:
    if band is None:
        return "全体"
    hi = "∞" if band[1] == float("inf") else f"{band[1]:.0f}"
    return f"[{band[0]:.0f},{hi})"


def _num(value: float, fmt: str) -> str:
    return "—" if np.isnan(value) else format(value, fmt)


def _md_table(header: Sequence[str], rows: Sequence[Sequence[str]]) -> str:
    lines = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    lines += ["| " + " | ".join(r) + " |" for r in rows]
    return "\n".join(lines)


def format_table(rows: Sequence[DirectionCvRow], *, with_band: bool) -> str:
    """結果を Markdown の表にする。with_band=False なら全体の行だけ、True なら帯別の行だけ。"""
    sel = [r for r in rows if (r.band is not None) == with_band]
    header = ["向き", "学習の仕方", "行数 n", "秒数 [s]", "MAE [%]", "符号つき平均 [%]"]
    if with_band:
        header.insert(0, "v0 の帯 [km/h]")
    body = []
    for r in sel:
        cells = [r.direction, r.training, str(r.n), f"{r.seconds:.0f}",
                 _num(r.mae, ".2f"), _num(r.mean_error, "+.2f")]
        if with_band:
            cells.insert(0, _band_label(r.band))
        body.append(cells)
    return _md_table(header, body)


def report_text(label: str, rows: Sequence[DirectionCvRow], n_rows: int, n_release: int) -> str:
    return "\n".join([
        f"### {label}（効いている行 {n_rows}、うち戻し {n_release}）",
        "",
        "全体",
        format_table(rows, with_band=False),
        "",
        "車速の帯別（v0 = 先頭の特徴量 = 現在車速）",
        format_table(rows, with_band=True),
    ])


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("csv", type=Path, help="手順 2 の走行ログ CSV（PATTERN_DRIVE を含む）")
    ap.add_argument("--model", type=Path, default=None,
                    help="特徴量 spec を読む pkl（省略時は config の feedforward.model_path）")
    args = ap.parse_args(argv)
    try:
        cfg = load_config(DEFAULT_CONFIG_PATH)
    except ConfigError as exc:
        print(f"設定エラー: {exc}")
        return 2
    model_path = args.model or Path(cfg.feedforward.model_path)
    if not model_path.exists():
        print(f"pkl が見つかりません: {model_path}")
        return 2
    accel_spec, brake_spec, weight_cfg, notes = load_pkl_specs(model_path)
    logs, patterns, phases = load_training_rows(args.csv, label="cmd")
    if not logs:
        print(f"{args.csv} に手順2（PATTERN_DRIVE）の行がありません")
        return 2
    veh = cfg.vehicle
    epochs = np.array([lg.timestamp.timestamp() for lg in logs])
    diffs = np.diff(epochs)
    diffs = diffs[diffs > 0.0]
    period_s = float(np.median(diffs)) if len(diffs) else 1.0
    pairs = directed_regime_data(
        logs, patterns, phases,
        accel_deadband_pct=veh.min_accel_opening_pct,
        brake_deadband_pct=veh.min_brake_opening_pct,
        accel_threshold_pct=DIRECTION_THRESHOLD_ACCEL_PCT,
        brake_threshold_pct=DIRECTION_THRESHOLD_PCT,
        accel_spec=accel_spec,
        brake_spec=brake_spec,
    )
    print(f"pkl: {model_path.name}（先読みホライズン アクセル "
          f"{','.join(f'{h:g}' for h in accel_spec.lookahead_horizons_s)} / ブレーキ "
          f"{','.join(f'{h:g}' for h in brake_spec.lookahead_horizons_s)} s）")
    for note in notes:
        print(f"注: {note}")
    if weight_cfg.get("enabled"):
        print("注: pkl は WLTP の重みつき（sample_weight）で学習。この交差検証は重みなし")
    print(f"CSV: {args.csv.name}（{len(logs)} 行・{len(np.unique(patterns))} パターン、"
          f"周期の中央値 {period_s:.3f}s、label=cmd）")
    print(f"u の原点（min） アクセル {veh.min_accel_opening_pct:g}% / "
          f"ブレーキ {veh.min_brake_opening_pct:g}%、"
          f"戻し判定しきい値 アクセル {DIRECTION_THRESHOLD_ACCEL_PCT:g}% / "
          f"ブレーキ {DIRECTION_THRESHOLD_PCT:g}%")
    print("誤差 = 予測 − 実測。正 = 式が開度を多めに出す")
    for data, release in pairs:
        print()
        if len(data.y) == 0:
            print(f"### {data.label}: 効いている行がありません")
            continue
        rows = direction_cv_table(data, release, period_s=period_s)
        print(report_text(data.label, rows, len(data.y), int(release.sum())))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
