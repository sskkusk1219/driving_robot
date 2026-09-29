"""手順2 のモデル作成: 先読みホライズンをペダル別に交差検証で自動選択する。

ProblemReport_20260921 手順6（2026-09-28）。これまで `config_testVehicle.yaml` の
`use_h0〜use_h3` で固定 true/false を人が選んでいたが、車が変われば応答速度も変わり
「先読みをいくつにすべきか」も変わるはずというユーザー方針を受け、パターン単位の
交差検証 MAE が最も下がるホライズンの組を、アクセル・ブレーキそれぞれで貪欲法
（前向き選択）で選ぶ。

選び方（ユーザー決定 2026-09-28）:
    1. 交差検証 MAE が最も良い組をそのまま採用する（最短ホライズンの下限は設定だけ用意し、
       既定は無効。`config.FeaturesSection.search_min_horizon_s`）。
    2. アクセル・ブレーキで別々のホライズンを選ぶ（データ上の最適が違うため）。
    3. 貪欲法（前向き選択）: レジーム判定のホライズン（h1。既定 1.0s）だけの組から始め、
       探索格子の中で CV-MAE が最も下がる 1 本を足す。相対改善が
       `search_min_improvement` 未満、または本数が `search_max_horizons` に達したら止める。

採否はオフラインのこの点数では決めない（実機の手順3 比較で判断する）。

段2 の実機破綻と案a（2026-09-28。ProblemReport_20260921 手順6 6-7/6-8）:
    上の CV-MAE だけで選んだ pkl が実機で破綻した。実車速が基準より遅れているのにアクセル
    FF が下がる（逆向き）モデルになっており、遅れが自分で広がって最大逸脱 126km/h に至った。
    学習データ（偏差 0 の自分の軌跡）には「全部の dv が同じ量だけずれる」場面が無いため、
    偏差への反応の向きは CV-MAE にも R² にも現れず、係数の打ち消し合いと Ridge 正則化の
    偶然で決まっていた。このため `search_pedal` は CV-MAE に加えて、各候補の
    `deviation_gain`（`ff_model.deviation_gain`。実車速のずれ 1km/h あたりの開度の反応。
    正 = 正しい向き）を `gain_check_speeds_kmh` の各速度で測り、`min_deviation_gain` を
    下回る候補を除外する（案a。ユーザー決定）。

学習セットの選び方は本番の A1（`ff_candidate.train_inverse_model_effective`）と同じ:
    そのペダルが効いている行（開度 ≥ 不感帯）だけをそのモデルに入れる。

CSV の読み方は `model_analysis.load_training_rows` と同じ行選び（PATTERN_DRIVE のみ）だが、
matplotlib 等の重い依存を読み込まないよう、ここに軽量な実装を置く。
"""

from __future__ import annotations

import csv
from collections.abc import Sequence
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Literal

import numpy as np
from sklearn.metrics import mean_absolute_error, r2_score
from sklearn.model_selection import GroupKFold

from tests.research.drive_log import SECTION_PATTERN_DRIVE, read_drive_logs
from tests.research.ff_model import (
    MIN_REGIME_SAMPLES,
    FeatureSpec,
    build_feature_matrix,
    deviation_gain,
    estimate_offsets,
    group_by_session,
    make_estimator,
)
from tests.research.learning_patterns import LearningDataError
from tests.research.research_types import DriveLog

__all__ = [
    "HorizonSearchResult",
    "HorizonSearchSettings",
    "SearchStep",
    "format_result_table",
    "read_pattern_groups",
    "search_ff_horizons",
    "search_pedal",
]


def read_pattern_groups(csv_path: Path) -> np.ndarray:
    """PATTERN_DRIVE 行の `pattern` 列（`read_drive_logs` と同じ行選び。学習行のグループ化用）。

    `label="cmd"`（既定の学習ラベル）の `read_drive_logs` と同じフィルタ（section 列があれば
    PATTERN_DRIVE のみ）を通すので、同じ CSV・同じ label なら行が 1 対 1 で対応する。
    """
    groups: list[str] = []
    with csv_path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if "section" in row and row["section"] != SECTION_PATTERN_DRIVE:
                continue
            groups.append(row.get("pattern", ""))
    return np.array(groups)


def _single_session_inputs(
    logs: list[DriveLog], patterns: np.ndarray
) -> tuple[np.ndarray, list[datetime], np.ndarray, np.ndarray, np.ndarray]:
    """単一セッションの学習行から speed・timestamps・accel_label・brake_label・groups を作る。

    手順2 は 1 走行 1 セッションを前提にする（複数セッションが混ざった CSV は対象外。
    多セッション CSV は ValueError で拒否する。理由は `_pedal_dataset` の docstring 参照）。
    """
    sessions = group_by_session(logs)
    if len(sessions) != 1:
        raise ValueError(
            f"horizon_search は単一セッションの CSV のみ対応しています"
            f"（{len(sessions)} セッション検出）。1 走行 1 CSV で渡してください。"
        )
    session_logs = sessions[0]
    if len(session_logs) != len(logs) or len(session_logs) != len(patterns):
        raise ValueError(
            "セッションの並べ替えで pattern 列と行の対応が取れませんでした"
            "（logs・patterns は同じ CSV を同じ読み方で作ってください）"
        )
    speed = np.clip(np.array([lg.actual_speed_kmh for lg in session_logs], dtype=float), 0.0, None)
    timestamps = [lg.timestamp for lg in session_logs]
    accel = np.array([lg.accel_opening for lg in session_logs], dtype=float)
    brake = np.array([lg.brake_opening for lg in session_logs], dtype=float)
    return speed, timestamps, accel, brake, patterns


def _r2(y: np.ndarray, pred: np.ndarray) -> float:
    if len(y) < 2 or float(np.var(y)) <= 1e-9:
        return float("nan")
    return float(r2_score(y, pred))


def _cv_mae(
    x: np.ndarray, y: np.ndarray, groups: np.ndarray, splits: int
) -> tuple[float, float, float]:
    """パターン単位の交差検証 MAE（平均・分割間の標準偏差）と R²（out-of-fold）を返す。

    パターンが 2 本未満で分割できない場合は、in-sample の当てはまりで代用する
    （散らばりが分からないので mae_std は 0.0）。
    """
    n_groups = len(np.unique(groups))
    k = min(splits, n_groups)
    if k < 2:
        model = make_estimator().fit(x, y)
        pred = model.predict(x)
        return float(mean_absolute_error(y, pred)), 0.0, _r2(y, pred)

    oof = np.full(len(y), np.nan)
    fold_maes: list[float] = []
    for train, test in GroupKFold(k).split(x, y, groups=groups):
        model = make_estimator().fit(x[train], y[train])
        pred = model.predict(x[test])
        oof[test] = pred
        fold_maes.append(float(mean_absolute_error(y[test], pred)))
    return float(mean_absolute_error(y, oof)), float(np.std(fold_maes)), _r2(y, oof)


@dataclass(frozen=True)
class SearchStep:
    """貪欲法の 1 段（開始点、または 1 本足した後）。"""

    added_horizon_s: float | None  # None = 開始点（regime_horizon_s のみ）
    lookahead_s: tuple[float, ...]
    cv_mae: float
    cv_mae_std: float
    r2: float
    n: int
    # 案a（手順6 段2。2026-09-28）: 実質Kp（deviation_gain）の最小値。NaN = 確認速度が
    # 学習行の v0 範囲に無く未確認（合否には数えない。合格として扱う）
    min_gain: float = float("nan")
    # この段で「MAE は改善するが実質Kp が逆向き」として除外した候補の数（表示用）
    n_gain_rejected: int = 0


@dataclass(frozen=True)
class HorizonSearchResult:
    """1 ペダル分の探索結果。"""

    pedal: str  # "accel" | "brake"
    spec: FeatureSpec
    steps: tuple[SearchStep, ...]

    @property
    def cv_mae(self) -> float:
        return self.steps[-1].cv_mae


@dataclass(frozen=True)
class _EvalResult:
    """`search_pedal.evaluate` の 1 候補ぶんの結果（CV スコア＋実質Kpの確認）。"""

    mae: float
    mae_std: float
    r2: float
    n: int
    gains: dict[float, float]  # 確認した速度 → deviation_gain（v0 範囲外の速度は含まない）

    @property
    def min_gain(self) -> float:
        return min(self.gains.values()) if self.gains else float("nan")

    def passes_gain_check(self, min_deviation_gain: float) -> bool:
        """`gains` が空（確認速度が v0 範囲に無い）なら合格扱い（判断材料が無いため）。"""
        return not self.gains or self.min_gain >= min_deviation_gain


def search_pedal(
    *,
    pedal: Literal["accel", "brake"],
    speed: np.ndarray,
    timestamps: list[datetime],
    label: np.ndarray,
    deadband_pct: float,
    groups: np.ndarray,
    regime_horizon_s: float,
    past_horizons_s: tuple[float, ...],
    past_as_delta: bool,
    include_v0_sq: bool,
    include_dv_regime_x_v0: bool,
    grid: Sequence[float],
    max_horizons: int,
    min_improvement: float,
    cv_splits: int,
    gain_check_speeds_kmh: Sequence[float] = (),
    min_deviation_gain: float = float("-inf"),
) -> HorizonSearchResult:
    """貪欲法（前向き選択）で 1 ペダルの先読みホライズンを選ぶ。

    `regime_horizon_s`（既定 h1=1.0s）だけの組から始め、`grid`（`regime_horizon_s` を除く）の
    中で交差検証 MAE が最も下がる 1 本を足す。相対改善が `min_improvement` 未満、または本数が
    `max_horizons` に達したら止める。学習行は A1 と同じ（開度 ≥ `deadband_pct` の行だけ）。

    案a（手順6 段2。2026-09-28）: `gain_check_speeds_kmh` を渡すと、各候補を全有効行で 1 回
    fit したモデルで `ff_model.deviation_gain` を測り（候補の学習行の v0 範囲に入る速度だけ）、
    最小値が `min_deviation_gain` を下回る候補を CV-MAE に関係なく除外する（実車速のずれに
    逆向きに反応する＝ずれが自分で広がる組を採らない）。既定（空リスト）は確認しない
    （後方互換。`min_deviation_gain` の既定 `-inf` も常に合格になるので無害）。開始点
    （`regime_horizon_s` のみ）が不合格でも打ち切らない（実測で、少数特徴の開始点はノイズで
    符号が弱く、ホライズンを 1 本足すだけで直る組が実在した）。格子のどの組を足しても直らない
    場合は最終結果（`spec`）が不合格のまま返る。この関数は最終防波堤ではなく、呼び出し元
    （`pattern_drive.build_ff_model`）が保存直前の pkl をもう一度確認して止める。

    Raises:
        LearningDataError: 開始点（`regime_horizon_s` のみ）の学習行が `MIN_REGIME_SAMPLES` 未満
    """
    mask_full = label >= deadband_pct
    cache: dict[tuple[float, ...], _EvalResult] = {}

    def evaluate(lookahead: tuple[float, ...]) -> _EvalResult:
        if lookahead in cache:
            return cache[lookahead]
        spec = FeatureSpec(
            lookahead_horizons_s=lookahead,
            past_horizons_s=past_horizons_s,
            regime_horizon_s=regime_horizon_s,
            include_v0_sq=include_v0_sq,
            include_dv_regime_x_v0=include_dv_regime_x_v0,
            past_as_delta=past_as_delta,
        )
        x, idx = build_feature_matrix(
            speed,
            estimate_offsets(timestamps, lookahead),
            estimate_offsets(timestamps, past_horizons_s),
            spec,
        )
        m = mask_full[idx]
        xx, yy, gg = x[m], label[idx][m], groups[idx][m]
        if len(yy) == 0:
            # 行が無い組み合わせ（開度 ≥ 不感帯の行がこの先読みの有効範囲に無い）。
            # MIN_REGIME_SAMPLES 未満として呼び出し側で LearningDataError にする
            result = _EvalResult(float("nan"), 0.0, float("nan"), 0, {})
        else:
            mae, mae_std, r2 = _cv_mae(xx, yy, gg, cv_splits)
            gains: dict[float, float] = {}
            if gain_check_speeds_kmh:
                # v0 は特徴量の先頭列（FeatureSpec.feature_names の並び）。候補の学習行の
                # v0 範囲の外は測らない（学習域外の外挿を確認扱いにしないため）
                v0_lo, v0_hi = float(xx[:, 0].min()), float(xx[:, 0].max())
                in_range = [v for v in gain_check_speeds_kmh if v0_lo <= v <= v0_hi]
                if in_range:
                    final_model = make_estimator().fit(xx, yy)
                    gains = {
                        v: deviation_gain(final_model, spec, v, pedal=pedal) for v in in_range
                    }
            result = _EvalResult(mae, mae_std, r2, int(len(yy)), gains)
        cache[lookahead] = result
        return result

    current = (regime_horizon_s,)
    start = evaluate(current)
    if start.n < MIN_REGIME_SAMPLES:
        raise LearningDataError(
            f"{pedal} が効いている行が不足しています"
            f"（{start.n} 点、開度 ≥ 不感帯 {deadband_pct:.2f}%）。"
            f"最低 {MIN_REGIME_SAMPLES} 点必要です（ホライズン自動選択）。"
        )
    # 開始点（h1 のみ）の実質Kpが下限を割っていても、ここでは止めない。実測（155259。
    # ProblemReport_20260921 6-7/6-8）で、開始点だけの少数特徴では符号が弱くノイズで
    # 反転する一方、そこにホライズンを 1 本足すだけで正しい向きに直る組が実在した
    # （それが段2 で実際に使われた pkl のブレーキ側）。開始点で打ち切ると、この「足せば
    # 直る」組を試す機会そのものが無くなってしまう。下の貪欲法が各候補ごとに合否を見て
    # 除外するので、最終的に選ばれる組が逆向きのまま残ることはない（万一、格子のどの組を
    # 足しても直らない場合は開始点のまま返り、呼び出し元の `pattern_drive.build_ff_model` が
    # 保存直前の pkl をもう一度確認して止める）。
    steps = [SearchStep(None, current, start.mae, start.mae_std, start.r2, start.n, start.min_gain)]
    remaining = [h for h in grid if h != regime_horizon_s]

    while len(current) < max_horizons and remaining:
        best: tuple[float, _EvalResult, tuple[float, ...]] | None = None
        n_gain_rejected = 0
        for h in remaining:
            cand = tuple(sorted((*current, h)))
            res = evaluate(cand)
            if res.n == 0:
                continue  # この組は有効行が無い（データ不足）→ 候補から除外
            if not res.passes_gain_check(min_deviation_gain):
                n_gain_rejected += 1
                continue  # MAE は良くても実質Kpが逆向き → 候補から除外（案a）
            if best is None or res.mae < best[1].mae:
                best = (h, res, cand)
        if best is None:
            break  # 足せる候補が無い（残り全部が有効行0か実質Kp不合格）→ ここまでで打ち切る
        h, res, cand = best
        if res.mae >= steps[-1].cv_mae * (1.0 - min_improvement):
            break
        current = cand
        remaining = [x for x in remaining if x != h]
        steps.append(
            SearchStep(
                h, current, res.mae, res.mae_std, res.r2, res.n, res.min_gain, n_gain_rejected
            )
        )

    spec = FeatureSpec(
        lookahead_horizons_s=current,
        past_horizons_s=past_horizons_s,
        regime_horizon_s=regime_horizon_s,
        include_v0_sq=include_v0_sq,
        include_dv_regime_x_v0=include_dv_regime_x_v0,
        past_as_delta=past_as_delta,
    )
    return HorizonSearchResult(pedal=pedal, spec=spec, steps=tuple(steps))


@dataclass(frozen=True)
class HorizonSearchSettings:
    """`search_pedal` に渡す共通設定（`config.FeaturesSection` から組み立てる）。"""

    grid: tuple[float, ...]
    max_horizons: int
    min_improvement: float
    cv_splits: int
    regime_horizon_s: float
    past_horizons_s: tuple[float, ...]
    past_as_delta: bool
    include_v0_sq: bool
    include_dv_regime_x_v0: bool
    # 案a（手順6 段2。2026-09-28）。既定は確認しない（後方互換。テストの `_settings()` など）
    gain_check_speeds_kmh: tuple[float, ...] = ()
    min_deviation_gain: float = float("-inf")


def search_ff_horizons(
    logs: list[DriveLog],
    patterns: np.ndarray,
    accel_deadband_pct: float,
    brake_deadband_pct: float,
    settings: HorizonSearchSettings,
) -> tuple[HorizonSearchResult, HorizonSearchResult]:
    """アクセル・ブレーキそれぞれの `search_pedal` を実行し、(アクセル, ブレーキ) を返す。"""
    speed, timestamps, accel, brake, groups = _single_session_inputs(logs, patterns)
    common = {
        "speed": speed,
        "timestamps": timestamps,
        "groups": groups,
        "regime_horizon_s": settings.regime_horizon_s,
        "past_horizons_s": settings.past_horizons_s,
        "past_as_delta": settings.past_as_delta,
        "include_v0_sq": settings.include_v0_sq,
        "include_dv_regime_x_v0": settings.include_dv_regime_x_v0,
        "grid": settings.grid,
        "max_horizons": settings.max_horizons,
        "min_improvement": settings.min_improvement,
        "cv_splits": settings.cv_splits,
        "gain_check_speeds_kmh": settings.gain_check_speeds_kmh,
        "min_deviation_gain": settings.min_deviation_gain,
    }
    accel_result = search_pedal(
        pedal="accel", label=accel, deadband_pct=accel_deadband_pct, **common
    )
    brake_result = search_pedal(
        pedal="brake", label=brake, deadband_pct=brake_deadband_pct, **common
    )
    return accel_result, brake_result


def format_result_table(result: HorizonSearchResult) -> str:
    """探索の各段（足したホライズン・MAE・R²・分割のばらつき・実質Kp）を Markdown 表にする。

    「実質Kp 最小」は案a（手順6 段2）の合否条件に使った値（符号つき。負なら本来は除外されて
    いるはずなので、正常な結果では表に負の値は出ない）。「除外(実質Kp)」はその段で MAE が
    より良かったのに実質Kpの下限を割って除外した候補の数（n/a = 未確認の候補・行不足のみ）。
    """
    header = [
        "段", "足したホライズン[s]", "先読み[s]", "CV-MAE", "分割間の標準偏差", "R²", "n",
        "実質Kp最小", "除外(実質Kp)",
    ]
    rows = []
    for i, step in enumerate(result.steps):
        added = "(開始)" if step.added_horizon_s is None else f"+{step.added_horizon_s:g}"
        rows.append(
            [
                str(i),
                added,
                str(list(step.lookahead_s)),
                f"{step.cv_mae:.4f}",
                f"{step.cv_mae_std:.4f}",
                f"{step.r2:.4f}" if step.r2 == step.r2 else "n/a",  # NaN チェック
                str(step.n),
                f"{step.min_gain:.3f}" if step.min_gain == step.min_gain else "n/a",  # NaN
                str(step.n_gain_rejected),
            ]
        )
    lines = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    lines += ["| " + " | ".join(r) + " |" for r in rows]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    """CLI: `.venv/bin/python -m tests.research.horizon_search <csv>`（手順2 は再実行しない）。

    `relearn`/`main` の手順2 が `horizon_search: true` のときに内部で呼ぶのと同じ探索を、
    config・不感帯の実測値を使って単独で確認するためのツール。
    """
    import argparse

    from tests.research.config import DEFAULT_CONFIG_PATH, load_config

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="手順2 の走行ログ CSV（PATTERN_DRIVE を含む）")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    args = parser.parse_args(argv)

    cfg = load_config(args.config)
    ft = cfg.features
    settings = HorizonSearchSettings(
        grid=ft.search_grid(),
        max_horizons=ft.search_max_horizons,
        min_improvement=ft.search_min_improvement,
        cv_splits=ft.search_cv_splits,
        regime_horizon_s=ft.h1_s,
        past_horizons_s=ft.past_horizons_s(),
        past_as_delta=ft.past_as_delta,
        include_v0_sq=ft.use_v0_sq,
        include_dv_regime_x_v0=ft.use_dv1_x_v0,
        gain_check_speeds_kmh=tuple(ft.search_gain_check_speeds_kmh),
        min_deviation_gain=ft.search_min_deviation_gain,
    )
    logs = read_drive_logs(args.csv)
    patterns = read_pattern_groups(args.csv)
    print(f"学習データ: {args.csv}（{len(logs)} 行・{len(np.unique(patterns))} パターン）")
    accel_result, brake_result = search_ff_horizons(
        logs, patterns, cfg.feedforward.accel_deadband_pct, cfg.feedforward.brake_deadband_pct,
        settings,
    )
    for result in (accel_result, brake_result):
        print(f"\n## {result.pedal}\n")
        print(format_result_table(result))
        print(f"\n選んだ先読み: {list(result.spec.lookahead_horizons_s)}")
    return 0


if __name__ == "__main__":
    import sys

    sys.exit(main())
