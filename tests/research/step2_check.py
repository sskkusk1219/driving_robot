from __future__ import annotations

import argparse
import asyncio
import math
import statistics
import sys
from collections.abc import Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from tests.research.config import DEFAULT_CONFIG_PATH, ResearchConfig, load_config
from tests.research.fwd_data import DEFAULT_ACCEL_LAG_S, DEFAULT_BRAKE_LAG_S, FwdRows
from tests.research.fwd_map import (
    LOW_RELEASE_BAND_MAX_KMH,
    RELEASE_GRID_PCT,
    ForwardTable,
    load_forward_map,
    release_seconds_in_band,
)
from tests.research.fwd_plan import FLAG_PREPOS, build_plan
from tests.research.fwd_tool import DeadbandLag, err_stats, read_logs, residuals, settings_of_table
from tests.research.pedal_lag import estimate_pedal_return
from tests.research.play_onset import OnsetResult, coast_fn_from_curve, onsets_from_csv
from tests.research.research_types import DrivingMode

STATUS_OK = "OK"
STATUS_NG = "NG"
STATUS_SKIP = "SKIP"

# B1 の乱数の外れ: 乱数ごとの学習データでの MAE が「残りの乱数の中央値 × この倍率」を超えたら外れ。
# 段2o-7 仮値。135724 の乱数ごとの値で見直す
SEED_OUTLIER_RATIO = 1.2

# 合否のしきい値（キー = "<項目id>.<metrics のキー>"。値は metrics の絶対値の上限。超えたら NG。
# 値が nan のときも NG = 測れていない）。None は判定しない（記録だけ）。
# 手順3 で C8 を使うときに影響する範囲だけを見る。車の特性の数値は使わず、「別の道筋で求めた値との
# 差」か「手順3 の中で使う量」で決める（他の車でも同じ基準が使える）
# 2026-10-23 ユーザー決定: 実車では効き始めの求め方で 2% ほど違いが出るため
# （手順2 `151009`: meta 9.95%・走行中 9.90%・計算し直し 9.35%）
B5_TOL_PCT = 2.0

THRESHOLDS: dict[str, float | None] = {
    # NN の MAE ÷ 表（C7。データから直接作った表）の MAE。1.0 を超えたら NN の方が悪い = 学習失敗
    "B1.ratio_mae": 1.0,
    "B1.ratio_p95": None,
    # 外れた乱数の数（SEED_OUTLIER_RATIO）。1 つでもあれば NG（build_ff_model は 1 回だけ
    # 学習し直す）
    "B1.seed_outlier_count": 0.0,
    # 惰行の差の最大は cfg.forward_map.coast_band_kmhs（手順3 で踏むか惰行かを分ける帯の幅）を
    # check_coast が使う。ここには置かない（config から読む）
    "B3.coast_diff_max": None,
    # yaml のクリープ加速度に正があるのに発進側の表が無い = 1。値の差では判定しない
    # （yaml のクリープカーブは止まりかけの定常の行、発進側の表は発進途中の行から作るので、
    # 正しい走行でも 0.3〜0.5 そろってずれる）
    "B3.creep_launch_missing": 0.0,
    # 遅れ [s]。0.005 は制御周期（50 ms）の 1/10 で、これより細かい差は手順3 の開度の列を変えない
    "B4.accel_lag_diff_s": 0.005,
    "B4.brake_lag_diff_s": 0.005,
    # 効き始め [%]。B5_TOL_PCT を超える差で NG
    # （段のように立ち上がる効きでの誤り、meta 5.40% ↔ 10.15% を検出する）
    "B5.accel_in_run_diff_pct": B5_TOL_PCT,
    "B5.brake_in_run_diff_pct": B5_TOL_PCT,
    "B5.accel_recomputed_diff_pct": B5_TOL_PCT,
    "B5.brake_recomputed_diff_pct": B5_TOL_PCT,
    # 戻る時間（遅れ [s]・戻る速さ [%/s]）。meta は同じ CSV から測った値なので、計算し直した値と
    # 浮動小数の誤差の範囲で一致する
    "B6.accel_delay_diff_s": 1e-6,
    "B6.brake_delay_diff_s": 1e-6,
    "B6.accel_rate_diff_pct_s": 1e-4,
    "B6.brake_rate_diff_pct_s": 1e-4,
    # 手順3 の列で切り替えのすき間が戻る時間に足りない回数（段B の処理が全部直すので 0 のはず）
    "D1.gap_violations": 0.0,
    # 停車保持開度 [%]。yaml の書式（小数 2 桁）の丸め誤差だけ許す
    "C1.stop_brake_diff_pct": 0.01,
    # 保存した pkl とパスが違う = 1（0 = 同じ）
    "C2.path_diff": 0.0,
    "C2.kind_mismatch": None,
    # 手順3 の列。NaN・先置きの超過（浮動小数の誤差 1e-9 だけ許す）は 1 つでも NG
    "D1.nan_count": 0.0,
    # 超過量は「置いた開度 − 上限」で負が正常なので、正の部分 prepos_excess_* で判定する
    "D1.prepos_excess_accel_pct": 1e-9,
    "D1.prepos_excess_brake_pct": 1e-9,
    # 範囲外・穴埋めの秒数 ÷ モードの秒数。仮値（段2o-7）: 正しく学習できた 3 本の最大が
    # 範囲外 89/1800 = 5%・穴埋め 289/1800 = 16% なので、その 2 倍
    "D1.out_of_range_ratio": 0.10,
    "D1.filled_ratio": 0.32,
}


class Step2CheckError(Exception):
    """手順2 の学習結果のチェックで NG があった（yaml は書き換えていない）。"""

    def __init__(self, items: Sequence[CheckItem]) -> None:
        self.items = list(items)
        super().__init__(format_ng(self.items))


@dataclass
class CheckItem:
    """チェック 1 項目の結果。value は表示用の値、reference は比べた相手、metrics は数値の差。"""

    id: str
    name: str
    status: str  # "OK" / "NG" / "SKIP"（材料が無い）
    value: str = ""
    reference: str = ""
    reason: str = ""
    metrics: dict[str, float] = field(default_factory=dict)


def _skip(item_id: str, name: str, reason: str) -> CheckItem:
    return CheckItem(item_id, name, STATUS_SKIP, "-", "-", reason)


def _judge(
    item_id: str, metrics: dict[str, float], limits: dict[str, float] | None = None
) -> str:
    """しきい値が入っている metrics が 1 つでも超えていれば NG。全部 None なら OK。
    しきい値があるのに値が nan のときも NG（測れていない）。limits は THRESHOLDS より優先する
    しきい値（config から読む値用）。"""
    for key, val in metrics.items():
        thr = (limits or {}).get(key, THRESHOLDS.get(f"{item_id}.{key}"))
        if thr is None:
            continue
        if not math.isfinite(val) or abs(val) > thr:
            return STATUS_NG
    return STATUS_OK


def _f(x: float, nd: int = 2) -> str:
    if math.isnan(x):
        return "-"
    return "inf" if math.isinf(x) else f"{x:.{nd}f}"


def _item(
    item_id: str,
    name: str,
    metrics: dict[str, float],
    value: str,
    reference: str,
    reason: str,
    limits: dict[str, float] | None = None,
) -> CheckItem:
    return CheckItem(
        item_id, name, _judge(item_id, metrics, limits), value, reference, reason, metrics
    )


def ng_items(items: Sequence[CheckItem]) -> list[CheckItem]:
    return [i for i in items if i.status == STATUS_NG]


def format_ng(items: Sequence[CheckItem]) -> str:
    """NG の項目と理由を 1 項目 1 行にする。"""
    lines = ["手順2 の学習結果のチェックで NG:"]
    for i in ng_items(items):
        lines.append(f"  {i.id} {i.name}: {i.value}（比べた相手: {i.reference}）{i.reason}")
    return "\n".join(lines)



# ─────────────────────────────────────────────────────────────────────
# B. 作ったモデルの中身
# ─────────────────────────────────────────────────────────────────────


def seed_outliers(seed_mae: dict[Any, float], ratio: float = SEED_OUTLIER_RATIO) -> list[int]:
    """乱数ごとの学習データでの MAE のうち、「残りの乱数の中央値 × ratio」を超えたものの乱数。
    乱数が 1 つだけなら比べる相手が無いので外れなしとする。"""
    out: list[int] = []
    for seed, mae in seed_mae.items():
        others = [m for k, m in seed_mae.items() if k != seed]
        if others and mae > statistics.median(others) * ratio:
            out.append(int(seed))
    return sorted(out)


def check_fit(
    table_c8: ForwardTable | None,
    table_c7: ForwardTable | None,
    rows: Sequence[FwdRows] | None,
) -> CheckItem:
    """B1: 学習データでの外れ（実測の加速度 − 表の加速度）の MAE・3 秒移動平均の p95。

    C8 と C7 で出し、比（C8/C7）も metrics に入れる（MAE の比 > 1.0 で NG）。乱数ごとの
    学習データでの MAE は C8 の meta の `seed_mae`（{乱数: MAE}。`fwd_final.build_final_table` が
    入れる）から読み、外れた乱数（`seed_outliers`）があれば NG。
    """
    name = "学習データでの外れ"
    if table_c8 is None or not rows or sum(len(r) for r in rows) == 0:
        return _skip("B1", name, "C8 の pkl か学習の行が無い")

    def stats(table: ForwardTable) -> tuple[float, float, int]:
        res = residuals(table, list(rows))
        st = err_stats(res["err"], res["ma"], np.ones(len(res["err"]), dtype=bool))
        return st.mae, st.ma_p95, st.n

    mae8, p958, n = stats(table_c8)
    metrics: dict[str, float] = {"n_rows": float(n), "c8_mae": mae8, "c8_p95": p958}
    ref = "C7 の表なし"
    if table_c7 is not None:
        mae7, p957, _ = stats(table_c7)
        metrics.update(
            c7_mae=mae7, c7_p95=p957,
            ratio_mae=mae8 / mae7 if mae7 > 0 else math.nan,
            ratio_p95=p958 / p957 if p957 > 0 else math.nan,
        )
        ref = f"C7 MAE {_f(mae7, 3)}・p95 {_f(p957, 3)}（比 {_f(metrics['ratio_mae'], 3)}／" \
              f"{_f(metrics['ratio_p95'], 3)}）"
    reason = "乱数ごとの MAE なし（meta に seed_mae が無い）"
    seed_mae = table_c8.meta.get("seed_mae")
    if seed_mae:
        outliers = seed_outliers(seed_mae)
        metrics["seed_outlier_count"] = float(len(outliers))
        reason = (
            "乱数ごとの MAE "
            + "・".join(f"{k}:{_f(float(v), 3)}" for k, v in seed_mae.items())
            + (f"／外れ: 乱数 {outliers}" if outliers else "／外れなし")
        )
    return _item(
        "B1", name, metrics,
        f"C8 MAE {_f(mae8, 3)}・p95 {_f(p958, 3)} km/h/s（{n} 行）", ref, reason,
    )


def _diff_stats(diff: np.ndarray) -> tuple[float, float]:
    return (float(np.max(diff)), float(np.median(diff))) if len(diff) else (math.nan, math.nan)


# 惰行の比べる範囲の下限 = クリープ車速 + この値 [km/h]。それより遅い所は表の車速の刻み
# （2.5 km/h）の補間の差が出るだけで、手順3 の判断に効かない
COAST_CHECK_MARGIN_KMH = 5.0


def check_coast(
    table: ForwardTable | None,
    coast_speeds: Sequence[float],
    coast_decel: Sequence[float],
    creep_accel: Sequence[float],
    *,
    creep_speed_kmh: float,
    mode_max_kmh: float | None,
    band_kmhs: float,
) -> CheckItem:
    """B3: 表の u=0 の加速度 `coast_accel(v)`（負が減速）と、yaml の実測カーブの差。

    惰行: yaml の惰行カーブ（減速量。正）の点のうち、車速が [クリープ車速 + 5, モードの最高車速]
    の点だけで、差 = |coast_accel(v) + 減速量|。差の最大が band_kmhs（手順3 で踏むか惰行かを分ける
    帯の幅。`forward_map.coast_band_kmhs`）を超えたら NG。モードの最高車速が無ければ上限なし。
    クリープ: 値では判定しない。yaml のクリープ加速度に正の値があるのに発進側の表
    （`table.launch_accel`）が無ければ NG。停止側の表（`table.stop_accel`）の有無は表示するだけ
    （2026-10-18 段2o-10）。
    """
    name = "惰行・クリープ"
    if table is None:
        return _skip("B3", name, "pkl が無い")
    has_creep = len(creep_accel) > 0
    if len(coast_speeds) == 0 and not has_creep:
        return _skip("B3", name, "yaml に惰行カーブもクリープカーブも無い")
    metrics: dict[str, float] = {}
    parts: list[str] = []
    lo = creep_speed_kmh + COAST_CHECK_MARGIN_KMH
    hi = math.inf if mode_max_kmh is None else float(mode_max_kmh)
    if len(coast_speeds) > 0:
        v_all = np.asarray(coast_speeds, dtype=float)
        sel = (v_all >= lo) & (v_all <= hi)
        v = v_all[sel]
        if len(v) == 0:
            parts.append(f"惰行 {lo:g}〜{hi:g} km/h に点なし")
        else:
            d = np.abs(
                np.asarray(table.coast_accel(v), dtype=float) + np.asarray(coast_decel)[sel]
            )
            mx, md = _diff_stats(d)
            metrics.update(coast_diff_max=mx, coast_diff_median=md, coast_n=float(len(v)))
            parts.append(f"惰行 最大 {_f(mx, 3)}・中央 {_f(md, 3)}（{len(v)} 点）")
    if has_creep:
        positive = any(float(x) > 0.0 for x in creep_accel)
        missing = positive and table.launch_accel is None
        metrics["creep_launch_missing"] = 1.0 if missing else 0.0
        parts.append(
            "クリープ加速度あり・発進側の表" + ("なし" if table.launch_accel is None else "あり")
            + "・停止側の表" + ("なし" if table.stop_accel is None else "あり")
            if positive else "クリープ加速度は正なし"
        )
    if not metrics:
        return _skip("B3", name, "比べる点が無い（" + "・".join(parts) + "）")
    return _item(
        "B3", name, metrics, "／".join(parts),
        f"yaml の惰行カーブ [km/h/s]（{lo:g}〜{'上限なし' if math.isinf(hi) else f'{hi:g}'} km/h）"
        f"・差の上限 {band_kmhs:g}",
        "", limits={"coast_diff_max": band_kmhs},
    )


def check_lags(
    table: ForwardTable | None,
    measured_accel_s: float | None,
    measured_brake_s: float | None,
    default_accel_s: float = DEFAULT_ACCEL_LAG_S,
    default_brake_s: float = DEFAULT_BRAKE_LAG_S,
) -> CheckItem:
    """B4: pkl の meta の accel_lag_s・brake_lag_s と、測れた遅れの差（測れなければ既定値）。

    測れた遅れは `pattern_drive.measure_pedal_lags` の accel_s・brake_s。差 = meta − 比べた値。
    """
    name = "遅れ"
    if table is None:
        return _skip("B4", name, "pkl が無い")
    metrics: dict[str, float] = {}
    parts: list[str] = []
    refs: list[str] = []
    for label, key, meas, dflt in (
        ("accel", "accel_lag_s", measured_accel_s, default_accel_s),
        ("brake", "brake_lag_s", measured_brake_s, default_brake_s),
    ):
        raw = table.meta.get(key)
        meta = float(raw) if raw is not None and math.isfinite(float(raw)) else math.nan
        cmp_val = dflt if meas is None else meas
        metrics[f"{label}_lag_diff_s"] = meta - cmp_val
        metrics[f"{label}_lag_measured"] = 0.0 if meas is None else 1.0
        parts.append(f"{label} meta {_f(meta, 3)}s")
        refs.append(f"{label} {'測定' if meas is not None else '既定'} {_f(cmp_val, 3)}s")
    value = "・".join(parts)
    return _item(
        "B4", name, metrics, value, "・".join(refs),
        f"差: accel {_f(metrics['accel_lag_diff_s'], 3)}"
        f"・brake {_f(metrics['brake_lag_diff_s'], 3)}",
    )


def check_returns(table: ForwardTable | None, csv_path: Path) -> CheckItem:
    """B6: pkl の meta の戻る時間（{accel,brake}_return_delay_s・_return_rate_pct_s）と、CSV から
    計算し直した値（`pedal_lag.estimate_pedal_return`）の差（2026-10-17 段2o-9）。

    計算し直しが測れなかったペダルは「省略」（理由を出す）。測れたのに meta に無ければ差は無限大
    （= NG）。両ペダルとも測れなければ SKIP。
    """
    name = "戻る時間"
    if table is None:
        return _skip("B6", name, "pkl が無い")
    res = estimate_pedal_return(csv_path)
    metrics: dict[str, float] = {}
    parts: list[str] = []
    refs: list[str] = []
    notes: list[str] = []
    for label, fit in (("accel", res.accel), ("brake", res.brake)):
        if not fit.measured:
            notes.append(f"{label} 省略（{fit.reason}）")
            continue
        for key, short, rec, unit in (
            (f"{label}_return_delay_s", "delay", fit.delay_s, "s"),
            (f"{label}_return_rate_pct_s", "rate", fit.rate_pct_s, "%/s"),
        ):
            raw = table.meta.get(key)
            meta = float(raw) if raw is not None and math.isfinite(float(raw)) else None
            metrics[f"{label}_{short}_diff_{'s' if short == 'delay' else 'pct_s'}"] = (
                math.inf if meta is None else meta - float(rec)  # type: ignore[arg-type]
            )
            parts.append(f"{label} {short} " + ("なし" if meta is None else f"{meta:.3f}{unit}"))
            refs.append(f"{label} {short} {float(rec):.3f}{unit}")  # type: ignore[arg-type]
    if not metrics:
        return _skip("B6", name, "計算し直しが測れない（" + "・".join(notes) + "）")
    return _item(
        "B6", name, metrics, "・".join(parts), "・".join(refs),
        "・".join(notes),
    )


def check_onsets(
    table: ForwardTable | None,
    pedal_min_a: float,
    pedal_min_b: float,
    in_run: tuple[float | None, float | None] | None = None,
    recomputed: tuple[OnsetResult, OnsetResult] | None = None,
) -> CheckItem:
    """B5: pkl の meta の効き始め（accel_onset_u_pct・brake_onset_u_pct は min からの量）を
    絶対開度（min ＋ u）にして、次の 2 つと比べる。差は絶対値 [%]。

    ① 走行中の最後の値（RampResult の最後の accel_onset_pct・brake_onset_pct。絶対開度）
    ② yaml の惰行カーブで計算し直した値（`play_onset.onsets_from_csv` の結果）

    差 > B5_TOL_PCT で NG。meta にキーが無いのに①か②が
    求まっているときは差を無限大として記録する（= NG）。①も②も無いペダルは比べず、理由を残す。
    全ペダルで比べる値が無ければ SKIP。
    """
    name = "効き始め"
    if table is None:
        return _skip("B5", name, "pkl が無い")
    if in_run is None and recomputed is None:
        return _skip("B5", name, "走行中の値も計算し直した値も無い")
    metrics: dict[str, float] = {}
    parts: list[str] = []
    refs: list[str] = []
    notes: list[str] = []
    in_run_t = in_run if in_run is not None else (None, None)
    rec_t = recomputed if recomputed is not None else (None, None)
    for label, key, pmin, run_v, rec in (
        ("accel", "accel_onset_u_pct", pedal_min_a, in_run_t[0], rec_t[0]),
        ("brake", "brake_onset_u_pct", pedal_min_b, in_run_t[1], rec_t[1]),
    ):
        raw = table.meta.get(key)
        meta_abs = pmin + float(raw) if raw is not None and math.isfinite(float(raw)) else None
        metrics[f"{label}_meta_abs_pct"] = math.nan if meta_abs is None else meta_abs
        parts.append(f"{label} meta " + ("なし" if meta_abs is None else f"{meta_abs:.2f}%"))
        if run_v is not None:
            metrics[f"{label}_in_run_diff_pct"] = (
                math.inf if meta_abs is None else abs(meta_abs - run_v)
            )
            refs.append(f"{label} 走行中 {run_v:.2f}%")
        elif in_run is not None:
            notes.append(f"{label} 走行中の値なし")
        if run_v is None and rec is None:
            notes.append(f"{label} は走行中の値も計算し直した値も無く比べない")
        if rec is not None:
            if rec.onset_pct is None:
                notes.append(f"{label} 計算し直しは求まらず（{rec.reason}）")
            else:
                metrics[f"{label}_recomputed_pct"] = float(rec.onset_pct)
                metrics[f"{label}_recomputed_diff_pct"] = (
                    math.inf if meta_abs is None else abs(meta_abs - float(rec.onset_pct))
                )
                refs.append(f"{label} 計算し直し {rec.onset_pct:.2f}%")
    if not any(k.endswith("_diff_pct") for k in metrics):
        return _skip("B5", name, "比べる値が求まっていない（" + "・".join(notes) + "）")
    diffs = [v for k, v in metrics.items() if k.endswith("_diff_pct")]
    return _item(
        "B5", name, metrics, "・".join(parts), "・".join(refs),
        "差の最大 " + ("inf" if math.inf in diffs else f"{max(diffs):.2f}") + "%"
        + ("／" + "・".join(notes) if notes else ""),
    )


# ─────────────────────────────────────────────────────────────────────
# C. yaml に書いた値
# ─────────────────────────────────────────────────────────────────────


def check_measured_in_yaml(cfg: ResearchConfig, pedal_result: Any | None) -> CheckItem:
    """C1: yaml を読み直した停車保持開度（絶対開度）と、2-0 の実測
    （`pedal_search.PedalSearchResult`）の差。pedal_result が None なら SKIP。
    クリープ車速は手順3 に効かないので見ない。"""
    name = "yaml の停車保持開度"
    if pedal_result is None:
        return _skip("C1", name, "2-0 の実測（PedalSearchResult）が無い")
    stop_diff = abs(cfg.stop_brake_opening_pct - float(pedal_result.stop_brake_opening_pct))
    metrics = {"stop_brake_diff_pct": stop_diff}
    return _item(
        "C1", name, metrics,
        f"停車保持 {cfg.stop_brake_opening_pct:.2f}%",
        f"実測 停車保持 {pedal_result.stop_brake_opening_pct:.2f}%",
        f"差 {stop_diff:.3f}%",
    )


def _same_file(a: Path, b: Path) -> bool:
    """同じファイルか。相対/絶対の違い・シンボリックリンクを吸収する（実在すれば samefile）。"""
    try:
        if a.exists() and b.exists():
            return a.samefile(b)
    except OSError:
        pass
    return a.absolute().resolve() == b.absolute().resolve()


def check_model_path(cfg: ResearchConfig, saved_path: str | Path | None) -> CheckItem:
    """C2: yaml の feedforward.model_path が保存した pkl と同じか、`load_forward_map` で読めるか。
    saved_path が None なら「読めるか」だけ見る。"""
    name = "model_path"
    yaml_path = Path(cfg.feedforward.model_path)
    metrics: dict[str, float] = {}
    notes: list[str] = []
    if saved_path is not None:
        same = _same_file(yaml_path, Path(saved_path))
        # 2026-10-16: 名前が same_path で 0 = 同じ だったため、同じパスなのに 0 と出て読み違えた。
        # 違いの有無を表す path_diff（0 = 同じ、1 = 違う）にした
        metrics["path_diff"] = 0.0 if same else 1.0
        notes.append("保存した pkl と同じ" if same else f"保存した pkl {saved_path} と違う")
    try:
        table = load_forward_map(yaml_path)
        metrics["readable"] = 1.0
        notes.append(f"読めた（kind={table.kind}）")
        want = {
            "C7": "table", "C8": "model", "C10": "model", "C11": "model", "C12": "model",
            "C13": "model", "C14": "model", "C15": "model",
        }.get(cfg.feedforward.candidate)
        if want is not None and table.kind != want:
            metrics["kind_mismatch"] = 1.0
            notes.append(f"candidate {cfg.feedforward.candidate} と kind が違う")
    except Exception as exc:  # noqa: BLE001 - 壊れた pkl は pickle 由来の例外も出るので全部「読めない」
        metrics["readable"] = 0.0
        notes.append(f"読めない: {type(exc).__name__}: {exc}")
    # 読めない pkl は事実としての NG。パスの違い・kind の違いは THRESHOLDS（C2.path_diff）で判定する
    status = STATUS_NG if metrics.get("readable", 0.0) == 0.0 else _judge("C2", metrics)
    return CheckItem(
        "C2", name, status, str(yaml_path),
        str(saved_path) if saved_path is not None else "-", "・".join(notes), metrics,
    )


# ─────────────────────────────────────────────────────────────────────
# D. 手順3 に渡す開度の列
# ─────────────────────────────────────────────────────────────────────


def check_plan(
    cfg: ResearchConfig, table: ForwardTable | None, mode: DrivingMode | None
) -> CheckItem:
    """D1: 手順3 と同じ引数（`mode_drive._prepare_forward_map_plan`）で開度の列を作って見る。

    範囲外・穴埋めの秒数、NaN の数、先置きの統計、先に置く開度の最大が（効き始め − margin）を
    超えていないか（超過量 = 置いた開度の最大 − (min ＋ 効き始め − margin)。0 以下が正常）。
    表の引き方は `ForwardMapFeedforward.build_plan` と同じ `fwd_plan.build_plan` を直接呼ぶ。
    """
    name = "手順3 の開度の列"
    if table is None:
        return _skip("D1", name, "pkl が無い")
    if mode is None:
        return _skip("D1", name, "走行モードが無い（DB から読めていない）")
    fm = cfg.forward_map
    # 2026-10-23: 手順3（mode_drive）と同じ表・引数で作る
    # （A2 の床・FEL・ぼかした基準・C15 の stop_lead 等）
    from tests.research.mode_drive import (  # noqa: PLC0415
        apply_table_options,
        candidate_is_stop_end,
        forward_map_plan_args,
    )

    ref_at, duration_s, plan_kwargs = forward_map_plan_args(
        cfg, mode, stop_end=candidate_is_stop_end(cfg.feedforward.candidate), table=table
    )
    plan = build_plan(ref_at, duration_s, apply_table_options(cfg, table), **plan_kwargs)
    nan_count = int(np.count_nonzero(np.isnan(plan.effort_pct))) + int(
        np.count_nonzero(np.isnan(plan.need_accel_kmhs))
    )
    metrics: dict[str, float] = {
        "duration_s": float(mode.total_duration),
        "out_of_range_s": plan.out_of_range_s(),
        "filled_s": plan.filled_s(),
        "nan_count": float(nan_count),
    }
    if mode.total_duration > 0:
        metrics["out_of_range_ratio"] = metrics["out_of_range_s"] / mode.total_duration
        metrics["filled_ratio"] = metrics["filled_s"] / mode.total_duration
    text = (
        f"範囲外 {metrics['out_of_range_s']:.1f}s・穴埋め {metrics['filled_s']:.1f}s"
        f"・NaN {nan_count}"
    )
    pre = plan.preposition
    if pre is None:
        text += "・先置きなし"
    else:
        metrics.update(
            prepos_placed=float(pre.n_placed), prepos_short=float(pre.n_short),
            prepos_no_time=float(pre.n_no_time), prepos_not_needed=float(pre.n_not_needed),
            prepos_no_onset=float(pre.n_no_onset), prepos_s=float(pre.prepos_s),
        )
        mask = plan.flag == FLAG_PREPOS
        for label, key, pmin, sign in (
            ("accel", "accel_onset_u_pct", cfg.vehicle.min_accel_opening_pct, 1.0),
            ("brake", "brake_onset_u_pct", cfg.vehicle.min_brake_opening_pct, -1.0),
        ):
            sel = mask & (sign * plan.effort_pct > 0.0)
            raw = table.meta.get(key)
            if not sel.any() or raw is None or not math.isfinite(float(raw)):
                continue
            limit = pmin + float(raw) - fm.preposition_margin_pct
            metrics[f"prepos_max_{label}_pct"] = float(np.max(sign * plan.effort_pct[sel]))
            metrics[f"prepos_over_{label}_pct"] = metrics[f"prepos_max_{label}_pct"] - limit
            metrics[f"prepos_excess_{label}_pct"] = max(0.0, metrics[f"prepos_over_{label}_pct"])
        text += f"・先置き: {pre.summary()}"
    gap = plan.return_gap
    if gap is not None:
        metrics.update(
            gap_switches=float(gap.n_switches), gap_cut=float(gap.n_cut),
            gap_cut_s=float(gap.cut_s), gap_delayed=float(gap.n_delayed),
            gap_delayed_s=float(gap.delayed_s), gap_violations=float(gap.n_violations),
        )
        text += f"・切り替えのすき間: {gap.summary()}"
    over = [v for k, v in metrics.items() if k.startswith("prepos_over_")]
    return _item(
        "D1", name, metrics, text, f"モード {mode.name}（{mode.total_duration:.0f}s）",
        "先置きの超過量の最大 " + (f"{max(over):.3f}%" if over else "-（先置きなし）"),
    )


# ─────────────────────────────────────────────────────────────────────
# まとめ
# ─────────────────────────────────────────────────────────────────────


def check_table_extras(table: ForwardTable | None, rows: Sequence[FwdRows] | None) -> CheckItem:
    """表の作り方のお知らせ（2026-10-20 段4-3。NG にしない・しきい値なし）。

    - 発進側・停止側の表の開度の元（meta の side_table_opening_source。cmd＝指令）
    - 停止側の戻しの層を作る設定か（meta の stop_release_layer_enabled。learning.
      stop_release_layer_enabled。無い古い pkl は層の有無から）
    - アクセルの戻し量をニューラルネットの入力にした設定か（meta の accel_release_input_enabled。
      learning.accel_release_input_enabled。無い古い pkl は戻しの表の有無から）
    - アクセルの踏み増し量をニューラルネットの入力にした設定か（meta の accel_push_input_enabled。
      learning.accel_push_input_enabled。2026-10-23 段5-7b。無い古い pkl は表の有無から）と、
      踏み増し量 0.3% 以上の行の秒数・頭打ち
    - 停止側の戻しの層ごとの秒数（meta の stop_release_layer_s。戻し量の格子の順）
    - アクセルの戻し量 0.3% 以上の行の秒数と、頭打ち（戻し量の p95）
    - ブレーキの戻しの行 7.5〜20 km/h の秒数（meta の brake_release_band_s。無い古い pkl は
      手順2 の行から数える）
    """
    name = "表の作り方（段4-3のお知らせ）"
    if table is None:
        return _skip("I1", name, "C8 の表なし")
    m = table.meta
    src = m.get("side_table_opening_source")
    layers = m.get("stop_release_layer_s")
    layer_on = m.get("stop_release_layer_enabled")
    if layer_on is None:  # 古い pkl（設定が meta に無い）は層の有無で見る
        layer_on = table.stop_release_accel is not None
    a_in = m.get("accel_release_input_enabled")
    if a_in is None:  # 古い pkl（設定が meta に無い）は戻しの表の有無で見る
        a_in = table.accel_release_grid is not None
    p_in = m.get("accel_push_input_enabled")
    if p_in is None:  # 古い pkl（設定が meta に無い）は踏み増しの表の有無で見る
        p_in = table.accel_push_grid is not None
    p_s = m.get("accel_push_rows_s")
    p_cap = m.get("accel_push_max_pct")
    grid = m.get("stop_release_grid_pct") or [float(x) for x in RELEASE_GRID_PCT]
    band_s = m.get("brake_release_band_s")
    lo = float(m.get("brake_release_band_lo_kmh") or m.get("stop_max_speed_kmh") or 7.5)
    hi = float(m.get("brake_release_band_hi_kmh") or LOW_RELEASE_BAND_MAX_KMH)
    if band_s is None and rows:
        band_s = sum(release_seconds_in_band(r, lo, hi) for r in rows)
    metrics: dict[str, float] = {}
    if layers:
        metrics["stop_release_total_s"] = float(sum(layers))
    a_s = m.get("accel_release_rows_s")
    a_cap = m.get("accel_release_max_pct")
    if a_s is not None:
        metrics["accel_release_rows_s"] = float(a_s)
    if a_cap is not None:
        metrics["accel_release_max_pct"] = float(a_cap)
    if p_s is not None:
        metrics["accel_push_rows_s"] = float(p_s)
    if p_cap is not None:
        metrics["accel_push_max_pct"] = float(p_cap)
    if band_s is not None:
        metrics["brake_release_band_s"] = float(band_s)
    stop_part = (
        "なし"
        if not layers
        else "／".join(f"{g:g}%:{_f(sec, 1)}s" for g, sec in zip(grid, layers, strict=False))
        + ("" if table.stop_release_accel is not None else "（層は作らず）")
    )
    parts = [
        f"開度の元 {src if src else '-（古い pkl は実開度）'}",
        f"停止側の戻しの層 {'作る' if layer_on else '作らない'}設定（{stop_part}）",
        f"アクセルの戻し量の入力 {'あり' if a_in else 'なし'}設定"
        + "・アクセルの戻し "
        + ("なし" if a_s is None else f"{_f(float(a_s), 1)}s・頭打ち {_f(float(a_cap or 0.0))}%"
           + ("" if table.accel_release_grid is not None else "（表は作らず）")),
        f"アクセルの踏み増し量の入力 {'あり' if p_in else 'なし'}設定"
        + "・アクセルの踏み増し "
        + ("なし" if p_s is None else f"{_f(float(p_s), 1)}s・頭打ち {_f(float(p_cap or 0.0))}%"
           + ("" if table.accel_push_grid is not None else "（表は作らず）")),
        f"ブレーキの戻し {lo:g}〜{hi:g} km/h "
        + ("-" if band_s is None else f"{_f(float(band_s), 1)}s"),
    ]
    return CheckItem(
        "I1", name, STATUS_OK, "・".join(parts), "-", "お知らせ（NG にしない）", metrics
    )


def run_checks(
    *,
    cfg: ResearchConfig,
    csv_path: Path,
    table_c8: ForwardTable | None,
    table_c7: ForwardTable | None = None,
    plan_mode: DrivingMode | None = None,
    ramp_results: Sequence[Any] | None = None,
    pedal_result: Any | None = None,
    saved_path: str | Path | None = None,
    yaml_cfg: ResearchConfig | None = None,
) -> list[CheckItem]:
    """走行が無くても出せる項目を計算し、走行の材料（ramp_results・pedal_result 等）があれば
    C1・B5① も出す。無い材料は SKIP。順に B1 B3 B4 B5 B6 C1 C2 D1 I1（I1 は段4-3 のお知らせ）。

    cfg は「手順3 が使う値」の cfg（惰行・クリープ加速カーブは今回の値。yaml を書く前なら
    新しい値を入れた写しを渡す）。yaml_cfg は C1 用の「yaml を読み直した cfg」（省略時は cfg）。
    plan_mode は WLTP モード（D1 と B3 の車速の上限に使う。None なら D1 は SKIP・B3 は上限なし）。
    """
    from tests.research.pattern_drive import measure_pedal_lags  # noqa: PLC0415

    min_a, min_b = cfg.vehicle.min_accel_opening_pct, cfg.vehicle.min_brake_opening_pct
    ff = cfg.feedforward

    rows: list[FwdRows] | None = None
    if table_c8 is not None:
        try:
            st = settings_of_table(table_c8, DeadbandLag(min_a, min_b))
            rows = read_logs([csv_path], st)
        except (OSError, ValueError, KeyError):
            rows = None

    try:
        lags = measure_pedal_lags(csv_path, accel_min_pct=min_a, brake_min_pct=min_b)
        meas_a, meas_b = lags.accel_s, lags.brake_s
    except (OSError, ValueError):
        meas_a = meas_b = None

    recomputed: tuple[OnsetResult, OnsetResult] | None = None
    if ff.coast_decel_speeds_kmh and ff.coast_decel_kmhs:
        try:
            recomputed = onsets_from_csv(
                csv_path, accel_min=min_a, brake_min=min_b,
                coast_fn=coast_fn_from_curve(ff.coast_decel_speeds_kmh, ff.coast_decel_kmhs),
            )
        except (OSError, ValueError, KeyError):
            recomputed = None
    in_run: tuple[float | None, float | None] | None = None
    if ramp_results:
        last = ramp_results[-1]
        in_run = (last.accel_onset_pct, last.brake_onset_pct)

    return [
        check_fit(table_c8, table_c7, rows),
        check_coast(
            table_c8, ff.coast_decel_speeds_kmh, ff.coast_decel_kmhs, ff.creep_accel_kmhs,
            creep_speed_kmh=ff.creep_speed_kmh,
            mode_max_kmh=None if plan_mode is None else plan_mode.max_speed,
            band_kmhs=cfg.forward_map.coast_band_kmhs,
        ),
        check_lags(table_c8, meas_a, meas_b),
        check_onsets(table_c8, min_a, min_b, in_run, recomputed),
        check_returns(table_c8, csv_path),
        check_measured_in_yaml(yaml_cfg if yaml_cfg is not None else cfg, pedal_result),
        check_model_path(cfg, saved_path),
        check_plan(cfg, table_c8, plan_mode),
        check_table_extras(table_c8, rows),
    ]


def run_checks_for_paths(
    *,
    cfg: ResearchConfig,
    csv_path: Path,
    model_path: str | Path,
    c7_path: str | Path | None = None,
    **kwargs: Any,
) -> list[CheckItem]:
    """pkl のパスから表を読んで `run_checks` を呼ぶ。C8（model_path）が読めなければ、C2 の NG
    1 項目だけを返す（他の項目は表が無いと出せない）。C7 が読めないときは C7 なしで進める。"""
    try:
        table_c8 = load_forward_map(Path(model_path))
    except Exception as exc:  # noqa: BLE001 - 壊れた pkl は pickle 由来の例外も出る
        return [
            CheckItem(
                "C2", "model_path", STATUS_NG, str(model_path), "-",
                f"読めない: {type(exc).__name__}: {exc}", {"readable": 0.0},
            )
        ]
    table_c7: ForwardTable | None = None
    if c7_path is not None:
        try:
            table_c7 = load_forward_map(Path(c7_path))
        except Exception:  # noqa: BLE001 - C7 が読めなくても C8 の判定は続ける
            table_c7 = None
    return run_checks(cfg=cfg, csv_path=csv_path, table_c8=table_c8, table_c7=table_c7, **kwargs)


def check_fel_coverage(coverage: Any | None) -> CheckItem:
    """FEL の埋まり方（2026-10-19 段2s-2b）。WLTP が通るマスのうち補正のあるマスにいる時間の割合。

    お知らせであって NG にはしない（しきい値なし。足りない場面は次の手順2 で学習用パターンの
    規則の係数を変えて増やす）。補正表が無いとき（学習の走りが止まった・走らなかった）は SKIP。
    reason に足りないマスの上位を入れる。
    """
    name = "FEL の埋まり方"
    if coverage is None:
        return _skip("F1", name, "補正表なし（学習の走りが無い・止まった）")
    parts = []
    metrics: dict[str, float] = {}
    keys = {"全体": "all", "アクセル": "accel", "ブレーキ": "brake",
            "ブレーキ<20 km/h": "brake_lt20", "停車前<20 km/h": "stop_lt20"}
    for nm, tot, cov in coverage.groups:
        r = cov / tot if tot > 0.0 else math.nan
        parts.append(f"{nm} {_f(r * 100, 0)}%")
        metrics[f"covered_{keys.get(nm, nm)}"] = r
    miss = coverage.missing[:3]
    reason = "お知らせ（NG にしない）。足りないマス上位: " + (
        "／".join(f"{m.text}（WLTP {m.wltp_s:.1f}s）" for m in miss) if miss else "なし"
    )
    return CheckItem(
        "F1", name, STATUS_OK, "・".join(parts), f"補正は学習 {coverage.min_cell_s:g}s 以上のマス",
        reason, metrics,
    )


def format_checks(items: Sequence[CheckItem]) -> str:
    """表（id・名前・判定・値・比べた相手・理由）と、数値の差（metrics）の一覧。"""
    head = ("id", "名前", "判定", "値", "比べた相手", "理由")
    body = [(i.id, i.name, i.status, i.value, i.reference, i.reason) for i in items]
    lines = [" | ".join(head), " | ".join("---" for _ in head)]
    lines += [" | ".join(r) for r in body]
    lines.append("")
    lines.append("数値の差（metrics）:")
    for i in items:
        if i.metrics:
            lines.append(
                f"  {i.id}: " + "・".join(f"{k}={_f(v, 4)}" for k, v in i.metrics.items())
            )
    return "\n".join(lines)


# ─────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────


async def _load_plan_mode_db(cfg: ResearchConfig) -> DrivingMode:
    from tests.research.mode_drive import load_mode  # noqa: PLC0415

    return await load_mode(cfg, cfg.modes.wltp_mode_name)


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="手順2 の結果のチェック（値の一覧づくり）")
    ap.add_argument("csv", type=Path, help="手順2 の走行ログ CSV")
    ap.add_argument("--model", type=Path, required=True, help="C8 の pkl（fwd_model_*.pkl）")
    ap.add_argument("--table", type=Path, default=None, help="C7 の pkl（fwd_table_*.pkl）")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH, help="yaml")
    ap.add_argument("--no-db", action="store_true", help="DB を使わない（D1 は SKIP）")
    args = ap.parse_args(argv)
    cfg = load_config(args.config)
    plan_mode: DrivingMode | None = None
    if not args.no_db:
        try:
            plan_mode = asyncio.run(_load_plan_mode_db(cfg))
        except Exception as exc:  # noqa: BLE001 - DB が無くても続ける
            print(f"注: モードを DB から読めない（{type(exc).__name__}）→ D1 は SKIP")
    items = run_checks_for_paths(
        cfg=cfg, csv_path=args.csv, model_path=args.model, c7_path=args.table,
        plan_mode=plan_mode, saved_path=args.model,
    )
    print(f"CSV: {args.csv.name}　C8: {args.model.name}"
          f"{'　C7: ' + args.table.name if args.table else ''}　yaml: {args.config.name}")
    print(format_checks(items))
    return 1 if ng_items(items) else 0


if __name__ == "__main__":
    sys.exit(main())
