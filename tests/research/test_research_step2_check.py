"""step2_check（手順2 の結果のチェック関数）のテスト。小さな合成データだけを使う。"""

from __future__ import annotations

import math
from datetime import UTC, datetime
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from tests.research import step2_check as sc
from tests.research.config import ResearchConfig
from tests.research.fwd_map import POINT_MEASURED, ForwardTable, save_table
from tests.research.play_onset import OnsetResult
from tests.research.research_types import DrivingMode, SpeedPoint


def _table(**meta: float) -> ForwardTable:
    """a = −2 − 0.01v + (u≥0: 0.3u / u<0: 0.6u)。u の格子は −30..30（刻み 1）。"""
    rep = np.array([0.0, 50.0, 100.0, 150.0])
    grid = np.arange(-30.0, 31.0, 1.0)
    base = -2.0 - 0.01 * rep[:, None]
    accel = base + np.where(grid >= 0, 0.3 * grid, 0.6 * grid)[None, :]
    return ForwardTable(
        speed_edges=np.array([-25.0, 25.0, 75.0, 125.0, 175.0]),
        speed_rep=rep,
        u_grid=grid,
        accel_grid=accel,
        seconds=np.full(accel.shape, 10.0),
        point_kind=np.full(accel.shape, POINT_MEASURED, dtype=int),
        meas_u_min=np.full(4, -30.0),
        meas_u_max=np.full(4, 30.0),
        meta=dict(meta),
    )


def _onset(pct: float | None) -> OnsetResult:
    return OnsetResult(pct, 0.3 if pct is not None else None, (), "" if pct is not None else "なし")


def test_judge_thresholds_and_limits(monkeypatch) -> None:
    assert sc._judge("B5", {"accel_in_run_diff_pct": 0.5}) == sc.STATUS_OK  # 境目ちょうどは OK
    assert sc._judge("B5", {"accel_in_run_diff_pct": 0.51}) == sc.STATUS_NG
    assert sc._judge("B5", {"accel_in_run_diff_pct": math.nan}) == sc.STATUS_NG
    assert sc._judge("B5", {"accel_in_run_diff_pct": math.inf}) == sc.STATUS_NG
    # None のしきい値は判定しない。limits は THRESHOLDS より優先する
    assert sc._judge("B3", {"coast_diff_max": 99.0}) == sc.STATUS_OK
    assert sc._judge("B3", {"coast_diff_max": 0.25}, {"coast_diff_max": 0.25}) == sc.STATUS_OK
    assert sc._judge("B3", {"coast_diff_max": 0.26}, {"coast_diff_max": 0.25}) == sc.STATUS_NG


def _shifted(delta: float) -> ForwardTable:
    t = _table()
    t.accel_grid = t.accel_grid + delta
    return t


def _fit_rows():  # type: ignore[no-untyped-def]
    from tests.research.fwd_data import FwdRows

    t = np.arange(0.0, 20.0, 0.1)
    v = np.full_like(t, 50.0)
    u = np.full_like(t, 4.0)
    truth = -2.0 - 0.5 + 0.3 * 4.0
    return FwdRows(v, u, np.full_like(t, truth + 0.1), t, 0.1, "x")


def test_b1_skip_and_ratio() -> None:
    assert sc.check_fit(None, None, []).status == sc.STATUS_SKIP
    rows = _fit_rows()
    item = sc.check_fit(_table(), _table(), [rows])
    assert item.metrics["c8_mae"] == pytest.approx(0.1, abs=1e-6)
    assert item.metrics["ratio_mae"] == pytest.approx(1.0)
    assert item.status == sc.STATUS_OK  # 比 1.0 ちょうどは OK（> 1.0 で NG）
    only8 = sc.check_fit(_table(), None, [rows])
    assert "ratio_mae" not in only8.metrics and only8.status == sc.STATUS_OK
    good = sc.check_fit(_shifted(0.1), _table(), [rows])  # NN の方が当てはまる
    assert good.metrics["ratio_mae"] < 1.0 and good.status == sc.STATUS_OK
    bad = sc.check_fit(_shifted(-0.1), _table(), [rows])  # NN の方が悪い
    assert bad.metrics["ratio_mae"] == pytest.approx(2.0)
    assert bad.status == sc.STATUS_NG


def test_seed_outliers_boundary_and_b1_ng() -> None:
    base = {0: 1.0, 1: 1.0, 2: 1.0, 3: 1.0}
    assert sc.seed_outliers({**base, 4: 1.2}) == []  # 残りの中央値 × 1.2 ちょうどは外れでない
    assert sc.seed_outliers({**base, 4: 1.21}) == [4]
    assert sc.seed_outliers({0: 5.0}) == []  # 比べる相手がない
    assert sc.SEED_OUTLIER_RATIO == 1.2
    rows = _fit_rows()
    ok = sc.check_fit(_table(seed_mae={**base, 4: 1.2}), _table(), [rows])
    assert ok.metrics["seed_outlier_count"] == 0.0 and ok.status == sc.STATUS_OK
    ng = sc.check_fit(_table(seed_mae={**base, 4: 1.21}), _table(), [rows])
    assert ng.metrics["seed_outlier_count"] == 1.0 and ng.status == sc.STATUS_NG
    assert "乱数 [4]" in ng.reason


# B3 の呼び出しの共通部分: クリープ車速 5 km/h → 比べる下限 10 km/h、モードの最高 100 km/h
_B3 = {"creep_speed_kmh": 5.0, "mode_max_kmh": 100.0, "band_kmhs": 0.25}


def test_b3_coast_only_between_creep_plus5_and_mode_max() -> None:
    t = _table()
    v = [0.0, 8.0, 50.0, 100.0, 135.0]
    # 表の coast_accel は −2.0・−2.08・−2.5・−3.0・−3.35。範囲外の 8 と 135 は大きくずらす
    decel = [9.0, 2.08 + 5.0, 2.5 + 0.2, 3.0, 3.35 + 5.0]
    item = sc.check_coast(t, v, decel, [], **_B3)
    assert item.metrics["coast_n"] == 2.0  # 50 と 100
    assert item.metrics["coast_diff_max"] == pytest.approx(0.2)
    assert item.status == sc.STATUS_OK
    decel[2] = 2.5 + 0.3
    assert sc.check_coast(t, v, decel, [], **_B3).status == sc.STATUS_NG
    # モードの最高車速が無ければ上限なし（135 も比べる）
    nolimit = sc.check_coast(t, v, decel, [], **{**_B3, "mode_max_kmh": None})
    assert nolimit.metrics["coast_n"] == 3.0


def test_b3_creep_needs_launch_table_but_not_value() -> None:
    t = _table()
    v, decel = [50.0], [2.5]
    no_launch = sc.check_coast(t, v, decel, [0.4, 0.2, 0.0], **_B3)
    assert no_launch.metrics["creep_launch_missing"] == 1.0
    assert no_launch.status == sc.STATUS_NG
    t.launch_accel = t.accel_grid.copy()
    with_launch = sc.check_coast(t, v, decel, [0.4, 0.2, 0.0], **_B3)
    assert with_launch.metrics["creep_launch_missing"] == 0.0
    assert with_launch.status == sc.STATUS_OK  # 値の差では判定しない
    # 停止側の表（2026-10-18 段2o-10）は有無を表示するだけ（NG にしない）
    assert "停止側の表なし" in with_launch.value
    t.stop_accel = t.accel_grid.copy()
    with_stop = sc.check_coast(t, v, decel, [0.4, 0.2, 0.0], **_B3)
    assert "停止側の表あり" in with_stop.value and with_stop.status == sc.STATUS_OK
    # クリープ加速度に正が無ければ、発進側が無くても NG にしない
    none_positive = sc.check_coast(_table(), v, decel, [0.0, 0.0], **_B3)
    assert none_positive.metrics["creep_launch_missing"] == 0.0
    assert none_positive.status == sc.STATUS_OK
    assert "creep_diff_max" not in none_positive.metrics


def test_b3_skip_cases() -> None:
    assert sc.check_coast(None, [50.0], [2.5], [], **_B3).status == sc.STATUS_SKIP
    assert sc.check_coast(_table(), [], [], [], **_B3).status == sc.STATUS_SKIP
    # 惰行カーブの点が全部範囲外で、クリープも無ければ比べる点が無い
    assert sc.check_coast(_table(), [0.0, 8.0], [2.0, 2.1], [], **_B3).status == sc.STATUS_SKIP


def test_b4_lags_measured_and_default() -> None:
    t = _table(accel_lag_s=0.23, brake_lag_s=0.05)
    item = sc.check_lags(t, 0.20, None, 0.05, 0.05)
    assert item.metrics["accel_lag_diff_s"] == pytest.approx(0.03)
    assert item.metrics["brake_lag_diff_s"] == pytest.approx(0.0)
    assert item.metrics["accel_lag_measured"] == 1.0
    assert item.metrics["brake_lag_measured"] == 0.0
    assert item.status == sc.STATUS_NG  # 0.03 > 0.005
    assert sc.check_lags(None, None, None).status == sc.STATUS_SKIP
    same = sc.check_lags(_table(accel_lag_s=0.2, brake_lag_s=0.05), 0.2, 0.05)
    assert same.status == sc.STATUS_OK
    just = sc.check_lags(_table(accel_lag_s=0.2049, brake_lag_s=0.05), 0.2, 0.05)
    assert just.status == sc.STATUS_OK  # 0.005 以内
    over = sc.check_lags(_table(accel_lag_s=0.21, brake_lag_s=0.05), 0.2, 0.05)
    assert over.status == sc.STATUS_NG
    nokey = sc.check_lags(_table(), 0.2, 0.05)  # meta にキーが無い = 測れていない
    assert nokey.status == sc.STATUS_NG


def test_b6_returns_meta_vs_recomputed(tmp_path: Path) -> None:
    from tests.research.test_research_pedal_lag import _write_return_csv  # noqa: PLC0415

    csv_path = _write_return_csv(tmp_path / "r.csv", [5, 10, 15, 25, 40, 20], 0.10, 100.0)
    from tests.research.pedal_lag import estimate_pedal_return  # noqa: PLC0415

    a = estimate_pedal_return(csv_path).accel
    meta = {"accel_return_delay_s": a.delay_s, "accel_return_rate_pct_s": a.rate_pct_s}
    ok = sc.check_returns(_table(**meta), csv_path)
    assert ok.status == sc.STATUS_OK and ok.id == "B6"
    assert abs(ok.metrics["accel_delay_diff_s"]) < 1e-9
    assert "brake 省略" in ok.reason  # ブレーキは測れない = 省略（理由つき）
    # meta の値がずれていれば NG
    bad = sc.check_returns(_table(accel_return_delay_s=a.delay_s + 0.01,
                                  accel_return_rate_pct_s=a.rate_pct_s), csv_path)
    assert bad.status == sc.STATUS_NG
    # 測れたのに meta に無ければ NG（無限大）
    nokey = sc.check_returns(_table(), csv_path)
    assert nokey.status == sc.STATUS_NG and math.isinf(nokey.metrics["accel_delay_diff_s"])
    # pkl なし・両方測れないときは SKIP
    assert sc.check_returns(None, csv_path).status == sc.STATUS_SKIP
    empty = tmp_path / "empty.csv"
    empty.write_text("elapsed_s\n", encoding="utf-8")
    assert sc.check_returns(_table(), empty).status == sc.STATUS_SKIP


def test_d1_gap_violations_threshold() -> None:
    assert sc._judge("D1", {"gap_violations": 0.0}) == sc.STATUS_OK
    assert sc._judge("D1", {"gap_violations": 1.0}) == sc.STATUS_NG


def test_b5_meta_540_vs_recomputed_1015_is_ng() -> None:
    t = _table(accel_onset_u_pct=1.20, brake_onset_u_pct=3.0)  # min 4.2 → 絶対 5.40・7.20
    item = sc.check_onsets(t, 4.2, 4.2, recomputed=(_onset(10.15), _onset(7.20)))
    assert item.metrics["accel_recomputed_diff_pct"] == pytest.approx(4.75)
    assert item.metrics["brake_recomputed_diff_pct"] == pytest.approx(0.0, abs=1e-9)
    assert item.status == sc.STATUS_NG
    # 走行中の値 10.05 とも合わない
    item2 = sc.check_onsets(t, 4.2, 4.2, in_run=(10.05, 7.2))
    assert item2.metrics["accel_in_run_diff_pct"] == pytest.approx(4.65)
    assert item2.status == sc.STATUS_NG


def test_b5_boundary_and_ok() -> None:
    t = _table(accel_onset_u_pct=1.5, brake_onset_u_pct=3.0)  # min 4.0 → 絶対 5.5・7.0
    edge = sc.check_onsets(t, 4.0, 4.0, in_run=(5.0, 7.0))  # アクセルの差ちょうど 0.5
    assert edge.metrics["accel_in_run_diff_pct"] == pytest.approx(0.5)
    assert edge.status == sc.STATUS_OK
    over = sc.check_onsets(t, 4.0, 4.0, in_run=(4.9, 7.0))
    assert over.status == sc.STATUS_NG
    ok = sc.check_onsets(t, 4.0, 4.0, in_run=(5.5, 7.0), recomputed=(_onset(5.6), _onset(6.8)))
    assert ok.status == sc.STATUS_OK


def test_b5_missing_key_and_skip() -> None:
    t = _table(accel_onset_u_pct=1.2)  # ブレーキのキーなし
    item = sc.check_onsets(t, 4.2, 4.2, in_run=(5.0, 7.0))
    assert item.metrics["accel_in_run_diff_pct"] == pytest.approx(0.4)
    assert item.metrics["brake_in_run_diff_pct"] == math.inf
    assert item.status == sc.STATUS_NG  # ① が求まるのに meta に無い
    item2 = sc.check_onsets(t, 4.2, 4.2, recomputed=(_onset(None), _onset(6.7)))
    assert item2.metrics["brake_recomputed_diff_pct"] == math.inf  # ② が求まるのに meta に無い
    assert item2.status == sc.STATUS_NG
    assert "accel_recomputed_diff_pct" not in item2.metrics
    assert sc.check_onsets(t, 4.2, 4.2).status == sc.STATUS_SKIP
    assert sc.check_onsets(t, 4.2, 4.2, in_run=(None, None)).status == sc.STATUS_SKIP
    assert sc.check_onsets(None, 4.2, 4.2, in_run=(1.0, 1.0)).status == sc.STATUS_SKIP
    # ① も ② も無いペダルは比べず理由を書く（もう一方が OK なら全体は OK）
    one = sc.check_onsets(
        _table(accel_onset_u_pct=1.0), 4.0, 4.0, in_run=(5.0, None),
        recomputed=(_onset(5.0), _onset(None)),
    )
    assert one.status == sc.STATUS_OK and "brake 計算し直しは求まらず" in one.reason


def test_c1_stop_hold_only() -> None:
    cfg = ResearchConfig()
    assert sc.check_measured_in_yaml(cfg, None).status == sc.STATUS_SKIP
    res = SimpleNamespace(
        stop_brake_opening_pct=cfg.stop_brake_opening_pct + 0.25,
        creep_speed_kmh=cfg.feedforward.creep_speed_kmh - 0.5,
    )
    item = sc.check_measured_in_yaml(cfg, res)
    assert item.metrics == {"stop_brake_diff_pct": pytest.approx(0.25)}  # クリープ車速は見ない
    assert item.status == sc.STATUS_NG
    same = SimpleNamespace(stop_brake_opening_pct=cfg.stop_brake_opening_pct + 0.005)
    assert sc.check_measured_in_yaml(cfg, same).status == sc.STATUS_OK


def test_c2_path_same_different_unreadable(tmp_path: Path, monkeypatch) -> None:
    pkl = save_table(_table(), tmp_path / "t.pkl")
    cfg = ResearchConfig()
    cfg.feedforward.model_path = str(pkl)
    cfg.feedforward.candidate = "C7"
    same = sc.check_model_path(cfg, pkl)
    assert same.status == sc.STATUS_OK and same.metrics["path_diff"] == 0.0
    diff = sc.check_model_path(cfg, tmp_path / "other.pkl")
    assert diff.metrics["path_diff"] == 1.0 and diff.metrics["readable"] == 1.0
    assert diff.status == sc.STATUS_NG
    # 相対パスと絶対パスでも同じファイルなら同じ
    monkeypatch.chdir(tmp_path)
    cfg.feedforward.model_path = "t.pkl"
    rel = sc.check_model_path(cfg, pkl)
    assert rel.metrics["path_diff"] == 0.0 and rel.status == sc.STATUS_OK
    only = sc.check_model_path(cfg, None)
    assert "path_diff" not in only.metrics and only.status == sc.STATUS_OK
    cfg.feedforward.model_path = str(tmp_path / "missing.pkl")
    assert sc.check_model_path(cfg, None).status == sc.STATUS_NG
    broken = tmp_path / "broken.pkl"
    broken.write_bytes(b"x")  # pickle として壊れている
    cfg.feedforward.model_path = str(broken)
    assert sc.check_model_path(cfg, None).status == sc.STATUS_NG


def test_run_checks_for_paths_unreadable_c8_is_single_c2_ng(tmp_path: Path) -> None:
    broken = tmp_path / "broken.pkl"
    broken.write_bytes(b"x")
    items = sc.run_checks_for_paths(
        cfg=ResearchConfig(), csv_path=tmp_path / "none.csv", model_path=broken
    )
    assert [i.id for i in items] == ["C2"] and items[0].status == sc.STATUS_NG


def _mode(points: list[tuple[float, float]]) -> DrivingMode:
    return DrivingMode(
        id="m", name="m", description="",
        reference_speed=[SpeedPoint(time_s=t, speed_kmh=v) for t, v in points],
        total_duration=points[-1][0], max_speed=max(v for _, v in points),
        created_at=datetime(2000, 1, 1, tzinfo=UTC),
    )


def test_d1_thresholds() -> None:
    ok = {"out_of_range_ratio": 0.10, "filled_ratio": 0.32, "nan_count": 0.0,
          "prepos_excess_accel_pct": 0.0, "prepos_excess_brake_pct": 0.0}
    assert sc._judge("D1", ok) == sc.STATUS_OK  # 境目ちょうどは OK
    for key, val in (("out_of_range_ratio", 0.11), ("filled_ratio", 0.33), ("nan_count", 1.0),
                     ("prepos_excess_accel_pct", 0.01), ("prepos_excess_brake_pct", 0.01)):
        assert sc._judge("D1", {**ok, key: val}) == sc.STATUS_NG, key


def test_d1_plan_metrics_and_skip() -> None:
    cfg = ResearchConfig()
    mode = _mode([(0.0, 0.0), (5.0, 0.0), (15.0, 40.0), (25.0, 40.0), (35.0, 0.0), (40.0, 0.0)])
    assert sc.check_plan(cfg, None, mode).status == sc.STATUS_SKIP
    assert sc.check_plan(cfg, _table(), None).status == sc.STATUS_SKIP
    item = sc.check_plan(cfg, _table(accel_onset_u_pct=1.0, brake_onset_u_pct=1.0), mode)
    assert item.id == "D1"
    assert item.metrics["nan_count"] == 0.0
    assert item.metrics["duration_s"] == 40.0
    assert item.metrics["out_of_range_ratio"] == pytest.approx(
        item.metrics["out_of_range_s"] / 40.0
    )
    assert item.metrics["filled_ratio"] == pytest.approx(item.metrics["filled_s"] / 40.0)
    assert "prepos_placed" in item.metrics
    assert item.metrics["gap_violations"] == 0.0 and "gap_switches" in item.metrics  # 段2o-9
    for key in ("prepos_over_accel_pct", "prepos_over_brake_pct"):
        if key in item.metrics:
            assert item.metrics[key] <= 1e-9  # 先に置く開度は効き始め − margin を超えない


def test_format_ng_and_error_lists_ng_items_only() -> None:
    items = [
        sc.CheckItem(
            "B5", "効き始め", sc.STATUS_NG, "accel meta 5.40%", "計算し直し 10.15%", "差 4.75%"
        ),
        sc.CheckItem("B4", "遅れ", sc.STATUS_OK, "v", "r", ""),
        sc.CheckItem("D1", "手順3", sc.STATUS_SKIP, "-", "-", "モード無し"),
    ]
    err = sc.Step2CheckError(items)
    assert "B5" in str(err) and "差 4.75%" in str(err)
    assert "B4" not in str(err) and "D1" not in str(err)
    assert [i.id for i in sc.ng_items(items)] == ["B5"]


def test_format_checks_lists_every_item() -> None:
    items = [
        sc.CheckItem("X1", "テスト", sc.STATUS_OK, "v", "r", "why", {"a": 1.5}),
        sc.CheckItem("X2", "材料なし", sc.STATUS_SKIP, "-", "-", "材料なし"),
    ]
    text = sc.format_checks(items)
    assert "X1" in text and "X2" in text and "SKIP" in text and "a=1.5000" in text


def test_i1_table_extras_is_informational_and_reports_seconds() -> None:
    """2026-10-20 段4-3: 表の作り方のお知らせ。開度の元・停止側の戻しの層の秒数・アクセルの戻しの
    秒数と頭打ち・ブレーキの戻しの 7.5〜20 km/h の秒数。NG にしない。"""
    t = _table(
        side_table_opening_source="cmd",
        stop_release_layer_s=[0.0, 3.5, 0.0, 1.0, 0.0],
        stop_release_grid_pct=[0.5, 1.0, 2.0, 4.0, 8.0],
        accel_release_rows_s=41.5,
        accel_release_max_pct=2.25,
        brake_release_band_s=4.0,
        brake_release_band_lo_kmh=7.5,
        brake_release_band_hi_kmh=20.0,
    )
    item = sc.check_table_extras(t, None)
    assert item.id == "I1" and item.status == sc.STATUS_OK
    assert "cmd" in item.value and "1%:3.5s" in item.value and "4%:1.0s" in item.value
    assert "41.5s" in item.value and "2.25%" in item.value and "7.5〜20 km/h 4.0s" in item.value
    assert item.metrics["stop_release_total_s"] == pytest.approx(4.5)
    assert item.metrics["accel_release_rows_s"] == pytest.approx(41.5)
    assert item.metrics["accel_release_max_pct"] == pytest.approx(2.25)
    assert item.metrics["brake_release_band_s"] == pytest.approx(4.0)
    # 古い pkl（meta にキーが無い）でも落ちず、NG にもならない
    old = sc.check_table_extras(_table(), None)
    assert old.status == sc.STATUS_OK and "なし" in old.value
    assert sc.check_table_extras(None, None).status == sc.STATUS_SKIP
    # 判定の対象に入っていない（しきい値なし）
    assert sc._judge("I1", item.metrics) == sc.STATUS_OK
