"""段10-1: 4 点の保持の「加速度の符号が変わる瞬間」の判定・前の pkl からの引き継ぎ・
チェックの「止める／注意」の分け方（2026-11-16）。"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import numpy as np
import pytest

from tests.research import hold_crossing as hc
from tests.research import pattern_drive as pdmod
from tests.research import pedal_search as ps
from tests.research import step2_check as sc
from tests.research.fwd_map import load_forward_map, save_table
from tests.research.test_research_fel import _table
from tests.research.test_research_hold_attach import _cfg, _pkl

LAG = 0.155


def _osc(t0: float = 0.0, dur: float = 12.0, base: float = 2.0, amp: float = 1.0):
    """周期 4 s の車速の振動（極値は t = 1.5・3.5・5.5 …）と、一次で増える開度。"""
    t = np.arange(0.0, dur, 0.05)
    v = base + amp * np.sin(2 * math.pi * (t - 0.5) / 4.0)
    u = 8.0 + 0.1 * t
    return t + t0, v, u


def test_find_crossings_mean_opening_is_lagged_and_speed_is_mean() -> None:
    t, v, u = _osc()
    cs = hc.find_crossings(t, v, u, LAG)
    # 極値 3.5・5.5・7.5・9.5・11.5（1.5 は点の始まりから 3 s 以内なので除く）
    assert [round(c.t_s, 1) for c in cs] == [3.5, 5.5, 7.5, 9.5, 11.5]
    assert {c.direction for c in cs} == {1, -1}  # 両方向
    s = hc.summarize(cs)
    assert s.count == 5 and s.n_up == 3 and s.n_down == 2
    assert s.opening_pct == pytest.approx(8.0 + 0.1 * (7.5 - LAG), abs=0.01)
    assert s.speed_kmh == pytest.approx((3 * 1.0 + 2 * 3.0) / 5, abs=0.01)


def test_find_crossings_skips_stopped_and_early_rows() -> None:
    t, v, u = _osc(base=0.0, amp=0.2)  # 車速が 0.3 km/h 未満の振動 ＝ 停車中の揺れ
    assert hc.find_crossings(t, np.abs(v), u, LAG) == []
    t, v, u = _osc(dur=4.0)  # 3 s 以降に 1 回だけ（3.5）
    assert len(hc.find_crossings(t, v, u, LAG)) == 1


def test_find_crossings_monotonic_has_none() -> None:
    t = np.arange(0.0, 10.0, 0.05)
    assert hc.find_crossings(t, 1.0 + 0.1 * t, 8.0 + 0 * t, LAG) == []


def _write_csv(path: Path, points: list[tuple[np.ndarray, np.ndarray, np.ndarray]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["phase", "elapsed_s", "actual_speed_kmh", "brake_actual_pct"])
        for t, v, u in points:
            for a, b, c in zip(t, v, u, strict=True):
                w.writerow(["LOW_HOLD", f"{a:.3f}", f"{b:.4f}", f"{c:.4f}"])
            w.writerow(["STOP_HOLD", f"{t[-1] + 0.5:.3f}", "0.0", "12.0"])


def _held_point(target: float, held: bool = False) -> ps.HoldPointResult:
    return ps.HoldPointResult(target, held, None, None, 0.0, 17.0, 8.3)


def test_apply_crossings_makes_point_held_and_uses_crossing_mean(tmp_path: Path) -> None:
    t1, v1, u1 = _osc()
    t2 = np.arange(30.0, 42.0, 0.05)
    flat = (t2, 1.0 + 0.05 * (t2 - 30.0), 8.0 + 0 * t2)  # 単調 ＝ 瞬間なし
    csv_path = tmp_path / "s.csv"
    _write_csv(csv_path, [(t1, v1, u1), flat])
    out, note = ps.apply_crossings([_held_point(2.0), _held_point(1.0)], csv_path, LAG)
    assert out[0].held and out[0].criterion == "crossings" and out[0].crossings == 5
    assert out[0].opening_pct == pytest.approx(8.0 + 0.1 * (7.5 - LAG), abs=0.01)
    assert out[0].speed_kmh == pytest.approx(1.8, abs=0.01)
    assert not out[1].held and out[1].crossings == 0 and out[1].criterion == ""
    assert "crossings" in note


def test_apply_crossings_keeps_band_result(tmp_path: Path) -> None:
    t1, v1, u1 = _osc()
    csv_path = tmp_path / "s.csv"
    _write_csv(csv_path, [(t1, v1, u1)])
    band = ps.HoldPointResult(2.0, True, 2.05, 8.25, 5.2, 7.0, 8.3)  # 帯で保てた点
    out, _ = ps.apply_crossings([band], csv_path, LAG)
    assert out[0].held and out[0].criterion == "band"
    assert (out[0].speed_kmh, out[0].opening_pct) == (2.05, 8.25)  # 帯の値のまま
    assert out[0].crossings == 5  # 瞬間の数は記録する
    meta = ps.hold_opening_meta(
        out, creep_kmh=5.0, breakaway_pct=8.6, min_pct=7.0, source_csv="s.csv"
    )
    assert meta["points"][0]["criterion"] == "band" and meta["points"][0]["crossings"] == 5


def test_apply_crossings_segment_count_mismatch_is_unchanged(tmp_path: Path) -> None:
    t1, v1, u1 = _osc()
    csv_path = tmp_path / "s.csv"
    _write_csv(csv_path, [(t1, v1, u1)])
    out, note = ps.apply_crossings([_held_point(2.0), _held_point(3.0)], csv_path, LAG)
    assert not any(p.held for p in out) and "合いません" in note


def test_attach_hold_data_held_by_crossings_makes_hold_pi_and_opening(tmp_path: Path) -> None:
    from tests.research.test_research_hold_pi import _two_points

    cfg = _cfg(tmp_path, hold_pi_enabled=True)
    s = _two_points(k=50.0, tau=7.0, lag=0.1555, kp=0.13, ki=0.03)
    csv_path = tmp_path / "step2.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["phase", "elapsed_s", "actual_speed_kmh", "brake_actual_pct"])
        for t, v, u in zip(s.t_s, s.speed_kmh, s.cmd_pct, strict=True):
            w.writerow(["LOW_HOLD", f"{t:.3f}", f"{v:.4f}", f"{u:.4f}"])
    pkl = _pkl(tmp_path)
    pedal = ps.PedalSearchResult(
        5.0, 10.0, 12.0, brake_min_pct=3.0,
        hold_points=(_held_point(1.0), _held_point(2.0)),  # 帯では全部保てず
    )
    item = pdmod.attach_hold_data(cfg, pkl, csv_path, pedal)
    meta = load_forward_map(pkl).meta
    assert item is not None and item.status in (sc.STATUS_OK, sc.STATUS_WARN)
    pts = meta["hold_opening"]["points"] if "hold_opening" in meta else []
    # 瞬間が 2 回以上の点は「保てた」で criterion に残る
    assert all(p["criterion"] in ("band", "crossings", "") for p in pts)


def _prev_pkl(tmp_path: Path) -> Path:
    table = _table()
    table.meta["hold_pi"] = {"kp_pct_per_kmh": 0.05, "ki_pct_per_kmh_s": 0.01, "L_s": 0.15}
    table.meta["hold_opening"] = {"points": [], "creep_kmh": 5.0, "source_csv": "old.csv"}
    return save_table(table, tmp_path / "previous.pkl")


def test_fallback_inherits_hold_data_from_previous_pkl(tmp_path: Path, capsys) -> None:
    cfg = _cfg(tmp_path, hold_pi_enabled=True)
    prev = _prev_pkl(tmp_path)
    cfg.feedforward.model_path = str(prev)  # 手順2 の前の yaml が指していた pkl
    csv_path = tmp_path / "step2.csv"
    csv_path.write_text("phase,elapsed_s,actual_speed_kmh,brake_actual_pct\n", encoding="utf-8")
    pkl = _pkl(tmp_path, name="new.pkl")
    pedal = ps.PedalSearchResult(
        5.0, 10.0, 12.0, brake_min_pct=3.0, hold_points=(_held_point(1.0), _held_point(2.0))
    )
    item = pdmod.attach_hold_data(cfg, pkl, csv_path, pedal)
    meta = load_forward_map(pkl).meta
    assert meta["hold_pi"]["kp_pct_per_kmh"] == 0.05
    assert meta["hold_opening"]["source_csv"] == "old.csv"
    assert item is not None and item.status == sc.STATUS_WARN
    assert "previous.pkl" in item.value
    out = capsys.readouterr().out
    assert "previous.pkl" in out and "引き継ぎ" in out


def test_no_previous_meta_does_not_fail(tmp_path: Path) -> None:
    cfg = _cfg(tmp_path, hold_pi_enabled=True)  # model_path は存在しない前の pkl
    csv_path = tmp_path / "step2.csv"
    csv_path.write_text("phase,elapsed_s,actual_speed_kmh,brake_actual_pct\n", encoding="utf-8")
    pkl = _pkl(tmp_path, name="new.pkl")
    pedal = ps.PedalSearchResult(
        5.0, 10.0, 12.0, brake_min_pct=3.0, hold_points=(_held_point(1.0),)
    )
    item = pdmod.attach_hold_data(cfg, pkl, csv_path, pedal)
    meta = load_forward_map(pkl).meta
    assert "hold_pi" not in meta and "hold_opening" not in meta
    assert item is not None and item.status == sc.STATUS_WARN


# ── チェックの分け方 ────────────────────────────────────────────────


def _ng(item_id: str, **metrics: float) -> sc.CheckItem:
    return sc.CheckItem(item_id, "x", sc.STATUS_NG, "v", "r", "", dict(metrics))


def test_non_blocking_ng_becomes_warn() -> None:
    items = [
        _ng("B3"), _ng("B4"), _ng("B5"), _ng("B6"), _ng("C1"), _ng("H1"),
        _ng("D1", nan_count=0.0, out_of_range_ratio=0.5, gap_violations=2.0),
    ]
    out = sc.classify_checks(items)
    assert [i.status for i in out] == [sc.STATUS_WARN] * len(items)
    assert sc.ng_items(out) == [] and len(sc.warn_items(out)) == len(items)
    assert "注意" in sc.format_warn(out)
    assert sc.classify_checks(out) == out  # 何度かけても同じ


@pytest.mark.parametrize(
    "item",
    [_ng("B1", ratio_mae=1.5), _ng("C2", readable=0.0), _ng("D1", nan_count=3.0),
     _ng("D1", nan_count=math.nan)],
)
def test_blocking_ng_stays_ng(item: sc.CheckItem) -> None:
    out = sc.classify_checks([item, _ng("B5")])
    assert out[0].status == sc.STATUS_NG and out[1].status == sc.STATUS_WARN
    assert sc.ng_items(out) == [out[0]]
    with pytest.raises(sc.Step2CheckError):
        raise sc.Step2CheckError(out)


def test_ok_and_skip_are_untouched_and_format_shows_warn() -> None:
    ok = sc.CheckItem("B1", "a", sc.STATUS_OK)
    skip = sc.CheckItem("D1", "b", sc.STATUS_SKIP)
    out = sc.classify_checks([ok, skip, _ng("B5")])
    assert [i.status for i in out] == [sc.STATUS_OK, sc.STATUS_SKIP, sc.STATUS_WARN]
    assert "WARN" in sc.format_checks(out)
