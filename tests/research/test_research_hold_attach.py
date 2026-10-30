"""手順2 の 4 点の保持: 1 刻み寄せの動かし方と、PI・保持の開度の自動付与（2026-10-30）。"""

from __future__ import annotations

import csv
from pathlib import Path

import pytest

from tests.research import config as cfgmod
from tests.research import hold_pi as hp
from tests.research import pattern_drive as pdmod
from tests.research import pedal_search as ps
from tests.research import step2_check
from tests.research.fwd_map import load_forward_map, save_table
from tests.research.test_research_fel import _table
from tests.research.test_research_hold_pi import _two_points

# ── 1 刻み寄せ ──────────────────────────────────────────────────────


def test_hold_dead_is_continuous_deadband() -> None:
    assert ps.hold_dead(0.4, 0.5) == 0.0
    assert ps.hold_dead(-0.5, 0.5) == 0.0
    assert ps.hold_dead(0.8, 0.5) == pytest.approx(0.3)
    assert ps.hold_dead(-0.8, 0.5) == pytest.approx(-0.3)
    assert ps.hold_dead(0.2, 0.0) == pytest.approx(0.2)


def test_min_step_moves_one_pulse_toward_error() -> None:
    # 遅すぎ（基準 − 実車速 ＞ 不感帯）→ 浅く（指令 [pulse] が減る）
    assert ps.hold_min_step_delta_pulse(1.0, 0.5, 0.05) == -ps.HOLD_MIN_STEP_PULSE
    # 速すぎ → 深く（増える）
    assert ps.hold_min_step_delta_pulse(-1.0, 0.5, 0.05) == ps.HOLD_MIN_STEP_PULSE
    # 大きさは偏差に依らず最小刻み 1 個
    assert ps.hold_min_step_delta_pulse(50.0, 0.5, 0.05) == -ps.HOLD_MIN_STEP_PULSE
    # 不感帯の中・dt が 0 以下は動かさない
    assert ps.hold_min_step_delta_pulse(0.5, 0.5, 0.05) == 0
    assert ps.hold_min_step_delta_pulse(-0.3, 0.5, 0.05) == 0
    assert ps.hold_min_step_delta_pulse(2.0, 0.5, 0.0) == 0
    # 最小刻みはアクチュエータの 1 pulse
    assert ps.HOLD_MIN_STEP_PULSE == 1


# ── 自動付与 ────────────────────────────────────────────────────────


def _cfg(tmp_path: Path, *, hold_pi_enabled: bool, points_enabled: bool = True):
    path = tmp_path / "cfg.yaml"
    cfgmod.load_config(path).save(
        {
            "pedal_search.hold_points_enabled": points_enabled,
            "forward_map.hold_pi_enabled": hold_pi_enabled,
            "vehicle.min_brake_opening_pct": 3.0,
        }
    )
    return cfgmod.load_config(path)


def _write_low_hold_csv(path: Path, *, with_rows: bool = True) -> None:
    with path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["phase", "elapsed_s", "actual_speed_kmh", "brake_actual_pct"])
        w.writerow(["STOP_HOLD", "0.0", "0.0", "12.0"])
        if with_rows:
            s = _two_points(k=50.0, tau=7.0, lag=0.1555, kp=0.13, ki=0.03)
            for t, v, u in zip(s.t_s, s.speed_kmh, s.cmd_pct, strict=True):
                w.writerow(["LOW_HOLD", f"{t:.3f}", f"{v:.4f}", f"{u:.4f}"])


def _pkl(tmp_path: Path, *, breakaway: float | None = 4.0) -> Path:
    table = _table()
    table.meta["brake_lag_s"] = 0.1555
    if breakaway is not None:
        table.meta["launch_breakaway_u_pct"] = -breakaway
    return save_table(table, tmp_path / "m.pkl")


def _pedal(held: bool = True, n: int = 2) -> ps.PedalSearchResult:
    pts = tuple(
        ps.HoldPointResult(1.0 + i, held, 1.0 + i if held else None, 8.0 if held else None,
                           5.0, 9.0, 8.3)
        for i in range(n)
    )
    return ps.PedalSearchResult(5.0, 10.0, 12.0, brake_min_pct=3.0, hold_points=pts)


def test_attach_hold_data_sets_pi_and_opening(tmp_path: Path) -> None:
    cfg = _cfg(tmp_path, hold_pi_enabled=True)
    csv_path = tmp_path / "step2.csv"
    _write_low_hold_csv(csv_path)
    pkl = _pkl(tmp_path)
    item = pdmod.attach_hold_data(cfg, pkl, csv_path, _pedal())
    assert item is not None and item.status == step2_check.STATUS_OK
    meta = load_forward_map(pkl).meta
    assert meta["hold_pi"]["source"] == "LOW_HOLD"
    assert meta["hold_pi"]["kp_pct_per_kmh"] > 0.0
    ho = meta["hold_opening"]
    assert ho["source_csv"] == "step2.csv"
    assert ho["breakaway_pct"] == pytest.approx(3.0 + 4.0)  # min + |launch_breakaway_u_pct|
    assert ho["min_pct"] == 3.0 and ho["creep_kmh"] == 5.0 and len(ho["points"]) == 2


@pytest.mark.parametrize("hold_pi_enabled", [True, False])
def test_attach_hold_data_failure_is_ng_only_when_hold_pi_enabled(
    tmp_path: Path, hold_pi_enabled: bool
) -> None:
    cfg = _cfg(tmp_path, hold_pi_enabled=hold_pi_enabled)
    csv_path = tmp_path / "step2.csv"
    _write_low_hold_csv(csv_path, with_rows=False)  # LOW_HOLD の行なし → HoldPiError
    pkl = _pkl(tmp_path)
    item = pdmod.attach_hold_data(cfg, pkl, csv_path, _pedal())
    assert item is not None
    want = step2_check.STATUS_NG if hold_pi_enabled else step2_check.STATUS_SKIP
    assert item.status == want
    assert "LOW_HOLD" in item.reason
    meta = load_forward_map(pkl).meta
    assert "hold_pi" not in meta
    assert "hold_opening" in meta  # 付けられた分は残す


def test_attach_hold_data_no_point_held_or_no_points(tmp_path: Path) -> None:
    cfg = _cfg(tmp_path, hold_pi_enabled=True)
    csv_path = tmp_path / "step2.csv"
    _write_low_hold_csv(csv_path)
    pkl = _pkl(tmp_path)
    item = pdmod.attach_hold_data(cfg, pkl, csv_path, _pedal(held=False))
    assert item is not None and item.status == step2_check.STATUS_NG
    assert "1 点も保てません" in item.reason
    item = pdmod.attach_hold_data(cfg, pkl, csv_path, _pedal(n=0))
    assert item is not None and item.status == step2_check.STATUS_NG
    assert "点がありません" in item.reason


def test_attach_hold_data_skips_when_disabled_or_no_pedal(tmp_path: Path) -> None:
    csv_path = tmp_path / "step2.csv"
    _write_low_hold_csv(csv_path)
    pkl = _pkl(tmp_path)
    off = _cfg(tmp_path, hold_pi_enabled=True, points_enabled=False)
    assert pdmod.attach_hold_data(off, pkl, csv_path, _pedal()) is None
    on = _cfg(tmp_path, hold_pi_enabled=True)
    assert pdmod.attach_hold_data(on, pkl, csv_path, None) is None
    assert "hold_pi" not in load_forward_map(pkl).meta


def test_attach_hold_pi_low_hold_needs_brake_lag(tmp_path: Path) -> None:
    csv_path = tmp_path / "step2.csv"
    _write_low_hold_csv(csv_path)
    with pytest.raises(hp.HoldPiError):
        hp.attach_hold_pi_low_hold({}, csv_path, deviation_c=6.0)
