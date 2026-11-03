"""案③（低速で効いている開度を保つ。2026-11-03 ProblemReport_20261103 段1）のテスト。
hold_scene.keep_top_kmh / keep_ff_pct と、mode_drive._ModeRun._keep_step の範囲・起点。"""

from __future__ import annotations

from types import SimpleNamespace
from typing import Any

import numpy as np
import pytest

from tests.research import config as cfgmod
from tests.research import drive_log as dlmod
from tests.research import mode_drive as mdmod
from tests.research.ff_candidate import ForwardMapFeedforward
from tests.research.fwd_plan import FLAG_STOP
from tests.research.hold_scene import HoldScenePI, keep_ff_pct, keep_top_kmh
from tests.research.test_research_mode_drive import _mode

STEP = 0.05
LAG = 0.155
C = 6.79
V_TOP = 2.85
BAND = 0.177


def _meta(used_flags: tuple[bool, ...] = (True, True, True, False)) -> dict:
    speeds = (1.1, 1.4, 2.85, 5.0)
    return {"hold_pi": {"per_step": [
        {"v_mean_kmh": v, "used": u} for v, u in zip(speeds, used_flags, strict=True)
    ]}}


# ── 純関数 ────────────────────────────────────────────────────────────


def test_keep_top_is_max_of_used_steps() -> None:
    assert keep_top_kmh(_meta()) == pytest.approx(2.85)  # used でない 5.0 は数えない
    assert keep_top_kmh(_meta((True, False, False, False))) == pytest.approx(1.1)


def test_keep_top_errors_when_missing() -> None:
    with pytest.raises(ValueError):
        keep_top_kmh({})
    with pytest.raises(ValueError):
        keep_top_kmh(_meta((False, False, False, False)))


def test_keep_ff_formula_and_clip() -> None:
    # 基準の加速度が起点と同じ → u0 のまま
    assert keep_ff_pct(8.0, -0.3, -0.3, C, 7.0, 22.0) == pytest.approx(8.0)
    # 基準の加速度が +c だけ上がる → 1% 浅く（上がるほどブレーキを戻す）
    assert keep_ff_pct(8.0, 0.0, C, C, 7.0, 22.0) == pytest.approx(7.0 + 0.0)
    assert keep_ff_pct(8.5, 0.0, C * 0.5, C, 7.0, 22.0) == pytest.approx(8.0)
    # 下がる → 深く
    assert keep_ff_pct(8.0, 0.0, -C * 0.5, C, 7.0, 22.0) == pytest.approx(8.5)
    # clip
    assert keep_ff_pct(8.0, 0.0, 10 * C, C, 7.0, 22.0) == pytest.approx(7.0)
    assert keep_ff_pct(8.0, 0.0, -100 * C, C, 7.0, 22.0) == pytest.approx(22.0)
    with pytest.raises(ValueError):
        keep_ff_pct(8.0, 0.0, 0.0, 0.0, 7.0, 22.0)


# ── 範囲・起点 ────────────────────────────────────────────────────────


def _run(points: list[tuple[float, float]], *, band: bool = False, stop_rows: tuple[int, ...] = (),
         enabled: bool = True) -> Any:
    run = mdmod._ModeRun.__new__(mdmod._ModeRun)  # noqa: SLF001
    mode = _mode(points)
    run.ref_t = mdmod.ReferenceSpeed(mode)
    pi = HoldScenePI(kp=0.1, ki=0.03, dt_s=STEP, lo_pct=7.0, hi_pct=22.0)
    run.hold = mdmod.HoldSceneSetup(
        pi=pi, lag_s=LAG, slope_max_kmhs=0.177, stop_kmh=0.5, creep_kmh=5.0, intervals=[],
        min_length_s=2.0, dropped_intervals=0, step_s=STEP, points=None, offset_pct=0.0,
        text="", plant_c=C, accel_band_kmhs=BAND,
        keep_enabled=enabled, keep_band=band, keep_window_s=0.3, keep_top_kmh=V_TOP,
    )
    flag = np.zeros(4000, dtype=int)
    for r in stop_rows:
        flag[r] = FLAG_STOP
    ff = ForwardMapFeedforward()
    ff._plan = SimpleNamespace(flag=flag, index_at=lambda t: int(round(t / STEP)))  # noqa: SLF001
    run.ff = ff
    run._keep_anchor = None  # noqa: SLF001
    run._keep_after_stop = False  # noqa: SLF001
    run.keep_cycles = 0
    from collections import deque

    run._keep_brake_win = deque()  # noqa: SLF001
    run._keep_speed_win = deque()  # noqa: SLF001
    return run


def _step(run: Any, k: int, speed: float, *, plan_effort: float = -8.0, scene: int = 0,
          brake: float = 8.0) -> float | None:
    """周期 k の _keep_step を呼び、周期の終わりの実開度の履歴を足す（本体と同じ順）。"""
    t = k * STEP
    out = run._keep_step(t, speed, run.ref_t.at(t), plan_effort, scene)  # noqa: SLF001
    run._keep_brake_win.append((t, brake))  # noqa: SLF001
    return out


FLAT = [(0.0, 1.0), (100.0, 1.0)]


def test_anchor_taken_once_after_history_covers_window() -> None:
    run = _run(FLAT)
    outs = [_step(run, k, 1.0) for k in range(40)]
    first = next(i for i, o in enumerate(outs) if o is not None)
    assert first >= 7  # 窓 0.3s ＋ L 0.155s 分の履歴が貯まるまでは通常の FF（None）
    assert all(o is None for o in outs[:first])
    assert all(o == pytest.approx(8.0) for o in outs[first:])  # u0 ＝ 8.0・a0 ＝ 0・基準の加速度 0
    assert run._keep_anchor == pytest.approx((8.0, 0.0))  # noqa: SLF001
    # 範囲の中で実開度が変わっても起点は取り直さない
    out = _step(run, 40, 1.0, brake=12.0)
    assert out == pytest.approx(8.0)
    assert run._keep_anchor[0] == pytest.approx(8.0)  # noqa: SLF001


def test_anchor_uses_lagged_brake_window_and_speed_slope() -> None:
    run = _run(FLAT)
    k0 = 30
    for k in range(k0):
        t = k * STEP
        _step(run, k, 1.0 + 0.2 * t, brake=8.0 if t <= k0 * STEP - LAG - 0.3 else 9.0)
    # 窓 [t−L−w, t−L] の実開度だけの平均（それより新しい 9.0 は入らない）
    out = _step(run, k0, 1.0 + 0.2 * k0 * STEP, brake=9.0)
    u0, a0 = run._keep_anchor  # noqa: SLF001
    assert 8.0 <= u0 < 9.0
    assert a0 == pytest.approx(0.2)
    assert out is not None


def test_rise_of_reference_accel_makes_brake_shallower() -> None:
    # 基準 1.0 一定 → 2 s 後から +0.5 km/h/s で上がる（ただし帯 0.177 を超えると範囲外）
    run = _run([(0.0, 1.0), (2.0, 1.0), (3.0, 1.1), (100.0, 1.1)])
    for k in range(30):
        _step(run, k, 1.0)
    # t = 1.5 s: L 先の基準の加速度 0 → 8.0。t = 2.0 − L ごろ〜: 上がり 0.1 km/h/s ≤ 帯
    assert _step(run, 30, 1.0) == pytest.approx(8.0)
    out = None
    for k in range(31, 45):
        out = _step(run, k, 1.0)
    assert out == pytest.approx(8.0 - 0.1 / C)  # 傾き 0.1 km/h/s ÷ c だけ浅く


def test_leaves_range_when_accel_exceeds_band_and_drops_anchor() -> None:
    run = _run([(0.0, 1.0), (2.0, 1.0), (3.0, 2.0), (100.0, 2.0)])  # +1 km/h/s ＞ 帯
    for k in range(30):
        _step(run, k, 1.0)
    assert run._keep_anchor is not None  # noqa: SLF001
    outs = [_step(run, k, 1.0) for k in range(30, 50)]
    assert outs[-1] is None
    assert run._keep_anchor is None  # noqa: SLF001


def test_out_of_range_conditions() -> None:
    run = _run(FLAT)
    for k in range(30):
        _step(run, k, 1.0)
    assert _step(run, 30, 1.0) is not None
    # 計画の行がブレーキでない（アクセル・惰行）
    assert _step(run, 31, 1.0, plan_effort=0.0) is None
    assert run._keep_anchor is None  # noqa: SLF001
    # 保持の場面 1・2・3（band off）は外
    for scene in (1, 2, 3):
        run = _run(FLAT)
        for k in range(30):
            _step(run, k, 1.0)
        assert _step(run, 30, 1.0, scene=scene) is None
    # 基準が V_top 以上
    run = _run([(0.0, 3.0), (100.0, 3.0)])
    for k in range(30):
        assert _step(run, k, 3.0) is None


def test_flag_stop_row_is_out_of_range() -> None:
    run = _run(FLAT, stop_rows=(30,))
    for k in range(30):
        _step(run, k, 1.0)
    assert _step(run, 29, 1.0) is not None
    assert _step(run, 30, 1.0) is None  # 停車保持の行
    assert run._keep_anchor is None  # noqa: SLF001
    # 停車保持の後（発進の準備）は基準が V_top を超えるまで入らない
    for k in range(31, 50):
        assert _step(run, k, 1.0, brake=9.0) is None
    assert run._keep_anchor is None  # noqa: SLF001


def test_reenters_after_reference_exceeds_v_top_following_stop() -> None:
    # 停車保持（行 30）→ 基準が 3.0 へ上がり（V_top 2.85 超）→ 1.0 へ戻る停車の手前で入り直す
    run = _run([(0.0, 1.0), (2.0, 1.0), (3.0, 3.0), (4.0, 3.0), (5.0, 1.0), (100.0, 1.0)],
               stop_rows=(30,))
    for k in range(31):
        _step(run, k, 1.0)
    assert run._keep_after_stop  # noqa: SLF001
    for k in range(31, 100):
        _step(run, k, 1.0, brake=9.0)
    assert not run._keep_after_stop  # noqa: SLF001
    out = None
    for k in range(100, 160):
        out = _step(run, k, 1.0, brake=9.0)
    assert out is not None and run._keep_anchor[0] == pytest.approx(9.0)  # noqa: SLF001


def test_band_scene_3_only_with_band_and_anchors_after_moving() -> None:
    run = _run(FLAT, band=True)
    for k in range(30):
        assert _step(run, k, 0.0, scene=3) is None  # 車が止まっている間は帯の FF
    assert run._keep_anchor is None  # noqa: SLF001
    out = None
    for k in range(30, 40):
        out = _step(run, k, 1.0, scene=3)
    assert out is not None and run._keep_anchor is not None  # noqa: SLF001
    # 帯の中は基準が上がっても（加速度 ＞ 帯）外さない
    run3 = _run([(0.0, 1.0), (1.0, 1.0), (3.0, 2.5), (100.0, 2.5)], band=True)
    outs = [_step(run3, k, 1.0, scene=3) for k in range(50)]
    assert outs[-1] is not None
    # scene 1・2 は band でも不可
    for scene in (1, 2):
        run2 = _run(FLAT, band=True)
        for k in range(30):
            _step(run2, k, 1.0)
        assert _step(run2, 30, 1.0, scene=scene) is None


def test_off_returns_none_and_touches_nothing() -> None:
    run = _run(FLAT, enabled=False)
    for k in range(40):
        assert _step(run, k, 1.0) is None
    assert run._keep_anchor is None  # noqa: SLF001
    assert not run._keep_speed_win  # noqa: SLF001


# ── 設定 ──────────────────────────────────────────────────────────────


def test_config_defaults_and_validation() -> None:
    fm = cfgmod.ForwardMapSection()
    assert fm.low_speed_keep_enabled is False
    assert fm.low_speed_keep_band is False
    assert fm.low_speed_keep_window_s == pytest.approx(0.3)
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.forward_map.low_speed_keep_enabled = True
    cfg.forward_map.hold_pi_enabled = False
    cfg.forward_map.hold_lead_in_enabled = False
    assert any("low_speed_keep_enabled" in e for e in cfgmod.validate_config(cfg))
    cfg.forward_map.hold_pi_enabled = True
    assert not any("low_speed_keep" in e for e in cfgmod.validate_config(cfg))
    cfg.forward_map.low_speed_keep_window_s = 0.0
    assert any("low_speed_keep_window_s" in e for e in cfgmod.validate_config(cfg))
    cfg.forward_map.low_speed_keep_window_s = 0.3
    cfg.forward_map.low_speed_keep_enabled = False
    cfg.forward_map.low_speed_keep_band = True
    assert any("low_speed_keep_band" in e for e in cfgmod.validate_config(cfg))


def test_log_columns_exist() -> None:
    for col in ("keep_active", "keep_anchor_pct", "keep_anchor_accel_kmhs", "keep_ff_pct"):
        assert col in dlmod.CSV_COLUMNS
