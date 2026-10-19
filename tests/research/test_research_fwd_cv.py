"""fwd_cv（交差検証の枠）のテスト。実ログには依存しない。"""

from __future__ import annotations

import dataclasses

import numpy as np
import torch

from tests.research.fwd_cv import (
    GRID_LOG,
    STAGES,
    WLTP_LOGS,
    CvConfig,
    fit_one_info,
    folds,
    make_windows,
    side_rows,
    side_weights,
    stop_approach_mask,
)
from tests.research.fwd_data import FwdRows
from tests.research.fwd_plan import simulate_speed
from tests.research.fwd_tool import DeadbandLag, Trace
from tests.research.fwd_torch import rollout_speed


def test_folds_hold_out_each_wltp_once_and_keep_grid_log():
    fs = folds()
    assert len(fs) == 3
    assert sorted(h for _, h in fs) == sorted(WLTP_LOGS)
    for train, held in fs:
        assert GRID_LOG in train
        assert held not in train
        assert len(train) == 3


def _rows() -> FwdRows:
    u = np.array([-2.0, 0.0, 0.0, 3.0, -1.0])
    n = len(u)
    return FwdRows(
        speed_kmh=np.arange(n, dtype=float) + 10,
        u_pct=u,
        accel_kmhs=np.zeros(n),
        t_s=np.arange(n) * 0.05,
        dt_s=0.05,
        source="x",
        dt_row_s=np.full(n, 0.05),
    )


def test_side_rows_coast_in_brake_flag():
    r = [_rows()]
    x, _, g, t = side_rows(r, "brake", coast_in_brake=True)
    assert len(x) == 4 and (x[:, 1] == 0).sum() == 2
    x, _, g, t = side_rows(r, "brake", coast_in_brake=False)
    assert len(x) == 2 and not (x[:, 1] == 0).any()
    assert len(g) == len(t) == 2
    x, _, _, _ = side_rows(r, "accel", coast_in_brake=False)
    assert len(x) == 3  # 惰行 2 行 + u = 3


def test_config_key_changes_per_field():
    base = CvConfig("a", (10,))
    keys = {base.key()}
    for change in (
        {"name": "b"},
        {"hidden": (30, 30)},
        {"activation": "tanh"},
        {"loss": "absolute"},
        {"coast_from": "accel"},
        {"val_split": "block"},
        {"seeds": (3, 4, 5)},
        {"alpha": 1e-3},
    ):
        keys.add(dataclasses.replace(base, **change).key())
    assert len(keys) == 9


def test_time_weight_key_compat():
    base = CvConfig("a", (50, 50))
    tw = dataclasses.replace(base, time_weight=True)
    assert "time_weight" not in base.key()
    assert tw.key() == base.key() + "|time_weight=True"


def test_stage32_has_12_and_shares_two_keys_with_stage3():
    assert len(STAGES[32]) == 12
    k3 = {c.key() for c in STAGES[3]}
    assert sum(c.key() in k3 for c in STAGES[32]) == 2
    assert len({c.key() for c in STAGES[32]}) == 12


def test_side_weights_ratio_and_mean():
    a = dataclasses.replace(_rows(), dt_s=0.1)
    b = _rows()  # dt_s = 0.05
    rl = [a, b]
    _, _, g, _ = side_rows(rl, "accel", coast_in_brake=True)
    w = side_weights(g, rl)
    assert np.isclose(w.mean(), 1.0)
    assert np.isclose(w[g == 0][0] / w[g == 1][0], 2.0)


def test_drop_stop_approach_key_compat():
    base = CvConfig("a", (50, 50))
    dr = dataclasses.replace(base, drop_stop_approach=True)
    assert "drop_stop_approach" not in base.key()
    assert dr.key() != base.key()
    assert dr.key() == base.key() + "|drop_stop_approach=True"
    both = dataclasses.replace(dr, time_weight=True)
    assert both.key() == base.key() + "|time_weight=True|drop_stop_approach=True"


def test_stop_approach_mask_boundaries():
    v = np.array([4.9, 5.0, 4.9, 4.9, 0.0, 30.0])
    u = np.array([-8.0, -8.0, -7.9, -20.0, -8.0, -20.0])
    assert stop_approach_mask(v, u).tolist() == [True, False, False, True, True, False]


def test_rollout_s_key_compat():
    base = CvConfig("a", (50, 50))
    ro = dataclasses.replace(base, rollout_s=10.0)
    assert "rollout_s" not in base.key()
    assert ro.key() == base.key() + "|rollout_s=10.0"
    both = dataclasses.replace(base, drop_stop_approach=True, rollout_s=10.0)
    assert both.key() == base.key() + "|drop_stop_approach=True|rollout_s=10.0"


def test_stage4_has_six_configs():
    cfgs = STAGES[4]
    assert len(cfgs) == 6
    assert sorted({c.rollout_s for c in cfgs}) == [0.0, 10.0, 30.0]
    assert all(c.drop_stop_approach and c.hidden == (50, 50) for c in cfgs)
    assert "4 2層50個 relu 外す" in {c.name for c in cfgs}
    assert "4 2層50個 relu 外す 積み上げ10秒" in {c.name for c in cfgs}


ST = DeadbandLag(4.0, 7.0, accel_lag_s=0.05, brake_lag_s=0.1)


def _trace(n: int, dt: float, *, v=None, u=None, t0: float = 0.0) -> Trace:
    t = t0 + np.arange(n) * dt
    v = np.full(n, 20.0) if v is None else v
    u = np.zeros(n) if u is None else u
    return Trace(t, v, u)


def test_windows_count_tail_alignment_and_short_segment_dropped():
    # 区間 25 点（0.1 秒刻み）→ 窓 1.0 秒 = 11 点: 頭から 2 つ（0〜10・11〜21）
    # ＋末尾そろえ（14〜24）。
    # 間の 1 点の停車（v=0）で切れて、後ろの 5 点の区間は短いので捨てる
    v = np.concatenate([np.full(25, 20.0), [0.0], np.full(5, 20.0)])
    tr = _trace(31, 0.1, v=v)
    groups, used, dropped = make_windows([tr], ST, 1.0)
    assert len(groups) == 1 and len(groups[0]) == 3
    assert groups[0].v.shape == (3, 11)
    assert np.isclose(used, 2.4) and np.isclose(dropped, 0.4)
    # 末尾そろえの窓は区間の最後の 11 点（添字 14〜24）
    assert np.allclose(groups[0].dt[:, 1:], 0.1) and np.all(groups[0].dt[:, 0] == 0)


def test_windows_exact_multiple_has_no_tail_and_groups_split_by_dt():
    a = _trace(21, 0.1)  # 21 点 = 11 点の窓 + 端数
    b = _trace(41, 0.05)  # 0.05 秒刻みなら n = 21 点
    groups, _, _ = make_windows([a, b], ST, 1.0)
    assert sorted(g.v.shape[1] for g in groups) == [11, 21]
    ga = next(g for g in groups if g.v.shape[1] == 11)
    assert len(ga) == 2  # 0〜10 と 10〜20（21 点 = 11 + 10 点: 端数の窓が重なって 2 つ目）


def test_windows_cut_at_stop_approach():
    v = np.full(40, 20.0)
    u = np.zeros(40)
    v[20], u[20] = 3.0, -9.0  # 停車まぎわの強いブレーキの行
    tr = _trace(40, 0.1, v=v, u=u)
    g_all, _, _ = make_windows([tr], ST, 0.5)
    g_cut, used, dropped = make_windows([tr], ST, 0.5, drop_mask_fn=stop_approach_mask)
    cut = np.concatenate([g.v for g in g_cut])
    assert not np.any(cut == 3.0)  # 外した行は窓に入らない
    assert any(np.any(g.v == 3.0) for g in g_all)
    assert np.isclose(used, 1.9 + 1.8) and dropped == 0.0


class _Plant:
    """解析的な加速度を持つ簡単な表の代わり（simulate_speed は table.accel(v, u) だけ使う）。"""

    @staticmethod
    def accel(v, u):
        return np.where(u >= 0, 0.5 * u, 0.9 * u) - 0.03 * v - 0.2 * (u == 0)


def _plant_fn(v, u):
    return torch.where(u >= 0, 0.5 * u, 0.9 * u) - 0.03 * v - 0.2 * (u == 0).to(v.dtype)


def test_rollout_speed_matches_simulate_speed():
    for dt in (0.05, 0.1):
        n = 120
        t = np.arange(n) * dt
        u = 8.0 * np.sin(t * 1.3) + np.where(t > 3.0, -2.0, 0.0)  # アクセル → ブレーキをまたぐ
        u[20:30] = 0.0  # 惰行
        v = np.abs(np.cumsum(np.sin(t))) * 0.1 + 10.0 + np.arange(n) * 1e-3
        tr = Trace(t, v, u)
        groups, _, _ = make_windows([tr], ST, 2.0)
        assert len(groups[0]) >= 3
        for k in range(len(groups[0])):
            g = groups[0]
            vw, uw, dw, ix = (
                torch.as_tensor(x[k : k + 1], dtype=torch.float64)
                if i < 3
                else torch.as_tensor(x[k : k + 1])
                for i, x in enumerate((g.v, g.u_lag, g.dt, g.v_idx))
            )
            got = rollout_speed(_plant_fn, vw[:, 0], uw, dw, ix)[0].numpy()
            a = int(np.flatnonzero(v == g.v[k, 0])[0])  # 窓の頭（v はすべて異なる値）
            want = simulate_speed(
                _Plant(), t, u, a, a + len(got), float(v[a]), ST.accel_lag_s, ST.brake_lag_s
            )
            assert np.allclose(got, want, atol=1e-6)


def test_fit_one_info_without_rollout_returns_none_info():
    rng = np.random.default_rng(0)
    n = 60
    u = rng.uniform(-5, 5, n)
    rows = FwdRows(
        speed_kmh=rng.uniform(5, 60, n),
        u_pct=u,
        accel_kmhs=0.3 * u,
        t_s=np.arange(n) * 0.1,
        dt_s=0.1,
        source="x",
    )
    fm, info = fit_one_info(CvConfig("t", (3,)), [rows], 0, max_iter=2)
    assert info is None and fm.accel_side is not None


def test_side_rows_both_sides_have_own_release_column():
    """2026-10-14 段2o-5: ブレーキ側の入力は (車速, u, ブレーキの戻し量) の 3 列。
    2026-10-20 段4-3: アクセル側も (車速, u, アクセルの戻し量) の 3 列。"""
    base = _rows()
    r = FwdRows(
        **{
            **base.__dict__,
            "release_pct": np.array([0.0, 0.0, 0.0, 0.0, 1.5]),
            "accel_release_pct": np.array([0.0, 0.7, 0.0, 0.0, 0.0]),
        }
    )
    xb, _, _, _ = side_rows([r], "brake", coast_in_brake=True)
    xa, _, _, _ = side_rows([r], "accel", coast_in_brake=True)
    assert xb.shape[1] == 3 and xa.shape[1] == 3
    assert xb[:, 2].tolist() == [0.0, 0.0, 0.0, 1.5]  # u <= 0 の 4 行
    assert xa[:, 2].tolist() == [0.7, 0.0, 0.0]  # u >= 0 の 3 行（添字 1・2・3）


def test_side_rows_exclude_launch_only_on_brake_side():
    base = _rows()
    r = FwdRows(**{**base.__dict__, "launch_row": np.array([True, False, False, True, True])})
    xb, _, _, _ = side_rows([r], "brake", coast_in_brake=True, exclude_launch=True)
    assert xb[:, 1].tolist() == [0.0, 0.0]  # u<=0 の 4 行から launch の 2 行（添字 0・4）を除く
    xb_all, _, _, _ = side_rows([r], "brake", coast_in_brake=True)
    assert len(xb_all) == 4
    xa, _, _, _ = side_rows([r], "accel", coast_in_brake=True, exclude_launch=True)
    assert len(xa) == 3  # アクセル側は発進の行も残す（u=3 の行は launch_row でも残る）


def test_cvconfig_brake_exclude_launch_default_on_and_key_compatible():
    assert CvConfig("t", (3,)).brake_exclude_launch is True
    assert "brake_exclude_launch" not in CvConfig("t", (3,)).key()
    assert CvConfig("t", (3,), brake_exclude_launch=False).key().endswith(
        "|brake_exclude_launch=False"
    )
