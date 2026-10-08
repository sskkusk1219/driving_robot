"""fwd_torch（PyTorch 版の学習）のテスト。合成データだけを使う。"""

from __future__ import annotations

import numpy as np
import torch

from tests.research.fwd_torch import (
    VALIDATION_FRACTION,
    WindowGroup,
    block_ids,
    finetune_rollout,
    fit_side,
    split_indices,
)


def test_block_ids_differ_by_log_and_by_10s():
    t = np.concatenate([np.arange(0, 30, 0.05), np.arange(0, 30, 0.05)])
    g = np.concatenate([np.zeros(600, dtype=int), np.ones(600, dtype=int)])
    b = block_ids(t, g, 10.0)
    assert b[0] == b[199]  # 同じログ・同じ 10 秒
    assert b[0] != b[200]  # 10 秒違い
    assert b[0] != b[600]  # ログ違い（時刻は同じ）
    assert len(np.unique(b)) == 6


def test_block_split_never_straddles_and_is_about_10_percent():
    t = np.arange(0, 300, 0.05)
    g = np.zeros(len(t), dtype=int)
    b = block_ids(t, g, 10.0)
    gen = torch.Generator().manual_seed(1)
    tr, val = split_indices(len(t), gen, b)
    assert set(np.unique(b[tr.numpy()])).isdisjoint(set(np.unique(b[val.numpy()])))
    assert len(tr) + len(val) == len(t)
    frac = len(val) / len(t)
    assert VALIDATION_FRACTION <= frac < VALIDATION_FRACTION + 0.08  # かたまり 1 つ分（約 3%）以内


def test_random_split_is_row_wise_10_percent():
    gen = torch.Generator().manual_seed(0)
    tr, val = split_indices(1000, gen, None)
    assert len(val) == 100 and len(tr) == 900
    assert set(tr.tolist()).isdisjoint(val.tolist())


def test_fit_side_runs_on_small_data():
    rng = np.random.default_rng(0)
    n = 400
    x = np.column_stack([rng.uniform(0, 80, n), rng.uniform(-10, 10, n)])
    y = -1.0 + 0.2 * x[:, 1] - 0.01 * x[:, 0]
    t = np.arange(n) * 0.05
    for blocks in (None, block_ids(t, np.zeros(n, dtype=int), 2.0)):
        side = fit_side(x, y, hidden=(5,), max_iter=3, seed=0, val_blocks=blocks)
        pred = side.predict(x[:7])
        assert pred.shape == (7,)
        assert np.all(np.isfinite(pred))


# ─────────────────────────── 段4: 積み上げでの追加学習 ───────────────────────────


def _synthetic_sides_and_windows():
    """真の加速度 a = 0.5u − 0.05(v − 20)（アクセル）／0.8u − 0.05(v − 20)（ブレーキ）。
    ネットは y を 0.5 ずらして学習させておく（開始時にずれている 2 本）。"""
    rng = np.random.default_rng(0)
    n = 500
    x = np.column_stack([rng.uniform(5, 40, n), rng.uniform(0, 8, n)])
    xb = np.column_stack([rng.uniform(5, 40, n), rng.uniform(-8, 0, n)])
    ya = 0.5 * x[:, 1] - 0.05 * (x[:, 0] - 20)
    yb = 0.8 * xb[:, 1] - 0.05 * (xb[:, 0] - 20)
    acc = fit_side(x, ya + 0.5, hidden=(4,), max_iter=40, seed=0)
    brk = fit_side(xb, yb + 0.5, hidden=(4,), max_iter=40, seed=1)
    # 窓: 真の動特性で車速を作る（遅れなし、刻み 0.1 秒、20 点）
    n_win, n_pt = 30, 20
    v = np.zeros((n_win, n_pt))
    u = np.zeros((n_win, n_pt))
    dt = np.full((n_win, n_pt), 0.1)
    dt[:, 0] = 0
    v[:, 0] = rng.uniform(15, 25, n_win)
    u[:, 1:] = rng.uniform(-6, 6, (n_win, 1)) + rng.normal(0, 0.5, (n_win, n_pt - 1))
    for k in range(1, n_pt):
        a = np.where(u[:, k] >= 0, 0.5 * u[:, k], 0.8 * u[:, k]) - 0.05 * (v[:, k - 1] - 20)
        v[:, k] = np.maximum(v[:, k - 1] + a * 0.1, 0.0)
    idx = np.tile(np.arange(n_pt) - 1, (n_win, 1)).clip(min=0)
    group = WindowGroup(v, u, dt, idx)
    pts = {"accel": (x, ya, None), "brake": (xb, yb, None)}
    return acc, brk, pts, [group]


def _weights(side):
    return [p.detach().clone() for p in side.net.parameters()]


def test_finetune_does_not_touch_input_and_reduces_rollout_error():
    acc, brk, pts, groups = _synthetic_sides_and_windows()
    before = _weights(acc) + _weights(brk)
    a2, b2, info = finetune_rollout(
        acc, brk, pts, groups, loss="absolute", seed=0, max_epochs=15, learning_rate=1e-2
    )
    after = _weights(acc) + _weights(brk)
    assert all(torch.equal(p, q) for p, q in zip(before, after, strict=True))  # 入力は不変
    assert a2 is not acc and b2 is not brk
    assert info["train_roll_end"] < info["train_roll_start"]
    assert info["epochs"] >= 1 and info["n_windows_train"] + info["n_windows_val"] == 30
    assert a2.x_mean is not None and np.array_equal(a2.x_mean, acc.x_mean)
    assert a2.y_std == acc.y_std


def test_finetune_lam_is_ratio_of_start_losses():
    acc, brk, pts, groups = _synthetic_sides_and_windows()
    _, _, info = finetune_rollout(acc, brk, pts, groups, seed=0, max_epochs=1)
    assert np.isclose(info["lam"], info["train_point_start"] / info["train_roll_start"])


def test_finetune_zero_epochs_keeps_weights():
    acc, brk, pts, groups = _synthetic_sides_and_windows()
    a2, b2, info = finetune_rollout(acc, brk, pts, groups, seed=0, max_epochs=0)
    for p, q in zip(_weights(acc) + _weights(brk), _weights(a2) + _weights(b2), strict=True):
        assert torch.equal(p, q)
    assert info["epochs"] == 0 and info["best_epoch"] == 0
    assert np.isclose(info["train_roll_end"], info["train_roll_start"])
