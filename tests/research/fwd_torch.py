"""順モデル（C8）の PyTorch 版の学習（段2i の改善。検討案 1・2・3・14・15）。

形は fwd_model.py と同じ: アクセル側（u >= 0）・ブレーキ側（u <= 0）の 2 本、
入力は 車速・u（標準化）、
目的変数の加速度も標準化して学習し予測で戻す。FittedModel と同じ `accel_side.predict` /
`brake_side.predict` を持つので、fwd_model.model_to_grid にそのまま渡せる。

段A は sklearn の MLPRegressor（Adam・学習率 1e-3・バッチ 200・L2 1e-4・検証 10%・10 回改善なしで
打ち切り・最良の重みに戻す）に合わせた再現。損失（二乗・絶対・Huber）と行の重み（除外は重み 0）は
引数で切り替える。
"""

from __future__ import annotations

import copy
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np
import torch
from torch import nn

LOSS_SQUARED = "squared"
LOSS_ABSOLUTE = "absolute"
LOSS_HUBER = "huber"
LOSSES = (LOSS_SQUARED, LOSS_ABSOLUTE, LOSS_HUBER)

ACTIVATIONS = {"logistic": nn.Sigmoid, "tanh": nn.Tanh, "relu": nn.ReLU}

# sklearn MLPRegressor の既定に合わせた値（段A の再現用。データ由来ではなく sklearn の既定）
LEARNING_RATE = 1e-3
BATCH_SIZE = 200
VALIDATION_FRACTION = 0.1
N_ITER_NO_CHANGE = 10
TOL = 1e-4
MAX_ITER = 500
ALPHA_L2 = 1e-4
HUBER_DELTA = 1.0  # 標準化した加速度での値（仮値。Huber を試すときに決める）

# 段4（積み上げたずれでの追加学習。finetune_rollout）の仮値。どれもデータ由来ではない
ROLLOUT_LEARNING_RATE = 1e-4  # 仮値（最初の学習の 1/10。追加学習なので小さく）
ROLLOUT_BATCH_WINDOWS = 32  # 仮値（計算時間の都合）
ROLLOUT_MAX_EPOCHS = 100  # 仮値（打ち切りは N_ITER_NO_CHANGE と同じ規則）
ROLLOUT_EVAL_CHUNK = 256  # 全窓で損失を測るときの 1 回あたりの窓数（メモリの都合）

# 検証のかたまりの長さ [s]。仮値（加速度の窓 1 秒より十分長い）。段0 で無作為抜きと比べて決める
VAL_BLOCK_S = 10.0


def set_threads(n: int) -> None:
    """torch の計算スレッド数（並列プロセスでは 1 にする）。呼ばなければ torch の既定。"""
    torch.set_num_threads(n)


def block_ids(t_s: np.ndarray, log_idx: np.ndarray, block_s: float = VAL_BLOCK_S) -> np.ndarray:
    """行ごとの「かたまり番号」。同じログの中で時刻を block_s 秒ごとに区切る。
    ログが違えば別の番号。"""
    t_s = np.asarray(t_s, dtype=float)
    log_idx = np.asarray(log_idx)
    out = np.empty(len(t_s), dtype=np.int64)
    base = 0
    for g in np.unique(log_idx):
        m = log_idx == g
        local = np.floor((t_s[m] - t_s[m].min()) / block_s).astype(np.int64)
        out[m] = base + local
        base += int(local.max()) + 1
    return out


def split_indices(
    n: int, gen: torch.Generator, val_blocks: np.ndarray | None = None
) -> tuple[torch.Tensor, torch.Tensor]:
    """早期打ち切りの (学習の添字, 検証の添字)。val_blocks なしは行の無作為抜き（約 10%）。
    あれば「かたまり」を無作為に選び、行数が約 10% になるまで取る（同じかたまりは丸ごと片側）。"""
    n_val = max(int(n * VALIDATION_FRACTION), 1)
    if val_blocks is None:
        perm = torch.randperm(n, generator=gen)
        return perm[n_val:], perm[:n_val]
    blocks = np.asarray(val_blocks)
    ids, counts = np.unique(blocks, return_counts=True)
    order = torch.randperm(len(ids), generator=gen).numpy()
    chosen, total = [], 0
    for k in order:
        if total >= n_val:
            break
        chosen.append(ids[k])
        total += int(counts[k])
    is_val = np.isin(blocks, np.array(chosen))
    return (
        torch.as_tensor(np.flatnonzero(~is_val), dtype=torch.long),
        torch.as_tensor(np.flatnonzero(is_val), dtype=torch.long),
    )


def make_net(
    hidden: Sequence[int], activation: str = "logistic", n_inputs: int = 2
) -> nn.Sequential:
    """n_inputs: 入力の数（アクセル側は 2＝車速・u、ブレーキ側は 2026-10-14 段2o-5 から
    3＝＋戻し量）。"""
    layers: list[nn.Module] = []
    n_in = n_inputs
    for h in hidden:
        layers += [nn.Linear(n_in, h), ACTIVATIONS[activation]()]
        n_in = h
    layers.append(nn.Linear(n_in, 1))
    return nn.Sequential(*layers)


@dataclass
class TorchSide:
    """片側の学習済みネット。predict は (N, 2)（車速・u）→ 加速度 [km/h/s]。
    ブレーキ側は (N, 3)（車速・u・戻し量。2026-10-14 段2o-5）。"""

    net: nn.Sequential
    x_mean: np.ndarray
    x_std: np.ndarray
    y_mean: float
    y_std: float
    n_iter: int = 0

    @property
    def n_inputs(self) -> int:
        return len(self.x_mean)

    def predict(self, x: np.ndarray) -> np.ndarray:
        zx = (np.asarray(x, dtype=float) - self.x_mean) / self.x_std
        z = torch.as_tensor(zx, dtype=torch.float32)
        with torch.no_grad():
            out = self.net(z).squeeze(1).numpy().astype(float)
        return out * self.y_std + self.y_mean


def _loss_fn(kind: str) -> nn.Module:
    if kind == LOSS_SQUARED:
        return nn.MSELoss(reduction="none")
    if kind == LOSS_ABSOLUTE:
        return nn.L1Loss(reduction="none")
    if kind == LOSS_HUBER:
        return nn.HuberLoss(reduction="none", delta=HUBER_DELTA)
    raise ValueError(f"損失の種類が不明: {kind}")


def fit_side(
    x: np.ndarray,
    y: np.ndarray,
    *,
    hidden: Sequence[int] = (10,),
    activation: str = "logistic",
    loss: str = LOSS_SQUARED,
    alpha: float = ALPHA_L2,
    seed: int = 0,
    max_iter: int = MAX_ITER,
    early_stopping: bool = True,
    weights: np.ndarray | None = None,
    batch_size: int = BATCH_SIZE,
    val_blocks: np.ndarray | None = None,
) -> TorchSide:
    """片側（accel か brake）の行 (x: (N, 2), y: (N,)) からネットを学習する。
    weights は行の重み（0 で学習から除く）。検証 10% は重み 0 の行も含めて無作為に分けるが、
    検証の損失も重みをかけて測る。val_blocks（行ごとのかたまり番号）を渡すと、検証はかたまり単位で抜く。"""
    x = np.asarray(x, dtype=float)
    y = np.asarray(y, dtype=float)
    w = np.ones(len(y)) if weights is None else np.asarray(weights, dtype=float)
    keep = w > 0
    x_mean, x_std = x[keep].mean(axis=0), x[keep].std(axis=0)
    x_std = np.where(x_std > 0, x_std, 1.0)
    y_mean, y_std = float(y[keep].mean()), float(y[keep].std()) or 1.0
    zx = torch.as_tensor((x - x_mean) / x_std, dtype=torch.float32)
    zy = torch.as_tensor((y - y_mean) / y_std, dtype=torch.float32)
    zw = torch.as_tensor(w, dtype=torch.float32)

    gen = torch.Generator().manual_seed(seed)
    torch.manual_seed(seed)
    net = make_net(hidden, activation, n_inputs=x.shape[1])
    opt = torch.optim.Adam(net.parameters(), lr=LEARNING_RATE)
    lf = _loss_fn(loss)

    n = len(y)
    if early_stopping:
        tr_idx, val_idx = split_indices(n, gen, val_blocks)
    else:
        tr_idx, val_idx = torch.randperm(n, generator=gen), torch.zeros(0, dtype=torch.long)

    def weighted_mean(idx: torch.Tensor) -> torch.Tensor:
        per = lf(net(zx[idx]).squeeze(1), zy[idx]) * zw[idx]
        return per.sum() / zw[idx].sum().clamp_min(1e-12)

    best, best_state, bad, n_iter = float("inf"), None, 0, 0
    for n_iter in range(1, max_iter + 1):  # noqa: B007
        net.train()
        order = tr_idx[torch.randperm(len(tr_idx), generator=gen)]
        for b in range(0, len(order), batch_size):
            idx = order[b : b + batch_size]
            if float(zw[idx].sum()) <= 0:
                continue
            per = lf(net(zx[idx]).squeeze(1), zy[idx]) * zw[idx]
            data_loss = per.sum() / zw[idx].sum()
            l2 = sum((p**2).sum() for name, p in net.named_parameters() if name.endswith("weight"))
            # sklearn と同じ: L2 は 0.5 × alpha × |W|² を行数で割って足す
            total = data_loss + 0.5 * alpha * l2 / len(idx)
            opt.zero_grad()
            total.backward()
            opt.step()
        if early_stopping:
            net.eval()
            with torch.no_grad():
                score = float(weighted_mean(val_idx))
            if score < best - TOL:
                best, bad = score, 0
                best_state = {k: v.clone() for k, v in net.state_dict().items()}
            else:
                bad += 1
                if bad >= N_ITER_NO_CHANGE:
                    break
        else:
            with torch.no_grad():
                score = float(weighted_mean(tr_idx))
            if score < best - TOL:
                best, bad = score, 0
            else:
                bad += 1
                if bad >= N_ITER_NO_CHANGE:
                    break
    if best_state is not None:
        net.load_state_dict(best_state)
    net.eval()
    return TorchSide(net, x_mean, x_std, y_mean, y_std, n_iter)


def timed_fit_side(*args, **kwargs) -> tuple[TorchSide, float]:
    t0 = time.perf_counter()
    side = fit_side(*args, **kwargs)
    return side, time.perf_counter() - t0


# ─────────────────────────── 段4: 積み上げたずれでの追加学習 ───────────────────────────


def rollout_speed(
    accel_fn: Callable[[torch.Tensor, torch.Tensor], torch.Tensor],
    v0: torch.Tensor,
    u_lag: torch.Tensor,
    dt: torch.Tensor,
    v_idx: torch.Tensor,
) -> torch.Tensor:
    """窓ごとに車速を積み上げる（fwd_plan.simulate_speed の hold なしと同じ規則）。
    accel_fn(v (B,), u (B,)) → 加速度 (B,) [km/h/s]。v0: (B,) 窓の頭の実車速。
    u_lag・dt: (B, T)。k 列目 = そのステップで使う遅らせた u・刻み（0 列目は使わない）。
    v_idx: (B, T) の LongTensor。k 列目 = 遅らせた車速として使う積み上げ済みの列番号（0..k-1）。
    vs[0] = v0、vs[k] = max(vs[k-1] + accel_fn(vs[v_idx[:, k]], u_lag[:, k]) × dt[:, k], 0)。
    返り値 (B, T)。勾配が流れるよう列をリストにためて stack する（in-place 代入しない）。"""
    cols = [v0]
    for k in range(1, u_lag.shape[1]):
        so_far = torch.stack(cols, dim=1)
        v_lag = so_far.gather(1, v_idx[:, k : k + 1]).squeeze(1)
        nxt = cols[-1] + accel_fn(v_lag, u_lag[:, k]) * dt[:, k]
        cols.append(torch.clamp(nxt, min=0.0))
    return torch.stack(cols, dim=1)


@dataclass
class WindowGroup:
    """同じ点数 n の窓のまとまり。配列はすべて (窓の数, n)。
    v: 実車速、u_lag: 遅らせた u、dt: 刻み、v_idx: 遅らせた車速の窓内の列番号。"""

    v: np.ndarray
    u_lag: np.ndarray
    dt: np.ndarray
    v_idx: np.ndarray

    def __len__(self) -> int:
        return len(self.v)


def _side_accel(side: TorchSide, v: torch.Tensor, u: torch.Tensor) -> torch.Tensor:
    """片側のネットを物理単位（km/h/s）で評価する。入力が 3 つの側（ブレーキ側）は
    戻し量・踏み増し量 0 で評価する（積み上げの追加学習は戻し量を使わない。2026-10-14 段2o-5）。"""
    cols = [v, u] + [torch.zeros_like(v) for _ in range(side.n_inputs - 2)]
    x = torch.stack(cols, dim=1)
    xm = torch.as_tensor(side.x_mean, dtype=x.dtype)
    xs = torch.as_tensor(side.x_std, dtype=x.dtype)
    return side.net((x - xm) / xs).squeeze(1) * side.y_std + side.y_mean


def finetune_rollout(
    accel_side: TorchSide,
    brake_side: TorchSide,
    point_data: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray | None]],
    window_groups: Sequence[WindowGroup],
    *,
    loss: str = LOSS_ABSOLUTE,
    alpha: float = ALPHA_L2,
    seed: int = 0,
    coast_from: str = "mean",
    max_epochs: int = ROLLOUT_MAX_EPOCHS,
    batch_size: int = BATCH_SIZE,
    batch_windows: int = ROLLOUT_BATCH_WINDOWS,
    learning_rate: float = ROLLOUT_LEARNING_RATE,
) -> tuple[TorchSide, TorchSide, dict]:
    """学習済みの 2 本から始めて、1 点の加速度の外れ L_point に「積み上げた車速と実車速の差
    （絶対誤差）L_roll [km/h]」を足した損失で同時に追加学習する。入力の TorchSide は変えない
    （コピーして学習）。標準化の定数も変えない。
    point_data: {"accel": (x, y, 重み or None), "brake": (...)}（fit_side に渡した行と同じ形）。
    合計 = L_point + lam × L_roll + L2。lam は開始時（更新前）の学習側の L_point / L_roll
    （開始時に 2 項が同じ大きさ。規則で決まる値で調整値ではない）。
    検証は 1 点の行・窓とも無作為 10%。エポックの終わりに検証の L_point + lam × L_roll
    （L2 なし）を測り、fit_side と同じ規則（TOL 以上の改善が N_ITER_NO_CHANGE 回続けて無ければ
    打ち切り）で最良の重みに戻す。最良の初期値は更新前の検証の損失（一度も良くならなければ
    開始時の重みのまま返る）。"""
    if coast_from not in ("mean", "accel"):
        raise ValueError(f"coast_from が不明: {coast_from}")
    t0 = time.perf_counter()
    nets = {"accel": copy.deepcopy(accel_side), "brake": copy.deepcopy(brake_side)}
    params = [p for s in nets.values() for p in s.net.parameters()]
    lf = _loss_fn(loss)
    gen = torch.Generator().manual_seed(seed)

    def accel_fn(v: torch.Tensor, u: torch.Tensor) -> torch.Tensor:
        a = _side_accel(nets["accel"], v, u)
        b = _side_accel(nets["brake"], v, u)
        zero = a if coast_from == "accel" else (a + b) / 2.0
        return torch.where(u > 0, a, torch.where(u < 0, b, zero))

    # 1 点の行: 側ごとに標準化して学習用・検証用に分ける
    pts: dict[str, dict] = {}
    for name, (x, y, w) in point_data.items():
        s = nets[name]
        x = np.asarray(x, dtype=float)
        zx = torch.as_tensor((x - s.x_mean) / s.x_std, dtype=torch.float32)
        zy = torch.as_tensor((np.asarray(y, dtype=float) - s.y_mean) / s.y_std, dtype=torch.float32)
        zw = torch.ones(len(zy)) if w is None else torch.as_tensor(w, dtype=torch.float32)
        tr, va = split_indices(len(zy), gen)
        pts[name] = {"zx": zx, "zy": zy, "zw": zw, "tr": tr, "va": va}

    # 窓: グループごとに torch へ。全体で無作為 10% を検証へ
    if not window_groups or sum(len(g) for g in window_groups) == 0:
        raise ValueError("積み上げに使える窓が無い")
    grp = [
        {
            "v": torch.as_tensor(g.v, dtype=torch.float32),
            "u": torch.as_tensor(g.u_lag, dtype=torch.float32),
            "dt": torch.as_tensor(g.dt, dtype=torch.float32),
            "idx": torch.as_tensor(g.v_idx, dtype=torch.long),
        }
        for g in window_groups
    ]
    owners = torch.cat([torch.full((len(g),), i) for i, g in enumerate(window_groups)])
    locals_ = torch.cat([torch.arange(len(g)) for g in window_groups])
    perm = torch.randperm(len(owners), generator=gen)
    n_win = len(owners)
    n_val_w = min(max(int(n_win * VALIDATION_FRACTION), 1), n_win - 1) if n_win > 1 else 0
    val_w, tr_w = perm[:n_val_w], perm[n_val_w:]
    if n_val_w == 0:  # 窓が 1 つしかないときは学習と検証で共用
        val_w = tr_w
    tr_by_group = [locals_[tr_w[owners[tr_w] == i]] for i in range(len(grp))]
    va_by_group = [locals_[val_w[owners[val_w] == i]] for i in range(len(grp))]

    def point_mean(name: str, idx: torch.Tensor) -> torch.Tensor:
        p = pts[name]
        per = lf(nets[name].net(p["zx"][idx]).squeeze(1), p["zy"][idx]) * p["zw"][idx]
        return per.sum() / p["zw"][idx].sum().clamp_min(1e-12)

    def roll_abs(i: int, idx: torch.Tensor) -> tuple[torch.Tensor, int]:
        """グループ i の窓 idx の |積み上げ − 実車速| の合計と要素数（列 0 は除く）。"""
        g = grp[i]
        v = g["v"][idx]
        vs = rollout_speed(accel_fn, v[:, 0], g["u"][idx], g["dt"][idx], g["idx"][idx])
        d = (vs - v)[:, 1:].abs()
        return d.sum(), d.numel()

    def roll_mean_all(by_group: list[torch.Tensor]) -> float:
        tot, cnt = 0.0, 0
        for i, idx in enumerate(by_group):
            for b in range(0, len(idx), ROLLOUT_EVAL_CHUNK):
                s, c = roll_abs(i, idx[b : b + ROLLOUT_EVAL_CHUNK])
                tot, cnt = tot + float(s), cnt + c
        return tot / cnt if cnt else 0.0

    def point_mean_all(key: str) -> float:
        with torch.no_grad():
            return sum(float(point_mean(n, pts[n][key])) for n in pts)

    def measure(by_group: list[torch.Tensor], key: str) -> tuple[float, float]:
        with torch.no_grad():
            return point_mean_all(key), roll_mean_all(by_group)

    lp0, lr0 = measure(tr_by_group, "tr")
    lam = lp0 / max(lr0, 1e-12)
    vlp0, vlr0 = measure(va_by_group, "va")
    best = vlp0 + lam * vlr0  # 更新前の検証の損失（これより良くならなければ開始時の重みのまま）
    best_state = {n: {k: v.clone() for k, v in s.net.state_dict().items()} for n, s in nets.items()}
    best_epoch, bad, n_epoch = 0, 0, 0

    opt = torch.optim.Adam(params, lr=learning_rate)
    for n_epoch in range(1, max_epochs + 1):  # noqa: B007
        # 窓のバッチはグループ内で作り、バッチの順はグループをまたいでシャッフル
        batches = []
        for i, idx in enumerate(tr_by_group):
            o = idx[torch.randperm(len(idx), generator=gen)]
            batches += [(i, o[b : b + batch_windows]) for b in range(0, len(o), batch_windows)]
        for bi in torch.randperm(len(batches), generator=gen).tolist():
            i, widx = batches[bi]
            s_abs, cnt = roll_abs(i, widx)
            total = lam * s_abs / cnt
            for name, p in pts.items():
                tr = p["tr"]
                idx = tr[torch.randint(len(tr), (min(batch_size, len(tr)),), generator=gen)]
                named = nets[name].net.named_parameters()
                l2 = sum((q**2).sum() for k, q in named if k.endswith("weight"))
                total = total + point_mean(name, idx) + 0.5 * alpha * l2 / len(idx)
            opt.zero_grad()
            total.backward()
            opt.step()
        vlp, vlr = measure(va_by_group, "va")
        score = vlp + lam * vlr
        if score < best - TOL:
            best, bad, best_epoch = score, 0, n_epoch
            best_state = {
                n: {k: v.clone() for k, v in s.net.state_dict().items()} for n, s in nets.items()
            }
        else:
            bad += 1
            if bad >= N_ITER_NO_CHANGE:
                break
    for n, s in nets.items():
        s.net.load_state_dict(best_state[n])
        s.net.eval()
    lp1, lr1 = measure(tr_by_group, "tr")
    vlp1, vlr1 = measure(va_by_group, "va")
    info = {
        "lam": lam,
        "train_point_start": lp0,
        "train_roll_start": lr0,
        "train_point_end": lp1,
        "train_roll_end": lr1,
        "val_point_start": vlp0,
        "val_roll_start": vlr0,
        "val_point_end": vlp1,
        "val_roll_end": vlr1,
        "epochs": n_epoch,
        "best_epoch": best_epoch,
        "seconds": time.perf_counter() - t0,
        "n_windows_train": int(len(tr_w)),
        "n_windows_val": int(len(val_w)),
    }
    return nets["accel"], nets["brake"], info
