"""簡易車両モデル（ProblemReport_20260912「今回実施する内容」3. 改善提案の模擬用）。

実車に触らずに FF の改善案を比べるための、ペダル開度 → 車速の最小モデル。

    加速度 = 惰行(v) + アクセル応答(v, アクセル開度 − 不感帯)
                     − ブレーキ応答(v, ブレーキ開度 − 不感帯)

    惰行(v)     … −惰行減速カーブ（config の coast_decel_*）。クリープ域は
                  tests/research/hardware.StubVehicle と同じ連続化（クリープ車速へ引き戻す力を
                  上 +creep_rate・下 −惰行減速で挟む）
    ペダル応答  … 不感帯を超えた開度 [%] → 惰行からの加速度の変化 [km/h/s]（正値）。
                  速度 × 開度の表（走行ログから同定）か、config のペダルゲインの比例（比較用）。
                  不感帯の中の開度は 0（惰行と同じ）
    遅れ        … 指令から delay_s 後に効き、ペダル応答に時定数 lag_s の一次遅れ

表の同定（identify_response）:
    走行ログのうち「そのペダルだけを踏み、前後 1s 開度がほぼ一定」の行で、時刻 t の開度と
    [t+0.5, t+1.0] の平均加速度を対応づける（2026-09-12 の議論: この窓で開度と加速度の R² が最大）。
    惰行からの変化 = ±(加速度 + 惰行減速) を「速度帯 × 不感帯超の開度の帯」ごとの中央値にし、
    原点 (0, 0) から単調増加の折れ線にする。データの無い速度帯は近い速度帯の折れ線を使い、
    最後の点より大きい開度は最後の傾きで延ばす。
"""

from __future__ import annotations

import csv
from collections.abc import Callable, Sequence
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from src.models.profile import FeedforwardParams
from tests.research.drive_log import cmd_opening
from tests.research.hardware import STUB_CREEP_STIFFNESS_PER_S

LOG_DT_S = 0.1  # 走行ログの刻み
SIM_DT_S = 0.05  # 閉ループ模擬の刻み（手順 3 の制御周期と同じ）

# 表の同定
RESPONSE_DELAY_S = 0.5  # 開度 t に対応づける加速度の窓の始まり [s]
RESPONSE_WINDOW_S = 0.5  # その窓の長さ [s]
STEADY_S = 1.0  # 開度が一定とみなす前後の時間 [s]
STEADY_TOL_PCT = 1.0  # その間の開度変化の許容 [%]
OTHER_PEDAL_OFF_PCT = 0.5  # もう一方のペダルを踏んでいないとみなす開度 [%]
MIN_IDENT_SPEED_KMH = 5.0  # これ未満はクリープが混ざるので同定に使わない
SPEED_EDGES_KMH = (5.0, 20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 140.0)
OVER_EDGES_PCT = (0.0, 1.0, 2.0, 3.0, 5.0, 8.0, 12.0, 20.0, 40.0, 80.0)
MIN_BIN_ROWS = 8  # 帯の中央値を採用する最少行数

# 表の開度軸（不感帯を超えた開度）
OVER_GRID_STEP_PCT = 0.5
OVER_GRID_MAX_PCT = 80.0
OVER_GRID = np.arange(0.0, OVER_GRID_MAX_PCT + OVER_GRID_STEP_PCT / 2, OVER_GRID_STEP_PCT)

MOVING_KMH = 0.5  # 再生の窓がこれ未満のままなら停車中として除く
PEDAL_ACCEL = "A"
PEDAL_BRAKE = "B"
PEDAL_COAST = "-"


# ─────────────────────────────────────────────────────────────────────
# 走行ログ
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class LogSeries:
    """走行ログの 1 区間（0.1s 刻み）。開度は指令値。"""

    name: str
    t: np.ndarray
    speed: np.ndarray
    accel: np.ndarray
    brake: np.ndarray


def read_log(path: Path, section: str, name: str = "") -> LogSeries:
    """走行 CSV の section の行を読む。時刻は mode_time_s（無ければ elapsed_s）。"""
    t: list[float] = []
    speed: list[float] = []
    accel: list[float] = []
    brake: list[float] = []
    with path.open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            if r.get("section") != section:
                continue
            t.append(float(r.get("mode_time_s") or r["elapsed_s"]))
            speed.append(float(r["actual_speed_kmh"]))
            accel.append(cmd_opening(r, "accel"))
            brake.append(cmd_opening(r, "brake"))
    return LogSeries(
        name=name or path.stem,
        t=np.array(t),
        speed=np.clip(np.array(speed), 0.0, None),
        accel=np.array(accel),
        brake=np.array(brake),
    )


# ─────────────────────────────────────────────────────────────────────
# ペダル応答
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class PedalResponse:
    """不感帯を超えた開度 [%] → 惰行からの加速度の変化 [km/h/s]（正値）。

    table[j, k] は速度 speeds_kmh[j]・開度 OVER_GRID[k] での値。速度方向・開度方向とも線形補間
    （速度は端でクランプ）。
    """

    label: str
    speeds_kmh: np.ndarray
    table: np.ndarray

    def __post_init__(self) -> None:
        if len(self.speeds_kmh) < 2:
            raise ValueError("speeds_kmh は 2 点以上必要です")
        if self.table.shape != (len(self.speeds_kmh), len(OVER_GRID)):
            raise ValueError(f"table の形が {self.table.shape} です")

    def at(self, speed_kmh: np.ndarray | float, over_pct: np.ndarray | float) -> np.ndarray:
        sp = self.speeds_kmh
        v = np.clip(np.asarray(speed_kmh, dtype=float), sp[0], sp[-1])
        j1 = np.clip(np.searchsorted(sp, v, side="right"), 1, len(sp) - 1)
        j0 = j1 - 1
        w = (v - sp[j0]) / (sp[j1] - sp[j0])
        pos = np.clip(np.asarray(over_pct, dtype=float), 0.0, OVER_GRID_MAX_PCT)
        pos = pos / OVER_GRID_STEP_PCT
        k0 = np.minimum(np.floor(pos).astype(int), len(OVER_GRID) - 2)
        frac = pos - k0

        def along(j: np.ndarray) -> np.ndarray:
            return self.table[j, k0] * (1.0 - frac) + self.table[j, k0 + 1] * frac

        return (1.0 - w) * along(j0) + w * along(j1)


def proportional_response(
    label: str, speeds_kmh: Sequence[float], gains: Sequence[float]
) -> PedalResponse:
    """config のペダルゲイン（開度 1% あたり）の比例: 変化 = ゲイン(v) × 不感帯超の開度。"""
    return PedalResponse(
        label=label,
        speeds_kmh=np.asarray(speeds_kmh, dtype=float),
        table=np.outer(np.asarray(gains, dtype=float), OVER_GRID),
    )


def scale_response(response: PedalResponse, factor: float) -> PedalResponse:
    """応答を factor 倍した表（改善案の比較で「車両モデルが外れていたら」を見るため）。"""
    return PedalResponse(
        label=f"{response.label}×{factor:g}",
        speeds_kmh=response.speeds_kmh,
        table=response.table * factor,
    )


def coast_decel(params: FeedforwardParams, speed_kmh: np.ndarray | float) -> np.ndarray:
    """coast_decel_at（src/models/profile.py）の配列版。カーブが 2 点未満なら定数。"""
    v = np.asarray(speed_kmh, dtype=float)
    n = min(len(params.coast_decel_speeds_kmh), len(params.coast_decel_kmhs))
    if n < 2:
        return np.full(v.shape, params.engine_brake_decel_kmhs)
    return np.interp(v, params.coast_decel_speeds_kmh[:n], params.coast_decel_kmhs[:n])


@dataclass(frozen=True)
class ResponseSamples:
    speed_kmh: np.ndarray
    over_pct: np.ndarray  # 不感帯を超えた開度
    delta_kmhs: np.ndarray  # 惰行からの加速度の変化（効く向きを正）


def response_samples(
    log: LogSeries, params: FeedforwardParams, *, is_accel: bool
) -> ResponseSamples:
    """同定に使う行（片方のペダルだけ・開度が前後 1s 一定・不感帯超・5 km/h 以上）。

    速度（表の速度軸と惰行減速を引く速度）は、加速度を測る窓の平均車速を使う。
    時刻 t の車速で惰行を引くと、強い減速中は窓までに速度が変わった分だけずれる。
    """
    n = len(log.speed)
    d = round(RESPONSE_DELAY_S / LOG_DT_S)
    w = round(RESPONSE_WINDOW_S / LOG_DT_S)
    s = round(STEADY_S / LOG_DT_S)
    i = np.arange(s, max(s, n - max(s, d + w)))
    pedal, other = (log.accel, log.brake) if is_accel else (log.brake, log.accel)
    db = params.accel_deadband_pct if is_accel else params.brake_deadband_pct
    v = (log.speed[i + d] + log.speed[i + d + w]) / 2.0
    acc = (log.speed[i + d + w] - log.speed[i + d]) / (w * LOG_DT_S)
    steady = (np.abs(pedal[i] - pedal[i - s]) < STEADY_TOL_PCT) & (
        np.abs(pedal[i + s] - pedal[i]) < STEADY_TOL_PCT
    )
    m = steady & (other[i] < OTHER_PEDAL_OFF_PCT) & (pedal[i] > db) & (v >= MIN_IDENT_SPEED_KMH)
    change = acc[m] + coast_decel(params, v[m])
    return ResponseSamples(
        speed_kmh=v[m], over_pct=pedal[i][m] - db, delta_kmhs=change if is_accel else -change
    )


def _curve_on_grid(over: np.ndarray, delta: np.ndarray) -> np.ndarray:
    """原点を含む折れ線を OVER_GRID に載せる。最後の点より先は最後の傾き（0 以上）で延ばす。"""
    out = np.interp(OVER_GRID, over, delta)
    slope = max(0.0, (delta[-1] - delta[-2]) / (over[-1] - over[-2]))
    beyond = OVER_GRID > over[-1]
    out[beyond] = delta[-1] + slope * (OVER_GRID[beyond] - over[-1])
    return out


def identify_response(
    label: str,
    logs: Sequence[LogSeries],
    params: FeedforwardParams,
    *,
    is_accel: bool,
) -> tuple[PedalResponse, np.ndarray]:
    """走行ログからペダル応答の表を作る。(表, 帯ごとの行数 [速度帯 × 開度帯]) を返す。"""
    parts = [response_samples(lg, params, is_accel=is_accel) for lg in logs]
    v = np.concatenate([p.speed_kmh for p in parts])
    u = np.concatenate([p.over_pct for p in parts])
    dlt = np.concatenate([p.delta_kmhs for p in parts])
    speed_bins = list(zip(SPEED_EDGES_KMH, SPEED_EDGES_KMH[1:], strict=False))
    over_bins = list(zip(OVER_EDGES_PCT, OVER_EDGES_PCT[1:], strict=False))
    counts = np.zeros((len(speed_bins), len(over_bins)), dtype=int)
    curves: list[np.ndarray | None] = []
    for j, (lo, hi) in enumerate(speed_bins):
        pts_u, pts_d = [0.0], [0.0]
        for k, (ulo, uhi) in enumerate(over_bins):
            m = (v >= lo) & (v < hi) & (u > ulo) & (u <= uhi)
            counts[j, k] = int(np.count_nonzero(m))
            if counts[j, k] >= MIN_BIN_ROWS:
                pts_u.append(float(np.median(u[m])))
                pts_d.append(float(np.median(dlt[m])))
        if len(pts_u) < 2:
            curves.append(None)
            continue
        mono = np.maximum.accumulate(np.maximum(np.array(pts_d), 0.0))
        curves.append(_curve_on_grid(np.array(pts_u), mono))
    have = [j for j, c in enumerate(curves) if c is not None]
    if not have:
        raise ValueError(f"{label}: 同定に使える行がありません")
    table = np.array([curves[min(have, key=lambda h, j=j: abs(h - j))] for j in range(len(curves))])
    centers = np.array([(lo + hi) / 2.0 for lo, hi in speed_bins])
    return PedalResponse(label=label, speeds_kmh=centers, table=table), counts


# ─────────────────────────────────────────────────────────────────────
# 車両
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class VehicleModel:
    name: str
    params: FeedforwardParams
    accel_response: PedalResponse
    brake_response: PedalResponse
    delay_s: float = 0.0  # 指令から効き始めるまで [s]
    lag_s: float = 0.0  # ペダル応答の一次遅れの時定数 [s]
    creep_stiffness_per_s: float = STUB_CREEP_STIFFNESS_PER_S

    def with_delay(self, delay_s: float, lag_s: float) -> VehicleModel:
        return replace(self, delay_s=delay_s, lag_s=lag_s)

    def coast_accel(self, speed_kmh: np.ndarray | float) -> np.ndarray:
        """両ペダルを離したときの加速度 [km/h/s]（StubVehicle と同じクリープ域の連続化）。"""
        p = self.params
        v = np.asarray(speed_kmh, dtype=float)
        pull = self.creep_stiffness_per_s * (p.creep_speed_kmh - v)
        return np.minimum(p.creep_rate_kmhs, np.maximum(-coast_decel(p, v), pull))

    def pedal_accel(
        self,
        speed_kmh: np.ndarray | float,
        accel_pct: np.ndarray | float,
        brake_pct: np.ndarray | float,
    ) -> np.ndarray:
        """ペダルによる惰行からの加速度の変化 [km/h/s]（不感帯の中は 0）。"""
        p = self.params
        a = np.asarray(accel_pct, dtype=float) - p.accel_deadband_pct
        b = np.asarray(brake_pct, dtype=float) - p.brake_deadband_pct
        return self.accel_response.at(speed_kmh, a) - self.brake_response.at(speed_kmh, b)


def pedal_of(accel: np.ndarray, brake: np.ndarray, params: FeedforwardParams) -> str:
    """区間の中で効いたペダル（アクセル優先）。"""
    if np.any(accel >= params.accel_deadband_pct):
        return PEDAL_ACCEL
    if np.any(brake >= params.brake_deadband_pct):
        return PEDAL_BRAKE
    return PEDAL_COAST


# ─────────────────────────────────────────────────────────────────────
# 開ループ再生（実走行の指令開度を入れ、horizon_s 先の車速を比べる）
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class ReplayResult:
    error_kmh: np.ndarray  # horizon_s 後の車速誤差（模擬 − 実測）
    pedal: np.ndarray  # 窓の中で効いたペダル A / B / -

    def summary(self, pedal: str | None = None) -> tuple[float, float, float, int]:
        """(RMSE, |誤差| p95, 平均, 窓の数)。pedal を指定するとその窓だけ。"""
        e = self.error_kmh if pedal is None else self.error_kmh[self.pedal == pedal]
        if e.size == 0:
            return float("nan"), float("nan"), float("nan"), 0
        return (
            float(np.sqrt(np.mean(e**2))),
            float(np.percentile(np.abs(e), 95)),
            float(np.mean(e)),
            int(e.size),
        )


def replay(
    model: VehicleModel, log: LogSeries, *, horizon_s: float = 5.0, every_s: float = 1.0
) -> ReplayResult:
    """every_s ごとに実車速から始め、指令開度だけで horizon_s 走らせる（全窓を同時に計算）。"""
    dt = LOG_DT_S
    h = round(horizon_s / dt)
    d = round(model.delay_s / dt)
    n = len(log.speed)
    starts = np.arange(0, max(0, n - h), round(every_s / dt))
    moving = np.array([log.speed[s : s + h + 1].max() >= MOVING_KMH for s in starts], dtype=bool)
    starts = starts[moving] if starts.size else starts
    if starts.size == 0:
        return ReplayResult(np.empty(0), np.empty(0, dtype=str))

    def target(x: np.ndarray, i: np.ndarray) -> np.ndarray:
        j = np.clip(i - d, 0, n - 1)
        return model.pedal_accel(x, log.accel[j], log.brake[j])

    alpha = dt / (model.lag_s + dt) if model.lag_s > 0.0 else 1.0
    x = log.speed[starts].copy()
    y = target(x, starts)
    for k in range(h):
        y = y + alpha * (target(x, starts + k) - y)
        x = np.maximum(0.0, x + (model.coast_accel(x) + y) * dt)
    pedal = np.array(
        [pedal_of(log.accel[s : s + h], log.brake[s : s + h], model.params) for s in starts]
    )
    return ReplayResult(error_kmh=x - log.speed[starts + h], pedal=pedal)


# ─────────────────────────────────────────────────────────────────────
# 閉ループ模擬
# ─────────────────────────────────────────────────────────────────────

Command = Callable[[int, float], tuple[float, float]]  # (周期番号, 車速) → (アクセル, ブレーキ)


@dataclass(frozen=True)
class SimRun:
    t: np.ndarray
    speed: np.ndarray  # その周期の始めの車速（指令を決めるときに見た車速）
    accel: np.ndarray  # 指令開度
    brake: np.ndarray
    stopped_at_s: float | None  # stop_above_kmh を超えて打ち切った時刻

    @property
    def end_s(self) -> float:
        return float(self.t[-1]) if self.t.size else 0.0


def simulate(
    model: VehicleModel,
    n_steps: int,
    command: Command,
    *,
    dt: float = SIM_DT_S,
    v_start: float = 0.0,
    stop_above_kmh: float | None = None,
) -> SimRun:
    """周期ごとに指令を決めて車両を進める。stop_above_kmh を超えたらそこで打ち切る。"""
    d = round(model.delay_s / dt)
    alpha = dt / (model.lag_s + dt) if model.lag_s > 0.0 else 1.0
    speed = np.empty(n_steps)
    accel = np.empty(n_steps)
    brake = np.empty(n_steps)
    v = v_start
    y: float | None = None
    stopped: float | None = None
    last = n_steps
    for i in range(n_steps):
        speed[i] = v
        if stop_above_kmh is not None and v > stop_above_kmh:
            stopped, last = i * dt, i + 1
            accel[i], brake[i] = 0.0, 0.0
            break
        accel[i], brake[i] = command(i, v)
        j = max(0, i - d)
        tgt = float(model.pedal_accel(v, accel[j], brake[j]))
        y = tgt if y is None else y + alpha * (tgt - y)
        v = max(0.0, v + (float(model.coast_accel(v)) + y) * dt)
    return SimRun(
        t=np.arange(last) * dt,
        speed=speed[:last],
        accel=accel[:last],
        brake=brake[:last],
        stopped_at_s=stopped,
    )
