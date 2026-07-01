"""PID ゲインの自動適合ドメインモジュール。

本系は FF（Ridge 逆モデル）が基準軌跡の名目開度を出し、PID は残差（追従誤差）だけを
補正する構成。したがって PID が見るプラントは「開度→車速」の物理特性そのものであり、
これは学習運転が開ループのステップ応答（アクセル一定保持→車速プラトー）として既に
記録している。本モジュールはそのログから一次遅れ+むだ時間（FOPDT）を同定し、
SIMC（Skogestad）則で Kp/Ki を解析算出する（モデルベース適合）。

`estimate_dynamics_params`（model_training.py）と同じ堅牢性方針:
観測が不足する場合は None を返し、呼び出し元が既存ゲインを保持する。
"""

from __future__ import annotations

import logging
import math
from dataclasses import dataclass, replace
from datetime import UTC, datetime

import numpy as np

from src.domain.control.kpi_monitor import (
    KPI_HARD_LIMIT_KMH,
    KPI_P95_LIMIT_KMH,
    KPI_REVERSAL_WINDOW_S,
)
from src.domain.model_training import DEFAULT_DT_S, _group_by_session
from src.models.drive_log import DriveLog
from src.models.driving_mode import DrivingMode, SpeedPoint
from src.models.profile import PIDGains, VehicleProfile

_logger = logging.getLogger(__name__)

# ── FOPDT 同定パラメータ ───────────────────────────────────────────────
SEG_MIN_OPENING_PCT: float = 5.0  # 保持区間とみなすアクセル開度の下限
SEG_MIN_SAMPLES: int = 10  # 区間を採用する最小サンプル数（100ms なら約 1 秒）
MIN_RISE_KMH: float = 1.0  # 区間内で必要な車速上昇量（これ未満はノイズ扱い）
ONSET_RISE_KMH: float = 0.3  # むだ時間判定: 車速がこの量を超えたら応答開始とみなす
TAU_MIN_S: float = 0.2  # 時定数のクランプ下限
TAU_MAX_S: float = 30.0  # 時定数のクランプ上限
L_MAX_S: float = 3.0  # むだ時間のクランプ上限
MIN_SEGMENTS_FOR_ID: int = 2  # FOPDT 同定に必要な最小区間数

# ── SIMC / ゲインクランプ ──────────────────────────────────────────────
KP_MIN: float = 0.0
KP_MAX: float = 50.0
KI_MIN: float = 0.0
KI_MAX: float = 50.0


@dataclass(frozen=True)
class FOPDT:
    """一次遅れ+むだ時間モデル（開度→車速）。

    Attributes:
        k: 定常ゲイン [km/h / %]（アクセル開度 1% あたりの定常車速増分）
        tau: 時定数 [s]
        theta: むだ時間 [s]
    """

    k: float
    tau: float
    theta: float


def _segment_fopdt(
    speed: np.ndarray, opening: np.ndarray, dt: float
) -> tuple[float, float, float] | None:
    """単一のアクセル保持区間から (K, τ, L) を推定する。失敗時は None。

    Args:
        speed: 区間内の実車速系列 [km/h]
        opening: 区間内のアクセル開度系列 [%]
        dt: サンプル周期 [s]
    """
    n = len(speed)
    if n < SEG_MIN_SAMPLES:
        return None

    # 保持開度: 区間後半 60% の中央値（ランプ区間の影響を避ける）
    held = float(np.median(opening[int(0.4 * n) :]))
    if held < SEG_MIN_OPENING_PCT:
        return None

    v_start = float(speed[0])
    v_steady = float(np.mean(speed[-max(3, n // 10) :]))
    rise = v_steady - v_start
    if rise < MIN_RISE_KMH:
        return None

    # 定常ゲイン: 保持開度あたりの車速増分
    k = rise / held
    if k <= 0.0:
        return None

    # むだ時間: 区間開始から車速が ONSET_RISE_KMH を超えるまで
    onset_idx = 0
    for i in range(n):
        if speed[i] - v_start >= ONSET_RISE_KMH:
            onset_idx = i
            break
    theta = min(onset_idx * dt, L_MAX_S)

    # 時定数: 残差 r = v_steady - v の対数線形フィット（r = r0·exp(-t/τ)）
    residual = v_steady - speed[onset_idx:]
    floor = 0.1 * rise
    mask = residual > floor
    if int(np.count_nonzero(mask)) < 3:
        return None
    t_rel = np.arange(len(residual))[mask] * dt
    ln_r = np.log(residual[mask])
    slope = float(np.polyfit(t_rel, ln_r, 1)[0])
    if slope >= 0.0:
        return None
    tau = -1.0 / slope
    tau = max(TAU_MIN_S, min(TAU_MAX_S, tau))

    return k, tau, theta


def _find_accel_segments(
    speed: np.ndarray, accel: np.ndarray, brake: np.ndarray, brake_db: float
) -> list[tuple[int, int]]:
    """ブレーキオフでアクセルを踏んでいる連続区間 (start, end) のリストを返す。"""
    active = (accel >= SEG_MIN_OPENING_PCT) & (brake < brake_db)
    segments: list[tuple[int, int]] = []
    start: int | None = None
    for i, on in enumerate(active):
        if on and start is None:
            start = i
        elif not on and start is not None:
            segments.append((start, i))
            start = None
    if start is not None:
        segments.append((start, len(active)))
    return segments


def identify_fopdt(logs: list[DriveLog], profile: VehicleProfile) -> FOPDT | None:
    """連続走行ログ（学習運転）からアクセル側 FOPDT を同定する。

    アクセル一定保持区間（ブレーキオフ・開度 >= SEG_MIN_OPENING_PCT・車速がプラトーへ）を
    抽出し、各区間の (K, τ, L) を中央値集約する。区間が不足する場合は None を返す
    （呼び出し元は既存ゲインを保持する）。
    """
    brake_db = profile.feedforward_params.brake_deadband_pct
    ks: list[float] = []
    taus: list[float] = []
    thetas: list[float] = []

    for session_logs in _group_by_session(logs):
        if len(session_logs) < SEG_MIN_SAMPLES:
            continue
        speed = np.clip(
            np.array([lg.actual_speed_kmh for lg in session_logs], dtype=float), 0.0, None
        )
        accel = np.array([lg.accel_opening for lg in session_logs], dtype=float)
        brake = np.array([lg.brake_opening for lg in session_logs], dtype=float)

        epochs = np.array([lg.timestamp.timestamp() for lg in session_logs])
        d = np.diff(epochs)
        d = d[d > 0.0]
        dt = float(np.median(d)) if len(d) > 0 else DEFAULT_DT_S
        if dt <= 0.0:
            dt = DEFAULT_DT_S

        for s, e in _find_accel_segments(speed, accel, brake, brake_db):
            fit = _segment_fopdt(speed[s:e], accel[s:e], dt)
            if fit is None:
                continue
            k, tau, theta = fit
            ks.append(k)
            taus.append(tau)
            thetas.append(theta)

    if len(ks) < MIN_SEGMENTS_FOR_ID:
        _logger.info(
            "FOPDT 同定をスキップ: 有効区間 %d < %d", len(ks), MIN_SEGMENTS_FOR_ID
        )
        return None

    return FOPDT(
        k=float(np.median(ks)),
        tau=float(np.median(taus)),
        theta=float(np.median(thetas)),
    )


def compute_pid_gains_simc(
    fopdt: FOPDT, profile: VehicleProfile, tau_c_factor: float = 0.5
) -> PIDGains:
    """FOPDT から SIMC（Skogestad）則で PI ゲインを算出する（Kd は初期 0）。

    閉ループ時定数 τc は安定性ノブ。τc = max(L, tau_c_factor·τ) とし、大きいほど
    ロバスト（ゲイン低下）。CAN 車速の量子化ノイズに対する微分の暴れを避けるため
    Kd は 0 とし、必要なら閉ループ絞り込みで導入する。

    Args:
        fopdt: 同定済みプラントモデル
        profile: 対象プロファイル（将来のクランプ調整用に受け取る）
        tau_c_factor: 閉ループ時定数の τ に対する比率
    """
    k = fopdt.k
    tau = fopdt.tau
    theta = fopdt.theta
    if k <= 0.0:
        # 非物理な同定結果。安全側に既存ゲインを返す。
        return profile.pid_gains

    tau_c = max(theta, tau_c_factor * tau)
    denom = tau_c + theta
    kc = (1.0 / k) * tau / denom if denom > 0.0 else 0.0
    tau_i = min(tau, 4.0 * denom)

    kp = kc
    ki = kc / tau_i if tau_i > 0.0 else 0.0

    kp = max(KP_MIN, min(KP_MAX, kp))
    ki = max(KI_MIN, min(KI_MAX, ki))

    return PIDGains(kp=kp, ki=ki, kd=0.0)


# ── 閉ループ検証/絞り込み ───────────────────────────────────────────────
TUNING_MODE_ID: str = "pid-tune"
# 規定パターンの各速度帯（max_speed に対する比率）。
_TRAJ_HI_RATIO: float = 0.6
_TRAJ_HI2_RATIO: float = 0.8
_TRAJ_LO_RATIO: float = 0.3
# 各保持区間長 [s]（加減速は max_decel_g から算出するため固定なのは保持のみ）。
_TRAJ_HOLD_S: float = 8.0
_TRAJ_HOLD_LO_S: float = 6.0
# 1G を km/h/s へ換算（learning_loop.G_TO_KMHS と同一）。
G_TO_KMHS: float = 9.81 * 3.6
# 規定パターンの加減速レートは上限Gのこの割合を目標とする（PID オーバーシュート余裕）。
DECEL_MARGIN: float = 0.8
# 加減速レートの下限 [km/h/s]（max_decel_g が極小でもパターンが破綻しないための床）。
MIN_RATE_KMHS: float = 2.0
# コスト無効値（サンプルなし・データ欠損）。最良として選ばれない十分大きな値。
INVALID_COST: float = 1.0e9


def build_tuning_trajectory(profile: VehicleProfile) -> DrivingMode:
    """PID 適合用の規定速度パターン（加速・保持・再加速・減速・保持・停止）を生成する。

    プロファイルの安全包絡を厳守する:
    - 全速度点は max_speed 以内。
    - 加減速区間のレートは `DECEL_MARGIN × max_decel_g`（≤ 上限G）。減速区間長を
      この目標レートから算出することで、基準軌跡が上限Gを超えないことを保証する。

    DB には永続化せず、メモリ上の DrivingMode として閉ループ走行に渡す。
    """
    v_max = profile.max_speed
    v_hi = min(_TRAJ_HI_RATIO * v_max, v_max)
    v_hi2 = min(_TRAJ_HI2_RATIO * v_max, v_max)
    v_lo = _TRAJ_LO_RATIO * v_max
    rate = max(DECEL_MARGIN * profile.max_decel_g * G_TO_KMHS, MIN_RATE_KMHS)  # [km/h/s]

    # 加速→保持→再加速→保持→減速→保持→停止。加減速区間は rate から所要時間を算出。
    t = 0.0
    points: list[tuple[float, float]] = [(0.0, 0.0)]
    t += v_hi / rate
    points.append((t, v_hi))
    t += _TRAJ_HOLD_S
    points.append((t, v_hi))
    t += (v_hi2 - v_hi) / rate
    points.append((t, v_hi2))
    t += _TRAJ_HOLD_S
    points.append((t, v_hi2))
    t += (v_hi2 - v_lo) / rate
    points.append((t, v_lo))
    t += _TRAJ_HOLD_LO_S
    points.append((t, v_lo))
    t += v_lo / rate
    points.append((t, 0.0))

    reference = [SpeedPoint(time_s=tt, speed_kmh=min(s, v_max)) for tt, s in points]
    return DrivingMode(
        id=TUNING_MODE_ID,
        name="PID自動適合パターン",
        description="PID 自動適合用の規定走行パターン（加速・保持・減速、上限G/最高車速厳守）",
        reference_speed=reference,
        total_duration=points[-1][0],
        max_speed=v_max,
        created_at=datetime.now(tz=UTC),
    )


def tuning_cost(kpi_summary: dict[str, float]) -> float:
    """走行後の KPI サマリを単一のスカラーコストへ写像する（小さいほど良い）。

    製品プライマリー KPI（kpi_monitor）の各しきい値で正規化し、ハード上限違反は
    支配的なペナルティとして重み付けする。サンプルが無い走行は INVALID_COST を返す。
    """
    if kpi_summary.get("n_samples", 0.0) <= 0.0:
        return INVALID_COST
    p95 = kpi_summary.get("p95_kmh", 0.0)
    max_dev = kpi_summary.get("max_abs_deviation_kmh", 0.0)
    reversal = kpi_summary.get("reversal_max_per_5s", 0.0)
    hard = kpi_summary.get("hard_limit_violations", 0.0)
    return (
        p95 / KPI_P95_LIMIT_KMH
        + 0.5 * max_dev / KPI_HARD_LIMIT_KMH
        + 0.2 * reversal / KPI_REVERSAL_WINDOW_S
        + 100.0 * hard
    )


class CoordinateDescentTuner:
    """KPI コストを最小化する座標降下（コンパス探索）チューナー。

    走行（評価）はハードに依存するため本クラスからは切り離し、呼び出し元が
    `next_candidate()` で候補ゲインを受け取り走行・コスト算出し `report()` で返す
    注入式とする。これによりハード無しで収束挙動を単体テストできる。

    各ラウンドで best を中心に kp/ki/kd を ±ステップ探索する。ラウンド内で改善が
    あれば best を更新して即再センタリング、無ければステップを半減する。ステップが
    min_step_frac 未満、または max_runs 到達で停止する。
    """

    # ステップの最小スケール（値が 0 でも動けるようにする基準量）。
    _BASE: dict[str, float] = {"kp": 1.0, "ki": 0.1, "kd": 0.1}

    def __init__(
        self,
        initial: PIDGains,
        *,
        max_runs: int = 15,
        init_step_frac: float = 0.3,
        min_step_frac: float = 0.05,
    ) -> None:
        self._best = initial
        self._best_cost = math.inf
        self._max_runs = max_runs
        self._step = init_step_frac
        self._min_step = min_step_frac
        self._runs = 0
        self._pending: list[PIDGains] = [initial]  # まずベースラインを測定
        self._improved_this_round = False

    def _gen_round(self) -> list[PIDGains]:
        cands: list[PIDGains] = []
        for p in ("kp", "ki", "kd"):
            cur = getattr(self._best, p)
            scale = abs(cur) + self._BASE[p]
            for d in (1.0, -1.0):
                val = max(0.0, cur + d * self._step * scale)
                cands.append(replace(self._best, **{p: val}))
        return cands

    def next_candidate(self) -> PIDGains | None:
        """次に評価すべき候補ゲインを返す。停止条件に達したら None。"""
        if self._runs >= self._max_runs:
            return None
        if not self._pending:
            if not self._improved_this_round:
                self._step *= 0.5
                if self._step < self._min_step:
                    return None
            self._improved_this_round = False
            self._pending = self._gen_round()
        return self._pending[0]

    def report(self, gains: PIDGains, cost: float) -> None:
        """候補ゲインの走行コストを報告する。"""
        self._runs += 1
        if self._pending:
            self._pending.pop(0)
        if cost < self._best_cost:
            self._best_cost = cost
            self._best = gains
            self._improved_this_round = True
            self._pending = []  # best 更新につき現ラウンドを破棄して再センタリング

    @property
    def best(self) -> PIDGains:
        return self._best

    @property
    def best_cost(self) -> float:
        return self._best_cost

    @property
    def runs(self) -> int:
        return self._runs
