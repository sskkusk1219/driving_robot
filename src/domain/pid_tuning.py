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

from src.domain.control.conversions import G_TO_KMHS
from src.domain.control.kpi_monitor import (
    KPI_HARD_LIMIT_KMH,
    KPI_P95_LIMIT_KMH,
    KPI_REVERSAL_LIMIT_PER_WINDOW,
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

# ── PID 先読み補償（pid_preview_s）クランプ ─────────────────────────────
# PID フィードバックのみに適用する基準軌跡の前倒し秒数。FF は now-frame で動くため
# ここでの前倒しは FB ループのむだ時間補償に限定される。二重補償による系統偏差を
# 避けるため上限を 1.0s に制限し、座標降下がそれ以上を探索しないようにする。
PID_PREVIEW_MIN_S: float = 0.0
PID_PREVIEW_MAX_S: float = 1.0


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


def initial_preview_from_fopdt(_fopdt: FOPDT) -> float:
    """PID 先読み補償(pid_preview_s)の初期値を返す。常に 0.0。

    旧実装は FOPDT のむだ時間 θ をそのまま初期値にしていたが、これは FF が構造的に
    内蔵するむだ時間補償と二重になり、実車速が基準を θ 秒だけ先行する系統偏差を生む
    （KPI 違反の主因）。PID 先読みは now-frame からの純粋な追加ノブとして 0.0 から
    始め、必要なら座標降下が [0, PID_PREVIEW_MAX_S] の範囲で探索する。引数は
    シグネチャ互換のため受けるが未使用（θ は fopdt_theta として別途保存される）。
    """
    return 0.0


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
    # brake_db（不感帯）以下は「ブレーキオフ」とみなす。strict < だと brake_db=0.0
    # （合法値）のとき brake>=0 で常に False になり区間が一切見つからない（D5 レビュー指摘）。
    active = (accel >= SEG_MIN_OPENING_PCT) & (brake <= brake_db)
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
        _logger.info("FOPDT 同定をスキップ: 有効区間 %d < %d", len(ks), MIN_SEGMENTS_FOR_ID)
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
# 規定パターンの各速度帯（max_speed に対する比率）。0.95 巡航帯は WLTP 巡航（131km/h≒
# 0.94×140）を包含し、125-140km/h×微小開度のサンプルを供給する（B-7-1: 旧 0.8 比=112 は
# 巡航帯に届かず、この帯のモデルデータ欠損＝高速域チャタの根本原因だった）。
_TRAJ_LAUNCH_RATIO: float = 0.6  # 発進加速の到達速度（例 84）
_TRAJ_MID_RATIO: float = 0.8  # 中速保持（例 112）
_TRAJ_CRUISE_RATIO: float = 0.95  # 巡航保持（例 133）。WLTP 巡航帯を含む
_TRAJ_LO_RATIO: float = 0.3  # 減速後の低速保持（例 42）
# 各保持区間長 [s]（加減速は max_decel_g から算出するため固定なのは保持のみ）。
_TRAJ_HOLD_S: float = 6.0
_TRAJ_HOLD_CRUISE_S: float = 10.0  # 巡航保持は長め（高速×微小開度データ供給＋チャタ帯評価）
_TRAJ_HOLD_LO_S: float = 6.0
# G_TO_KMHS(1Gをkm/h/sへ換算)は src.domain.control.conversions の共通定数を使う（A2 レビュー指摘）。
# 加減速レートは `係数 × max_decel_g`。区間ごとに高率/低率を混在させ、単一レート（旧実装）では
# 得られなかった多様な過渡（WLTP のランプは 0.5〜3.8km/h/s）を適合走行に含める（B-7-1）。
# 全係数 ≤ _TRAJ_RATE_FAST(0.8) ≤ 1.0 なので基準軌跡は上限Gを超えない。
_TRAJ_RATE_FAST: float = 0.8  # 発進・主減速（きびきび）
_TRAJ_RATE_SLOW: float = 0.35  # 中速→巡航への緩加速（定常付近の微小開度応答を採る）
_TRAJ_RATE_STOP: float = 0.4  # 低速→停止の緩減速
# 旧名互換（外部参照・過去のレビュー引用のため残置）。主減速レートの係数。
DECEL_MARGIN: float = _TRAJ_RATE_FAST
# 加減速レートの下限 [km/h/s]（max_decel_g が極小でもパターンが破綻しないための床）。
MIN_RATE_KMHS: float = 2.0
# コスト無効値（サンプルなし・データ欠損）。最良として選ばれない十分大きな値。
INVALID_COST: float = 1.0e9


def build_tuning_trajectory(profile: VehicleProfile) -> DrivingMode:
    """PID 適合用の規定速度パターン（発進・保持・巡航・減速・保持・停止）を生成する。

    プロファイルの安全包絡を厳守する:
    - 全速度点は max_speed 以内。
    - 各加減速区間のレートは `係数 × max_decel_g`（係数 ≤ 0.8 ≤ 1.0）。区間長をこの目標レート
      から算出することで、基準軌跡が上限Gを超えないことを保証する。

    構成（B-7-1）: 0 →[fast] 0.6v →hold→[slow] 0.8v →hold→[slow] 0.95v →hold(巡航,長め)→
    [fast] 0.3v →hold→[stop] 0。0.95v の巡航保持で WLTP 巡航帯（125-140km/h）の微小開度応答を
    採取し、高速域チャタの原因だったモデルデータ欠損を埋める。加減速レートを区間で振り分け、
    多様なランプ過渡を適合対象に含める。

    DB には永続化せず、メモリ上の DrivingMode として閉ループ走行に渡す。
    """
    v_max = profile.max_speed
    v_launch = min(_TRAJ_LAUNCH_RATIO * v_max, v_max)
    v_mid = min(_TRAJ_MID_RATIO * v_max, v_max)
    v_cruise = min(_TRAJ_CRUISE_RATIO * v_max, v_max)
    v_lo = _TRAJ_LO_RATIO * v_max

    def rate(coeff: float) -> float:
        return max(coeff * profile.max_decel_g * G_TO_KMHS, MIN_RATE_KMHS)  # [km/h/s]

    # 加速→保持→緩加速→保持→緩加速→巡航保持→主減速→保持→緩停止。
    # 各加減速は (Δ速度 / 区間レート) で所要時間を算出する。
    t = 0.0
    points: list[tuple[float, float]] = [(0.0, 0.0)]
    t += v_launch / rate(_TRAJ_RATE_FAST)
    points.append((t, v_launch))
    t += _TRAJ_HOLD_S
    points.append((t, v_launch))
    t += (v_mid - v_launch) / rate(_TRAJ_RATE_SLOW)
    points.append((t, v_mid))
    t += _TRAJ_HOLD_S
    points.append((t, v_mid))
    t += (v_cruise - v_mid) / rate(_TRAJ_RATE_SLOW)
    points.append((t, v_cruise))
    t += _TRAJ_HOLD_CRUISE_S
    points.append((t, v_cruise))
    t += (v_cruise - v_lo) / rate(_TRAJ_RATE_FAST)
    points.append((t, v_lo))
    t += _TRAJ_HOLD_LO_S
    points.append((t, v_lo))
    t += v_lo / rate(_TRAJ_RATE_STOP)
    points.append((t, 0.0))

    reference = [SpeedPoint(time_s=tt, speed_kmh=min(s, v_max)) for tt, s in points]
    return DrivingMode(
        id=TUNING_MODE_ID,
        name="PID自動適合パターン",
        description="PID 自動適合用の規定走行パターン（発進・保持・巡航・減速、上限G厳守）",
        reference_speed=reference,
        total_duration=points[-1][0],
        max_speed=v_max,
        created_at=datetime.now(tz=UTC),
    )


_SEG_GRID_S: float = 0.1  # 代表区間抽出の密サンプリング周期
_SEG_WINDOW_BUDGET_FRAC: float = 0.7  # 代表窓に割く時間予算の割合（残りはランプ余裕）
_SEG_ACTIVITY_SMOOTH_S: float = 2.0  # アクティビティ平滑化の移動平均窓 [s]


def _merge_windows(windows: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """時間窓 [(start,end),...] を開始時刻順に並べ、重複/隣接をマージする。"""
    if not windows:
        return []
    ordered = sorted(windows)
    merged = [ordered[0]]
    for ws, we in ordered[1:]:
        ls, le = merged[-1]
        if ws <= le:
            merged[-1] = (ls, max(le, we))
        else:
            merged.append((ws, we))
    return merged


def build_tuning_trajectory_from_mode(
    mode: DrivingMode, max_duration_s: float = 120.0
) -> DrivingMode:
    """本番モードから代表区間を切り出した PID 適合用トラジェクトリを生成する。

    座標降下の評価走行を本番モード（例: WLTP_ExHi 323s/131km/h）そのもので行うと
    1候補あたりの走行が長すぎて 19 回反復が非現実的になる。かといって規定パターン
    （42s/最高112km/h、`build_tuning_trajectory`）で適合すると本番モードと速度域・過渡が
    異なり適合結果が転移しない（実機 session 717e2609 で確認）。本関数は本番モードの
    「最高速度を含む窓」と「加減速アクティビティが最大の窓」を切り出し、先頭 0→接続速度・
    窓間・末尾→0 を安全なランプで繋いだ ≤ max_duration_s の代表走行を作る。

    安全包絡: 切り出した窓は本番モードのスライスそのものなので max_speed / 減速G を
    自動的に満たす。追加するランプの傾きはモード自身の最大 |dv/dt| 以内に収める
    （＝モードが既に含む最急過渡より急にはしない）ため、全体が元モードの包絡内に留まる。

    mode がすでに max_duration_s 以下、または点数不足なら（名前だけ適合用に替えて）そのまま返す。
    """
    src = mode.reference_speed
    if len(src) < 2 or mode.total_duration <= max_duration_s:
        return DrivingMode(
            id=TUNING_MODE_ID,
            name="PID自動適合パターン(本番モード)",
            description=(
                f"本番モード '{mode.name}' をそのまま使用（既に {max_duration_s:.0f}s 以下）"
            ),
            reference_speed=list(src),
            total_duration=mode.total_duration,
            max_speed=mode.max_speed,
            created_at=datetime.now(tz=UTC),
        )

    src_t = np.array([p.time_s for p in src], dtype=float)
    src_v = np.array([p.speed_kmh for p in src], dtype=float)
    total = float(src_t[-1])
    grid_t = np.arange(0.0, total, _SEG_GRID_S)
    grid_v = np.interp(grid_t, src_t, src_v)

    # ランプ率: モード自身の最大 |dv/dt| 以内（安全包絡内）に収める。
    seg_rates = np.abs(np.diff(src_v) / np.diff(src_t))
    ramp_rate = max(MIN_RATE_KMHS, float(np.max(seg_rates)))
    v_max = mode.max_speed

    # ランプ余裕を確保して窓予算を決める。最悪ケース（先頭+末尾+窓間で最大3本、各 v_max 分）を
    # 見込んで予約し、残りを2窓へ等分する。予算が枯れる小モードは単一窓へフォールバック。
    ramp_reserve = 3.0 * v_max / ramp_rate
    win_budget = min(_SEG_WINDOW_BUDGET_FRAC * max_duration_s, max_duration_s - ramp_reserve)
    win_budget = max(win_budget, 0.0)

    # 窓中心: 最高速度点 と 平滑化アクティビティ最大点。
    activity = np.abs(np.gradient(grid_v, grid_t))
    k = max(1, int(_SEG_ACTIVITY_SMOOTH_S / _SEG_GRID_S))
    smooth = np.convolve(activity, np.ones(k) / k, mode="same")
    c_speed = float(grid_t[int(np.argmax(grid_v))])
    c_activity = float(grid_t[int(np.argmax(smooth))])

    half = win_budget / 4.0  # 2窓 × 半幅2つ
    raw_windows = [
        (max(0.0, c - half), min(total, c + half)) for c in (c_speed, c_activity) if half > 0.0
    ]
    windows = _merge_windows(raw_windows) or [
        (max(0.0, c_speed - half), min(total, c_speed + half))
    ]

    # 組み立て: 0 発進 → 各窓（間はランプ）→ 0 停止。窓内は元モードの相対タイミング/傾きを保つ。
    points: list[tuple[float, float]] = [(0.0, 0.0)]
    t = 0.0
    prev_v = 0.0
    for ws, we in windows:
        v_start = float(np.interp(ws, src_t, src_v))
        t += abs(v_start - prev_v) / ramp_rate
        points.append((t, v_start))
        win_start_t = t
        interior = [(float(tp), float(vp)) for tp, vp in zip(src_t, src_v) if ws < tp < we]
        for tp, vp in interior:
            points.append((win_start_t + (tp - ws), vp))
        v_end = float(np.interp(we, src_t, src_v))
        points.append((win_start_t + (we - ws), v_end))
        t = win_start_t + (we - ws)
        prev_v = v_end
    # 末尾: 0 へ減速
    t += abs(prev_v) / ramp_rate
    points.append((t, 0.0))

    # 時間が重複する点（ゼロ長ランプ＝窓が速度0で始まる/窓間の接続速度が一致する等）を畳んで
    # 時間を狭義単調増加にする。同時刻に複数速度がある場合は後の速度（＝目標）を採用する。
    cleaned: list[tuple[float, float]] = [points[0]]
    for tt, s in points[1:]:
        if tt - cleaned[-1][0] > 1e-6:
            cleaned.append((tt, s))
        else:
            cleaned[-1] = (cleaned[-1][0], s)

    # max_speed 以内・非負を保証してから SpeedPoint 化。
    reference = [SpeedPoint(time_s=tt, speed_kmh=min(max(0.0, s), v_max)) for tt, s in cleaned]
    return DrivingMode(
        id=TUNING_MODE_ID,
        name="PID自動適合パターン(本番モード代表区間)",
        description=(
            f"本番モード '{mode.name}' の最高速度窓＋加減速最大窓を切り出した代表走行"
            f"（≤{max_duration_s:.0f}s、元モードの安全包絡内）"
        ),
        reference_speed=reference,
        total_duration=cleaned[-1][0],
        max_speed=v_max,
        created_at=datetime.now(tz=UTC),
    )


VERIFY_MODE_ID: str = "verify"
# 検証専用パターン: 巡航保持長・停止保持長 [s]。
_VERIFY_MAX_CRUISE_HOLD_S: float = 12.0  # 最高速巡航の保持（≥10s で微小開度域を採取）
_VERIFY_CRUISE_HOLD_S: float = 7.0  # 中間巡航の保持
_VERIFY_STOP_HOLD_S: float = 6.0  # 完全停止の保持（クリープ発進・停車保持の検証）
# 検証パターンの巡航代表点の目標数（cap 以下で分布から選ぶ）。
_VERIFY_N_CRUISE_POINTS: int = 4


def _mode_ramp_rates(modes: list[DrivingMode]) -> tuple[float, float, float]:
    """全モードの加減速ランプ率 [km/h/s] の (p10, p50, p90) を返す（>2s の区間のみ）。"""
    rates: list[float] = []
    for mode in modes:
        pts = mode.reference_speed
        if len(pts) < 2:
            continue
        t = np.array([p.time_s for p in pts], dtype=float)
        v = np.array([p.speed_kmh for p in pts], dtype=float)
        if t[-1] <= t[0]:
            continue
        grid_t = np.arange(t[0], t[-1], 0.1)
        grid_v = np.interp(grid_t, t, v)
        a = np.convolve(np.gradient(grid_v, grid_t), np.ones(10) / 10, mode="same")
        mask = np.abs(a) > 0.15
        rates.extend(np.abs(a[mask]).tolist())
    if not rates:
        return 0.5, 1.7, 3.5
    arr = np.array(rates)
    return (
        float(np.percentile(arr, 10)),
        float(np.percentile(arr, 50)),
        float(np.percentile(arr, 90)),
    )


def _mode_cruise_speeds(modes: list[DrivingMode], cap: float) -> list[float]:
    """全モードの巡航速度（|a|<0.2 が 3s 以上続く区間の中央値、cap 以下）の代表点を返す。"""
    cruise: list[float] = []
    for mode in modes:
        pts = mode.reference_speed
        if len(pts) < 2:
            continue
        t = np.array([p.time_s for p in pts], dtype=float)
        v = np.array([p.speed_kmh for p in pts], dtype=float)
        if t[-1] <= t[0]:
            continue
        grid_t = np.arange(t[0], t[-1], 0.1)
        grid_v = np.interp(grid_t, t, v)
        a = np.convolve(np.gradient(grid_v, grid_t), np.ones(10) / 10, mode="same")
        flat = (np.abs(a) < 0.2) & (grid_v > 5.0)
        i = 0
        while i < len(flat):
            if flat[i]:
                j = i
                while j < len(flat) and flat[j]:
                    j += 1
                if (j - i) * 0.1 >= 3.0:
                    med = float(np.median(grid_v[i:j]))
                    if med <= cap - 2.0:
                        cruise.append(med)
                i = j
            else:
                i += 1
    if not cruise:
        return [0.3 * cap, 0.5 * cap, 0.7 * cap]
    # cap 以下の巡航速度を等分位で代表点に絞る（昇順・重複除去）。
    arr = np.sort(np.array(cruise))
    qs = np.linspace(100.0 / (2 * _VERIFY_N_CRUISE_POINTS), 100.0, _VERIFY_N_CRUISE_POINTS)
    reps = sorted({round(float(np.percentile(arr, q)), 0) for q in qs})
    return [r for r in reps if r <= cap - 2.0]


def build_verification_trajectory(
    modes: list[DrivingMode], profile: VehicleProfile, budget_s: float = 300.0
) -> DrivingMode:
    """学習サイクルの検証走行（VERIFY）用の専用パターンを生成する。

    登録全モードの包絡（最高速度・巡航速度分布・加減速率分布・停止）を約 budget_s に合成する。
    ユーザー選択なし・自動生成。対象モードそのもののリハーサルはしない（12_AMA=87分級で
    サイクルが破綻するため）。プロファイルの安全包絡（≤max_speed・≤0.8×max_decel_g・
    0 始端 0 終端）を厳守する。cap 超の巡航点は自動除外（max_speed=100 なら最高速巡航=100）。

    構成: 0→低速巡航→（中間巡航を昇順・レート多様）→最高速巡航(≥10s)→減速（緩/エンブレ/
    ハードブレーキ）→完全停止・停止保持→再発進→最終停止。
    """
    cap = min(
        max((m.max_speed for m in modes), default=profile.max_speed), profile.max_speed
    )
    cap = max(cap, 10.0)
    r_p10, r_p50, r_p90 = _mode_ramp_rates(modes)
    rate_cap = 0.8 * profile.max_decel_g * G_TO_KMHS
    accel_lo = max(MIN_RATE_KMHS, min(r_p10, rate_cap))
    accel_mid = max(MIN_RATE_KMHS, min(r_p50, rate_cap))
    accel_hi = max(MIN_RATE_KMHS, min(r_p90, rate_cap))
    cruise_pts = _mode_cruise_speeds(modes, cap)

    # (目標速度, ランプ率) の列を組み立てる。保持は別途付与する。
    def hold_for(v: float) -> float:
        return _VERIFY_MAX_CRUISE_HOLD_S if v >= cap - 2.0 else _VERIFY_CRUISE_HOLD_S

    # 発進: 低速巡航（クリープ帯を抜ける緩加速）
    launch_v = min(20.0, 0.2 * cap)
    segments: list[tuple[float, float, float]] = [(launch_v, accel_lo, _VERIFY_CRUISE_HOLD_S)]
    # 中間巡航を昇順に、レートを lo/mid/hi で回して多様化
    rates_cycle = [accel_mid, accel_hi, accel_mid, accel_lo]
    for idx, cv in enumerate(cruise_pts):
        segments.append((cv, rates_cycle[idx % len(rates_cycle)], hold_for(cv)))
    # 最高速巡航（≥10s）
    segments.append((cap, accel_mid, _VERIFY_MAX_CRUISE_HOLD_S))
    # 減速フェーズ: 緩減速（アクセル調整域）→ ハードブレーキ（p90）→ 中速巡航
    mid_cruise = cruise_pts[len(cruise_pts) // 2] if cruise_pts else 0.5 * cap
    segments.append((min(0.85 * cap, cap), accel_lo, _VERIFY_CRUISE_HOLD_S))  # 高速緩減速
    segments.append((mid_cruise, accel_hi, _VERIFY_CRUISE_HOLD_S))  # ハードブレーキ
    segments.append((0.3 * cap, accel_mid, _VERIFY_CRUISE_HOLD_S))  # 中率減速
    # 完全停止・停止保持 → 再発進 → 最終停止
    segments.append((0.0, accel_mid, _VERIFY_STOP_HOLD_S))
    segments.append((min(0.4 * cap, cap), accel_mid, _VERIFY_CRUISE_HOLD_S))
    segments.append((0.0, accel_hi, 0.0))

    # 予算 budget_s に収める: ランプ時間は安全（レート）に直結するため保持のみ圧縮する。
    # 最高速巡航は ≥10s・停止保持は ≥4s を床にして役割（微小開度採取・停車保持）を守る。
    ramp_total = 0.0
    v = 0.0
    for target, rate, _hold in segments:
        ramp_total += abs(min(max(0.0, target), cap) - v) / max(rate, MIN_RATE_KMHS)
        v = min(max(0.0, target), cap)
    hold_total = sum(h for _, _, h in segments)
    hold_scale = 1.0
    if ramp_total + hold_total > budget_s and hold_total > 0.0:
        hold_scale = max(0.0, (budget_s - ramp_total)) / hold_total

    points: list[tuple[float, float]] = [(0.0, 0.0)]
    t = 0.0
    v = 0.0
    for target, rate, hold in segments:
        target = min(max(0.0, target), cap)
        t += abs(target - v) / max(rate, MIN_RATE_KMHS)
        points.append((t, target))
        v = target
        if hold > 0.0:
            scaled = hold * hold_scale
            if hold >= _VERIFY_MAX_CRUISE_HOLD_S:
                scaled = max(scaled, 10.0)  # 最高速巡航は最低 10s
            elif hold >= _VERIFY_STOP_HOLD_S:
                scaled = max(scaled, 4.0)  # 停止保持は最低 4s
            t += scaled
            points.append((t, target))

    reference = [SpeedPoint(time_s=tt, speed_kmh=min(max(0.0, s), cap)) for tt, s in points]
    return DrivingMode(
        id=VERIFY_MODE_ID,
        name="検証専用パターン",
        description=(
            f"登録全モードの包絡（最高{cap:.0f}km/h・巡航{len(cruise_pts)}点・停止1回）を"
            f"合成した検証走行（総{points[-1][0]:.0f}s、安全包絡内）"
        ),
        reference_speed=reference,
        total_duration=points[-1][0],
        max_speed=cap,
        created_at=datetime.now(tz=UTC),
    )


# ペダル活動度（アクセル ON-OFF 立ち上がり回数/min）の重み（B-7-4）。実機ゲートB 2走行
# （session ff431d46: 21.7回/min、343e80c1: 16.7回/min）で机上校正した。この重みだと
# ハンチング時の寄与は 1.7〜2.2 で p95 項（2.25〜2.5）と同オーダーになり「滑らかさ」を最適化
# 対象に含める一方、支配項の reversal（21〜24）や p95 を上書きしない。滑らかな解（〜5回/min）
# では 0.5 程度に留まり、20→5回/min の改善が p95 換算 0.3km/h 相当の価値になる。
PEDAL_ACTIVITY_WEIGHT: float = 0.1

# アクセル⇔ブレーキ交互踏み（不要切替）回数/min の重み（B-8-4 移管）。実機 2 セッション
# （e3773d9c: 39.4回/min のシーソー、2af4c09b: 18.8回/min）で机上校正した。0.5 だと
# シーソー走行の寄与は 9〜20 と大きく、滑らかな解（軌跡要求 1〜3回/min → 0.5〜1.5）では
# 支配的でない。ゲイン候補間で軌跡要求切替は共通の定数オフセットなので降下方向を歪めず、
# 過剰なペダル踏み替えを起こすゲインだけを罰する。
PEDAL_SEESAW_WEIGHT: float = 0.5


def tuning_cost(kpi_summary: dict[str, float]) -> float:
    """走行後の KPI サマリを単一のスカラーコストへ写像する（小さいほど良い）。

    製品プライマリー KPI（kpi_monitor）の各しきい値で正規化する。サンプルが無い走行は
    INVALID_COST を返す。

    ペダル活動度項（B-7-4）: アクセルの 0→非0 立ち上がり回数/min に `PEDAL_ACTIVITY_WEIGHT`
    を掛けて加える。偏差 KPI を満たしても人間の運転よりペダル ON-OFF が過剰（実機で高速域
    36回/min のハンチング）な解をチューナーが選ばないよう「滑らかさ」を最適化対象に含める。
    summary にペダル情報が無い（accel_opening 未計測）走行では 0 で影響しない。

    reversal は KPI 上限値（KPI_REVERSAL_LIMIT_PER_WINDOW=1回/5s窓）で正規化する。
    窓長 KPI_REVERSAL_WINDOW_S(=5.0s) で割ると単位が秒になってしまい、KPI 限界値
    ちょうどの走行（reversal=1）でも 0.2 の 1/5（0.04）しか計上されず振動ペナルティが
    5倍過小評価される（D4 レビュー指摘）。

    reversal 重みは B-6 で 0.2→1.0 に引き上げた。実機ゲートB（session ff431d46）で
    チューナーが規定パターン上の振動（14回/5s窓）を受容し、本番走行で符号反転 23回/5s窓
    （KPI 上限の 23 倍）が残った。旧 0.2 では 0.2×14=2.8 が p95 項に対して小さく、振動を
    許容する候補が最良に選ばれていた。1.0 にすると振動 KPI 超過を p95 と同格でペナルティする。

    ハード上限（1.0km/h 超）ペナルティは Stage B で「回数の不連続段差」から
    「超過量×時間の積分（連続）」へ置換した。旧 `100×hard違反回数` は座標降下に勾配を
    与えず、超過量が僅かに減っても回数が変わらなければコストが動かないため局所解で停滞し、
    stage2_cost≈643（≈6回違反残り）で収束していた（実機 session 717e2609/ccb3bf89 で確認）。
    連続項 `10×積分[km/h·s]` が「どれだけ・どれだけの間」超過したかに滑らかに反応して勾配を
    与え、`2×max_dev` が最悪スパイクを直接押し下げ、`1×(hard>0)` の小段差が「違反ゼロ達成」
    への離散インセンティブを残す。
    """
    if kpi_summary.get("n_samples", 0.0) <= 0.0:
        return INVALID_COST
    p95 = kpi_summary.get("p95_kmh", 0.0)
    max_dev = kpi_summary.get("max_abs_deviation_kmh", 0.0)
    reversal = kpi_summary.get("reversal_max_per_5s", 0.0)
    hard = kpi_summary.get("hard_limit_violations", 0.0)
    over_integral = kpi_summary.get("over_limit_integral_kmhs", 0.0)
    accel_on_per_min = kpi_summary.get("accel_on_per_min", 0.0)
    pedal_switch_per_min = kpi_summary.get("pedal_switch_per_min", 0.0)
    return (
        p95 / KPI_P95_LIMIT_KMH
        + 1.0 * reversal / KPI_REVERSAL_LIMIT_PER_WINDOW
        + 10.0 * over_integral
        + 2.0 * max_dev / KPI_HARD_LIMIT_KMH
        + (1.0 if hard > 0.0 else 0.0)
        + PEDAL_ACTIVITY_WEIGHT * accel_on_per_min
        + PEDAL_SEESAW_WEIGHT * pedal_switch_per_min
    )


@dataclass(frozen=True)
class TuningParams:
    """座標降下が探索する4パラメータ（PID ゲイン + PID 先読み補償秒数）。"""

    kp: float
    ki: float
    kd: float
    pid_preview_s: float = 0.0

    @property
    def gains(self) -> PIDGains:
        return PIDGains(kp=self.kp, ki=self.ki, kd=self.kd)

    @classmethod
    def from_profile(cls, profile: VehicleProfile) -> TuningParams:
        g = profile.pid_gains
        return cls(
            kp=g.kp,
            ki=g.ki,
            kd=g.kd,
            pid_preview_s=profile.dynamics_params.pid_preview_s,
        )


class CoordinateDescentTuner:
    """KPI コストを最小化する巡回座標降下（cyclic coordinate descent）チューナー。

    走行（評価）はハードに依存するため本クラスからは切り離し、呼び出し元が
    `next_candidate()` で候補パラメータを受け取り走行・コスト算出し `report()` で返す
    注入式とする。これによりハード無しで収束挙動を単体テストできる。

    kp → ki → kd → pid_preview_s の順に各座標を巡回する。座標ごとにまず `+` 候補を
    評価し、改善（cost < best_cost）すれば採用して直ちに次の座標へ進む。改善しなければ
    `−` 候補を試し、それでも改善しなければ次の座標へ進む（残り方向は捨てる）。クランプ後
    の値が現在の best と同値になる候補は走行させずスキップする。全座標を1巡して一度も
    改善がなければステップを半減する。ステップが min_step_frac 未満、または max_runs
    到達で停止する。
    """

    _PARAMS: tuple[str, ...] = ("kp", "ki", "kd", "pid_preview_s")
    # ステップの最小スケール（値が 0 でも動けるようにする基準量）。
    _BASE: dict[str, float] = {"kp": 1.0, "ki": 0.1, "kd": 0.5, "pid_preview_s": 0.25}
    # 各パラメータのクランプ範囲 (min, max)。
    _CLAMP: dict[str, tuple[float, float]] = {
        "kp": (KP_MIN, math.inf),
        "ki": (KI_MIN, math.inf),
        "kd": (0.0, math.inf),
        "pid_preview_s": (PID_PREVIEW_MIN_S, PID_PREVIEW_MAX_S),
    }

    def __init__(
        self,
        initial: TuningParams,
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
        self._baseline_done = False
        self._param_idx = 0
        self._direction_idx = 0
        self._improved_this_cycle = False

    def next_candidate(self) -> TuningParams | None:
        """次に評価すべき候補パラメータを返す。停止条件に達したら None。"""
        if self._runs >= self._max_runs:
            return None
        if not self._baseline_done:
            return self._best

        while True:
            if self._param_idx >= len(self._PARAMS):
                if not self._improved_this_cycle:
                    self._step *= 0.5
                    if self._step < self._min_step:
                        return None
                self._improved_this_cycle = False
                self._param_idx = 0
                self._direction_idx = 0
                continue
            if self._direction_idx >= 2:
                self._param_idx += 1
                self._direction_idx = 0
                continue

            p = self._PARAMS[self._param_idx]
            direction = (1.0, -1.0)[self._direction_idx]
            cur = getattr(self._best, p)
            scale = abs(cur) + self._BASE[p]
            lo, hi = self._CLAMP[p]
            val = max(lo, min(hi, cur + direction * self._step * scale))
            if val == cur:
                # クランプで best と同値になる候補は無駄走行なのでスキップ
                self._direction_idx += 1
                continue
            return replace(self._best, **{p: val})

    def report(self, params: TuningParams, cost: float) -> None:
        """候補パラメータの走行コストを報告する。"""
        self._runs += 1
        is_new_best = cost < self._best_cost
        if is_new_best:
            self._best_cost = cost
            self._best = params

        if not self._baseline_done:
            # ベースライン測定は座標巡回に数えない（kp から通常どおり開始する）。
            self._baseline_done = True
            return

        if is_new_best:
            self._improved_this_cycle = True
            self._param_idx += 1
            self._direction_idx = 0
        else:
            self._direction_idx += 1

    @property
    def best(self) -> TuningParams:
        return self._best

    @property
    def best_cost(self) -> float:
        return self._best_cost

    @property
    def runs(self) -> int:
        return self._runs
