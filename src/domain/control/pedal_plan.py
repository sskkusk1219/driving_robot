"""ペダル操作計画（ペダルプラン）のドメインモジュール。

人間のドライバーは基準軌跡を毎サイクル追いかけるのではなく、走り出す前に「どこで踏み、
どこで離し、どこでブレーキを踏むか」の骨格を決め、あとは少しずつ調整する。本モジュールは
その骨格を走行開始時にオフライン生成する。

構成（純ドメイン・I/O なし）:
  - PlanPhase   : DRIVE(駆動) / COAST(惰行=クリープ・エンジンブレーキ) / BRAKE(制動) /
                  STOP_HOLD(停車保持)
  - PedalPlan   : 時刻グリッド上のフェーズ列と名目 effort 列（effort_at / phase_at で参照）
  - PedalPlanner: 基準軌跡＋車両定数＋FF モデルから PedalPlan を生成

フェーズ分類は基準速度の必要加速度 a_req と車両の惰行加速度 a_coast(v)（クリープ／エンジン
ブレーキ）を比べて決める。微小フェーズはマージして「軌跡が要求しない不要なペダル切替」を
構造的に排除する。名目 effort は FF モデルを全軌跡でオフライン評価し、ゼロ位相ローパスで
滑らかにしてからフェーズ整合クランプする（DRIVE は非負・BRAKE は非正・COAST は 0）。

effort の符号は FF/PID/ILC と同じ（+: 加速 [%]、−: 制動 [%]）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum

import numpy as np

from src.domain.control.conversions import VEHICLE_STOP_SPEED_KMH
from src.domain.control.feedforward import FeedforwardController
from src.domain.control.ilc import zero_phase_lowpass
from src.models.driving_mode import DrivingMode
from src.models.profile import FeedforwardParams

# ── ペダルプラン既定パラメータ ─────────────────────────────────────────────
PLAN_DT_S: float = 0.1  # プラングリッド周期 [s]（drive_logs / ILC と一致）
# 必要加速度 a_req の平滑窓 [s]。人間は瞬時勾配でなく傾向で踏むため 1s で均す。
PLAN_ACCEL_SMOOTH_S: float = 1.0
# フェーズ分類のヒステリシス幅 [km/h/s]。a_req が a_coast(v) から ±この幅を超えて初めて
# DRIVE/BRAKE に振り分ける（惰行帯を確保し、計測ノイズでフェーズがチャタらない）。
PHASE_MARGIN_KMHS: float = 0.15
# 最小フェーズ長 [s]。これ未満のフェーズは隣接する長いフェーズへ吸収し、基準軌跡の
# 微小なうねり由来の短時間フェーズ（サブ秒フリッカ）を消す。STOP_HOLD は対象外（常に保持）。
# 実機で問題化したシーソー（0.6〜1s 周期）は閉ループの PID 過補正由来であり基準軌跡には
# 存在しない（トリム帯＋ゲイン低下で対処）。基準軌跡から導くフェーズは物理的に妥当な区間
# なので、US06 等の正当な短時間ハードブレーキを潰さないよう 1.0s に留める（2.0s だと US06 の
# 生 4.1回/min→1.7回/min まで実ブレーキを吸収してしまう。1.0s では 2.5回/min で保存、
# 2026-07-11 全12モードで机上確認）。
MIN_PHASE_S: float = 1.0
# 名目 effort のゼロ位相ローパスのカットオフ [Hz]。ゆっくり滑らかな踏み変化に整形する。
PLAN_LOWPASS_HZ: float = 0.25


class PlanPhase(StrEnum):
    """ペダルプランのフェーズ（区間ごとのペダル権限）。"""

    DRIVE = "drive"  # アクセルで駆動（緩減速の調整踏みを含む）
    COAST = "coast"  # ペダルなし（クリープ発進・クリープ保持・エンジンブレーキ減速）
    BRAKE = "brake"  # ブレーキで減速
    STOP_HOLD = "stop"  # 停車保持（stop_brake_opening_pct を保持）


@dataclass
class PedalPlan:
    """時刻グリッド上のフェーズ列と名目 effort 列。

    efforts[i] / phases[i] は時刻 i×dt_s の名目 effort [%] とフェーズ。走行中は
    effort_at / phase_at で now-frame 参照する。
    """

    dt_s: float = PLAN_DT_S
    efforts: list[float] = field(default_factory=list)
    phases: list[PlanPhase] = field(default_factory=list)

    def __post_init__(self) -> None:
        self._times = np.arange(len(self.efforts), dtype=float) * self.dt_s
        self._efforts_arr = np.asarray(self.efforts, dtype=float)

    @property
    def duration_s(self) -> float:
        """プランが覆う時間長 [s]。"""
        return max(0, len(self.efforts) - 1) * self.dt_s

    def effort_at(self, elapsed_s: float) -> float:
        """elapsed_s における名目 effort [%]。範囲外は端点クランプ。空プランは 0.0。"""
        if self._efforts_arr.size == 0:
            return 0.0
        return float(np.interp(elapsed_s, self._times, self._efforts_arr))

    def phase_at(self, elapsed_s: float) -> PlanPhase:
        """elapsed_s におけるフェーズ（最近傍グリッド）。空プランは COAST。"""
        if not self.phases:
            return PlanPhase.COAST
        idx = int(round(elapsed_s / self.dt_s))
        idx = max(0, min(len(self.phases) - 1, idx))
        return self.phases[idx]


def coast_accel(v: float, params: FeedforwardParams) -> float:
    """速度 v [km/h] での惰行加速度 a_coast [km/h/s]（ペダル未操作時）。

    クリープ速度未満はクリープが車を押す（+creep_rate）、以上はエンジンブレーキで減速する
    （−engine_brake_decel）。フェーズ分類の基準線。
    """
    if v < params.creep_speed_kmh:
        return params.creep_rate_kmhs
    return -params.engine_brake_decel_kmhs


def required_accel(
    speeds: np.ndarray, dt_s: float, smooth_s: float = PLAN_ACCEL_SMOOTH_S
) -> np.ndarray:
    """基準速度列から必要加速度 a_req [km/h/s] を求める（移動平均で平滑化）。"""
    if speeds.size < 2:
        return np.zeros_like(speeds, dtype=float)
    a: np.ndarray = np.gradient(speeds.astype(float), dt_s)
    w = max(1, int(round(smooth_s / dt_s)))
    if w <= 1:
        return a
    kernel = np.ones(w) / w
    return np.convolve(a, kernel, mode="same")


def classify_phases(
    speeds: np.ndarray,
    accels: np.ndarray,
    params: FeedforwardParams,
    *,
    margin: float = PHASE_MARGIN_KMHS,
) -> list[PlanPhase]:
    """基準速度・必要加速度からフェーズ列を分類する（純関数・analyze_session と共用）。

    v<停車速度 は STOP_HOLD。それ以外は a_req を惰行加速度 a_coast(v) と比べ、
    a_coast+margin を超えれば DRIVE、a_coast−margin を下回れば BRAKE、間は COAST。
    """
    phases: list[PlanPhase] = []
    for v, a in zip(speeds, accels):
        if v < VEHICLE_STOP_SPEED_KMH:
            phases.append(PlanPhase.STOP_HOLD)
            continue
        ac = coast_accel(float(v), params)
        if a > ac + margin:
            phases.append(PlanPhase.DRIVE)
        elif a < ac - margin:
            phases.append(PlanPhase.BRAKE)
        else:
            phases.append(PlanPhase.COAST)
    return phases


def merge_micro_phases(
    phases: list[PlanPhase], dt_s: float, min_phase_s: float = MIN_PHASE_S
) -> list[PlanPhase]:
    """min_phase_s 未満の連続フェーズを長い隣接フェーズへ吸収する。

    STOP_HOLD は吸収しない（停車保持は常に維持）。反復して短フェーズが無くなるまで均す。
    これで DRIVE⇄BRAKE の短時間踏み替え（軌跡が要求しない不要切替）が消える。
    """
    if not phases:
        return []
    min_len = max(1, int(round(min_phase_s / dt_s)))

    def runs(seq: list[PlanPhase]) -> list[list[int]]:
        out: list[list[int]] = []
        start = 0
        for i in range(1, len(seq) + 1):
            if i == len(seq) or seq[i] != seq[start]:
                out.append([start, i])  # [start, end)
                start = i
        return out

    result = list(phases)
    changed = True
    while changed:
        changed = False
        segments = runs(result)
        for idx, (s, e) in enumerate(segments):
            length = e - s
            ph = result[s]
            if ph == PlanPhase.STOP_HOLD or length >= min_len:
                continue
            # 隣接する非 STOP_HOLD の長い方へ吸収する。両隣が STOP_HOLD なら COAST 化。
            left = segments[idx - 1] if idx > 0 else None
            right = segments[idx + 1] if idx < len(segments) - 1 else None
            candidates: list[tuple[int, PlanPhase]] = []
            if left is not None and result[left[0]] != PlanPhase.STOP_HOLD:
                candidates.append((left[1] - left[0], result[left[0]]))
            if right is not None and result[right[0]] != PlanPhase.STOP_HOLD:
                candidates.append((right[1] - right[0], result[right[0]]))
            fill = max(candidates, key=lambda c: c[0])[1] if candidates else PlanPhase.COAST
            for i in range(s, e):
                result[i] = fill
            changed = True
            break  # セグメント構造が変わったので runs を取り直す
    return result


def _clamp_effort_by_phase(
    efforts: np.ndarray, phases: list[PlanPhase], params: FeedforwardParams
) -> list[float]:
    """フェーズ整合クランプ: DRIVE≥0 / BRAKE≤0 / COAST=0 / STOP_HOLD=停車保持ブレーキ。

    ローパスの滲みで DRIVE 区間に負値・BRAKE 区間に正値が漏れるのを断つ。停車保持は
    プランが支配し、stop_brake_opening_pct（最低でも brake_deadband_pct）の制動 effort を置く。
    """
    stop_effort = -max(params.stop_brake_opening_pct, params.brake_deadband_pct)
    out: list[float] = []
    for e, ph in zip(efforts, phases):
        val = float(e)
        if ph == PlanPhase.DRIVE:
            out.append(max(0.0, val))
        elif ph == PlanPhase.BRAKE:
            out.append(min(0.0, val))
        elif ph == PlanPhase.STOP_HOLD:
            out.append(stop_effort)
        else:  # COAST
            out.append(0.0)
    return out


class PedalPlanner:
    """基準軌跡＋車両定数＋FF モデルから PedalPlan を生成する。"""

    @staticmethod
    def build(
        mode: DrivingMode,
        ff: FeedforwardController,
        params: FeedforwardParams,
        *,
        dt_s: float = PLAN_DT_S,
    ) -> PedalPlan:
        """走行開始時に一度だけ呼ぶ（非リアルタイム）。

        Args:
            mode: 基準軌跡を持つ走行モード。
            ff: ロード済み FF コントローラ（未ロードなら effort は 0 系列になる）。
            params: 車両物理定数（クリープ・エンジンブレーキ・停車保持等）。
            dt_s: プラングリッド周期 [s]。

        Returns:
            PedalPlan。基準軌跡が空なら空プラン。
        """
        pts = mode.reference_speed
        if not pts:
            return PedalPlan(dt_s=dt_s, efforts=[], phases=[])

        src_t = np.array([p.time_s for p in pts], dtype=float)
        src_v = np.array([p.speed_kmh for p in pts], dtype=float)
        total = float(src_t[-1])
        n = max(1, int(round(total / dt_s)) + 1)
        grid_t = np.arange(n, dtype=float) * dt_s
        grid_v = np.interp(grid_t, src_t, src_v)

        a_req = required_accel(grid_v, dt_s)
        phases = classify_phases(grid_v, a_req, params)
        phases = merge_micro_phases(phases, dt_s)

        if ff.has_model:
            raw = np.array(
                [
                    ff.predict_effort(
                        float(grid_v[i]),
                        [float(np.interp(grid_t[i] + h, src_t, src_v)) for h in ff.horizons],
                        [float(np.interp(grid_t[i] - h, src_t, src_v)) for h in ff.past_horizons],
                    )
                    for i in range(n)
                ],
                dtype=float,
            )
            smoothed = zero_phase_lowpass(raw, PLAN_LOWPASS_HZ, dt_s)
        else:
            smoothed = np.zeros(n, dtype=float)

        efforts = _clamp_effort_by_phase(smoothed, phases, params)
        return PedalPlan(dt_s=dt_s, efforts=efforts, phases=phases)


__all__ = [
    "MIN_PHASE_S",
    "PHASE_MARGIN_KMHS",
    "PLAN_ACCEL_SMOOTH_S",
    "PLAN_DT_S",
    "PLAN_LOWPASS_HZ",
    "PedalPlan",
    "PedalPlanner",
    "PlanPhase",
    "classify_phases",
    "coast_accel",
    "merge_micro_phases",
    "required_accel",
]
