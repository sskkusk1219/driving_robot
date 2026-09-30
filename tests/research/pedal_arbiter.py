"""符号付き努力量をアクセル/ブレーキ開度へ写像するペダル調停（研究用の移植版）。

引用元: src/domain/control/pedal_arbiter.py（PedalArbiter / ArbiterOutput）。
tests/research は src を import しない決まりのため、アルゴリズムだけを移植している。

本番の調停は 6 つの機能（切替ヒステリシス・不感帯補償・レートリミット・微小変化の保持・
再踏込ディレイ・惰行の解放レート）を常に全部使う。こちらは機能ごとに個別のスイッチ
（ArbiterSection.enable_*）を持ち、1 つずつ有効にして実機で試せる
（ProblemReport_20260921 段3: 1 つずつ実機で試すため）。

加えて研究専用の 2 機能（src の調停にはない）を持つ: 加速度帯の保持（段3c）と
向きのヒステリシス（段3d）（ProblemReport_20260921 6-15）。

全スイッチ off のときは mode_drive.split_effort（効果量の符号で振り分けて開度上限で
クランプするだけ）と完全に同じ出力になる。
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from tests.research.config import ArbiterSection


@dataclass
class ArbiterOutput:
    """調停結果。accel_opening と brake_opening は排他（高々一方のみ非ゼロ）。"""

    accel_opening: float  # [%]
    brake_opening: float  # [%]
    saturated_high: bool  # 加速側の要求が上限・ディレイ・レート制限で削られた
    saturated_low: bool  # 制動側の要求が上限・レート制限で削られた


def plan_accel_kmhs(
    ref_at: Callable[[float], float], t_s: float, horizon_s: float, step_s: float = 0.1
) -> float:
    """基準車速 0〜H 秒先の直線近似の傾き [km/h/s]（ProblemReport_20260921 6-15、案E）。

    ref_at(t_s + k*step_s)（k=0..N、N=round(horizon_s/step_s)）に最小二乗で直線を当てる。
    """
    n = max(1, round(horizon_s / step_s))
    ts = [k * step_s for k in range(n + 1)]
    vs = [ref_at(t_s + t) for t in ts]
    t_mean = sum(ts) / len(ts)
    v_mean = sum(vs) / len(vs)
    denom = sum((t - t_mean) ** 2 for t in ts)
    return sum((t - t_mean) * (v - v_mean) for t, v in zip(ts, vs, strict=True)) / denom


def enabled_arbiter_features(section: ArbiterSection) -> list[str]:
    """有効な調停機能を、設定値つきの日本語ラベルで返す（表示・レポート用）。"""
    features: list[str] = []
    if section.enable_hysteresis:
        features.append(f"切替ヒステリシス ±{section.switch_hysteresis_pct:g}%")
    if section.enable_deadband_compensation:
        features.append("不感帯補償")
    if section.enable_rate_limit:
        features.append(
            f"レートリミット アクセル {section.accel_rate_limit_pct_s:g} / "
            f"ブレーキ {section.brake_rate_limit_pct_s:g} %/s"
        )
    if section.enable_min_step:
        features.append(f"微小変化の保持 {section.accel_min_step_pct:g}%")
    if section.enable_reengage_dwell:
        features.append(f"再踏込ディレイ {section.accel_reengage_dwell_s:g}s")
    if section.enable_release_rate:
        features.append(f"アクセル解放レート {section.accel_release_rate_pct_s:g} %/s")
    if section.enable_accel_band:
        features.append(
            f"加速度帯の保持 ±{section.accel_band_kmhs:g}km/h/s"
            f"（{section.accel_band_horizon_s:g}s先の傾き）"
            f"・偏差変化 {section.accel_band_dev_escape_kmh:g}km/h"
            f"・開度差 {section.accel_band_open_escape_pct:g}%"
        )
    if section.enable_direction_hysteresis:
        features.append(f"向きのヒステリシス {section.accel_direction_hysteresis_pct:g}%")
    return features


class PedalArbiter:
    """努力量 → ペダル開度の写像。機能ごとに ArbiterSection のスイッチで有効化する。"""

    def __init__(
        self,
        section: ArbiterSection,
        accel_deadband_pct: float,
        brake_deadband_pct: float,
        max_accel_opening: float,
        max_brake_opening: float,
        nominal_dt_s: float = 0.05,
    ) -> None:
        self._s = section
        self._accel_deadband = accel_deadband_pct
        self._brake_deadband = brake_deadband_pct
        self._max_accel = max(0.0, max_accel_opening)
        self._max_brake = max(0.0, max_brake_opening)
        # サイクルスキップ後の大きな dt でレートが暴れないよう、公称周期の 0.5〜4 倍にクランプする
        self._nominal_dt_s = max(0.0, nominal_dt_s)
        self.reset()

    def reset(self) -> None:
        """内部状態（時刻・前回開度・ディレイ起点）をリセットする。走行開始時に呼ぶ。"""
        self._time_s = 0.0
        self._last_accel_opening = 0.0
        self._last_brake_opening = 0.0
        # ブレーキが最後に非ゼロだった内部時刻。再踏込ディレイの起点
        self._last_brake_active_at: float | None = None
        # 加速度帯: 前回アクセルを動かした時の計画加速度・偏差（アクセル以外の周期で消える）
        self._band_anchor_accel: float | None = None
        self._band_anchor_dev: float | None = None
        # 向きのヒステリシス: 前回の動きの向き（+1 増 / -1 減 / 0 未定）
        self._accel_dir = 0

    def arbitrate(
        self,
        effort: float,
        dt: float,
        *,
        plan_accel_kmhs: float | None = None,
        deviation_kmh: float | None = None,
    ) -> ArbiterOutput:
        """努力量 [%]（+加速/−制動）をペダル開度に写像する。dt は前サイクルからの経過 [s]。

        plan_accel_kmhs（計画加速度）と deviation_kmh（車速−基準）は加速度帯の保持用。
        None なら加速度帯は働かない。
        """
        s = self._s
        if dt <= 0.0:
            dt = self._nominal_dt_s
        elif self._nominal_dt_s > 0.0:
            dt = max(0.5 * self._nominal_dt_s, min(4.0 * self._nominal_dt_s, dt))
        self._time_s += dt

        # 1. ペダル選択。ヒステリシス off は h=0（符号だけで決める）
        h = max(0.0, s.switch_hysteresis_pct) if s.enable_hysteresis else 0.0
        saturated_high = False
        saturated_low = False
        if effort > h:
            pedal = "accel"
        elif effort < -h:
            pedal = "brake"
        else:
            pedal = "coast"

        # 2. 再踏込ディレイ: ブレーキ直後のアクセルを一定時間抑止（制動側には設けない）
        if (
            s.enable_reengage_dwell
            and pedal == "accel"
            and self._last_brake_active_at is not None
            and self._time_s - self._last_brake_active_at < s.accel_reengage_dwell_s
        ):
            pedal = "coast"
            saturated_high = True

        if pedal == "accel":
            requested = self._magnitude(effort, self._accel_deadband)
            applied = requested
            if s.enable_rate_limit:
                applied = self._rate_limit(
                    requested, self._last_accel_opening, s.accel_rate_limit_pct_s, dt
                )
            applied = min(applied, self._max_accel)
            if applied < requested:
                saturated_high = True
            if s.enable_min_step and abs(applied - self._last_accel_opening) < max(
                0.0, s.accel_min_step_pct
            ):
                applied = self._last_accel_opening
            if s.enable_accel_band and plan_accel_kmhs is not None and deviation_kmh is not None:
                if (
                    self._band_anchor_accel is not None
                    and self._band_anchor_dev is not None
                    and self._last_accel_opening > 0.0
                    and abs(plan_accel_kmhs - self._band_anchor_accel) <= s.accel_band_kmhs
                    and abs(deviation_kmh - self._band_anchor_dev) <= s.accel_band_dev_escape_kmh
                    # 要求開度が保持中の開度から離れたら追従（保持で開度差が溜まるのを防ぐ）
                    and abs(applied - self._last_accel_opening) < s.accel_band_open_escape_pct
                ):
                    applied = self._last_accel_opening
                else:
                    self._band_anchor_accel = plan_accel_kmhs
                    self._band_anchor_dev = deviation_kmh
            if s.enable_direction_hysteresis and self._last_accel_opening > 0.0:
                applied = self._apply_direction_hysteresis(applied, self._last_accel_opening)
            accel, brake = applied, 0.0
        elif pedal == "brake":
            requested = self._magnitude(-effort, self._brake_deadband)
            applied = requested
            if s.enable_rate_limit:
                applied = self._rate_limit(
                    requested, self._last_brake_opening, s.brake_rate_limit_pct_s, dt
                )
            applied = min(applied, self._max_brake)
            if applied < requested:
                saturated_low = True
            # ブレーキ選択時はアクセルを即 0 解放（同時踏み禁止）
            accel, brake = 0.0, applied
        else:
            # 惰行: 解放レート off は即 0、on は rate_pct_s [%/s] で漸減
            if s.enable_release_rate:
                accel = self._release_accel(
                    self._last_accel_opening, s.accel_release_rate_pct_s, dt
                )
            else:
                accel = 0.0
            brake = 0.0

        if pedal != "accel":
            self._band_anchor_accel = None
            self._band_anchor_dev = None
            self._accel_dir = 0
        self._last_accel_opening = accel
        self._last_brake_opening = brake
        if brake > 0.0:
            self._last_brake_active_at = self._time_s
        return ArbiterOutput(accel, brake, saturated_high, saturated_low)

    def _apply_direction_hysteresis(self, applied: float, last: float) -> float:
        """同じ向きの動きはすぐ通し、逆向きは w [%] 以上動く時だけ向きを変えて通す。"""
        w = self._s.accel_direction_hysteresis_pct
        diff = applied - last
        if self._accel_dir == 0:
            if abs(diff) >= w:
                self._accel_dir = 1 if diff > 0 else -1
                return applied
            return last
        if diff * self._accel_dir >= 0.0:
            return applied
        if abs(diff) >= w:
            self._accel_dir = -self._accel_dir
            return applied
        return last

    def _magnitude(self, magnitude: float, deadband_pct: float) -> float:
        """不感帯補償が on なら逆補償、off なら大きさをそのまま通す（負は 0）。"""
        if self._s.enable_deadband_compensation:
            return self._apply_deadband(magnitude, deadband_pct)
        return max(0.0, magnitude)

    @staticmethod
    def _apply_deadband(magnitude: float, deadband_pct: float) -> float:
        """不感帯逆補償: 死帯 (0, deadband) に落ちないよう 0 か deadband 以上へ丸める。"""
        db = max(0.0, deadband_pct)
        if db == 0.0:
            return max(0.0, magnitude)
        if magnitude < db / 2.0:
            return 0.0
        return max(db, magnitude)

    @staticmethod
    def _rate_limit(requested: float, previous: float, rate_pct_s: float, dt: float) -> float:
        """開度の増加方向のみレート制限する。減少（解放）方向は無制限。"""
        if rate_pct_s <= 0.0 or dt <= 0.0:
            return requested
        max_step = rate_pct_s * dt
        if requested > previous + max_step:
            return previous + max_step
        return requested

    @staticmethod
    def _release_accel(previous: float, rate_pct_s: float, dt: float) -> float:
        """惰行遷移時のアクセル解放。rate_pct_s<=0 は即 0、dt<=0 は前回値を保持。"""
        if previous <= 0.0 or rate_pct_s <= 0.0:
            return 0.0
        if dt <= 0.0:
            return previous
        return max(0.0, previous - rate_pct_s * dt)
