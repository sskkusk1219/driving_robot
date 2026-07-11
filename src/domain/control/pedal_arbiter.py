"""符号付き努力量をアクセル/ブレーキ開度へ写像するペダル調停コンポーネント。

FF と PID の出力を「努力量」（+: 加速 [%]、−: 制動 [%]）として合成した後、
ペダルへの写像はこのコンポーネントだけが行う。これにより:

- アクセル・ブレーキの同時踏みが構造的に発生しない（常にどちらか一方のみ非ゼロ）
- 切替ヒステリシス・再踏込ディレイ・レートリミットで振動抑制 KPI
  （偏差符号反転 ≤1 回/5 秒）を機構として支える
- 不感帯逆補償により最終指令が物理死帯 (0, deadband) に落ちない

従来の「FF が (accel, brake) ペアを出し、DriveLoop が PID を符号分割して加算し、
両方 >0 ならブレーキを消す」方式は、FF アクセル中の減速権限喪失とブレーキの
バンバン制御を生むため廃止した（コードレビュー 2026-06-11 指摘 #1/#11）。
"""

from __future__ import annotations

from dataclasses import dataclass

from src.models.profile import FeedforwardParams


@dataclass
class ArbiterOutput:
    """調停結果。accel_opening と brake_opening は排他（高々一方のみ非ゼロ）。"""

    accel_opening: float  # [%] 0 または不感帯以上
    brake_opening: float  # [%] 同上
    saturated_high: bool  # 加速側の要求が上限・ディレイ・レート制限で削られた
    saturated_low: bool  # 制動側の要求が上限・レート制限で削られた


class PedalArbiter:
    """努力量 → ペダル開度の写像と振動抑制を担うドメインコンポーネント。"""

    def __init__(
        self,
        params: FeedforwardParams,
        max_accel_opening: float,
        max_brake_opening: float,
        nominal_dt_s: float = 0.05,
    ) -> None:
        self._params = params
        self._max_accel = max(0.0, max_accel_opening)
        self._max_brake = max(0.0, max_brake_opening)
        # レートリミットに使う dt の基準周期。バスストール等でサイクルがスキップされた後の
        # 大きな dt をそのまま使うと 1 サイクルで大きくペダルが動いてしまうため、
        # PIDController と同じ方式（公称周期の 0.5〜4 倍）にクランプする（D6 レビュー指摘）。
        self._nominal_dt_s = max(0.0, nominal_dt_s)
        self._time_s = 0.0
        self._last_accel_opening = 0.0
        self._last_brake_opening = 0.0
        # ブレーキが最後に非ゼロだった内部時刻。アクセル再踏込ディレイの起点。
        self._last_brake_active_at: float | None = None

    def reset(self) -> None:
        """内部状態（時刻・前回開度・ディレイ起点）をリセットする。走行開始時に呼ぶ。"""
        self._time_s = 0.0
        self._last_accel_opening = 0.0
        self._last_brake_opening = 0.0
        self._last_brake_active_at = None

    def arbitrate(self, effort: float, dt: float) -> ArbiterOutput:
        """努力量 [%]（+加速/−制動）をペダル開度に写像する。

        Args:
            effort: FF+PID 合成後の符号付き努力量
            dt: 前サイクルからの経過時間 [s]（レートリミットとディレイに使用）
        """
        p = self._params
        if dt <= 0.0:
            dt = self._nominal_dt_s
        elif self._nominal_dt_s > 0.0:
            # スケジューラジッタ・サイクルスキップ（バスストール復帰）の外れ値を抑える。
            # クランプしないと復帰後の大きな dt がレートリミットの上限 (rate_pct_s * dt) を
            # 押し上げ、1 サイクルでペダルが最大速度で動いてしまう。
            dt = max(0.5 * self._nominal_dt_s, min(4.0 * self._nominal_dt_s, dt))
        self._time_s += dt

        # 1. ペダル選択（ヒステリシス）: ±h の帯内は惰行。帯内で努力量の符号が
        #    チャタついてもペダル反転が起きない（符号反転 KPI の機構的保証）。
        h = max(0.0, p.switch_hysteresis_pct)
        saturated_high = False
        saturated_low = False
        if effort > h:
            pedal = "accel"
        elif effort < -h:
            pedal = "brake"
        else:
            pedal = "coast"

        # 2. アクセル再踏込ディレイ: 制動直後の加速はポンピングの典型パターンのため
        #    一定時間抑止する。制動側にはディレイを設けない（減速権限を遅延させない）。
        if pedal == "accel" and self._last_brake_active_at is not None:
            since_brake = self._time_s - self._last_brake_active_at
            if since_brake < p.accel_reengage_dwell_s:
                pedal = "coast"
                saturated_high = True  # 加速要求がディレイで削られた → 積分停止

        if pedal == "accel":
            requested = self._apply_deadband(effort, p.accel_deadband_pct)
            applied = self._rate_limit(
                requested, self._last_accel_opening, p.accel_rate_limit_pct_s, dt
            )
            applied = min(applied, self._max_accel)
            if applied < requested:
                saturated_high = True
            # 微小変化は前回開度を保持（サーボの震えと ON-OFF チャタを抑える・B-7-3）。
            if abs(applied - self._last_accel_opening) < max(0.0, p.accel_min_step_pct):
                applied = self._last_accel_opening
            accel, brake = applied, 0.0
        elif pedal == "brake":
            requested = self._apply_deadband(-effort, p.brake_deadband_pct)
            applied = self._rate_limit(
                requested, self._last_brake_opening, p.brake_rate_limit_pct_s, dt
            )
            applied = min(applied, self._max_brake)
            if applied < requested:
                saturated_low = True
            # ブレーキ選択時はアクセルを即 0 解放（同時踏み禁止・減速権限を最優先）。
            accel, brake = 0.0, applied
        else:
            # 惰行: アクセルからの遷移は即 0 解放せず解放レートで漸減する（B-7-3）。定常巡航で
            # 努力量がヒステリシス帯を跨ぐたびに即 0 解放されると ON-OFF パルスになり、実機で
            # 高速域 36回/min のハンチングを生む。1〜2サイクルの惰行では accel を残し滑らかに保つ。
            # ブレーキは惰行では踏んでいないため 0。
            accel = self._release_accel(self._last_accel_opening, p.accel_release_rate_pct_s, dt)
            brake = 0.0

        # 非選択ペダルは即座に解放する（レート制限しない）。解放を遅らせると
        # 切替の過渡で両ペダルが同時に非ゼロになるため。
        self._last_accel_opening = accel
        self._last_brake_opening = brake
        if brake > 0.0:
            self._last_brake_active_at = self._time_s

        return ArbiterOutput(
            accel_opening=accel,
            brake_opening=brake,
            saturated_high=saturated_high,
            saturated_low=saturated_low,
        )

    @staticmethod
    def _apply_deadband(magnitude: float, deadband_pct: float) -> float:
        """不感帯逆補償: 指令が物理死帯 (0, deadband) に落ちないよう写像する。

        死帯内の微小指令はペダルを動かしても力が出ず、PID 積分器の巻き上げと
        その後の段付き応答（lurch）を招くため、0 か deadband 以上に丸める。
        """
        db = max(0.0, deadband_pct)
        if db == 0.0:
            return max(0.0, magnitude)
        if magnitude < db / 2.0:
            return 0.0
        return max(db, magnitude)

    @staticmethod
    def _rate_limit(requested: float, previous: float, rate_pct_s: float, dt: float) -> float:
        """開度の増加方向のみレート制限する。減少（解放）方向は安全側のため無制限。"""
        if rate_pct_s <= 0.0 or dt <= 0.0:
            return requested
        max_step = rate_pct_s * dt
        if requested > previous + max_step:
            return previous + max_step
        return requested

    @staticmethod
    def _release_accel(previous: float, rate_pct_s: float, dt: float) -> float:
        """惰行遷移時のアクセル解放を rate_pct_s [%/s] で 0 へ漸減する（B-7-3）。

        rate_pct_s<=0 は即 0 解放（従来挙動）。dt<=0 は前回値を保持。
        """
        if previous <= 0.0:
            return 0.0
        if rate_pct_s <= 0.0:
            return 0.0
        if dt <= 0.0:
            return previous
        return max(0.0, previous - rate_pct_s * dt)
