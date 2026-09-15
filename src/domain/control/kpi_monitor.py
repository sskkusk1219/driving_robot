"""プライマリー KPI（車速追従精度・振動抑制）の実行時計測モニタ。

product-requirements.md のプライマリー KPI:
- 速度偏差絶対値の 95 パーセンタイルが 0.4 km/h 以内（2026-07-16 に 0.2 から変更）
- 速度偏差絶対値が 1.0 km/h を超えない（例外なし）
- 速度偏差の符号反転が 5 秒に 1 回以下

走行終了後のオフライン解析（しかもログは 2 サイクルに 1 回へ間引かれる）では
KPI 違反が観測すらできないため、制御サイクル内で全サンプルを集計する
（コードレビュー 2026-06-11 指摘 #7）。

メモリは固定: ヒストグラムは固定長ビン、反転時刻は 5 秒窓 deque のみ保持するため、
10 時間 × 20Hz = 72 万サンプルでも増加しない。
"""

from __future__ import annotations

import logging
from collections import deque
from collections.abc import Mapping

_logger = logging.getLogger(__name__)

# 0.2→0.4（2026-07-16 ユーザー承認）: 生 10Hz 車速の高周波変動が p95≈0.4km/h あり
# （±0.5s 移動平均との差、7/15 実走 0cd3e2e4 で実測）、0.2 は計測変動と同水準以下で
# 制御では到達できないため。速度フィルタは導入せず閾値側を実測床に合わせた。
KPI_P95_LIMIT_KMH: float = 0.4
KPI_HARD_LIMIT_KMH: float = 1.0
# ハード違反の解除しきい値。境界での warning 連発を防ぐリリースヒステリシス。
_HARD_LIMIT_RELEASE_KMH: float = 0.9
KPI_REVERSAL_WINDOW_S: float = 5.0
# プライマリー KPI の上限値（この回数を超えたら違反）。窓の長さ(KPI_REVERSAL_WINDOW_S)とは
# 別物であり、コスト関数等で正規化に使う場合はこちらを使うこと（窓長で割ると単位が
# 秒になり「回数の上限で正規化」という意図と食い違う）。
KPI_REVERSAL_LIMIT_PER_WINDOW: float = 1.0
# 符号反転としてカウントする最小振幅 [km/h]。**±この幅を両側で超えて初めて 1 回**と数える
# （帯の中では直前符号を保持するヒステリシス方式）。
# 2026-09-09 に 0.05 → 0.3 へ変更（ProblemReport_20260908 第2ラウンド）。CAN 車速は
# フィルタなしの生値で 10Hz 隣接差の実測 std が 0.22-0.25km/h あり、0.05 では制御の振動では
# なく測定ノイズのゼロクロスを数えていた。実機 3ca20d43 の最大反転（9回/5s）が発生したのは
# t=62s＝120km/h 巡航＝**最も追従が良い区間**で、|偏差| がノイズに埋もれるほど反転が増える
# という逆立ちした指標になっていた（追従を良くするほど不合格に近づく）。0.3 は実測ノイズ
# std の約 1.3σ で、同じログでの反転は 9→4 回/5s になる。
SIGN_REVERSAL_AMPLITUDE_KMH: float = 0.3


def kpi_passed(kpi: Mapping[str, float]) -> bool:
    """プライマリー KPI 3 項目（p95≤0.4 / max≤1.0・例外なし / 反転≤1回/5s）を満たすか。

    走行 1 本の合否をドメイン層の単一ソースとして返す。学習サイクルの best 選択
    （LearningCycleService._plan_learn_run_key）と ILC の採否
    （PedalPlanService._decide_outcome）が**同じ判定**を使うために切り出した。
    第3ラウンドでは前者だけが KPI 合否を見ており、後者は reward スカラー 1 本で
    判定していたため、p95 がサイクル最良の走行が reward で捨てられていた
    （docs/Problem/引き継ぎ20260909.md 優先D）。

    Args:
        kpi: KPIMonitor.summary() 相当の辞書。

    Returns:
        3 項目すべてを満たせば True。サンプルが無ければ False。
    """
    if kpi.get("n_samples", 0.0) <= 0.0:
        return False
    return (
        kpi.get("p95_kmh", 1e9) <= KPI_P95_LIMIT_KMH
        and kpi.get("max_abs_deviation_kmh", 1e9) <= KPI_HARD_LIMIT_KMH
        and kpi.get("reversal_max_per_5s", 1e9) <= KPI_REVERSAL_LIMIT_PER_WINDOW
    )


_BIN_WIDTH_KMH: float = 0.01
_MAX_BIN_KMH: float = 10.0
# 超過積分の dt クランプ上限 [s]。pause 復帰やサイクルスキップの大ギャップ対策。
_DT_CLAMP_S: float = 0.5
# ペダル「踏んでいる」とみなすアクセル開度のしきい値 [%]（ON-OFF 立ち上がりカウント用）。
# scripts/analyze_session.py のハンチング解析と揃える。
_PEDAL_ON_THRESHOLD_PCT: float = 0.5
# アクセル⇔ブレーキの交互踏み（シーソー）とみなす切替の時間窓 [s]。前にアクティブだった
# ペダルと異なるペダルがこの秒数以内に踏まれたら「不要切替」の候補としてカウントする。
_PEDAL_SWITCH_WINDOW_S: float = 2.0

# ── 滑らかさ指標（2026-09-08 追加: ProblemReport_20260908 対応）──────────────
# 「人が運転しているような滑らかさ」の記録専用指標。KPI 合否（_kpi_passed）のゲートには
# 入れない（learning_cycle.py 参照。KPI 優先方針のため、まずは記録・表示のみ）。
# 開度の向き反転とみなす最小変化量 [%]。丸め誤差・量子化ノイズによる符号チャタつきを
# 反転として数えないための不感帯（analyze_session.py のハンチング解析と揃える）。
_PEDAL_REVERSAL_NOISE_FLOOR_PCT: float = 0.05
# 開度変化率ヒストグラムのビン幅・上限 [%/s]（固定メモリの p95 推定用。p95_kmh と同じ手法）。
_PEDAL_RATE_BIN_WIDTH_PCT_S: float = 1.0
_PEDAL_RATE_MAX_BIN_PCT_S: float = 500.0
# 滑らかさの合否目安（ユーザー確認 2026-09-08）。KPI ゲートには使わず、記録・WebUI 表示用。
KPI_PEDAL_REVERSALS_LIMIT_PER_MIN: float = 10.0
KPI_PEDAL_RATE_P95_LIMIT_PCT_S: float = 5.0
KPI_EFFORT_RATE_RMS_LIMIT_PCT_S: float = 3.0

# フェーズ文字列（PlanPhase.value と一致。kpi_monitor を pedal_plan から独立させるため
# import せず文字列で受ける）。フェーズ逸脱量の集計に使う（STOP_HOLD/None は逸脱対象外）。
_PHASE_DRIVE: str = "drive"
_PHASE_COAST: str = "coast"
_PHASE_BRAKE: str = "brake"


class KPIMonitor:
    """速度偏差の P95・最大値・符号反転率を逐次集計するドメインコンポーネント。"""

    def __init__(self) -> None:
        n_bins = int(_MAX_BIN_KMH / _BIN_WIDTH_KMH) + 1  # 最終ビンはオーバーフロー
        self._bins: list[int] = [0] * n_bins
        self._n: int = 0
        self._max_abs_deviation: float = 0.0
        self._last_sign: int = 0
        self._reversal_times: deque[float] = deque()
        self._reversal_max_per_window: int = 0
        self._hard_violations: int = 0
        self._in_hard_violation: bool = False
        # ハード上限超過の連続量。回数(_hard_violations)の不連続ペナルティでは座標降下が
        # 局所解で停滞するため、tuning_cost へ勾配を与える「超過量×時間」の積分を保持する。
        self._over_limit_integral_kmhs: float = 0.0
        self._time_over_limit_s: float = 0.0
        self._last_now_s: float | None = None
        self._first_now_s: float | None = None
        # ペダル活動度（ハンチング指標）。アクセルの 0→非0 立ち上がり回数と総移動量[%]。
        # tuning_cost が「滑らかな操作」を最適化対象に含めるために集計する（B-7-4）。
        self._accel_on_count: int = 0
        self._pedal_travel_pct: float = 0.0
        self._prev_accel_on: bool | None = None
        self._prev_accel_opening: float | None = None
        # アクセル⇔ブレーキ交互踏み（シーソー）回数。前にアクティブだったペダルと異なる
        # ペダルが _PEDAL_SWITCH_WINDOW_S 以内に踏まれた回数を数える（不要切替の実測。
        # 軌跡が要求する切替との差が「不要切替」＝滑らかさの逆指標）。
        self._pedal_switch_count: int = 0
        self._last_active_pedal: int = 0  # 1=accel, -1=brake, 0=どちらも踏んでいない
        self._last_active_pedal_s: float | None = None
        # effort 内訳の集計（エピソード型プラン学習の報酬・トリム寄与率の可観測化）。
        # applied effort（フェーズ権限クランプ後・調停器前の符号付き合成値）の変化率 RMS、
        # プラン／トリム effort の RMS、フェーズ逸脱量の積分を逐次集計する（固定メモリ）。
        self._prev_applied_effort: float | None = None
        self._effort_rate_sq_integral: float = 0.0  # Σ((Δapplied/dt)²·dt) = Σ(Δapplied)²/dt
        self._plan_sq_sum: float = 0.0
        self._trim_sq_sum: float = 0.0
        self._effort_sample_count: int = 0
        # フェーズ逸脱量の積分 [%·s]。速い補正層（安全網）がフェーズ権限を無視して介入した量。
        self._phase_violation_integral: float = 0.0

        # 滑らかさ指標（記録専用・KPI ゲート対象外）: 開度変化率の p95 用ヒストグラムと
        # 向き反転回数。アクセル・ブレーキそれぞれ独立に集計する。
        n_rate_bins = int(_PEDAL_RATE_MAX_BIN_PCT_S / _PEDAL_RATE_BIN_WIDTH_PCT_S) + 1
        self._accel_rate_bins: list[int] = [0] * n_rate_bins
        self._brake_rate_bins: list[int] = [0] * n_rate_bins
        self._prev_brake_opening: float | None = None
        self._accel_reversal_count: int = 0
        self._brake_reversal_count: int = 0
        self._accel_last_delta_sign: int = 0
        self._brake_last_delta_sign: int = 0

    def update(
        self,
        ref_kmh: float,
        actual_kmh: float,
        now_s: float,
        accel_opening: float | None = None,
        brake_opening: float | None = None,
        plan_effort_pct: float | None = None,
        trim_effort_pct: float | None = None,
        applied_effort_pct: float | None = None,
        phase: str | None = None,
    ) -> None:
        """1 サイクル分の偏差（と任意でペダル開度・effort 内訳）を集計する。

        Args:
            ref_kmh: 基準車速 [km/h]
            actual_kmh: 実車速 [km/h]
            now_s: 単調増加時刻 [s]（イベントループ時刻）
            accel_opening: 調停後のアクセル開度 [%]。渡すとペダル活動度（ON-OFF 立ち上がり
                回数・総移動量）を集計する。None なら偏差 KPI のみ（後方互換）。
            brake_opening: 調停後のブレーキ開度 [%]。accel_opening と両方渡すと
                アクセル⇔ブレーキ交互踏み（不要切替）を集計する。None なら集計しない。
            plan_effort_pct: プラン名目 effort [%]（符号付き）。RMS を集計する。
            trim_effort_pct: トリム補正 effort [%]（符号付き）。RMS を集計し、トリム寄与率
                （PID フィードバック量の指標）を出す。
            applied_effort_pct: フェーズ権限クランプ後・調停器前の合成 effort [%]。変化率 RMS
                （滑らかさ）とフェーズ逸脱量を集計する。
            phase: プランのフェーズ文字列（PlanPhase.value）。フェーズ逸脱量の判定に使う。
                None なら逸脱は 0（学習運転・ブートストラップ）。
        """
        if self._first_now_s is None:
            self._first_now_s = now_s
        deviation = actual_kmh - ref_kmh
        abs_dev = abs(deviation)

        self._n += 1
        if abs_dev > self._max_abs_deviation:
            self._max_abs_deviation = abs_dev
        bin_idx = min(int(abs_dev / _BIN_WIDTH_KMH), len(self._bins) - 1)
        self._bins[bin_idx] += 1

        # dt は前回サンプルからの経過で、pause/サイクルスキップ由来の大ギャップが積分を
        # 膨らませないよう [0, _DT_CLAMP_S] にクランプする。超過積分・effort 変化率・
        # フェーズ逸脱の各積分で共用する。
        dt_clamped = 0.0
        if self._last_now_s is not None:
            dt_clamped = now_s - self._last_now_s
            if dt_clamped < 0.0:
                dt_clamped = 0.0
            elif dt_clamped > _DT_CLAMP_S:
                dt_clamped = _DT_CLAMP_S
            over = abs_dev - KPI_HARD_LIMIT_KMH
            if over > 0.0:
                self._over_limit_integral_kmhs += over * dt_clamped
                self._time_over_limit_s += dt_clamped
        self._last_now_s = now_s

        # 符号反転（±SIGN_REVERSAL_AMPLITUDE_KMH を両側で超えたものだけ）。任意 5 秒窓の
        # 最大反転回数を追跡する。
        if deviation > SIGN_REVERSAL_AMPLITUDE_KMH:
            sign = 1
        elif deviation < -SIGN_REVERSAL_AMPLITUDE_KMH:
            sign = -1
        else:
            sign = 0
        if sign != 0:
            if self._last_sign != 0 and sign != self._last_sign:
                self._reversal_times.append(now_s)
                while (
                    self._reversal_times and now_s - self._reversal_times[0] > KPI_REVERSAL_WINDOW_S
                ):
                    self._reversal_times.popleft()
                self._reversal_max_per_window = max(
                    self._reversal_max_per_window, len(self._reversal_times)
                )
            self._last_sign = sign

        # ペダル活動度（ハンチング指標）。アクセルの 0→非0 立ち上がりと総移動量を集計する。
        if accel_opening is not None:
            on = accel_opening > _PEDAL_ON_THRESHOLD_PCT
            if self._prev_accel_on is not None and on and not self._prev_accel_on:
                self._accel_on_count += 1
            self._prev_accel_on = on
            if self._prev_accel_opening is not None:
                d_accel = accel_opening - self._prev_accel_opening
                self._pedal_travel_pct += abs(d_accel)
                if dt_clamped > 0.0:
                    self._record_rate_bin(self._accel_rate_bins, d_accel / dt_clamped)
                self._accel_last_delta_sign = self._update_reversal_count(
                    d_accel, self._accel_last_delta_sign, is_accel=True
                )
            self._prev_accel_opening = accel_opening

            # アクセル⇔ブレーキ交互踏み（シーソー）。今アクティブなペダルが、直前にアクティブ
            # だった別ペダルと _PEDAL_SWITCH_WINDOW_S 以内で切り替わったらカウント。
            if brake_opening is not None:
                if accel_opening > _PEDAL_ON_THRESHOLD_PCT:
                    current_pedal = 1
                elif brake_opening > _PEDAL_ON_THRESHOLD_PCT:
                    current_pedal = -1
                else:
                    current_pedal = 0
                if current_pedal != 0:
                    if (
                        self._last_active_pedal != 0
                        and current_pedal != self._last_active_pedal
                        and self._last_active_pedal_s is not None
                        and now_s - self._last_active_pedal_s <= _PEDAL_SWITCH_WINDOW_S
                    ):
                        self._pedal_switch_count += 1
                    self._last_active_pedal = current_pedal
                    self._last_active_pedal_s = now_s

        # ブレーキ側の変化率・向き反転（滑らかさ指標）。accel_opening の有無とは独立に
        # 集計する（アクセル側は上のブロックで扱う）。
        if brake_opening is not None:
            if self._prev_brake_opening is not None:
                d_brake = brake_opening - self._prev_brake_opening
                if dt_clamped > 0.0:
                    self._record_rate_bin(self._brake_rate_bins, d_brake / dt_clamped)
                self._brake_last_delta_sign = self._update_reversal_count(
                    d_brake, self._brake_last_delta_sign, is_accel=False
                )
            self._prev_brake_opening = brake_opening

        # effort 内訳（プラン学習の報酬・トリム寄与率）。applied の変化率 RMS（滑らかさ）、
        # プラン／トリムの RMS、フェーズ逸脱量を集計する。
        if applied_effort_pct is not None:
            self._effort_sample_count += 1
            if self._prev_applied_effort is not None and dt_clamped > 0.0:
                d = applied_effort_pct - self._prev_applied_effort
                # (Δapplied/dt)²·dt = (Δapplied)²/dt。duration で割って sqrt すると
                # 変化率 [%/s] の RMS になる。
                self._effort_rate_sq_integral += d * d / dt_clamped
            self._prev_applied_effort = applied_effort_pct
            # フェーズ逸脱: COAST で踏んだ量・DRIVE のブレーキ量・BRAKE のアクセル量。
            # STOP_HOLD/None は逸脱対象外（プラン支配でトリム無効）。
            violation = 0.0
            if phase == _PHASE_COAST:
                violation = abs(applied_effort_pct)
            elif phase == _PHASE_DRIVE:
                violation = max(0.0, -applied_effort_pct)
            elif phase == _PHASE_BRAKE:
                violation = max(0.0, applied_effort_pct)
            if violation > 0.0 and dt_clamped > 0.0:
                self._phase_violation_integral += violation * dt_clamped
        if plan_effort_pct is not None:
            self._plan_sq_sum += plan_effort_pct * plan_effort_pct
        if trim_effort_pct is not None:
            self._trim_sq_sum += trim_effort_pct * trim_effort_pct

        # ハード上限（1.0 km/h、例外なし）の即時警告。違反継続中の連発はしない。
        if abs_dev > KPI_HARD_LIMIT_KMH and not self._in_hard_violation:
            self._in_hard_violation = True
            self._hard_violations += 1
            _logger.warning(
                "KPI ハード上限違反: |偏差| %.2f km/h > %.1f km/h (ref=%.1f actual=%.1f)",
                abs_dev,
                KPI_HARD_LIMIT_KMH,
                ref_kmh,
                actual_kmh,
            )
        elif abs_dev < _HARD_LIMIT_RELEASE_KMH:
            self._in_hard_violation = False

    def p95_kmh(self) -> float:
        """偏差絶対値の 95 パーセンタイル [km/h]（ビン上端値、保守側）を返す。"""
        return self._bin_p95(self._bins, self._n, _BIN_WIDTH_KMH, _MAX_BIN_KMH)

    @staticmethod
    def _bin_p95(bins: list[int], n: int, bin_width: float, max_bin: float) -> float:
        """固定長ヒストグラムから 95 パーセンタイル（ビン上端値、保守側）を返す。"""
        if n == 0:
            return 0.0
        target = 0.95 * n
        cumulative = 0
        for i, count in enumerate(bins):
            cumulative += count
            if cumulative >= target:
                return (i + 1) * bin_width
        return max_bin

    def _record_rate_bin(self, bins: list[int], rate_pct_s: float) -> None:
        """開度変化率 [%/s] を p95 推定用の固定長ヒストグラムへ記録する。"""
        idx = min(int(abs(rate_pct_s) / _PEDAL_RATE_BIN_WIDTH_PCT_S), len(bins) - 1)
        bins[idx] += 1

    def _update_reversal_count(
        self, delta_pct: float, last_sign: int, *, is_accel: bool
    ) -> int:
        """開度変化の向き反転を数える。ノイズ床未満の変化は無視して符号を据え置く。

        Returns:
            更新後の直前向き符号（次回呼び出しへ渡す）。
        """
        if abs(delta_pct) < _PEDAL_REVERSAL_NOISE_FLOOR_PCT:
            return last_sign
        sign = 1 if delta_pct > 0.0 else -1
        if last_sign != 0 and sign != last_sign:
            if is_accel:
                self._accel_reversal_count += 1
            else:
                self._brake_reversal_count += 1
        return sign

    def summary(self) -> dict[str, float]:
        """走行全体の KPI サマリを返す。走行終了時にログ・公開する。"""
        duration_s = 0.0
        if self._first_now_s is not None and self._last_now_s is not None:
            duration_s = max(0.0, self._last_now_s - self._first_now_s)
        accel_on_per_min = (
            self._accel_on_count / (duration_s / 60.0) if duration_s > 0.0 else 0.0
        )
        pedal_switch_per_min = (
            self._pedal_switch_count / (duration_s / 60.0) if duration_s > 0.0 else 0.0
        )
        effort_rate_rms_pct_s = (
            (self._effort_rate_sq_integral / duration_s) ** 0.5 if duration_s > 0.0 else 0.0
        )
        plan_rms_pct = (
            (self._plan_sq_sum / self._effort_sample_count) ** 0.5
            if self._effort_sample_count > 0
            else 0.0
        )
        trim_rms_pct = (
            (self._trim_sq_sum / self._effort_sample_count) ** 0.5
            if self._effort_sample_count > 0
            else 0.0
        )
        # トリム寄与率＝PID フィードバック量の指標。分母 0 割れ回避に下限を置く。
        trim_share = trim_rms_pct / max(plan_rms_pct, 0.1)
        phase_violation_pct = (
            self._phase_violation_integral / duration_s if duration_s > 0.0 else 0.0
        )
        accel_reversals_per_min = (
            self._accel_reversal_count / (duration_s / 60.0) if duration_s > 0.0 else 0.0
        )
        brake_reversals_per_min = (
            self._brake_reversal_count / (duration_s / 60.0) if duration_s > 0.0 else 0.0
        )
        accel_rate_p95_pct_s = self._bin_p95(
            self._accel_rate_bins,
            sum(self._accel_rate_bins),
            _PEDAL_RATE_BIN_WIDTH_PCT_S,
            _PEDAL_RATE_MAX_BIN_PCT_S,
        )
        brake_rate_p95_pct_s = self._bin_p95(
            self._brake_rate_bins,
            sum(self._brake_rate_bins),
            _PEDAL_RATE_BIN_WIDTH_PCT_S,
            _PEDAL_RATE_MAX_BIN_PCT_S,
        )
        return {
            "n_samples": float(self._n),
            "max_abs_deviation_kmh": self._max_abs_deviation,
            "p95_kmh": self.p95_kmh(),
            "reversal_max_per_5s": float(self._reversal_max_per_window),
            "hard_limit_violations": float(self._hard_violations),
            "over_limit_integral_kmhs": self._over_limit_integral_kmhs,
            "time_over_limit_s": self._time_over_limit_s,
            "accel_on_count": float(self._accel_on_count),
            "accel_on_per_min": accel_on_per_min,
            "pedal_travel_pct": self._pedal_travel_pct,
            "pedal_switch_count": float(self._pedal_switch_count),
            "pedal_switch_per_min": pedal_switch_per_min,
            "effort_rate_rms_pct_s": effort_rate_rms_pct_s,
            "plan_rms_pct": plan_rms_pct,
            "trim_rms_pct": trim_rms_pct,
            "trim_share": trim_share,
            "phase_violation_pct": phase_violation_pct,
            "accel_reversals_per_min": accel_reversals_per_min,
            "brake_reversals_per_min": brake_reversals_per_min,
            "accel_rate_p95_pct_s": accel_rate_p95_pct_s,
            "brake_rate_p95_pct_s": brake_rate_p95_pct_s,
        }
