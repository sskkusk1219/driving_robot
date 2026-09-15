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

import math
from dataclasses import dataclass, field
from enum import StrEnum

import numpy as np

from src.domain.control.conversions import VEHICLE_STOP_SPEED_KMH
from src.domain.control.feedforward import FeedforwardController
from src.models.driving_mode import DrivingMode
from src.models.profile import FeedforwardParams, coast_decel_at, pedal_gain_at

# ── ペダルプラン既定パラメータ ─────────────────────────────────────────────
PLAN_DT_S: float = 0.1  # プラングリッド周期 [s]（drive_logs / ILC と一致）
# 必要加速度 a_req の平滑窓 [s]（センタリング移動平均）。人間は瞬時勾配でなく傾向で踏む。
# 2026-09-09 に「コーナーのにじみの一因」と見て 0.3 へ縮める案を検討したが、閉ループ模擬
# （scripts/simulate_control）で **1.0 のほうが一貫して良い**と出たため据え置いた
# （lead=0.2s/LPF=1.0Hz で p95 1.54 vs 0.3s の 1.57。全 12 通りの組み合わせで同傾向）。
# センタリング窓なので位相遅れは無く、フェーズ分類（classify_phases）を安定させる効果の
# ほうが大きい。コーナーのにじみは後段の zero_phase_lowpass がほぼ全てだった
# （PLAN_LOWPASS_HZ のコメント参照）。
PLAN_ACCEL_SMOOTH_S: float = 1.0
# a_req 平滑窓の中心位置。1.0=センタリング（前後 ±smooth_s/2 を見る＝非因果）、
# 0.0=完全な後方窓（過去だけを見る＝因果）。窓の**幅**は上のとおり 1.0s が最良なので、
# 折返し点の早抜けに効くのは幅ではなく「未来をどれだけ先食いするか」＝中心位置のほう、
# という見立てだった（docs/Problem/引き継ぎ20260909.md 優先A 案2）。
#
# 2026-09-10 掃引の結果、**この見立ては否定されたので 1.0（センタリング）に据え置く**。
# scripts/simulate_control --sweep-lead-mode（__verify_pattern__、lead=0.20s）で
#   center=1.00: p95 3.21 / max 5.70    center=0.50: p95 3.28 / max 5.80
#   center=0.25: p95 3.36 / max 5.84    center=0.00: p95 3.84 / max 5.85
# と、後方窓へ寄せるほど p95・max とも単調に悪化した（lead=0.42s でも同傾向）。
# センタリング窓は確かに 0.2s 先食いする（三角波の頂点 5.00s に対し a_req は 4.80s で
# 減速へ転じる）が、因果窓にすると今度は 0.2s 遅れるぶん頂点での不足が増える。
# 引数は掃引できるよう残す（実機の傾向が変わったら再評価する）。
PLAN_ACCEL_SMOOTH_CENTER: float = 1.0
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
#
# 2026-09-09: 0.25 → 1.0。ゼロ位相（forward-backward）は位相遅れが 0 な代わりに
# **非因果**で、コーナーを前後に対称ににじませる。0.25Hz は RC=1/(2π·0.25)=0.64s を
# 両方向に掛けるため、実効的に前後 ~1.3s の「先食い」が出ていた。実機 9eee549b（best run
# a7dc98f6）の実測:
#   - t=82.0s: 解析 effort −5.03% に対しプラン −2.58%（**49% 減衰**）。基準はまだ
#     −5.0km/h/s で減速中なのに t≈79.6s からブレーキを抜き始め、+4.70km/h の誤差。
#   - t=146s の軌跡頂点: 基準はまだ +2.0km/h/s なのに t≈143s からアクセルを抜き始め、
#     −4.60km/h の誤差。
# この 2 エピソードだけで p95 のほぼ全量を占めていた。1.0Hz（RC=0.16s）ならコーナーは
# 保ちつつ、FF モデル出力のギザつきは落とせる。閉ループ模擬（scripts/simulate_control、
# lead=0.2s）で p95 1.75（0.25Hz）→ 1.54（1.0Hz）。2.0Hz でも p95 はほぼ同じだが開度
# 変化率が上がるため 1.0Hz を採る。
PLAN_LOWPASS_HZ: float = 1.0


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

    クリープ速度未満はクリープが車を押す（+creep_rate）、以上は惰行減速カーブ
    （coast_decel_at: 同定済みなら速度依存の補間、未同定は engine_brake_decel 定数）で
    減速する。フェーズ分類の基準線。速度依存を無視すると、実惰行が基準より強い速度域で
    緩減速を BRAKE と誤分類し、プランが必要な正 effort を出せなくなる（sample_004 実機）。
    """
    if v < params.creep_speed_kmh:
        return params.creep_rate_kmhs
    return -coast_decel_at(params, v)


def required_accel(
    speeds: np.ndarray,
    dt_s: float,
    smooth_s: float | None = None,
    center_ratio: float | None = None,
) -> np.ndarray:
    """基準速度列から必要加速度 a_req [km/h/s] を求める（移動平均で平滑化）。

    smooth_s / center_ratio が None なら PLAN_ACCEL_SMOOTH_S / PLAN_ACCEL_SMOOTH_CENTER を
    **呼び出し時に**参照する。既定値として引数に束縛すると def 実行時に固定され、
    テストや実験で定数を差し替えても効かない。

    center_ratio は窓の中心位置で、1.0 がセンタリング（従来の `mode="same"` 相当＝前後
    ±smooth_s/2 を見る非因果平滑）、0.0 が完全な後方窓（過去のみ＝因果）。センタリング窓は
    位相遅れが 0 な代わりに未来を先食いするため、折返し点では「基準がまだ加速を要求して
    いるのに a_req が先に下がる」＝早抜けの一因になる。
    """
    if smooth_s is None:
        smooth_s = PLAN_ACCEL_SMOOTH_S
    if center_ratio is None:
        center_ratio = PLAN_ACCEL_SMOOTH_CENTER
    if speeds.size < 2:
        return np.zeros_like(speeds, dtype=float)
    a: np.ndarray = np.gradient(speeds.astype(float), dt_s)
    w = max(1, int(round(smooth_s / dt_s)))
    if w <= 1:
        return a
    kernel = np.ones(w) / w
    # mode="full" の要素 i は a[i-w+1 .. i]（＝後方窓）を覆う。出力 i に full[i+offset] を
    # 使うと窓は a[i+offset-w+1 .. i+offset] になるので、offset=0 が後方窓（因果）、
    # offset=(w-1)/2 がセンタリング、offset=w-1 が完全な前方窓になる。
    full = np.convolve(a, kernel, mode="full")
    ratio = min(1.0, max(0.0, center_ratio))
    offset = int(round((w - 1) * 0.5 * ratio))
    return full[offset : offset + a.size]


# 解析 effort とモデル出力のブレンド境界 [%]。|解析 effort| がこの下限以下ならモデルを
# 使わず解析値 100%、上限以上ならモデル 100%、間は線形にブレンドする。
# 根拠（実機 3ca20d43 / 学習運転 3eebcff3）: 学習運転のペダル開度サンプルは 10% 以上に
# 集中し、0.5-10% は速度域あたり 0.7-6.2 秒しかない。逆FFモデル（accel R²=0.93）が
# 検証済みなのは高開度側だけで、低開度側は外挿になる。ところが WLTP は減速時間の 75% が
# 「惰行より緩い減速」＝低開度でアクセルを当てる領域で、まさに外挿域を使う。
# そこでモデルが強い高開度側は温存し、外挿しかできない低開度側だけ解析値に置き換える。
ANALYTIC_BLEND_LO_PCT: float = 5.0
ANALYTIC_BLEND_HI_PCT: float = 15.0


def analytic_efforts(
    speeds: np.ndarray, accels: np.ndarray, params: FeedforwardParams
) -> np.ndarray:
    """惰行カーブとペダルゲインから必要 effort [%] を解析的に求める（同定なしは NaN）。

    effort = (a_req − a_coast(v)) / ペダルゲイン(v) + 不感帯。Δa=0（＝惰行そのまま）で
    effort=0 が構造的に保証されるため、学習データが無い低開度域が「モデルの外挿」ではなく
    「原点との内挿」になる。不感帯を足すのは PedalArbiter が開度を max(deadband, |effort|)
    に丸めるため（ゲインは不感帯超の開度で同定している。model_training.
    _estimate_pedal_gain_curve 参照）。

    ペダルゲイン未同定の向き・停車域は np.nan を返し、呼び出し元がモデル出力へ
    フォールバックできるようにする。
    """
    out = np.full(len(speeds), np.nan, dtype=float)
    accel_db = max(0.0, params.accel_deadband_pct)
    brake_db = max(0.0, params.brake_deadband_pct)
    for i, (v, a) in enumerate(zip(speeds, accels)):
        v_f = float(v)
        if v_f < VEHICLE_STOP_SPEED_KMH:
            continue  # 停車保持は clamp_effort_by_phase の保持 effort が支配する
        delta_a = float(a) - coast_accel(v_f, params)
        if delta_a > 0.0:
            gain = pedal_gain_at(params, v_f, is_accel=True)
            if gain is not None:
                out[i] = delta_a / gain + accel_db
        elif delta_a < 0.0:
            gain = pedal_gain_at(params, v_f, is_accel=False)
            if gain is not None:
                out[i] = -(-delta_a / gain + brake_db)
        else:
            out[i] = 0.0
    return out


def blend_model_and_analytic(model: np.ndarray, analytic: np.ndarray) -> np.ndarray:
    """モデル出力と解析 effort を |解析 effort| の大きさで線形ブレンドする。

    低開度側（ANALYTIC_BLEND_LO_PCT 以下）は解析値 100%、高開度側
    （ANALYTIC_BLEND_HI_PCT 以上）はモデル 100%。解析値が NaN（ゲイン未同定・停車域）の
    要素はモデル出力をそのまま使う＝ペダルゲイン未同定なら従来と完全に同じ結果になる。
    """
    analytic = np.asarray(analytic, dtype=float)
    model = np.asarray(model, dtype=float)
    span = max(ANALYTIC_BLEND_HI_PCT - ANALYTIC_BLEND_LO_PCT, 1e-9)
    w_model = np.clip((np.abs(analytic) - ANALYTIC_BLEND_LO_PCT) / span, 0.0, 1.0)
    blended = w_model * model + (1.0 - w_model) * analytic
    return np.where(np.isnan(analytic), model, blended)


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


def fold_times(plan_efforts: list[float], phases: list[PlanPhase], dt_s: float) -> list[float]:
    """プランの「折返し点」の時刻 [s] を返す（駆動⇄制動が入れ替わる点）。

    折返し点は基準軌跡の中で要求 effort の時間微分が最大の点であり、時間軸の誤差 Δt が
    そのまま |Δeffort| = |d(effort)/dt|·Δt の指令誤差になる。ここで前倒しすると
    「基準がまだ加速を要求しているのにアクセルを抜く」真逆の操作になるため、
    走行時は折返し点に近づくほど前倒しを絞る（DriveLoop._plan_lead_at）。

    検出は 2 系統の OR:
      - effort の符号反転（0 を挟む往復も含む。直近の非ゼロ符号を保持して判定する）
      - フェーズが DRIVE ⇄ BRAKE へ変わる点（COAST を挟む場合も折返しとみなす）

    Args:
        plan_efforts: プランの名目 effort 列 [%]。
        phases: 同じ長さのフェーズ列。
        dt_s: グリッド周期 [s]。

    Returns:
        折返し時刻の昇順リスト。
    """
    times: set[float] = set()
    last_sign = 0
    last_side: PlanPhase | None = None
    for i, effort in enumerate(plan_efforts):
        sign = 1 if effort > 0.0 else (-1 if effort < 0.0 else 0)
        if sign != 0:
            if last_sign != 0 and sign != last_sign:
                times.add(i * dt_s)
            last_sign = sign
        phase = phases[i] if i < len(phases) else None
        if phase in (PlanPhase.DRIVE, PlanPhase.BRAKE):
            if last_side is not None and phase != last_side:
                times.add(i * dt_s)
            last_side = phase
    return sorted(times)


def zero_phase_lowpass(x: np.ndarray, cutoff_hz: float, dt_s: float) -> np.ndarray:
    """1次 IIR の forward-backward 適用でゼロ位相ローパスする（位相遅れ 0）。

    cutoff_hz<=0 や短すぎる系列はそのまま返す。RC=1/(2π·fc)、α=dt/(RC+dt) の指数平滑を
    前方・後方に適用して群遅延を打ち消す。ペダルプランの名目 effort 平滑とプラン更新則
    （plan_update.py）で共用する。
    """
    if cutoff_hz <= 0.0 or dt_s <= 0.0 or x.size < 2:
        return np.asarray(x, dtype=float)
    rc = 1.0 / (2.0 * math.pi * cutoff_hz)
    alpha = dt_s / (rc + dt_s)
    forward = _ewma(x, alpha)
    backward = _ewma(forward[::-1], alpha)[::-1]
    return backward


def _ewma(x: np.ndarray, alpha: float) -> np.ndarray:
    """指数加重移動平均（1次 IIR ローパス）。y[i] = y[i-1] + α(x[i]−y[i-1])。"""
    y = np.empty_like(x, dtype=float)
    acc = float(x[0])
    for i in range(x.size):
        acc += alpha * (float(x[i]) - acc)
        y[i] = acc
    return y


def snap_efforts_to_deadband(
    efforts: list[float], params: FeedforwardParams
) -> list[float]:
    """プラン effort を PedalArbiter._apply_deadband と同一規則で不感帯へ量子化する。

    アービタは |effort|<deadband/2 を開度 0 に、それ以上を max(deadband, |effort|) に丸める
    （物理的な遊びを踏み越えない指令はペダルを動かさない）。プラン合成はこれを知らずに
    ±数% の「計画したのに実際は何も起きない」effort を出し得るため、生成・更新の最終段で
    同じ規則を適用してプランと実効値を一致させる（reward・トリム基準の整合、sample 2026-07-14
    実機: −2.0km/h/s 要求にプラン −2.4% → brake_deadband=6.0 未満で制動ゼロ）。

    アービタ側の規則を変更したら、ここも同期して更新すること。符号は変えない
    （0 化または絶対値の拡大のみ）ため、clamp_effort_by_phase 後に適用してもフェーズ権限
    （DRIVE≥0/BRAKE≤0/COAST=0）は保たれる。
    """
    accel_db = max(0.0, params.accel_deadband_pct)
    brake_db = max(0.0, params.brake_deadband_pct)
    out: list[float] = []
    for e in efforts:
        if e > 0.0:
            out.append(0.0 if accel_db > 0.0 and e < accel_db / 2.0 else max(accel_db, e))
        elif e < 0.0:
            mag = -e
            if brake_db > 0.0 and mag < brake_db / 2.0:
                out.append(0.0)
            else:
                out.append(-max(brake_db, mag))
        else:
            out.append(0.0)
    return out


def clamp_effort_by_phase(
    efforts: np.ndarray, phases: list[PlanPhase], params: FeedforwardParams
) -> list[float]:
    """フェーズ整合クランプ: DRIVE≥0 / BRAKE≤0 / COAST=0 / STOP_HOLD=停車保持ブレーキ。

    ローパスの滲みで DRIVE 区間に負値・BRAKE 区間に正値が漏れるのを断つ。停車保持は
    プランが支配し、stop_brake_opening_pct（最低でも brake_deadband_pct）の制動 effort を置く。
    プラン生成・プラン更新（plan_update.py）の両方で使い、フェーズ権限を機構的に保証する。
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

        a_req = required_accel(grid_v, dt_s, PLAN_ACCEL_SMOOTH_S)
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
        else:
            raw = np.zeros(n, dtype=float)

        # 低開度域（モデルにとっては外挿域）を惰行カーブ基準の解析値で置き換える。
        # ペダルゲイン未同定なら analytic は全 NaN ＝ raw がそのまま残り従来動作。
        analytic = analytic_efforts(grid_v, a_req, params)
        if ff.has_model:
            blended = blend_model_and_analytic(raw, analytic)
        else:
            blended = np.where(np.isnan(analytic), 0.0, analytic)
        smoothed = zero_phase_lowpass(blended, PLAN_LOWPASS_HZ, dt_s)

        efforts = clamp_effort_by_phase(smoothed, phases, params)
        efforts = snap_efforts_to_deadband(efforts, params)
        return PedalPlan(dt_s=dt_s, efforts=efforts, phases=phases)


__all__ = [
    "ANALYTIC_BLEND_HI_PCT",
    "ANALYTIC_BLEND_LO_PCT",
    "MIN_PHASE_S",
    "PHASE_MARGIN_KMHS",
    "PLAN_ACCEL_SMOOTH_CENTER",
    "PLAN_ACCEL_SMOOTH_S",
    "PLAN_DT_S",
    "PLAN_LOWPASS_HZ",
    "PedalPlan",
    "PedalPlanner",
    "PlanPhase",
    "analytic_efforts",
    "blend_model_and_analytic",
    "clamp_effort_by_phase",
    "classify_phases",
    "coast_accel",
    "fold_times",
    "merge_micro_phases",
    "required_accel",
    "snap_efforts_to_deadband",
    "zero_phase_lowpass",
]
