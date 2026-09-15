"""エピソード型方策改善の報酬スコア（ドメイン純関数）。

1走行＝1エピソード、ペダルプラン＝方策とみなし、走行1本の KPI サマリから単一の報酬
（大きいほど良い）を算出する。走行後にこの報酬が過去最良を上回ったときだけ次回プランを
採用し、下回れば最良プランへロールバックする（PedalPlanService、単調改善保証）。

報酬 4 項（明電舎技報 No.364 の深層強化学習の報酬定義を参考にユーザーが選定）:
  ① 追従性（W_TRACK・支配項）: KPI（p95 / max / ハード超過積分）で正規化
  ② 滑らかさ（W_SMOOTH）: applied effort の変化率 RMS
  ③ 踏み替えなし（W_SWITCH）: アクセル⇔ブレーキ交互踏み回数/min
  ④ フェーズ逸脱なし（W_PHASE）: 速い補正層がフェーズ権限を無視して介入した量

**役割分離**: 本モジュールの `reward_score`（最大化）は「走行後のプラン採否」を判定する。
`pid_tuning.tuning_cost`（最小化）は「座標降下のゲイン探索」に使う別物で、実機 2 セッションで
校正済みの重みを持つため統合・変更しない。両者は KPI しきい値（kpi_monitor）を単一ソースと
して共有する。

人間の運転スタイル（クリープ発進・開度一定・エンジンブレーキ優先・停止中保持）そのものは
報酬に入れない。フェーズ分類・フェーズ権限・STOP_HOLD・凍結帯という構造的制約で既に保証
されており（報酬＝お願い より強い）、報酬への重複計上は追従支配項とのバランスを崩す。
構造保証の外にある「安全網介入によるスタイル逸脱」のみを ④ で計測する。
"""

from __future__ import annotations

import statistics
from collections.abc import Mapping, Sequence

from src.domain.control.kpi_monitor import KPI_HARD_LIMIT_KMH, KPI_P95_LIMIT_KMH

# ── 報酬の重み ─────────────────────────────────────────────────────────
W_TRACK: float = 1.0  # 追従項（支配項）。KPI 限界時に追従項 ≈ 3 で他項を上回る
# 追従項の内訳。実機 iter 10（通算ベスト）での寄与は
#   p95/0.4 = 3.55（2.4%）／2×max = 8.11（5.5%）／10×超過積分 = 94.31（**89.0%**）
# だった。
#
# **超過積分の重み 10.0 は下げない。** 報酬の 89% がここなのは「最大逸脱（PRD: 全走行区間で
# 1.0km/h を超えないこと・例外なし）を最重視している」という意味で、PRD の優先度と整合して
# いる。13 反復で max が下がらなかったのは重み付けの問題ではなく、ILC が動かせるのがプランの
# effort **値**だけで時間軸を動かせないため＝原因に手が届かなかったから
# （docs/Problem/引き継ぎ20260909.md 4-①(b)）。ここを緩めると max を守る唯一の圧力を失う。
#
# 2026-09-10: p95 項の係数だけ 1.0 → 4.0 に上げた。p95 は係数上 2.4% しか効かず、iter 12 は
# p95 1.43（サイクル最良）を出したのに max 5.14 の一撃で reward が伸びず不採用になっていた。
# 採否そのものは辞書順（PedalPlanService._decide_outcome）で直したが、候補生成の勾配としても
# p95 が効くようにする。
W_TRACK_P95: float = 4.0
W_TRACK_MAX: float = 2.0
W_TRACK_OVER: float = 10.0
W_SMOOTH: float = 0.4  # 滑らかさ項（effort 変化率 RMS）（2026-09-08: 0.2→0.4。
# ProblemReport_20260908「操作が雑」対応で、プラン学習の探索を滑らかさ側へ寄せる）
W_SWITCH: float = 0.2  # 踏み替え項（交互踏み回数/min）
W_PHASE: float = 0.2  # フェーズ逸脱項（安全網介入量）
# effort 変化率 RMS の正規化 [%/s]。トリムのスルーレート 3%/s・プランのローパス 0.25Hz から
# 定常の変化率を机上見積もりした基準値。実機実績で将来調整する。
EFFORT_RATE_NORM_PCT_S: float = 2.0

# ── 走行間ばらつき（σ）の推定パラメータ ────────────────────────────────────
# 同一プランでも報酬は走行ごとにばらつく。この σ を採否判定の許容幅に使い、ノイズ相当の
# 悪化で学習が止まらないようにする（reward_noise_sigma / PedalPlanService）。
REWARD_SIGMA_WINDOW: int = 5  # σ 推定に使う直近サンプル数
REWARD_SIGMA_MIN_SAMPLES: int = 3  # σ を返すのに必要な最小サンプル数
# 「ノイズでは説明しにくい悪化」の境界。reward > best − K·σ なら探索を継続し、それ未満で
# のみ最良プランへロールバックする。片側 2σ ≒ 97.7% 相当。
REWARD_NOISE_SIGMA_K: float = 2.0


def reward_score(kpi: Mapping[str, float]) -> float:
    """走行 1 本の KPI サマリから報酬（大きいほど良い）を返す。

    追従項は tuning_cost と同じ KPI 正規化（p95 / max / ハード超過積分）を用い、
    符号を反転して「大きいほど良い」に揃える。キー欠損（学習運転などペダル・effort 情報が
    無い走行の summary）は 0 扱いで、追従項のみで評価される。

    Args:
        kpi: KPIMonitor.summary() 相当の辞書。

    Returns:
        報酬スコア（負の重み付きコスト。大きいほど良い走行）。
    """
    track = (
        W_TRACK_P95 * kpi.get("p95_kmh", 0.0) / KPI_P95_LIMIT_KMH
        + W_TRACK_MAX * kpi.get("max_abs_deviation_kmh", 0.0) / KPI_HARD_LIMIT_KMH
        + W_TRACK_OVER * kpi.get("over_limit_integral_kmhs", 0.0)
    )
    smooth = kpi.get("effort_rate_rms_pct_s", 0.0) / EFFORT_RATE_NORM_PCT_S
    switch = kpi.get("pedal_switch_per_min", 0.0)
    phase = kpi.get("phase_violation_pct", 0.0)
    return -(
        W_TRACK * track + W_SMOOTH * smooth + W_SWITCH * switch + W_PHASE * phase
    )


def reward_noise_sigma(
    rewards: Sequence[float], *, min_samples: int = REWARD_SIGMA_MIN_SAMPLES
) -> float | None:
    """直近の報酬列から走行間ばらつき σ を推定する。サンプル不足なら None。

    同一プランを走っても報酬は走行ごとにばらつく（実機 2026-07-23 の本番モードでは
    同一プランの 4 本が −53.5 / −99.3 / −76.1 / −88.1 で σ≈19）。このばらつきを
    「悪化」と誤認すると学習が止まるため、採否判定の許容幅に使う（PedalPlanService）。

    Args:
        rewards: 報酬の時系列（古い順）。直近 REWARD_SIGMA_WINDOW 件のみ使う。
        min_samples: σ を返すのに必要な最小サンプル数。

    Returns:
        標本標準偏差。サンプル不足なら None（呼び出し側は「判定不能＝探索を続ける」に倒す）。
    """
    window = list(rewards)[-REWARD_SIGMA_WINDOW:]
    if len(window) < max(2, min_samples):
        return None
    return float(statistics.stdev(window))


def reward_scale_key() -> str:
    """報酬の比較可能性を決める正規化条件のキー。

    `reward_score` は KPI しきい値で正規化するため、しきい値を変えると報酬のスケール自体が
    変わり、変更前に記録した最良報酬と変更後の報酬は比較できない（2026-07-16 の
    p95 0.2→0.4 で実際に発生し、旧スケールの best_reward が学習を阻害した）。永続化した
    報酬にこのキーを添えておき、不一致なら最良報酬を再ベースラインする。

    2026-09-10: **追従項の重みもキーに含める**ように直した。旧実装はしきい値だけから
    キーを作っていたため、`W_TRACK_P95` などの係数を変えても再ベースラインが走らず、
    旧スケールの best_reward が残って学習が止まる状態だった
    （`docs/Problem/引き継ぎ20260909.md:498` は「重みを変えるとキーが変わる」と書いているが、
    現コードはそうなっていなかった）。重みを変えたらキーも変わるので、必ずここを通ること。
    """
    return (
        f"p95={KPI_P95_LIMIT_KMH:g};hard={KPI_HARD_LIMIT_KMH:g}"
        f";w={W_TRACK_P95:g}/{W_TRACK_MAX:g}/{W_TRACK_OVER:g}"
    )


__all__ = [
    "EFFORT_RATE_NORM_PCT_S",
    "REWARD_NOISE_SIGMA_K",
    "REWARD_SIGMA_MIN_SAMPLES",
    "REWARD_SIGMA_WINDOW",
    "W_PHASE",
    "W_SMOOTH",
    "W_SWITCH",
    "W_TRACK",
    "W_TRACK_MAX",
    "W_TRACK_OVER",
    "W_TRACK_P95",
    "reward_noise_sigma",
    "reward_scale_key",
    "reward_score",
]
