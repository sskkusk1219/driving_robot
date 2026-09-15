"""エピソード型プラン更新則（ドメイン純関数）。ILCLearner の後継。

シャシダイナモの同一モード反復走行は再現性が高い。1走行＝1エピソードとみなし、走行で
実際に適用した effort（フェーズ権限クランプ後・調停器前の符号付き合成値）と追従残差から、
次回のペダルプラン全体を更新する:

    次回プラン = snap( clamp_by_phase(
        FF基準 + clip( Q(u_applied + L·e(t+Δ)) − FF基準, ±Δlimit ) ) )

- u_applied: 走行中に記録した applied effort（PedalArbiter は非可逆なので開度から復元せず
  記録値を使う）
- Q: ゼロ位相ローパス（forward-backward 1次 IIR、pedal_plan と共用）
- L: 学習ゲイン = factor / fopdt_k（プラント定常ゲインで正規化。旧 ILC 則を継承）
- Δ: リード時間 = theta + tau_factor·tau（e(t+Δ) を参照して因果を揃える。theta のみでは
  一次遅れ tau 分の補正遅れが残り、反復ごとに位相がずれて発散する。lead_time_from_fopdt 参照）
- **フェーズ列は基準（FF由来）プランのまま固定維持**し、更新後 effort に
  clamp_effort_by_phase を再適用する。DRIVE≥0 / BRAKE≤0 / COAST=0 / STOP_HOLD=停車保持が
  保たれ、プラン更新による不要なペダル切替の再発が構造的に起こらない。
- **クランプは FF由来プランとの差分 ±Δlimit**。プラン絶対値は数十%になるため絶対クランプは
  使えない。旧 ILC の ±10% 補正クランプの意味的等価物で、発散を機構的に FF 近傍へ拘束する。
- snap: フェーズクランプ後にアービタと同一規則の不感帯量子化（pedal_plan.
  snap_efforts_to_deadband）を適用し、「計画したのに実際は何も起きない」微小 effort を
  排除する（2026-07-14 実機解析: F2 対策）。

採否（reward による単調改善）は上位の PedalPlanService が担う。本モジュールは「更新後の
候補プラン」を計算するだけ（純関数）。
"""

from __future__ import annotations

from collections.abc import Sequence

import numpy as np

from src.domain.control.pedal_plan import (
    PedalPlan,
    PlanPhase,
    clamp_effort_by_phase,
    snap_efforts_to_deadband,
    zero_phase_lowpass,
)
from src.models.profile import FeedforwardParams, pedal_gain_at

# ── プラン更新の既定パラメータ ─────────────────────────────────────────────
# Q（学習フィルタ）のカットオフ [Hz]。
# 2026-09-10 実測で 0.2 → 0.5。zero_phase_lowpass は forward-backward なので位相遅れは 0 だが
# **非因果**で、RC=1/(2π·fc) を両方向に掛ける。0.2Hz は RC=0.80s ＝ 折返し点の前後 ±0.8s へ
# 補正がにじみ、**頂点に鋭いピークを立てられない**（docs/Problem/制御フロー.md ⑤-c）。
# 実機 5ac4f31d の頂点区間（t=143.0-146.5s、ピーク偏差 −5.14）で候補プランを作り直したところ、
# 基準プランの平均 9.53% に対する踏み増し量は
#   0.2Hz: +3.01% / 0.5Hz: +3.98% / 1.0Hz: +4.10%
# で、0.2→0.5 が支配的（+32%）、0.5→1.0 は頭打ち。必要開度の目安 16.9% に対し
# 0.5Hz で頂点最大 17.20% まで届く。1.0Hz は伸びが小さい割にプランの高周波成分を増やす
# （滑らかさ項 W_SMOOTH に効く）ので 0.5 を採る。
PLAN_UPDATE_CUTOFF_HZ: float = 0.5
PLAN_DELTA_LIMIT_PCT: float = 8.0  # FF由来プランからの差分クランプ [%]（2026-09-09 修正: 4.0→8.0。
# 2026-09-08 に 10.0→4.0 と絞ったが、実機 3ca20d43 では最も補正が必要な区間（緩減速の入口
# t=81-82s と軌跡頂点 t=146.6s）で ±4.0 に 4.1 秒張り付き、必要な補正量が切られていた。
# 惰行カーブ基準の解析プラン（pedal_plan.analytic_efforts）で基準プラン自体が正しくなり
# 大きな補正が要らなくなったので、上限は張り付かない側に緩める）
# 2026-09-10 追記: 「頂点で ±8% が binding している」という見立て（引き継ぎ20260909.md ⑤-b）は
# **実測で否定された**。同じ頂点区間で Δlimit を 8/12/15% と振っても踏み増し量は
# +3.98 → +4.00 → +4.00%（0.5Hz 時）でほとんど動かない。binding しているのは Δlimit ではなく
# 上の Q カットオフだったので、8.0 のまま据え置く（効かないクランプを緩めても得は無く、
# 学習が基準プランから離れられる幅だけが広がる）。
PLAN_L_GAIN_FACTOR: float = 0.25  # 学習ゲインの無次元係数（下記 l_gain_at 参照）
# （2026-09-08 修正: 0.4→0.25。1 反復目で改善しても 2 反復目から発散する事例があり、
#  多段収束に寄せて緩めた）
# むだ時間シフト Δ = θ + PLAN_LEAD_RESPONSE_S [s]。
# 2026-09-09 修正: 旧 Δ = θ + tau_factor·τ を廃止。τ は identify_fopdt が「アクセル一定保持で
# 車速がプラトーに達する」前提で求める値だが、実機の学習運転は 17 区間すべてで到達しておらず
# τ も k も保持区間の長さを測っているだけだった（models.profile.robust_kp_at の docstring）。
# 応答時定数の代わりに固定値を置く。
#
# 2026-09-10 実測で決定: 0.5 → 0.0（＝ Δ = θ）。
# 第3ラウンドの時点では「暫定・実測の裏付けなし」と書いていたが、
# scripts/replay_plan_update.py の集計バグ（args.tau_factor 未定義で必ず AttributeError）を
# 直して全 5 サイクル 26 ペアで比較したところ、Δ=θ のほうが「次回プラン差分 Δplan が前走行の
# 誤差に正しく追従している」相関が強いペアが 18/26 で優位だった。さらに追加リードを
# 0.25/0.5/1.0/1.5s と増やすほど単調に悪化する（Δ=θ の勝ちが 18→18→20→22）。
# 再現: .venv/bin/python -m scripts.replay_plan_update --all-cycles [--response-s X]
#
# 意味: 積分系プラントでは時刻 t の指令が θ 秒後の加速度に効くので、時刻 t のプランを直すのに
# 見るべき誤差は e(t+θ)。「そこから応答が立ち上がるまで」を足した分は**過剰な前倒し**で、
# 折返し点での早抜け（最大逸脱の第1要因）に加算されていた
# （docs/Problem/引き継ぎ20260909.md 4-③ の前倒し 3 段のうち 1 段）。
PLAN_LEAD_RESPONSE_S: float = 0.0
# 更新に必要な最小ログ数（これ未満は残差推定に足りないので更新しない）。
MIN_LOGS_FOR_UPDATE: int = 50
# 完走判定: 末尾ログ時刻がプラン長のこの割合以上でないと更新しない（途中データでの尾部破壊防止）。
MIN_COVERAGE_RATIO: float = 0.9


def l_gain_at(
    params: FeedforwardParams,
    v_kmh: float,
    delta_s: float,
    *,
    is_accel: bool,
    factor: float = PLAN_L_GAIN_FACTOR,
    fallback: float = PLAN_L_GAIN_FACTOR,
) -> float:
    """速度 v での ILC 学習ゲイン L [%/(km/h)] を返す。

    プラン effort を Δu だけ変えると、リード時間 Δ のあいだに車速は k'(v)·Δu·Δ だけ動く
    （k' = 同定済みペダルゲイン [km/h/s per %]）。よって速度誤差 e を消すのに必要な
    effort 変化は e/(k'·Δ) で、学習ゲインは

        L = factor / (k'(v) · Δ)

    2026-09-09 修正: 旧実装は L = factor/fopdt_k だった。実機 9eee549b では
    fopdt_k=3.51 から L=0.071 %/(km/h) となり、4km/h の誤差に対してプラン補正が 0.28% しか
    出ない＝実質学習していない状態だった（PLAN_DELTA_LIMIT_PCT=8.0 に対して桁が違う）。
    しかも fopdt_k は学習運転がプラトーに達しないと保持区間長を測るだけの値になる
    （models.profile.robust_kp_at の docstring）。

    ペダルゲイン未同定・Δ≤0 のときは fallback をそのまま返す（従来動作）。
    """
    if delta_s <= 0.0:
        return fallback
    gain = pedal_gain_at(params, v_kmh, is_accel=is_accel)
    if gain is None or gain <= 0.0:
        return fallback
    return factor / (gain * delta_s)


def l_gain_from_fopdt(fopdt_k: float | None, factor: float = PLAN_L_GAIN_FACTOR) -> float:
    """旧・FOPDT 由来の学習ゲイン L = factor / fopdt_k。

    ペダルゲイン曲線が未同定のときのフォールバック値としてのみ使う（l_gain_at 参照）。
    """
    if fopdt_k is None or fopdt_k <= 0.0:
        return factor
    return factor / fopdt_k


def lead_time_from_fopdt(
    fopdt_theta: float | None,
    fopdt_tau: float | None = None,
    *,
    response_s: float = PLAN_LEAD_RESPONSE_S,
) -> float:
    """むだ時間シフト Δ = theta + response_s [s]。

    プラン更新は「前走行の時刻 t+Δ の誤差」を「時刻 t のプラン」に反映する。Δ にはプラント
    のむだ時間 θ と、そこから応答が立ち上がるまでの時間を見込む。

    2026-09-09 修正: 旧実装の Δ = θ + tau_factor·τ から τ 項を外した（PLAN_LEAD_RESPONSE_S
    のコメント参照）。引数 fopdt_tau は呼び出し互換のため受けるが**未使用**。
    theta 未同定なら response_s のみ。
    """
    theta = max(0.0, fopdt_theta) if fopdt_theta is not None else 0.0
    return theta + max(0.0, response_s)


def update_plan(
    base_plan: PedalPlan,
    log_times_s: Sequence[float],
    applied_efforts: Sequence[float],
    ref_speeds: Sequence[float],
    actual_speeds: Sequence[float],
    params: FeedforwardParams,
    *,
    l_gain: float,
    delta_s: float,
    delta_limit_pct: float = PLAN_DELTA_LIMIT_PCT,
    cutoff_hz: float = PLAN_UPDATE_CUTOFF_HZ,
) -> PedalPlan | None:
    """走行実測から次回の候補プランを計算する。ガード不成立時は None（更新スキップ）。

    Args:
        base_plan: FF由来の基準プラン（更新時に現行 FF モデルで再生成したもの）。
            この phases を固定維持し、effort だけを更新する。
        log_times_s: 走行開始基準の相対秒（~0.1s 刻み・ジッタあり、単調増加想定）。
        applied_efforts: 各時刻の applied effort [%]（フェーズ権限クランプ後・調停器前）。
        ref_speeds: 各時刻の基準車速 [km/h]。
        actual_speeds: 各時刻の実車速 [km/h]。
        params: 車両物理定数（フェーズ整合クランプの停車保持値等に使う）。
        l_gain: 学習ゲイン L（>0）。
        delta_s: むだ時間シフト Δ [s]。
        delta_limit_pct: FF由来プランからの差分の上限 [%]。
        cutoff_hz: ゼロ位相ローパスのカットオフ [Hz]。

    Returns:
        更新後の候補 PedalPlan（phases は base_plan と同一）。ガード不成立なら None。
    """
    n = len(base_plan.efforts)
    if n == 0 or base_plan.dt_s <= 0.0:
        return None
    if len(log_times_s) < MIN_LOGS_FOR_UPDATE:
        return None
    if len(log_times_s) != len(applied_efforts) or len(log_times_s) != len(ref_speeds):
        return None
    if len(log_times_s) != len(actual_speeds):
        return None
    if log_times_s[-1] < MIN_COVERAGE_RATIO * base_plan.duration_s:
        return None

    dt = base_plan.dt_s
    grid = np.arange(n, dtype=float) * dt
    t_arr = np.asarray(log_times_s, dtype=float)
    u_arr = np.asarray(applied_efforts, dtype=float)
    err_arr = np.asarray(ref_speeds, dtype=float) - np.asarray(actual_speeds, dtype=float)

    u_grid = np.interp(grid, t_arr, u_arr)  # 実測 applied → 0.1s グリッド
    e_shifted = np.interp(grid + delta_s, t_arr, err_arr)  # e(t+Δ)
    # 学習ゲインは速度とフェーズ（駆動/制動）でプラントゲインが 2〜10 倍変わるため
    # グリッド点ごとに評価する。ペダルゲイン未同定なら引数 l_gain の一律値（従来動作）。
    v_grid = np.interp(grid, t_arr, np.asarray(ref_speeds, dtype=float))
    l_grid = np.array(
        [
            l_gain_at(
                params,
                float(v),
                delta_s,
                is_accel=(ph != PlanPhase.BRAKE),
                fallback=l_gain,
            )
            for v, ph in zip(v_grid, base_plan.phases, strict=False)
        ],
        dtype=float,
    )
    raw = u_grid + l_grid * e_shifted
    filtered = zero_phase_lowpass(raw, cutoff_hz, dt)

    base = np.asarray(base_plan.efforts, dtype=float)
    delta = np.clip(filtered - base, -abs(delta_limit_pct), abs(delta_limit_pct))
    efforts = clamp_effort_by_phase(base + delta, base_plan.phases, params)
    efforts = snap_efforts_to_deadband(efforts, params)
    return PedalPlan(dt_s=dt, efforts=efforts, phases=list(base_plan.phases))


__all__ = [
    "MIN_COVERAGE_RATIO",
    "MIN_LOGS_FOR_UPDATE",
    "PLAN_DELTA_LIMIT_PCT",
    "PLAN_L_GAIN_FACTOR",
    "PLAN_LEAD_RESPONSE_S",
    "PLAN_UPDATE_CUTOFF_HZ",
    "l_gain_at",
    "l_gain_from_fopdt",
    "lead_time_from_fopdt",
    "update_plan",
]
