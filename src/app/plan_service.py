"""エピソード型プラン学習のアプリケーションサービス（ILCService の後継）。

走行開始時に profile×mode の保存プランをロードし（prepare）、自動走行の正常完了後に
実効 effort＋残差から次回プランを更新して永続化する（update_from_session）。学習は性能向上
手段であり安全の前提にしない（ロード失敗・データ不足は FF 由来プランで走行継続）。

**採否は 3 値**（2026-07-23）:
  - ACCEPT  : reward > 最良 → 最良を更新し、実測から次回候補を生成
  - EXPLORE : 最良は下回るが走行間ばらつき σ の範囲内（reward > 最良 − K·σ）→ 最良は
              据え置いたまま、実測から次回候補を生成して探索を続ける
  - ROLLBACK: ノイズでは説明できない悪化 → 最良プランへ復帰して凍結

旧実装は 2 値（採用／ロールバック）で、ロールバック時は候補を一切生成せず最良プランに
固定していた。すると次走行は同一プラン＝同一結果になって再び棄却され、学習が永久に停止する。
さらに best_reward は「ノイズを含む評価の最大値」で上方バイアスを持つため、ノイズ床に達した
時点でこの停止が確定する。実機 2026-07-23 では PLAN_LEARN 3 本が全て棄却され p95 が
1.84/1.95/1.88 で停滞した（従来はモデル再学習のたびに履歴がリセットされ、この欠陥が
毎サイクル帳消しになって見えていなかった）。EXPLORE は ROLLBACK 直後の走行が最良プランで
走ることを利用して「最良プランの周辺を探索する」挙動になり、候補は ±Δlimit クランプで
拘束されるため発散しない。

モデル再学習（model_path 変化）をまたいでも保存プランと reward 履歴は引き継ぐ
（2026-07-16）。reward は KPI サマリのみから算出されモデル非依存のため、同一車両×同一
軌跡なら更新前後で公平に比較でき単調改善保証は壊れない。旧仕様の遅延無効化（不一致で
FF 由来へ戻す）は学習サイクルのたびに本番プランを振り出しへ戻し、初回 p95=2.74・収束まで
本番 7 走行を要していた（7/15 実測）。モデル大幅変化で旧プランが不適合になっても、速い
補正層が max≤1.0 の安全網となり、reward 悪化なら次回はロールバックで最良プラン維持となる
ため、実害は「収束が遅くなる」に留まる。

ただし **KPI しきい値を変えると reward のスケール自体が変わる**ため、変更前に記録した
best_reward と変更後の reward は比較できない。履歴に `reward_scale`（reward_scale_key）を
添えて記録し、不一致なら best_reward だけを再ベースラインする（プラン・best_efforts・
iteration・履歴は保持）。
"""

from __future__ import annotations

import logging
from collections.abc import Mapping
from enum import Enum
from typing import Any, Protocol

from src.domain.control.kpi_monitor import kpi_passed
from src.domain.control.pedal_plan import (
    PedalPlan,
    clamp_effort_by_phase,
    snap_efforts_to_deadband,
)
from src.domain.control.plan_update import (
    MIN_LOGS_FOR_UPDATE,
    l_gain_from_fopdt,
    lead_time_from_fopdt,
    update_plan,
)
from src.domain.control.reward import (
    REWARD_NOISE_SIGMA_K,
    reward_noise_sigma,
    reward_scale_key,
    reward_score,
)
from src.models.drive_log import DriveLog
from src.models.driving_mode import DrivingMode
from src.models.profile import VehicleProfile

_logger = logging.getLogger(__name__)


class _Outcome(Enum):
    """走行 1 本の採否判定。"""

    ACCEPT = "accept"  # 最良を更新（＋実測から次回候補を生成）
    EXPLORE = "explore"  # 最良は据え置き、探索は継続（実測から次回候補を生成）
    ROLLBACK = "rollback"  # 明確な悪化。最良プランへ復帰して凍結


_OUTCOME_LABEL = {
    _Outcome.ACCEPT: "採用",
    _Outcome.EXPLORE: "探索継続",
    _Outcome.ROLLBACK: "ロールバック",
}


def _history_matches_scale(history: list[dict[str, Any]], scale_key: str) -> bool:
    """履歴の最新エントリが現行の報酬スケールで記録されているか。

    本機能より前に書かれたエントリは `reward_scale` を持たないため False になり、
    呼び出し側で再ベースラインされる（現 DB の旧スケール best_reward が対象）。
    """
    if not history:
        return False
    last = history[-1]
    return isinstance(last, dict) and last.get("reward_scale") == scale_key


def _same_scale_rewards(history: list[dict[str, Any]], scale_key: str) -> list[float]:
    """現行スケールで記録された報酬だけを古い順に取り出す（σ 推定用）。

    履歴は JSONB 由来で任意の値が入りうるため、数値化できないエントリは無視する。
    """
    out: list[float] = []
    for entry in history:
        if not isinstance(entry, dict) or entry.get("reward_scale") != scale_key:
            continue
        try:
            out.append(float(entry["reward"]))
        except (KeyError, TypeError, ValueError):
            continue
    return out


def _run_key(kpi_summary: Mapping[str, float], reward: float) -> tuple[bool, float, float]:
    """走行の比較キー（大きいほど良い）: (KPI合否, −p95, reward) の辞書順。

    LearningCycleService._plan_learn_run_key と**同じ順序**にすること。第3ラウンドでは
    学習サイクルの best 選択だけが辞書順で、ILC の採否は reward スカラー 1 本だったため、
    p95 1.43（サイクル最良）を出した iter 12 が max 5.14 のせいで reward が伸びずに
    不採用となり、iter 13 でロールバックされてサイクルが進捗ゼロで終わった
    （docs/Problem/引き継ぎ20260909.md 優先D）。
    """
    p95 = float(kpi_summary.get("p95_kmh", float("inf")))
    return (kpi_passed(kpi_summary), -p95, reward)


def _prev_best_key(
    history: list[dict[str, Any]], prev_best_reward: float | None
) -> tuple[bool, float, float] | None:
    """履歴から「これまでの最良走行」の比較キーを復元する。

    履歴エントリは best を更新した走行（outcome=accept）に `p95_kmh` / `max_kmh` /
    `reversal_max_per_5s` を持つ。反転回数を持たない旧エントリは KPI 合否を再構成できない
    ので、合否は False 扱い（＝p95 と reward だけで比べる）に倒す。
    """
    if prev_best_reward is None:
        return None
    best: tuple[bool, float, float] | None = None
    for entry in history:
        if not isinstance(entry, dict) or not entry.get("accepted"):
            continue
        try:
            reward = float(entry["reward"])
            p95 = float(entry["p95_kmh"])
        except (KeyError, TypeError, ValueError):
            continue
        # `or` で既定値へ落とすと 0.0（合法値・反転 0 回）が falsy で握り潰される。
        # 欠損だけを既定値にしたいので None を明示的に判定する。
        def _num(key: str) -> float:
            value = entry.get(key)
            return 1e9 if value is None else float(value)

        passed = kpi_passed(
            {
                "n_samples": 1.0,
                "p95_kmh": p95,
                "max_abs_deviation_kmh": _num("max_kmh"),
                "reversal_max_per_5s": _num("reversal_max_per_5s"),
            }
        )
        key = (passed, -p95, reward)
        if best is None or key > best:
            best = key
    return best


def _decide_outcome(
    reward: float,
    prev_best_reward: float | None,
    history: list[dict[str, Any]],
    scale_key: str,
    kpi_summary: Mapping[str, float],
) -> _Outcome:
    """走行の KPI と過去実績から採否を決める（ACCEPT / EXPLORE / ROLLBACK）。

    比較は `(KPI合否, −p95, reward)` の辞書順（`_run_key`）で、学習サイクル側の best 選択と
    揃えてある。reward 単独で比べると、最大逸脱と p95 が逆を向いたときに p95 の改善を
    「悪化」と誤判定する（実機 979fb2f5 は 3 本走って進捗ゼロで終わった）。

    最良の更新は厳密（キーが上回れば ACCEPT）だが、下回った場合でも走行間ばらつき σ の
    範囲内（reward > best_reward − K·σ）なら EXPLORE として学習を続ける。σ を推定できる
    だけの同一スケール履歴が無い初期も EXPLORE 側に倒す（判定不能を「悪化」と決めつけない）。
    ROLLBACK は「ノイズでは説明できない reward の悪化」に限る——p95 が少し悪いだけで
    プランを凍結すると探索が止まるため、辞書順は ACCEPT の判定にだけ使う。
    """
    key = _run_key(kpi_summary, reward)
    best_key = _prev_best_key(history, prev_best_reward)
    if best_key is None and prev_best_reward is not None:
        # 履歴から最良走行の KPI を復元できない（p95 を持たない旧エントリだけ、など）。
        # 上位 2 要素を今回と同じにして reward 単独の比較へ落とす＝従来動作。
        best_key = (key[0], key[1], prev_best_reward)
    if best_key is None or key > best_key:
        return _Outcome.ACCEPT
    sigma = reward_noise_sigma(_same_scale_rewards(history, scale_key))
    if sigma is None:
        return _Outcome.EXPLORE
    if prev_best_reward is None or reward > prev_best_reward - REWARD_NOISE_SIGMA_K * sigma:
        return _Outcome.EXPLORE
    return _Outcome.ROLLBACK


class PedalPlanRepositoryProtocol(Protocol):
    async def get(self, profile_id: str, mode_id: str) -> Any | None: ...

    async def reset_for_mode(self, mode_id: str) -> None: ...

    async def upsert(
        self,
        profile_id: str,
        mode_id: str,
        plan: PedalPlan,
        *,
        iteration: int,
        best_efforts: list[float],
        best_reward: float | None,
        reward_history: list[dict[str, Any]],
        model_path: str | None,
    ) -> None: ...


class SessionLogReaderProtocol(Protocol):
    async def list_logs(self, session_id: str, limit: int = ...) -> list[DriveLog]: ...


class PedalPlanService:
    """保存プランのロード（走行前）と学習（走行後）を担うサービス。"""

    def __init__(
        self,
        plan_repo: PedalPlanRepositoryProtocol,
        session_repo: SessionLogReaderProtocol,
    ) -> None:
        self._repo = plan_repo
        self._session_repo = session_repo

    async def prepare(
        self, profile: VehicleProfile, mode: DrivingMode
    ) -> PedalPlan | None:
        """走行開始時に保存プランをロードして返す。

        レコードが無い/無効/空、ロード失敗の場合は None（FF 由来プランで走行）。
        モデル再学習後（model_path 不一致）も保存プランを返す（プラン引き継ぎ、
        モジュール docstring 参照）。
        """
        try:
            rec = await self._repo.get(profile.id, mode.id)
        except Exception:
            _logger.exception("保存プランのロードに失敗: FF 由来プランで走行を継続")
            return None
        if rec is None or not getattr(rec, "enabled", False):
            return None
        plan: PedalPlan | None = getattr(rec, "plan", None)
        if plan is None or not plan.efforts:
            return None
        if getattr(rec, "model_path", None) != profile.model_path:
            _logger.info(
                "モデル更新 (%s → %s) をまたいで保存プランを継続使用",
                getattr(rec, "model_path", None),
                profile.model_path,
            )
        plan = self._reconcile_with_profile(plan, profile)
        _logger.info(
            "保存プランを適用: profile=%s mode=%s 反復%d回目",
            profile.id,
            mode.id,
            getattr(rec, "iteration", 0),
        )
        return plan

    @staticmethod
    def _reconcile_with_profile(plan: PedalPlan, profile: VehicleProfile) -> PedalPlan:
        """保存プランへ現行プロファイル由来の値を再適用する（2026-09-09 追加）。

        保存プランは学習サイクル・モデル再学習をまたいで持続する（本モジュール docstring）
        一方、車両プロファイルの物理定数は学習運転のたびに再同定される。両者を突き合わせる
        経路が無かったため、**プランが古いプロファイルの値を持ち続ける**事故が起きていた。

        実機 9eee549b: 保存プランの STOP_HOLD effort が一律 −16.0% のままで、現行
        プロファイルの stop_brake_opening_pct=24.0 が反映されていなかった。停車保持
        ブレーキが不足し、基準 0km/h に対し実車速 1.75km/h のクリープが 6.3 秒続いて
        p95 を押し上げていた（当該サイクルの |err|>1.0 エピソード第 3 位）。

        学習した effort 形状（DRIVE/BRAKE の値）はそのまま保持し、プロファイル由来の値
        だけを最新化する: フェーズ整合クランプ（STOP_HOLD の保持 effort）→ 不感帯スナップ。
        どちらも冪等なので、整合していれば何も変わらない。
        """
        if not plan.efforts:
            return plan
        params = profile.feedforward_params
        reconciled = snap_efforts_to_deadband(
            clamp_effort_by_phase(plan.efforts, plan.phases, params), params
        )
        changed = sum(
            1 for a, b in zip(plan.efforts, reconciled, strict=False) if a != b
        )
        if changed == 0:
            return plan
        _logger.info(
            "保存プランをプロファイルへ再整合: profile=%s %d/%d 点を更新"
            "（停車保持 %.1f%% / 不感帯 accel %.1f%% brake %.1f%%）",
            profile.id,
            changed,
            len(plan.efforts),
            params.stop_brake_opening_pct,
            params.accel_deadband_pct,
            params.brake_deadband_pct,
        )
        return PedalPlan(dt_s=plan.dt_s, efforts=reconciled, phases=list(plan.phases))

    async def reset_for_mode(self, mode_id: str) -> None:
        """指定モードの保存プランを全プロファイルぶん削除する（基準軌跡の変更時に呼ぶ）。

        モデル更新はプランを引き継ぐが（モジュール docstring）、軌跡の変更は別物:
        旧軌跡で獲得した best_reward が新軌跡の走行では超えられず、ロールバック機構が
        古いプランに固着するため、履歴ごと削除して学習をやり直す必要がある。
        """
        await self._repo.reset_for_mode(mode_id)

    async def freeze_to_best(self, profile_id: str, mode_id: str) -> None:
        """保存プランを記録済みの最良プラン（best_efforts）へ明示的に確定する。

        ACCEPT/EXPLORE 走行後の候補プラン（次回探索用）は必ずしも best_efforts と一致しない
        （EXPLORE は最良を据え置いたまま候補だけ進める）。反復学習フェーズを抜けた直後は
        「最新の候補」ではなく「これまでの最良」を凍結し、続く PID 仕上げ・本番走行が
        確実にベストプランを使うようにする（2026-09-08: PLAN_LEARN フェーズ末尾で呼ぶ）。
        """
        try:
            rec = await self._repo.get(profile_id, mode_id)
        except Exception:
            _logger.exception("プラン凍結に失敗: 保存プランをロードできません")
            return
        if rec is None:
            return
        best_efforts = list(getattr(rec, "best_efforts", []) or [])
        plan: PedalPlan | None = getattr(rec, "plan", None)
        if not best_efforts or plan is None:
            return
        if list(plan.efforts) == best_efforts:
            return
        frozen_plan = PedalPlan(dt_s=plan.dt_s, efforts=best_efforts, phases=list(plan.phases))
        await self._repo.upsert(
            profile_id,
            mode_id,
            frozen_plan,
            iteration=int(getattr(rec, "iteration", 0)),
            best_efforts=best_efforts,
            best_reward=getattr(rec, "best_reward", None),
            reward_history=list(getattr(rec, "reward_history", []) or []),
            model_path=getattr(rec, "model_path", None),
        )
        _logger.info(
            "プランを最良（best_efforts）へ確定しました: profile=%s mode=%s", profile_id, mode_id
        )

    async def update_from_session(
        self,
        session_id: str,
        profile: VehicleProfile,
        mode: DrivingMode,
        kpi_summary: dict[str, float],
        used_plan: PedalPlan,
        base_plan: PedalPlan,
    ) -> None:
        """走行正常完了後に reward 判定してプランを更新・永続化する（fire-and-forget 前提）。"""
        try:
            await self._update(session_id, profile, mode, kpi_summary, used_plan, base_plan)
        except Exception:
            _logger.exception("プラン学習に失敗（走行は正常完了済み・次回は前回プランを使用）")

    async def _update(
        self,
        session_id: str,
        profile: VehicleProfile,
        mode: DrivingMode,
        kpi_summary: dict[str, float],
        used_plan: PedalPlan,
        base_plan: PedalPlan,
    ) -> None:
        if not used_plan.efforts or not base_plan.efforts:
            return

        rec = await self._repo.get(profile.id, mode.id)
        if rec is not None and not getattr(rec, "enabled", True):
            _logger.info(
                "プラン学習が無効のため更新しない: profile=%s mode=%s", profile.id, mode.id
            )
            return

        reward = reward_score(kpi_summary)
        scale_key = reward_scale_key()
        # reward は KPI のみ由来でモデル非依存のため、モデル再学習（model_path 変化）を
        # またいでも best_reward・履歴・iteration を継続する（model_path は記録として更新）。
        is_new = rec is None
        history: list[dict[str, Any]] = (
            list(getattr(rec, "reward_history", []) or []) if not is_new else []
        )
        if is_new:
            prev_best_reward: float | None = None
            prev_best_efforts = list(used_plan.efforts)
            iteration_base = 0
        else:
            prev_best_reward = getattr(rec, "best_reward", None)
            prev_best_efforts = list(getattr(rec, "best_efforts", []) or used_plan.efforts)
            iteration_base = int(getattr(rec, "iteration", 0))
            if prev_best_reward is not None and not _history_matches_scale(
                history, scale_key
            ):
                # KPI しきい値変更で報酬のスケールが変わっており、旧スケールの最良報酬とは
                # 比較できない。プラン（best_efforts）は残したまま基準だけ張り直す。
                _logger.info(
                    "報酬スケール変更を検出: best_reward=%.4f を再ベースライン (scale=%s)",
                    prev_best_reward,
                    scale_key,
                )
                prev_best_reward = None

        outcome = _decide_outcome(
            reward, prev_best_reward, history, scale_key, kpi_summary
        )

        if outcome is _Outcome.ROLLBACK:
            # ノイズでは説明できない悪化: 最良プランへ復帰して次回はそこから再開する。
            best_efforts = prev_best_efforts
            best_reward: float | None = prev_best_reward
            new_efforts = list(prev_best_efforts)
        else:
            # ACCEPT / EXPLORE: どちらも走行実測から次回候補を生成して学習を進める。
            # EXPLORE は最良を更新しないが探索は止めない（止めると同一プランの再走行を
            # 繰り返すだけで、ノイズ床に達した時点で学習が永久に停止する）。
            if outcome is _Outcome.ACCEPT:
                best_efforts = list(used_plan.efforts)
                best_reward = reward
            else:
                best_efforts = prev_best_efforts
                best_reward = prev_best_reward
            candidate = await self._propose_candidate(session_id, profile, base_plan)
            if candidate is not None:
                new_efforts = list(candidate.efforts)
            elif outcome is _Outcome.ACCEPT:
                # ログ不足等で候補を作れない場合は最良プラン（今回走行）を据え置く。
                new_efforts = list(used_plan.efforts)
            else:
                new_efforts = list(prev_best_efforts)

        # フェーズ列は基準軌跡由来で固定（プラン更新で不要切替が再発しない）。
        new_plan = PedalPlan(
            dt_s=base_plan.dt_s, efforts=new_efforts, phases=list(base_plan.phases)
        )
        history.append(
            {
                "iteration": iteration_base + 1,
                "reward": reward,
                "p95_kmh": kpi_summary.get("p95_kmh"),
                "max_kmh": kpi_summary.get("max_abs_deviation_kmh"),
                # KPI 合否を後から再構成するのに要る（_prev_best_key）。
                "reversal_max_per_5s": kpi_summary.get("reversal_max_per_5s"),
                "pedal_switch_per_min": kpi_summary.get("pedal_switch_per_min"),
                "phase_violation_pct": kpi_summary.get("phase_violation_pct"),
                "trim_share": kpi_summary.get("trim_share"),
                "outcome": outcome.value,
                # accepted は「最良を更新したか」の後方互換フィールド（WebUI・分析スクリプト）。
                "accepted": outcome is _Outcome.ACCEPT,
                "reward_scale": scale_key,
            }
        )
        await self._repo.upsert(
            profile.id,
            mode.id,
            new_plan,
            iteration=iteration_base + 1,
            best_efforts=best_efforts,
            best_reward=best_reward,
            reward_history=history,
            model_path=profile.model_path,
        )
        _logger.info(
            "プラン学習: profile=%s mode=%s 反復%d → reward=%.4f (%s)",
            profile.id,
            mode.id,
            iteration_base + 1,
            reward,
            _OUTCOME_LABEL[outcome],
        )

    async def _propose_candidate(
        self, session_id: str, profile: VehicleProfile, base_plan: PedalPlan
    ) -> PedalPlan | None:
        """走行ログの実効 effort＋残差から次回候補プランを計算する。ログ不足なら None。"""
        logs = await self._session_repo.list_logs(session_id, limit=1_000_000)
        rows = [
            log
            for log in logs
            if log.ref_speed_kmh is not None and log.applied_effort_pct is not None
        ]
        if len(rows) < MIN_LOGS_FOR_UPDATE:
            _logger.info("プラン更新をスキップ: 有効ログ %d 件が不足", len(rows))
            return None
        t0 = rows[0].timestamp
        times = [(log.timestamp - t0).total_seconds() for log in rows]
        applied = [float(log.applied_effort_pct) for log in rows]  # type: ignore[arg-type]
        refs = [float(log.ref_speed_kmh) for log in rows]  # type: ignore[arg-type]
        actuals = [float(log.actual_speed_kmh) for log in rows]

        dyn = profile.dynamics_params
        l_gain = l_gain_from_fopdt(dyn.fopdt_k)
        delta_s = lead_time_from_fopdt(dyn.fopdt_theta, dyn.fopdt_tau)
        return update_plan(
            base_plan,
            times,
            applied,
            refs,
            actuals,
            profile.feedforward_params,
            l_gain=l_gain,
            delta_s=delta_s,
        )


__all__ = [
    "PedalPlanRepositoryProtocol",
    "PedalPlanService",
    "SessionLogReaderProtocol",
]
