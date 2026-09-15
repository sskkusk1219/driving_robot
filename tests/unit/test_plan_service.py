"""PedalPlanService のユニットテスト。

prepare の分岐と、update の 3 値判定（ACCEPT / EXPLORE / ROLLBACK）・報酬スケールの
再ベースライン・新規学習を検証する。
"""

from datetime import UTC, datetime, timedelta
from types import SimpleNamespace
from unittest.mock import AsyncMock

import pytest

from src.app.plan_service import PedalPlanService, _decide_outcome, _Outcome
from src.domain.control.pedal_plan import PedalPlan, PlanPhase
from src.domain.control.reward import reward_scale_key
from src.models.calibration import CalibrationData
from src.models.drive_log import DriveLog
from src.models.driving_mode import DrivingMode, SpeedPoint
from src.models.profile import (
    DynamicsParams,
    FeedforwardParams,
    PIDGains,
    StopConfig,
    VehicleProfile,
)

N = 60


def _plan(effort: float = 10.0) -> PedalPlan:
    return PedalPlan(dt_s=0.1, efforts=[effort] * N, phases=[PlanPhase.DRIVE] * N)


def _profile(model_path: str | None = "/models/x.pkl") -> VehicleProfile:
    return VehicleProfile(
        id="p1",
        name="t",
        max_accel_opening=80.0,
        max_brake_opening=80.0,
        max_speed=100.0,
        max_decel_g=0.4,
        pid_gains=PIDGains(kp=1.0, ki=0.1, kd=0.0),
        stop_config=StopConfig(deviation_threshold_kmh=2.0, deviation_duration_s=4.0),
        calibration=CalibrationData(
            accel_zero_pos=0,
            accel_full_pos=5000,
            accel_stroke=5000,
            brake_zero_pos=0,
            brake_full_pos=5000,
            brake_stroke=5000,
            calibrated_at=datetime.now(tz=UTC),
            is_valid=True,
        ),
        model_path=model_path,
        created_at=datetime.now(tz=UTC),
        updated_at=datetime.now(tz=UTC),
        feedforward_params=FeedforwardParams(),
        dynamics_params=DynamicsParams(fopdt_k=2.16, fopdt_tau=2.08, fopdt_theta=0.3),
    )


def _mode() -> DrivingMode:
    return DrivingMode(
        id="m1",
        name="m",
        description="",
        reference_speed=[SpeedPoint(0.0, 40.0), SpeedPoint(6.0, 40.0)],
        total_duration=6.0,
        max_speed=40.0,
        created_at=datetime.now(tz=UTC),
    )


def _logs(applied: float = 10.0) -> list[DriveLog]:
    t0 = datetime.now(tz=UTC)
    out: list[DriveLog] = []
    for i in range(N):
        out.append(
            DriveLog(
                id=i,
                session_id="s1",
                timestamp=t0 + timedelta(seconds=i * 0.1),
                ref_speed_kmh=40.0,
                actual_speed_kmh=39.0,
                accel_opening=applied,
                brake_opening=0.0,
                accel_pos=0,
                brake_pos=0,
                accel_current=0.0,
                brake_current=0.0,
                plan_effort_pct=applied,
                trim_effort_pct=0.0,
                applied_effort_pct=applied,
                phase="drive",
            )
        )
    return out


def _service(rec: object | None, logs: list[DriveLog] | None = None):  # type: ignore[no-untyped-def]
    repo = AsyncMock()
    repo.get = AsyncMock(return_value=rec)
    repo.upsert = AsyncMock()
    session_repo = AsyncMock()
    session_repo.list_logs = AsyncMock(return_value=logs if logs is not None else _logs())
    return PedalPlanService(repo, session_repo), repo, session_repo


_GOOD_KPI = {"p95_kmh": 0.05, "max_abs_deviation_kmh": 0.1}
_BAD_KPI = {"p95_kmh": 0.5, "max_abs_deviation_kmh": 2.0}


def _hist(*rewards: float, scale: str | None = None) -> list[dict[str, object]]:
    """現行スケールで記録された報酬履歴を作る（σ 推定・再ベースライン判定の入力）。

    scale=None なら現行キー。σ を推定させるには REWARD_SIGMA_MIN_SAMPLES 件以上必要。
    """
    key = reward_scale_key() if scale is None else scale
    return [
        {"iteration": i + 1, "reward": r, "outcome": "accept", "reward_scale": key}
        for i, r in enumerate(rewards)
    ]


class TestPrepare:
    @pytest.mark.asyncio
    async def test_none_when_no_record(self) -> None:
        svc, _repo, _sr = _service(None)
        assert await svc.prepare(_profile(), _mode()) is None

    @pytest.mark.asyncio
    async def test_none_when_disabled(self) -> None:
        rec = SimpleNamespace(enabled=False, plan=_plan(), model_path="/models/x.pkl", iteration=1)
        svc, _repo, _sr = _service(rec)
        assert await svc.prepare(_profile(), _mode()) is None

    @pytest.mark.asyncio
    async def test_none_when_empty_plan(self) -> None:
        rec = SimpleNamespace(
            enabled=True, plan=PedalPlan(), model_path="/models/x.pkl", iteration=0
        )
        svc, _repo, _sr = _service(rec)
        assert await svc.prepare(_profile(), _mode()) is None

    @pytest.mark.asyncio
    async def test_returns_plan_when_model_path_mismatch(self) -> None:
        """プラン引き継ぎ（2026-07-16）: モデル再学習をまたいでも保存プランで走る。

        旧仕様の遅延無効化は学習サイクルのたびに本番プランを振り出しへ戻していた。
        """
        plan = _plan()
        rec = SimpleNamespace(enabled=True, plan=plan, model_path="/models/OLD.pkl", iteration=2)
        svc, _repo, _sr = _service(rec)
        # profile の model_path は /models/x.pkl で不一致だが、プランは継続使用する
        assert await svc.prepare(_profile(), _mode()) is plan

    @pytest.mark.asyncio
    async def test_returns_plan_when_valid(self) -> None:
        plan = _plan()
        rec = SimpleNamespace(enabled=True, plan=plan, model_path="/models/x.pkl", iteration=2)
        svc, _repo, _sr = _service(rec)
        assert await svc.prepare(_profile(), _mode()) is plan

    @pytest.mark.asyncio
    async def test_none_on_load_exception(self) -> None:
        repo = AsyncMock()
        repo.get = AsyncMock(side_effect=RuntimeError("db down"))
        svc = PedalPlanService(repo, AsyncMock())
        assert await svc.prepare(_profile(), _mode()) is None


class TestUpdateFromSession:
    @pytest.mark.asyncio
    async def test_new_learning_start_when_no_record(self) -> None:
        svc, repo, _sr = _service(None)
        await svc.update_from_session(
            "s1", _profile(), _mode(), _GOOD_KPI, used_plan=_plan(), base_plan=_plan()
        )
        repo.upsert.assert_awaited_once()
        kw = repo.upsert.await_args.kwargs
        assert kw["iteration"] == 1
        assert kw["best_reward"] is not None
        assert kw["reward_history"][-1]["accepted"] is True

    @pytest.mark.asyncio
    async def test_accept_when_reward_improves(self) -> None:
        rec = SimpleNamespace(
            enabled=True,
            plan=_plan(),
            model_path="/models/x.pkl",
            iteration=1,
            best_reward=-1.0,  # 過去最良が悪い → 今回の良走行が改善
            best_efforts=[9.0] * N,
            reward_history=_hist(-1.0),
        )
        svc, repo, _sr = _service(rec)
        await svc.update_from_session(
            "s1", _profile(), _mode(), _GOOD_KPI, used_plan=_plan(11.0), base_plan=_plan()
        )
        kw = repo.upsert.await_args.kwargs
        assert kw["iteration"] == 2
        assert kw["reward_history"][-1]["accepted"] is True
        # 採用: best_efforts は今回走行（used_plan）に更新される
        assert kw["best_efforts"] == [11.0] * N

    @pytest.mark.asyncio
    async def test_rollback_when_reward_clearly_worsens(self) -> None:
        """ノイズでは説明できない悪化は最良プランへ復帰して凍結する。

        σ 推定に足る同一スケール履歴（ばらつき小）を与えたうえで、そこから大きく外れた
        悪化を起こす。
        """
        rec = SimpleNamespace(
            enabled=True,
            plan=_plan(),
            model_path="/models/x.pkl",
            iteration=3,
            best_reward=-0.1,  # 過去最良が良い
            best_efforts=[7.0] * N,
            reward_history=_hist(-0.3, -0.2, -0.1),  # σ≈0.1 → 許容幅は 2σ=0.2 程度
        )
        svc, repo, _sr = _service(rec)
        await svc.update_from_session(
            "s1", _profile(), _mode(), _BAD_KPI, used_plan=_plan(20.0), base_plan=_plan()
        )
        kw = repo.upsert.await_args.kwargs
        plan_arg = repo.upsert.await_args.args[2]  # new_plan は位置引数
        assert kw["reward_history"][-1]["outcome"] == "rollback"
        assert kw["reward_history"][-1]["accepted"] is False
        # ロールバック: 候補・最良とも過去の best_efforts に戻る
        assert kw["best_efforts"] == [7.0] * N
        assert plan_arg.efforts == [7.0] * N
        assert kw["best_reward"] == -0.1

    @pytest.mark.asyncio
    async def test_explore_when_worsening_is_within_noise(self) -> None:
        """最良をわずかに下回る（σ 範囲内）走行は、最良据え置きのまま探索を継続する。

        欠陥②（2026-07-23 実機）の中核。旧実装はここでプランを凍結したため、次走行が
        同一プラン＝同一結果になって永久に改善しなくなった。
        """
        rec = SimpleNamespace(
            enabled=True,
            plan=_plan(),
            model_path="/models/x.pkl",
            iteration=3,
            best_reward=-0.1,
            best_efforts=[7.0] * N,
            # ばらつきの大きい履歴 → 許容幅が広く、多少の悪化はノイズ相当と判断される
            reward_history=_hist(-30.0, -0.1, -20.0),
        )
        svc, repo, _sr = _service(rec)
        await svc.update_from_session(
            "s1", _profile(), _mode(), _BAD_KPI, used_plan=_plan(20.0), base_plan=_plan()
        )
        kw = repo.upsert.await_args.kwargs
        plan_arg = repo.upsert.await_args.args[2]
        assert kw["reward_history"][-1]["outcome"] == "explore"
        assert kw["reward_history"][-1]["accepted"] is False  # 最良は更新しない
        # 最良は据え置き、しかし次回プランは実測から生成した候補（＝凍結しない）
        assert kw["best_efforts"] == [7.0] * N
        assert kw["best_reward"] == -0.1
        assert plan_arg.efforts != [7.0] * N

    @pytest.mark.asyncio
    async def test_explore_when_sigma_not_estimable(self) -> None:
        """σ を推定できるだけの同一スケール履歴が無い初期は探索側に倒す。"""
        rec = SimpleNamespace(
            enabled=True,
            plan=_plan(),
            model_path="/models/x.pkl",
            iteration=2,
            best_reward=-0.1,
            best_efforts=[7.0] * N,
            reward_history=_hist(-0.2, -0.1),  # 2 件（min_samples=3 未満）
        )
        svc, repo, _sr = _service(rec)
        await svc.update_from_session(
            "s1", _profile(), _mode(), _BAD_KPI, used_plan=_plan(20.0), base_plan=_plan()
        )
        kw = repo.upsert.await_args.kwargs
        plan_arg = repo.upsert.await_args.args[2]
        assert kw["reward_history"][-1]["outcome"] == "explore"
        assert plan_arg.efforts != [7.0] * N

    @pytest.mark.asyncio
    async def test_model_path_mismatch_continues_history(self) -> None:
        """プラン引き継ぎ（2026-07-16）: モデル再学習をまたいでも reward 履歴と継続比較する。

        reward は KPI のみ由来でモデル非依存のため、旧モデル時代の best_reward との
        単調改善比較は公平に成立する。
        """
        rec = SimpleNamespace(
            enabled=True,
            plan=_plan(),
            model_path="/models/OLD.pkl",  # profile 側は /models/x.pkl（再学習後）
            iteration=4,
            best_reward=-1.0,
            best_efforts=[9.0] * N,
            reward_history=_hist(-1.0),
        )
        svc, repo, _sr = _service(rec)
        await svc.update_from_session(
            "s1", _profile(), _mode(), _GOOD_KPI, used_plan=_plan(11.0), base_plan=_plan()
        )
        kw = repo.upsert.await_args.kwargs
        # 新規学習扱い（iteration=1・履歴リセット）にならず継続する
        assert kw["iteration"] == 5
        assert len(kw["reward_history"]) == 2
        assert kw["reward_history"][-1]["accepted"] is True
        # model_path は記録として現行値へ更新される
        assert kw["model_path"] == "/models/x.pkl"

    @pytest.mark.asyncio
    async def test_model_path_mismatch_rollback_keeps_old_best(self) -> None:
        """モデル再学習をまたいだ明確な悪化は旧モデル時代の最良プランへロールバックする。"""
        rec = SimpleNamespace(
            enabled=True,
            plan=_plan(),
            model_path="/models/OLD.pkl",
            iteration=4,
            best_reward=-0.1,  # 旧モデル時代の良い最良
            best_efforts=[7.0] * N,
            reward_history=_hist(-0.3, -0.2, -0.1),
        )
        svc, repo, _sr = _service(rec)
        await svc.update_from_session(
            "s1", _profile(), _mode(), _BAD_KPI, used_plan=_plan(20.0), base_plan=_plan()
        )
        kw = repo.upsert.await_args.kwargs
        assert kw["reward_history"][-1]["outcome"] == "rollback"
        assert kw["best_efforts"] == [7.0] * N
        assert kw["best_reward"] == -0.1

    @pytest.mark.asyncio
    async def test_guard_skip_keeps_used_plan_on_accept(self) -> None:
        """ログ不足で候補を作れない場合、採用でも used_plan を据え置く。"""
        svc, repo, _sr = _service(None, logs=_logs()[:10])  # 10 件 < 50
        await svc.update_from_session(
            "s1", _profile(), _mode(), _GOOD_KPI, used_plan=_plan(12.0), base_plan=_plan()
        )
        plan_arg = repo.upsert.await_args.args[2]  # new_plan は位置引数
        assert plan_arg.efforts == [12.0] * N

    @pytest.mark.asyncio
    async def test_disabled_record_skips_update(self) -> None:
        rec = SimpleNamespace(
            enabled=False,
            plan=_plan(),
            model_path="/models/x.pkl",
            iteration=1,
            best_reward=-0.5,
            best_efforts=[9.0] * N,
            reward_history=[],
        )
        svc, repo, _sr = _service(rec)
        await svc.update_from_session(
            "s1", _profile(), _mode(), _GOOD_KPI, used_plan=_plan(), base_plan=_plan()
        )
        repo.upsert.assert_not_awaited()

    @pytest.mark.asyncio
    async def test_empty_plan_skips(self) -> None:
        svc, repo, _sr = _service(None)
        await svc.update_from_session(
            "s1", _profile(), _mode(), _GOOD_KPI, used_plan=PedalPlan(), base_plan=PedalPlan()
        )
        repo.upsert.assert_not_awaited()


class TestRewardScaleRebaseline:
    """KPI しきい値変更で報酬スケールが変わったときの再ベースライン（2026-07-23）。"""

    def _rec(self, history: list[dict[str, object]]) -> SimpleNamespace:
        return SimpleNamespace(
            enabled=True,
            plan=_plan(),
            model_path="/models/x.pkl",
            iteration=7,
            best_reward=-0.001,  # 旧スケールの極端に良い最良（新スケールでは超えられない）
            best_efforts=[7.0] * N,
            reward_history=history,
        )

    @pytest.mark.asyncio
    async def test_legacy_history_without_scale_is_rebaselined(self) -> None:
        """本機能より前の履歴（reward_scale なし）は最良報酬を張り直す。

        現 DB の best_reward（p95 しきい値 0.2 時代の −48.14 / −199.78）が該当し、
        これが学習を阻害していた。
        """
        rec = self._rec([{"iteration": 7, "reward": -0.001, "accepted": True}])
        svc, repo, _sr = _service(rec)
        await svc.update_from_session(
            "s1", _profile(), _mode(), _BAD_KPI, used_plan=_plan(11.0), base_plan=_plan()
        )
        kw = repo.upsert.await_args.kwargs
        # 旧最良と比較せず、今回走行が新しい基準になる
        assert kw["reward_history"][-1]["outcome"] == "accept"
        assert kw["best_reward"] is not None
        assert kw["best_reward"] < -0.001

    @pytest.mark.asyncio
    async def test_different_scale_is_rebaselined(self) -> None:
        rec = self._rec(_hist(-0.001, scale="p95=0.2;hard=1"))
        svc, repo, _sr = _service(rec)
        await svc.update_from_session(
            "s1", _profile(), _mode(), _BAD_KPI, used_plan=_plan(11.0), base_plan=_plan()
        )
        kw = repo.upsert.await_args.kwargs
        assert kw["reward_history"][-1]["outcome"] == "accept"

    @pytest.mark.asyncio
    async def test_rebaseline_keeps_plan_iteration_and_history(self) -> None:
        """再ベースラインは最良報酬だけを無効化し、プラン資産は保持する。"""
        rec = self._rec(_hist(-0.001, scale="p95=0.2;hard=1"))
        svc, repo, _sr = _service(rec)
        await svc.update_from_session(
            "s1", _profile(), _mode(), _BAD_KPI, used_plan=_plan(11.0), base_plan=_plan()
        )
        kw = repo.upsert.await_args.kwargs
        assert kw["iteration"] == 8  # 継続
        assert len(kw["reward_history"]) == 2  # 旧履歴を保持したまま追記
        assert kw["best_efforts"] == [11.0] * N  # ACCEPT なので今回走行が最良に

    @pytest.mark.asyncio
    async def test_current_scale_is_not_rebaselined(self) -> None:
        """現行スケールの履歴なら従来どおり最良報酬と比較する（再ベースラインしない）。"""
        rec = self._rec(_hist(-0.003, -0.002, -0.001))
        svc, repo, _sr = _service(rec)
        await svc.update_from_session(
            "s1", _profile(), _mode(), _BAD_KPI, used_plan=_plan(11.0), base_plan=_plan()
        )
        kw = repo.upsert.await_args.kwargs
        assert kw["reward_history"][-1]["outcome"] == "rollback"
        assert kw["best_reward"] == -0.001

    @pytest.mark.asyncio
    async def test_new_entries_record_current_scale(self) -> None:
        svc, repo, _sr = _service(None)
        await svc.update_from_session(
            "s1", _profile(), _mode(), _GOOD_KPI, used_plan=_plan(), base_plan=_plan()
        )
        kw = repo.upsert.await_args.kwargs
        assert kw["reward_history"][-1]["reward_scale"] == reward_scale_key()


class TestDeadlockRegression:
    """欠陥②の回帰テスト（2026-07-23 実機シーケンスの再現）。

    実機ではシステムモードの PLAN_LEARN 3 本が全て棄却され、毎回同一プランへロールバック
    したため p95 が 1.84/1.95/1.88 で停滞した。棄却時に候補を生成しないと、次走行は
    同一プラン＝同一結果になり永久に抜け出せない。
    """

    @pytest.mark.asyncio
    async def test_repeated_small_regressions_keep_updating_plan(self) -> None:
        """最良をわずかに下回る走行が続いても、毎回プランが更新され続ける。"""
        history = _hist(-30.0, -0.1, -20.0)  # ばらつきのある同一スケール履歴
        best_efforts = [7.0] * N
        for i in range(3):
            rec = SimpleNamespace(
                enabled=True,
                plan=_plan(),
                model_path="/models/x.pkl",
                iteration=5 + i,
                best_reward=-0.1,
                best_efforts=best_efforts,
                reward_history=history,
            )
            svc, repo, _sr = _service(rec)
            await svc.update_from_session(
                "s1", _profile(), _mode(), _BAD_KPI, used_plan=_plan(20.0), base_plan=_plan()
            )
            kw = repo.upsert.await_args.kwargs
            plan_arg = repo.upsert.await_args.args[2]
            history = kw["reward_history"]
            # 毎回「凍結」ではなく候補プランが保存される
            assert kw["reward_history"][-1]["outcome"] == "explore"
            assert plan_arg.efforts != best_efforts
        assert len(history) == 6  # 3 本ぶん追記されている


class TestFreezeToBest:
    """freeze_to_best: PLAN_LEARN フェーズ終了時に保存プランを best_efforts へ確定する。"""

    @pytest.mark.asyncio
    async def test_overwrites_current_plan_with_best_efforts(self) -> None:
        best_efforts = [3.0] * N
        rec = SimpleNamespace(
            enabled=True,
            plan=_plan(20.0),  # 現行の候補プラン（best とは異なる）
            model_path="/models/x.pkl",
            iteration=5,
            best_reward=-0.1,
            best_efforts=best_efforts,
            reward_history=_hist(-0.1),
        )
        svc, repo, _sr = _service(rec)
        await svc.freeze_to_best("p1", "m1")
        assert repo.upsert.await_count == 1
        plan_arg = repo.upsert.await_args.args[2]
        kw = repo.upsert.await_args.kwargs
        assert plan_arg.efforts == best_efforts
        assert kw["best_efforts"] == best_efforts
        assert kw["best_reward"] == -0.1
        assert kw["iteration"] == 5

    @pytest.mark.asyncio
    async def test_noop_when_already_best(self) -> None:
        best_efforts = [3.0] * N
        rec = SimpleNamespace(
            enabled=True,
            plan=PedalPlan(dt_s=0.1, efforts=list(best_efforts), phases=[PlanPhase.DRIVE] * N),
            model_path="/models/x.pkl",
            iteration=5,
            best_reward=-0.1,
            best_efforts=best_efforts,
            reward_history=_hist(-0.1),
        )
        svc, repo, _sr = _service(rec)
        await svc.freeze_to_best("p1", "m1")
        assert repo.upsert.await_count == 0

    @pytest.mark.asyncio
    async def test_noop_when_no_record(self) -> None:
        svc, repo, _sr = _service(None)
        await svc.freeze_to_best("p1", "m1")
        assert repo.upsert.await_count == 0

    @pytest.mark.asyncio
    async def test_noop_when_no_best_efforts(self) -> None:
        rec = SimpleNamespace(
            enabled=True,
            plan=_plan(20.0),
            model_path="/models/x.pkl",
            iteration=0,
            best_reward=None,
            best_efforts=[],
            reward_history=[],
        )
        svc, repo, _sr = _service(rec)
        await svc.freeze_to_best("p1", "m1")
        assert repo.upsert.await_count == 0

    @pytest.mark.asyncio
    async def test_noop_on_load_exception(self) -> None:
        repo = AsyncMock()
        repo.get = AsyncMock(side_effect=RuntimeError("db down"))
        svc = PedalPlanService(repo, AsyncMock())
        await svc.freeze_to_best("p1", "m1")
        assert repo.upsert.await_count == 0


class TestPrepareReconcilesProfile:
    """保存プランへ現行プロファイル由来の値を再適用する（実機 9eee549b の回帰）。

    保存プランは学習サイクル・モデル再学習をまたいで持続する一方、車両プロファイルの
    物理定数は学習運転のたびに再同定される。突き合わせる経路が無かったため、保存プランの
    STOP_HOLD effort が −16.0% のまま（現行プロファイルは 24.0%）で停車保持ブレーキが
    不足し、基準 0km/h に対し実車速 1.75km/h のクリープが 6.3 秒続いていた。
    """

    @staticmethod
    def _stale_plan() -> PedalPlan:
        """停車保持が古い値（−16.0）のまま保存されたプラン。"""
        return PedalPlan(
            dt_s=0.1,
            efforts=[20.0, -8.0, -16.0, -16.0],
            phases=[
                PlanPhase.DRIVE,
                PlanPhase.BRAKE,
                PlanPhase.STOP_HOLD,
                PlanPhase.STOP_HOLD,
            ],
        )

    @pytest.mark.asyncio
    async def test_stop_hold_follows_current_profile(self) -> None:
        """STOP_HOLD が現行 stop_brake_opening_pct へ更新される。"""
        profile = _profile()
        profile.feedforward_params = FeedforwardParams(stop_brake_opening_pct=24.0)
        rec = SimpleNamespace(
            enabled=True, plan=self._stale_plan(), model_path=profile.model_path, iteration=6
        )
        svc, _repo, _sr = _service(rec)
        out = await svc.prepare(profile, _mode())
        assert out is not None
        assert out.efforts[2] == pytest.approx(-24.0)
        assert out.efforts[3] == pytest.approx(-24.0)

    @pytest.mark.asyncio
    async def test_learned_drive_and_brake_efforts_preserved(self) -> None:
        """学習した DRIVE/BRAKE の effort 形状はそのまま残す。"""
        profile = _profile()
        profile.feedforward_params = FeedforwardParams(stop_brake_opening_pct=24.0)
        rec = SimpleNamespace(
            enabled=True, plan=self._stale_plan(), model_path=profile.model_path, iteration=6
        )
        svc, _repo, _sr = _service(rec)
        out = await svc.prepare(profile, _mode())
        assert out is not None
        assert out.efforts[0] == pytest.approx(20.0)
        assert out.efforts[1] == pytest.approx(-8.0)

    @pytest.mark.asyncio
    async def test_consistent_plan_is_returned_unchanged(self) -> None:
        """既に整合しているプランは同一オブジェクトを返す（冪等・無駄な再構築なし）。"""
        profile = _profile()
        profile.feedforward_params = FeedforwardParams(stop_brake_opening_pct=16.0)
        plan = self._stale_plan()
        rec = SimpleNamespace(
            enabled=True, plan=plan, model_path=profile.model_path, iteration=6
        )
        svc, _repo, _sr = _service(rec)
        out = await svc.prepare(profile, _mode())
        assert out is plan


class TestLexicographicOutcome:
    """ILC の採否が (KPI合否, −p95, reward) の辞書順であること（優先D）。

    第3ラウンドの 979fb2f5 では、p95 1.43（サイクル最良）を出した iter 12 が max 5.14 の
    せいで reward が伸びず不採用になり、iter 13 でロールバックされて 3 本走って進捗ゼロで
    終わった。学習サイクル側の best 選択は辞書順に直っていたのに、ILC の採否だけが
    reward スカラー 1 本のまま食い違っていた。
    """

    @staticmethod
    def _entry(reward: float, p95: float, max_kmh: float, reversal: float) -> dict[str, object]:
        return {
            "iteration": 1,
            "reward": reward,
            "p95_kmh": p95,
            "max_kmh": max_kmh,
            "reversal_max_per_5s": reversal,
            "outcome": "accept",
            "accepted": True,
            "reward_scale": reward_scale_key(),
        }

    def test_better_p95_accepted_even_if_reward_worse(self) -> None:
        """p95 が改善していれば、max 悪化で reward が下がっても採用する。"""
        history = [self._entry(-100.0, 2.12, 4.52, 8.0)]
        kpi = {
            "n_samples": 100.0,
            "p95_kmh": 1.43,
            "max_abs_deviation_kmh": 5.14,
            "reversal_max_per_5s": 7.0,
        }
        outcome = _decide_outcome(-140.0, -100.0, history, reward_scale_key(), kpi)
        assert outcome is _Outcome.ACCEPT

    def test_worse_p95_is_not_accepted(self) -> None:
        history = [self._entry(-100.0, 1.42, 4.06, 8.0)]
        kpi = {
            "n_samples": 100.0,
            "p95_kmh": 2.20,
            "max_abs_deviation_kmh": 5.01,
            "reversal_max_per_5s": 8.0,
        }
        outcome = _decide_outcome(-150.0, -100.0, history, reward_scale_key(), kpi)
        assert outcome is not _Outcome.ACCEPT

    def test_kpi_pass_beats_better_p95(self) -> None:
        """KPI 合格は p95 の大小より優先される（辞書順の第 1 要素）。"""
        history = [self._entry(-10.0, 0.30, 0.5, 0.0)]  # 合格済み
        kpi = {
            "n_samples": 100.0,
            "p95_kmh": 0.20,  # p95 はより良いが…
            "max_abs_deviation_kmh": 2.0,  # max が 1.0 超で不合格
            "reversal_max_per_5s": 0.0,
        }
        outcome = _decide_outcome(-5.0, -10.0, history, reward_scale_key(), kpi)
        assert outcome is not _Outcome.ACCEPT


class TestRewardScaleKeyVersioning:
    def test_key_changes_when_track_weights_change(self, monkeypatch: pytest.MonkeyPatch) -> None:
        """追従項の重みを変えたらキーが変わる＝best_reward が再ベースラインされる。

        旧実装はしきい値だけからキーを作っており、係数を変えても再ベースラインが走らず
        旧スケールの best_reward が残って学習が止まる状態だった。
        """
        before = reward_scale_key()
        monkeypatch.setattr("src.domain.control.reward.W_TRACK_P95", 1.0)
        assert reward_scale_key() != before
