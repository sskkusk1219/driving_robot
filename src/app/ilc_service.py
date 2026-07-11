"""反復学習制御（ILC）のアプリケーションサービス。

走行開始時に profile×mode の補正テーブルをロードして ILCController を用意し（prepare）、
走行正常完了後に drive_logs の残差から次回テーブルを学習して永続化する（learn_from_session）。
学習は反復再現性のある誤差だけを蓄積するため、性能向上手段であり安全の前提にはしない
（ロード失敗・データ不足・発散時は補正 0 で走行継続）。
"""

from __future__ import annotations

import logging
from typing import Protocol

from src.domain.control.ilc import (
    ILC_CUTOFF_HZ,
    ILC_DT_S,
    ILCController,
    ILCLearner,
    ILCTable,
    is_diverged,
    l_gain_from_fopdt,
)
from src.models.drive_log import DriveLog
from src.models.driving_mode import DrivingMode
from src.models.profile import VehicleProfile

_logger = logging.getLogger(__name__)

# 学習に必要な最小ログ数（これ未満は残差推定に足りないので学習しない）。
_MIN_LOGS_FOR_LEARN: int = 50


class ILCRepositoryProtocol(Protocol):
    async def get(self, profile_id: str, mode_id: str) -> object | None: ...

    async def upsert(
        self, profile_id: str, mode_id: str, table: ILCTable, kpi_history: list[dict[str, object]]
    ) -> None: ...


class SessionLogReaderProtocol(Protocol):
    async def list_logs(self, session_id: str, limit: int = ...) -> list[DriveLog]: ...


class ILCService:
    """ILC テーブルのロード（走行前）と学習（走行後）を担うサービス。"""

    def __init__(
        self,
        ilc_repo: ILCRepositoryProtocol,
        session_repo: SessionLogReaderProtocol,
        learner: ILCLearner | None = None,
    ) -> None:
        self._repo = ilc_repo
        self._session_repo = session_repo
        self._learner = learner if learner is not None else ILCLearner()

    async def prepare(
        self, profile: VehicleProfile, mode: DrivingMode
    ) -> ILCController | None:
        """走行開始時に補正テーブルをロードして ILCController を返す。

        テーブルが無い/無効/空、またはロード失敗時は None（補正なしで走行）。
        """
        try:
            rec = await self._repo.get(profile.id, mode.id)
        except Exception:
            _logger.exception("ILC テーブルのロードに失敗: 補正なしで走行を継続")
            return None
        if rec is None or not getattr(rec, "enabled", False):
            return None
        table = getattr(rec, "table", None)
        if table is None or not table.efforts:
            return None
        _logger.info(
            "ILC 補正を適用: profile=%s mode=%s 反復%d回目",
            profile.id,
            mode.id,
            table.iteration,
        )
        return ILCController(table)

    async def learn_from_session(
        self,
        session_id: str,
        profile: VehicleProfile,
        mode: DrivingMode,
        kpi_summary: dict[str, float],
    ) -> None:
        """走行正常完了後に残差から次回テーブルを学習して upsert する。

        例外は走行停止処理に伝播させない（fire-and-forget 前提。ここで握りつぶしログ）。
        """
        try:
            await self._learn(session_id, profile, mode, kpi_summary)
        except Exception:
            _logger.exception("ILC 学習に失敗（走行は正常完了済み・次回は前回テーブルを使用）")

    async def _learn(
        self,
        session_id: str,
        profile: VehicleProfile,
        mode: DrivingMode,
        kpi_summary: dict[str, float],
    ) -> None:
        logs = await self._session_repo.list_logs(session_id, limit=1_000_000)
        pairs = [
            (log.timestamp, log.ref_speed_kmh, log.actual_speed_kmh)
            for log in logs
            if log.ref_speed_kmh is not None
        ]
        if len(pairs) < _MIN_LOGS_FOR_LEARN:
            _logger.info("ILC 学習をスキップ: 有効ログ %d 件が不足", len(pairs))
            return

        t0 = pairs[0][0]
        times = [(ts - t0).total_seconds() for ts, _ref, _act in pairs]
        errors = [float(ref - act) for _ts, ref, act in pairs]  # e = ref - actual

        rec = await self._repo.get(profile.id, mode.id)
        table: ILCTable = getattr(rec, "table", None) or ILCTable(efforts=[], dt_s=ILC_DT_S)
        history: list[dict[str, object]] = list(getattr(rec, "kpi_history", []) or [])
        if rec is not None and not getattr(rec, "enabled", True):
            _logger.info("ILC 無効のため学習しない: profile=%s mode=%s", profile.id, mode.id)
            return

        new_p95 = kpi_summary.get("p95_kmh")
        if is_diverged(table.best_p95_kmh, new_p95):
            _logger.warning(
                "ILC 発散検知: 今回 p95=%.3f > 最良 p95=%.3f×1.2 のため学習スキップ（据え置き）",
                new_p95 if new_p95 is not None else float("nan"),
                table.best_p95_kmh if table.best_p95_kmh is not None else float("nan"),
            )
            return

        dyn = profile.dynamics_params
        l_gain = l_gain_from_fopdt(dyn.fopdt_k)
        delta_s = dyn.fopdt_theta if dyn.fopdt_theta is not None else 0.0
        new_table = self._learner.update(
            table,
            times,
            errors,
            l_gain=l_gain,
            delta_s=delta_s,
            cutoff_hz=ILC_CUTOFF_HZ,
            dt_s=ILC_DT_S,
            new_p95_kmh=new_p95,
        )
        history.append(
            {
                "iteration": new_table.iteration,
                "p95_kmh": new_p95,
                "max_kmh": kpi_summary.get("max_abs_deviation_kmh"),
                "reversal_max_per_5s": kpi_summary.get("reversal_max_per_5s"),
            }
        )
        await self._repo.upsert(profile.id, mode.id, new_table, history)
        _logger.info(
            "ILC 学習完了: profile=%s mode=%s → 反復%d回目 (p95=%s)",
            profile.id,
            mode.id,
            new_table.iteration,
            f"{new_p95:.3f}" if new_p95 is not None else "N/A",
        )


__all__ = ["ILCService", "ILCRepositoryProtocol", "SessionLogReaderProtocol"]
