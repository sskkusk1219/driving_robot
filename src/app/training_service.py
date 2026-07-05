"""運転モデル学習＋PID自動適合をルーターから独立させたアプリケーションサービス。

FastAPI に依存しない。学習運転ログから逆モデルを再学習し、観測可能な物理定数・
FOPDT同定によるSIMC初期ゲインを算出してプロファイルへ永続化・制御スタックへ反映する。
2段階学習フロー（learning_cycle.py）の各訓練フェーズ、および `/learning/train`
ルーター（手動パス）の双方から呼ばれる共通処理。
"""

from __future__ import annotations

import asyncio
from dataclasses import dataclass
from typing import Protocol

from src.domain.model_training import (
    DEFAULT_FEATURE_SPEC,
    FeatureSpec,
    estimate_dynamics_params,
    train_inverse_model,
)
from src.domain.pid_tuning import compute_pid_gains_simc, identify_fopdt, initial_preview_from_fopdt
from src.infra.settings import ModelSettings
from src.models.drive_log import DriveLog
from src.models.profile import DynamicsParams, FeedforwardParams, PIDGains, VehicleProfile


def feature_spec_from_settings(settings: ModelSettings) -> FeatureSpec:
    """`ModelSettings`(config/settings.toml の `[model]` セクション)を `FeatureSpec` へ変換する。"""
    return FeatureSpec(
        lookahead_horizons_s=settings.lookahead_horizons_s,
        past_horizons_s=settings.past_horizons_s,
        regime_horizon_s=settings.regime_horizon_s,
        include_v0_sq=settings.include_v0_sq,
        include_dv_regime_x_v0=settings.include_dv_regime_x_v0,
        accel_horizons_s=settings.accel_horizons_s,
    )


class ProfileRepoProtocol(Protocol):
    async def get_by_id(self, profile_id: str) -> VehicleProfile | None: ...
    async def update(self, profile: VehicleProfile) -> VehicleProfile | None: ...


class SessionRepoProtocol(Protocol):
    async def list_logs_for_training(
        self,
        profile_id: str,
        session_ids: list[str] | None = None,
        limit: int = 100_000,
    ) -> list[DriveLog]: ...


class ControllerProtocol(Protocol):
    def refresh_active_profile(self, profile: VehicleProfile) -> bool: ...


@dataclass
class TrainResult:
    model_path: str
    metrics: dict[str, dict[str, float]]
    feedforward_params: FeedforwardParams
    pid_gains: PIDGains
    pid_auto_tuned: bool
    dynamics_params: DynamicsParams


async def train_and_apply(
    *,
    profile_repo: ProfileRepoProtocol,
    session_repo: SessionRepoProtocol,
    controller: ControllerProtocol,
    profile_id: str,
    session_ids: list[str],
    update_pid_gains: bool = True,
    feature_spec: FeatureSpec = DEFAULT_FEATURE_SPEC,
) -> TrainResult:
    """指定セッションのログから逆モデル・物理定数を再学習し、プロファイルへ永続化する。

    Args:
        profile_id: 対象プロファイルの UUID 文字列（呼び出し元が存在確認済みであること）
        session_ids: 学習に使うセッション ID 列（空であってはならない。呼び出し元が解決する）
        update_pid_gains: True の場合、FOPDT同定・SIMC計算で `profile.pid_gains` を上書きする。
            False の場合はこれらをスキップし、既存ゲイン（座標降下の適合結果等）を保持する
            （2段階学習フローの再学習フェーズで1段目の適合結果を消さないために使う）。
        feature_spec: 特徴量構成。既定は現行9特徴（`DEFAULT_FEATURE_SPEC`）。本番の変更は
            `config/settings.toml` の `[model]` セクション経由でのみ行う（オフライン評価後に判断）。

    Returns:
        学習結果（モデルパス・メトリクス・物理定数・PIDゲイン・自動適合の有無）。

    Raises:
        LearningDataError: 学習サンプルが不足しモデル構築できない場合
        RuntimeError: アクティブプロファイルへの反映に失敗した場合（シーケンスバグの検出）
    """
    profile = await profile_repo.get_by_id(profile_id)
    if profile is None:
        raise ValueError(f"プロファイル {profile_id!r} が見つかりません")

    logs = await session_repo.list_logs_for_training(profile_id, session_ids)
    model_path, metrics = await asyncio.to_thread(
        train_inverse_model, logs, profile, "data/models", feature_spec
    )

    # 観測可能な物理定数のみログから推定して上書き（不足項目は既存値を保持）
    new_params = await asyncio.to_thread(estimate_dynamics_params, logs, profile.feedforward_params)

    pid_auto_tuned = False
    if update_pid_gains:
        # 学習ログのアクセル保持区間から FOPDT を同定し SIMC で PID を自動適合する。
        # 同定不能（区間不足）なら既存ゲイン・dynamics_params を保持する。
        # CPU 同期処理のため to_thread。
        fopdt = await asyncio.to_thread(identify_fopdt, logs, profile)
        if fopdt is not None:
            new_gains = compute_pid_gains_simc(fopdt, profile)
            if new_gains != profile.pid_gains:
                profile.pid_gains = new_gains
                pid_auto_tuned = True
            # 先読み補償(preview_time_s)の初期値としてθを設定する。SIMC ゲイン自体は
            # θフルのまま計算済み（ループのむだ時間は不変のため割り引かない）。
            profile.dynamics_params = DynamicsParams(
                preview_time_s=initial_preview_from_fopdt(fopdt),
                fopdt_k=fopdt.k,
                fopdt_tau=fopdt.tau,
                fopdt_theta=fopdt.theta,
            )

    profile.model_path = model_path
    profile.feedforward_params = new_params
    updated = await profile_repo.update(profile)

    # 学習結果をアクティブプロファイルの制御スタックへ即時反映する。
    # DB だけ更新して in-memory を放置すると、学習直後の走行が旧モデル
    # （または FF なし）で実行される（コードレビュー 2026-06-11 指摘 #10）。
    applied_profile = updated if updated is not None else profile
    if not controller.refresh_active_profile(applied_profile):
        raise RuntimeError(
            "学習結果をアクティブプロファイルへ反映できませんでした"
            "（アクティブでない、または走行系状態のため）"
        )

    return TrainResult(
        model_path=model_path,
        metrics=metrics,
        feedforward_params=new_params,
        pid_gains=profile.pid_gains,
        pid_auto_tuned=pid_auto_tuned,
        dynamics_params=profile.dynamics_params,
    )
