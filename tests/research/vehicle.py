"""開度の定義と、研究用設定（config_testVehicle.yaml）→ 本番 VehicleProfile への変換。

開度の定義（tests 環境。本番のキャリブレーションは使わない）:
    0%   = 原点復帰位置（0 pulse）
    100% = ストローク限界（本番 robot_controller._ACTUATOR_PULSE_MAX = 9500 pulse）
原点からペダルに触れるまでの隙間とペダル自体の遊びは、不感帯（feedforward.*_deadband_pct）
として扱い、手順 2-0 のペダル探索で実測する。

手順 2 以降は本番のドメインコード（LearningDriveManager / model_training）をそのまま呼ぶ
（走行ループ自体は pattern_loop.py が自前実装）。それらが受け取る VehicleProfile を、
DB ではなく YAML から組み立てる。
"""

from __future__ import annotations

from datetime import UTC, datetime

from src.app.robot_controller import _ACTUATOR_PULSE_MAX
from src.domain.control.conversions import opening_to_position
from src.models.calibration import CalibrationData
from src.models.profile import FeedforwardParams, PIDGains, StopConfig, VehicleProfile
from tests.research.config import ResearchConfig

STROKE_LIMIT_PULSE: int = _ACTUATOR_PULSE_MAX  # 開度 100% の位置 [pulse]（両軸共通）


def opening_to_pulse(opening_pct: float) -> int:
    """開度 [%] → 位置 [pulse]（0% = 原点、100% = ストローク限界）。"""
    return opening_to_position(opening_pct, 0, STROKE_LIMIT_PULSE)


def pulse_to_opening(pulse: int) -> float:
    """位置 [pulse] → 開度 [%]。"""
    return pulse * 100.0 / STROKE_LIMIT_PULSE


def calibration_data() -> CalibrationData:
    """PatternLoop（本番 LearningLoop 相当）が必須とする CalibrationData を
    「原点=0%・9500=100%」で作る。"""
    return CalibrationData(
        accel_zero_pos=0,
        accel_full_pos=STROKE_LIMIT_PULSE,
        accel_stroke=STROKE_LIMIT_PULSE,
        brake_zero_pos=0,
        brake_full_pos=STROKE_LIMIT_PULSE,
        brake_stroke=STROKE_LIMIT_PULSE,
        calibrated_at=datetime.now(tz=UTC),
        is_valid=True,
    )


def feedforward_params(cfg: ResearchConfig) -> FeedforwardParams:
    """feedforward セクション → FeedforwardParams（キー名は 1:1）。"""
    ff = cfg.feedforward
    return FeedforwardParams(
        creep_speed_kmh=ff.creep_speed_kmh,
        creep_rate_kmhs=ff.creep_rate_kmhs,
        engine_brake_decel_kmhs=ff.engine_brake_decel_kmhs,
        coast_decel_speeds_kmh=tuple(ff.coast_decel_speeds_kmh),
        coast_decel_kmhs=tuple(ff.coast_decel_kmhs),
        pedal_gain_speeds_kmh=tuple(ff.pedal_gain_speeds_kmh),
        accel_gain_kmhs_per_pct=tuple(ff.accel_gain_kmhs_per_pct),
        brake_gain_kmhs_per_pct=tuple(ff.brake_gain_kmhs_per_pct),
        stop_brake_opening_pct=ff.stop_brake_opening_pct,
        brake_deadband_pct=ff.brake_deadband_pct,
        accel_deadband_pct=ff.accel_deadband_pct,
    )


def build_vehicle_profile(cfg: ResearchConfig) -> VehicleProfile:
    """YAML 全体から本番の VehicleProfile を組み立てる。id はプロファイル名。"""
    v = cfg.vehicle
    now = datetime.now(tz=UTC)
    return VehicleProfile(
        id=v.name,
        name=v.name,
        max_accel_opening=v.max_accel_opening_pct,
        max_brake_opening=v.max_brake_opening_pct,
        max_speed=v.max_speed_kmh,
        max_decel_g=v.max_decel_g,
        pid_gains=PIDGains(kp=cfg.pid.kp, ki=cfg.pid.ki, kd=cfg.pid.kd),
        stop_config=StopConfig(
            deviation_threshold_kmh=v.stop_deviation_threshold_kmh,
            deviation_duration_s=v.stop_deviation_duration_s,
        ),
        calibration=calibration_data(),
        model_path=cfg.feedforward.model_path if cfg.feedforward.is_model_trained else None,
        created_at=now,
        updated_at=now,
        feedforward_params=feedforward_params(cfg),
    )
