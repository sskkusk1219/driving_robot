"""D1・V4: train_candidate_model.py のユニットテスト。

先読みホライズンのずらし（V4・C3 用）、A1（そのペダルが効いている行だけ）でのラベル選択が
dv の符号ではなく開度のしきい値で決まること、CLI が指令/実開度どちらのラベルでも pkl を
保存できることを確かめる。
"""

from __future__ import annotations

from datetime import UTC, datetime
from pathlib import Path

import numpy as np
import pytest

from tests.research import config as cfgmod
from tests.research import drive_log as dlmod
from tests.research import train_candidate_model as tcm
from tests.research.axis_monitor import AxisMonitor
from tests.research.ff_model import DEFAULT_FEATURE_SPEC
from tests.research.research_types import DriveLogData
from tests.research.test_research_model_analysis import _logs
from tests.research.vehicle import opening_to_pulse


def test_shifted_spec_adds_offset_only_to_lookahead_and_regime() -> None:
    base = DEFAULT_FEATURE_SPEC
    shifted = tcm.shifted_spec(base, 0.5)
    assert shifted.lookahead_horizons_s == tuple(h + 0.5 for h in base.lookahead_horizons_s)
    assert shifted.regime_horizon_s == pytest.approx(base.regime_horizon_s + 0.5)
    assert shifted.past_horizons_s == base.past_horizons_s  # 過去ホライズンは変えない


def test_shifted_spec_zero_returns_same_object() -> None:
    base = DEFAULT_FEATURE_SPEC
    assert tcm.shifted_spec(base, 0.0) is base


def test_effective_regime_data_splits_by_deadband_not_dv_sign() -> None:
    """A1: dv の符号ではなく、そのペダルの開度が不感帯以上かで分ける。"""
    logs, patterns, phases = _logs()
    accel, brake = tcm.effective_regime_data(
        logs, patterns, phases,
        accel_deadband_pct=10.0, brake_deadband_pct=13.68, spec=DEFAULT_FEATURE_SPEC,
    )
    assert (accel.y >= 10.0).all()
    assert (brake.y >= 13.68).all()
    # ブレーキ 10% 区間（不感帯未満）はどちらのモデルにも入らない
    assert len(accel.y) + len(brake.y) < len(logs)


def test_pattern_cv_returns_none_when_single_pattern() -> None:
    logs, patterns, phases = _logs()
    accel, _ = tcm.effective_regime_data(
        logs, patterns, phases, accel_deadband_pct=10.0, brake_deadband_pct=13.68,
        spec=DEFAULT_FEATURE_SPEC,
    )
    assert len(np.unique(accel.pattern)) == 1
    assert tcm.pattern_cv(accel) is None


def test_pattern_cv_covers_multiple_patterns(tmp_path: Path) -> None:
    csv_path = tmp_path / "log.csv"
    _write_training_csv(csv_path)
    from tests.research.model_analysis import load_training_rows

    logs, patterns, phases = load_training_rows(csv_path, label="cmd")
    _, brake = tcm.effective_regime_data(
        logs, patterns, phases, accel_deadband_pct=10.0, brake_deadband_pct=13.68,
        spec=DEFAULT_FEATURE_SPEC,
    )
    assert len(np.unique(brake.pattern)) == 2  # 2:BRAKE_A / 3:BRAKE_B
    cv = tcm.pattern_cv(brake)
    assert cv is not None
    assert cv.n == len(brake.y)
    assert cv.mae >= 0.0


def _sample(
    speed: float, accel_cmd: float, brake_cmd: float, pattern: str,
    accel_actual: float, brake_actual: float,
) -> dlmod.DriveSample:
    data = DriveLogData(
        ref_speed_kmh=None, actual_speed_kmh=speed,
        accel_opening=accel_cmd, brake_opening=brake_cmd,
        accel_pos=opening_to_pulse(accel_cmd), brake_pos=opening_to_pulse(brake_cmd),
        accel_current=0.0, brake_current=0.0,
    )
    return dlmod.DriveSample(
        elapsed_s=0.0, timestamp=datetime.now(tz=UTC), data=data,
        section=dlmod.SECTION_PATTERN_DRIVE, phase="DRIVE", pattern=pattern,
        monitor_accel=AxisMonitor(
            position_pulse=opening_to_pulse(accel_actual), current_ma=0.0,
            alarm_code=0, servo_on=True, moving=False, pos_done=True,
        ),
        monitor_brake=AxisMonitor(
            position_pulse=opening_to_pulse(brake_actual), current_ma=0.0,
            alarm_code=0, servo_on=True, moving=False, pos_done=True,
        ),
    )


def _write_training_csv(path: Path) -> None:
    samples = []
    for i in range(100):
        samples.append(_sample(
            10.0 + 0.2 * i, accel_cmd=15.0, brake_cmd=0.0, pattern="1:ACCEL_SWEEP",
            accel_actual=15.0, brake_actual=0.0,
        ))
    for i in range(50):
        samples.append(_sample(
            30.0 - 0.2 * i, accel_cmd=0.0, brake_cmd=20.0, pattern="2:BRAKE_A",
            accel_actual=0.0, brake_actual=20.0,
        ))
    for i in range(50):
        samples.append(_sample(
            20.0 - 0.2 * i, accel_cmd=0.0, brake_cmd=20.0, pattern="3:BRAKE_B",
            accel_actual=0.0, brake_actual=20.0,
        ))
    dlmod.write_csv(samples, path)


@pytest.mark.parametrize("label", ["cmd", "actual"])
def test_main_trains_and_saves_pkl(tmp_path: Path, label: str) -> None:
    csv_path = tmp_path / "log.csv"
    _write_training_csv(csv_path)
    cfg_path = tmp_path / "cfg.yaml"
    cfgmod.load_config(cfg_path)  # 既定値で作る
    out_dir = tmp_path / "models"

    rc = tcm.main([
        str(csv_path), "--label", label, "--config", str(cfg_path), "--out-dir", str(out_dir),
    ])
    assert rc == 0
    assert list(out_dir.glob("*.pkl"))
