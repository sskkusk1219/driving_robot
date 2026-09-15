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

from src.domain.model_training import DEFAULT_FEATURE_SPEC
from src.models.drive_log import DriveLogData
from tests.research import config as cfgmod
from tests.research import drive_log as dlmod
from tests.research import train_candidate_model as tcm
from tests.research.axis_monitor import AxisMonitor
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


# ── C6: --cruise-curve-from ────────────────────────────────────────


def _cruise_hold_sample(
    elapsed_s: float, speed: float, opening: float, pattern: str
) -> dlmod.DriveSample:
    """定速階段（CRUISE_HOLD）の合成 1 行（アクセル開度一定・ブレーキ 0）。"""
    data = DriveLogData(
        ref_speed_kmh=None, actual_speed_kmh=speed,
        accel_opening=opening, brake_opening=0.0,
        accel_pos=opening_to_pulse(opening), brake_pos=opening_to_pulse(0.0),
        accel_current=0.0, brake_current=0.0,
    )
    return dlmod.DriveSample(
        elapsed_s=elapsed_s, timestamp=datetime.now(tz=UTC), data=data,
        section=dlmod.SECTION_PATTERN_DRIVE, phase="CRUISE_HOLD", pattern=pattern,
        monitor_accel=AxisMonitor(
            position_pulse=opening_to_pulse(opening), current_ma=0.0,
            alarm_code=0, servo_on=True, moving=False, pos_done=True,
        ),
        monitor_brake=AxisMonitor(
            position_pulse=opening_to_pulse(0.0), current_ma=0.0,
            alarm_code=0, servo_on=True, moving=False, pos_done=True,
        ),
    )


def _cruise_hold_samples(
    pattern: str, steps: list[tuple[float, float]], count_per_step: int
) -> list[dlmod.DriveSample]:
    """1 パターン内で複数車速を順に保持する合成行（`extract_hold_steps` は 1 パターン内で
    `learning.cruise_hold_speeds_kmh` を順に消化するため、車速ごとに別パターンにはしない）。

    settle_s=0.3・hold_s=0.5（テスト用の小さな custom config）に余裕を持たせた行数
    （test_research_cruise_curve.py の合成行と同じ考え方）。
    """
    samples: list[dlmod.DriveSample] = []
    t = 0.0
    for speed, opening in steps:
        for _ in range(count_per_step):
            samples.append(_cruise_hold_sample(round(t, 1), speed, opening, pattern))
            t = round(t + 0.1, 1)
    return samples


def _write_training_csv_with_cruise_hold(path: Path) -> None:
    """`_write_training_csv` に定速階段（2 車速）を足した CSV（C6 用の実測テーブルが作れる）。"""
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
    samples += _cruise_hold_samples("4:CRUISE_TRIM", [(15.0, 12.0), (25.0, 16.0)], 12)
    dlmod.write_csv(samples, path)


def _write_small_cruise_hold_config(cfg_path: Path) -> None:
    """テストが速く終わるよう settle/hold を小さくした最小 YAML（他セクションは既定値）。"""
    cfg_path.write_text(
        "learning:\n"
        "  cruise_hold_speeds_kmh: [15.0, 25.0]\n"
        "  cruise_hold_settle_tol_kmh: 1.0\n"
        "  cruise_hold_settle_s: 0.3\n"
        "  cruise_hold_hold_s: 0.5\n"
        "  cruise_hold_step_timeout_s: 5.0\n",
        encoding="utf-8",
    )


def test_main_with_cruise_curve_from_writes_c6_pkl_and_speed_bands(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    csv_path = tmp_path / "log.csv"
    _write_training_csv_with_cruise_hold(csv_path)
    cfg_path = tmp_path / "cfg.yaml"
    _write_small_cruise_hold_config(cfg_path)
    out_dir = tmp_path / "models"

    rc = tcm.main([
        str(csv_path), "--label", "cmd", "--config", str(cfg_path), "--out-dir", str(out_dir),
        "--cruise-curve-from", str(csv_path),
    ])
    assert rc == 0

    pkls = list(out_dir.glob("*.pkl"))
    assert pkls
    import pickle  # noqa: PLC0415 - このテストだけで使う

    with pkls[0].open("rb") as f:
        payload = pickle.load(f)  # noqa: S301 - テストで作った自前のファイル
    assert "cruise_curve" in payload

    out = capsys.readouterr().out
    assert "feedforward.candidate: C6" in out
    assert "実測テーブル" in out
    assert "km/h: MAE=" in out  # 速度帯別 CV の表示
    assert "参考値" in out


def test_main_without_cruise_curve_from_has_no_c6_markers(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    """C1（既定）は cruise_curve キー無し・C6 の注記も出ない。"""
    csv_path = tmp_path / "log.csv"
    _write_training_csv(csv_path)
    cfg_path = tmp_path / "cfg.yaml"
    cfgmod.load_config(cfg_path)
    out_dir = tmp_path / "models"

    rc = tcm.main([
        str(csv_path), "--label", "cmd", "--config", str(cfg_path), "--out-dir", str(out_dir),
    ])
    assert rc == 0

    pkls = list(out_dir.glob("*.pkl"))
    import pickle  # noqa: PLC0415

    with pkls[0].open("rb") as f:
        payload = pickle.load(f)  # noqa: S301
    assert "cruise_curve" not in payload

    out = capsys.readouterr().out
    assert "feedforward.candidate: C6" not in out
    assert "参考値" not in out
