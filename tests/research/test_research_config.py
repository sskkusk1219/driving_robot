"""研究開発用ハーネスの設定ローダ／CLI のユニットテスト。"""

from __future__ import annotations

from pathlib import Path

import pytest

from tests.research import config as cfgmod
from tests.research import main as mainmod


def _load_default() -> cfgmod.ResearchConfig:
    return cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)


def test_default_config_loads_and_validates() -> None:
    """同梱の config_testVehicle.yaml はそのまま検証を通る。"""
    cfg = _load_default()
    assert cfg.vehicle.max_speed_kmh == 140.0
    assert cfg.vehicle.max_decel_g == 0.4
    assert cfg.feedforward.model_path.endswith(".pkl")
    assert cfgmod.validate_config(cfg) == []


def test_log_interval_must_be_multiple_of_loop_interval() -> None:
    cfg = _load_default()
    cfg.control.log_interval_ms = 130
    problems = cfgmod.validate_config(cfg)
    assert any("整数倍" in p for p in problems)


def test_p95_above_hard_limit_is_rejected() -> None:
    cfg = _load_default()
    cfg.kpi.p95_deviation_kmh = 1.5
    problems = cfgmod.validate_config(cfg)
    assert any("max_abs_deviation_kmh" in p for p in problems)


def test_unknown_candidate_is_rejected() -> None:
    cfg = _load_default()
    assert cfg.feedforward.candidate == "C1"  # 既定
    cfg.feedforward.candidate = "C9"
    problems = cfgmod.validate_config(cfg)
    assert any("candidate" in p for p in problems)
    cfg.feedforward.candidate = "C5"
    assert not any("candidate" in p for p in cfgmod.validate_config(cfg))
    cfg.feedforward.candidate = "C6"  # 2026-09-15 追加（骨格を実測テーブルにする案）
    assert not any("candidate" in p for p in cfgmod.validate_config(cfg))


def test_curve_length_mismatch_is_rejected() -> None:
    cfg = _load_default()
    cfg.feedforward.coast_decel_speeds_kmh = [10.0, 20.0, 30.0]
    cfg.feedforward.coast_decel_kmhs = [1.0, 2.0]
    problems = cfgmod.validate_config(cfg)
    assert any("点数" in p for p in problems)


def test_decel_stop_thresholds_must_be_ordered() -> None:
    cfg = _load_default()
    cfg.decel_stop.release_above_g = 0.15  # 目標 0.2G より小さい
    assert any("release_above_g" in p for p in cfgmod.validate_config(cfg))
    cfg.decel_stop.release_above_g = 0.5  # vehicle.max_decel_g 0.4 を超える
    assert any("release_above_g" in p for p in cfgmod.validate_config(cfg))
    cfg.decel_stop.release_above_g = 0.3
    cfg.decel_stop.press_margin_g = 0.25  # 目標以上の margin
    assert any("press_margin_g" in p for p in cfgmod.validate_config(cfg))


def test_learning_offsets_are_validated() -> None:
    cfg = _load_default()
    cfg.learning.brake_hold_offsets_pct = [1.0, 0.5]  # 昇順でない
    assert any("brake_hold_offsets_pct" in p for p in cfgmod.validate_config(cfg))
    cfg.learning.brake_hold_offsets_pct = [0.5, 1.0]
    cfg.learning.cruise_trim_offsets_pct = []  # 空
    assert any("cruise_trim_offsets_pct" in p for p in cfgmod.validate_config(cfg))
    cfg.learning.cruise_trim_offsets_pct = [1.0]
    cfg.learning.brake_gain_min_offset_pct = 0.0  # 正値でない
    assert any("brake_gain_min_offset_pct" in p for p in cfgmod.validate_config(cfg))


def test_added_pattern_settings_are_validated() -> None:
    """A2・A5 の追加段: 空リストは可（足さない）、昇順でないと不可、開始車速は 0〜最高速。"""
    cfg = _load_default()
    assert cfg.learning.accel_sweep_add_offsets_pct == [2.0, 5.0, 8.0]
    assert cfg.learning.brake_hold_low_offsets_pct == [0.5, 2.0, 4.0]
    assert cfg.learning.brake_hold_low_start_kmh == 60.0
    assert cfg.learning.timeout_s == 1800.0
    cfg.learning.accel_sweep_add_offsets_pct = []
    cfg.learning.brake_hold_low_offsets_pct = []
    assert cfgmod.validate_config(cfg) == []
    cfg.learning.accel_sweep_add_offsets_pct = [5.0, 2.0]
    assert any("accel_sweep_add_offsets_pct" in p for p in cfgmod.validate_config(cfg))
    cfg.learning.accel_sweep_add_offsets_pct = [2.0]
    cfg.learning.brake_hold_low_start_kmh = cfg.vehicle.max_speed_kmh
    assert any("brake_hold_low_start_kmh" in p for p in cfgmod.validate_config(cfg))


def test_cruise_hold_settings_are_validated() -> None:
    """2026-09-14 定速階段（段2）: 空リストは可、車速は 0〜最高速の昇順、ゲイン・時定数は正値。"""
    cfg = _load_default()
    lr = cfg.learning
    assert lr.cruise_hold_speeds_kmh == [
        30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 110.0, 120.0, 130.0
    ]
    assert cfgmod.validate_config(cfg) == []

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

    lr.cruise_hold_speeds_kmh = []
    assert cfgmod.validate_config(cfg) == []  # 空は可（足さない）
    lr.cruise_hold_speeds_kmh = [40.0, 30.0]  # 昇順でない
    assert problems_about("cruise_hold_speeds_kmh")
    lr.cruise_hold_speeds_kmh = [30.0, cfg.vehicle.max_speed_kmh]  # 最高速以上
    assert problems_about("cruise_hold_speeds_kmh")
    lr.cruise_hold_speeds_kmh = [30.0, 40.0]

    lr.cruise_hold_settle_tol_kmh = 0.0
    assert problems_about("cruise_hold_settle_tol_kmh")
    lr.cruise_hold_settle_tol_kmh = 1.0
    lr.cruise_hold_settle_s = 0.0
    assert problems_about("cruise_hold_settle_s")
    lr.cruise_hold_settle_s = 3.0
    lr.cruise_hold_hold_s = 0.0
    assert problems_about("cruise_hold_hold_s")
    lr.cruise_hold_hold_s = 8.0
    lr.cruise_hold_step_timeout_s = 0.0
    assert problems_about("cruise_hold_step_timeout_s")
    lr.cruise_hold_step_timeout_s = 30.0
    lr.cruise_hold_kp = 0.0
    assert problems_about("cruise_hold_kp")
    lr.cruise_hold_kp = 0.3
    lr.cruise_hold_ki = -0.1
    assert problems_about("cruise_hold_ki")
    lr.cruise_hold_ki = 0.05
    lr.cruise_hold_max_rate_pct_per_s = 0.0
    assert problems_about("cruise_hold_max_rate_pct_per_s")
    lr.cruise_hold_max_rate_pct_per_s = 1.0
    lr.cruise_hold_initial_offset_pct = -1.0
    assert problems_about("cruise_hold_initial_offset_pct")
    lr.cruise_hold_initial_offset_pct = 7.0
    assert cfgmod.validate_config(cfg) == []


def test_a3a4_settings_are_validated() -> None:
    """A3・A4: 空リストは可、階段は降順・高ブレーキは昇順、開始車速は 0〜最高速。"""
    cfg = _load_default()
    lr = cfg.learning
    assert (lr.trim_stair_start_kmh, lr.trim_stair_offsets_pct, lr.trim_stair_step_s) == (
        [120.0, 90.0, 50.0], [8.0, 5.0, 2.0], 8.0
    )
    assert lr.brake_hold_hard_offsets_pct == [7.0, 17.0, 27.0, 37.0]
    assert (lr.brake_hold_hard_start_kmh, lr.brake_hold_hard_accel_offset_pct) == (20.0, 8.0)

    def problems_about(key: str) -> bool:
        return any(key in p for p in cfgmod.validate_config(cfg))

    lr.trim_stair_start_kmh, lr.trim_stair_offsets_pct = [], []
    lr.brake_hold_hard_offsets_pct = []
    assert cfgmod.validate_config(cfg) == []
    lr.trim_stair_start_kmh = [120.0]
    assert problems_about("trim_stair_offsets_pct も要る")
    lr.trim_stair_offsets_pct = [2.0, 8.0]
    assert problems_about("trim_stair_offsets_pct は")
    lr.trim_stair_offsets_pct = [8.0, 2.0]
    lr.trim_stair_start_kmh = [cfg.vehicle.max_speed_kmh]
    assert problems_about("trim_stair_start_kmh")
    lr.trim_stair_start_kmh = [120.0]
    lr.trim_stair_step_s = 0.0
    assert problems_about("trim_stair_step_s")
    lr.trim_stair_step_s = 8.0
    lr.brake_hold_hard_offsets_pct = [17.0, 7.0]
    assert problems_about("brake_hold_hard_offsets_pct")
    lr.brake_hold_hard_offsets_pct = [7.0]
    lr.brake_hold_hard_start_kmh = 0.0
    assert problems_about("brake_hold_hard_start_kmh")
    lr.brake_hold_hard_start_kmh = 20.0
    lr.brake_hold_hard_accel_offset_pct = 0.0
    assert problems_about("brake_hold_hard_accel_offset_pct")
    lr.brake_hold_hard_accel_offset_pct = 8.0
    assert cfgmod.validate_config(cfg) == []


def test_negative_standby_margin_is_rejected() -> None:
    cfg = _load_default()
    cfg.mode_drive.standby_margin_pct = -0.5
    assert any("standby_margin_pct" in p for p in cfgmod.validate_config(cfg))
    cfg.mode_drive.standby_margin_pct = 0.0
    assert not any("standby_margin_pct" in p for p in cfgmod.validate_config(cfg))


def test_checks_section_loads_from_default_yaml() -> None:
    """既定 YAML は開発時に UPS を使わないため checks.init_ups/pre_ups だけ false。"""
    cfg = _load_default()
    assert cfg.checks.init_ups is False
    assert cfg.checks.pre_ups is False
    assert cfg.checks.init_servo_comm is True
    assert cfg.checks.init_clear_errors is True
    assert cfg.checks.init_servo_on is True
    assert cfg.checks.init_can is True
    assert cfg.checks.init_home_return is True
    assert cfg.checks.pre_communication is True
    assert cfg.checks.pre_servo_state is True
    assert cfg.checks.pre_profile is True
    assert cfg.checks.pre_actuator_position is True
    assert cfg.checks.pre_brake_stop is True
    assert cfg.checks.pre_vehicle_stopped is True
    assert cfgmod.validate_config(cfg) == []


def test_pre_ups_true_requires_init_ups_true() -> None:
    cfg = _load_default()
    cfg.checks.pre_ups = True  # init_ups は既定 YAML のまま false
    assert any("checks.init_ups" in p for p in cfgmod.validate_config(cfg))
    cfg.checks.init_ups = True
    assert not any("checks.init_ups" in p for p in cfgmod.validate_config(cfg))


def test_negative_gain_is_rejected() -> None:
    cfg = _load_default()
    cfg.pid.kp = -1.0
    assert any("pid.kp" in p for p in cfgmod.validate_config(cfg))


def test_unknown_key_raises(tmp_path: Path) -> None:
    path = tmp_path / "cfg.yaml"
    path.write_text("pid:\n  kp: 1.0\n  kq: 2.0\n", encoding="utf-8")
    with pytest.raises(cfgmod.ConfigError, match="pid.kq"):
        cfgmod.load_config(path)


def test_unknown_section_raises(tmp_path: Path) -> None:
    path = tmp_path / "cfg.yaml"
    path.write_text("nonesuch:\n  a: 1\n", encoding="utf-8")
    with pytest.raises(cfgmod.ConfigError, match="nonesuch"):
        cfgmod.load_config(path)


def test_missing_config_is_copied_from_default(tmp_path: Path) -> None:
    path = tmp_path / "sub" / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    assert path.exists()
    assert cfg.source_path == path
    assert cfg.vehicle.max_speed_kmh == 140.0


def test_save_preserves_comments_and_layout(tmp_path: Path) -> None:
    """値の書き戻しでコメント・行順・インデントが壊れないこと。"""
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    before = path.read_text(encoding="utf-8").splitlines()

    changed = cfg.save(
        {
            "pid.kp": 4.762333295145048,
            "pid.ki": 0.5720877379661851,
            "feedforward.coast_decel_speeds_kmh": [5.0, 15.0, 25.0],
            "feedforward.coast_decel_kmhs": [1.6, 1.81, 2.7],
            "output.plot": False,
            "control.loop_interval_ms": 20,
            "modes.wltp_mode_name": "01_WLTP_Low,Mid,Hi,ExHi",
        }
    )
    assert len(changed) == 7
    after = path.read_text(encoding="utf-8").splitlines()
    assert len(before) == len(after)
    # コメントは残る
    assert any("比例ゲイン" in line for line in after)
    assert any("惰行カーブ 速度グリッド" in line for line in after)

    reloaded = cfgmod.load_config(path)
    assert reloaded.pid.kp == pytest.approx(4.76233, rel=1e-4)
    assert reloaded.feedforward.coast_decel_speeds_kmh == [5.0, 15.0, 25.0]
    assert reloaded.output.plot is False
    assert reloaded.control.loop_interval_ms == 20
    assert reloaded.modes.wltp_mode_name == "01_WLTP_Low,Mid,Hi,ExHi"


def test_save_unknown_key_raises(tmp_path: Path) -> None:
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    with pytest.raises(cfgmod.ConfigError, match="見つかりません"):
        cfg.save({"pid.no_such_gain": 1.0})


def test_save_nested_mapping_key_raises(tmp_path: Path) -> None:
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    with pytest.raises(cfgmod.ConfigError, match="入れ子"):
        cfg.save({"pid": 1.0})


def test_split_comment_ignores_hash_inside_quotes() -> None:
    value, comment = cfgmod._split_comment(' "a#b"  # 実コメント')
    assert value.strip() == '"a#b"'
    assert comment.strip() == "# 実コメント"


def test_split_comment_ignores_hash_inside_list() -> None:
    value, comment = cfgmod._split_comment(" [1, 2]  # 説明")
    assert value.strip() == "[1, 2]"
    assert comment.strip() == "# 説明"


def test_format_float_keeps_decimal_point() -> None:
    assert cfgmod._format_value(4.0) == "4.0"
    assert cfgmod._format_value(0.5720877379661851) == "0.572088"
    assert cfgmod._format_value(True) == "true"
    assert cfgmod._format_value(50) == "50"


def test_save_accepts_rounding_to_six_significant_digits(tmp_path: Path) -> None:
    """有効 6 桁に丸めて書くので、桁の多い値でも書き戻し検証で落ちないこと。"""
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    cfg.save({"feedforward.creep_speed_kmh": 123.456789})
    assert cfgmod.load_config(path).feedforward.creep_speed_kmh == pytest.approx(123.457)


# ── CLI ──────────────────────────────────────────────────────────────


def test_resolve_steps_defaults_to_step0() -> None:
    args = mainmod.build_parser().parse_args([])
    assert [s.number for s in mainmod.resolve_steps(args)] == [0]


def test_resolve_steps_upto() -> None:
    args = mainmod.build_parser().parse_args(["--upto", "3"])
    assert [s.number for s in mainmod.resolve_steps(args)] == [0, 1, 2, 3]


def test_resolve_steps_only_and_list() -> None:
    parser = mainmod.build_parser()
    assert [s.number for s in mainmod.resolve_steps(parser.parse_args(["--only", "4"]))] == [4]
    args = parser.parse_args(["--steps", "0,2,3"])
    assert [s.number for s in mainmod.resolve_steps(args)] == [0, 2, 3]


def test_resolve_steps_rejects_unknown_number() -> None:
    args = mainmod.build_parser().parse_args(["--only", "99"])
    with pytest.raises(SystemExit, match="存在しません"):
        mainmod.resolve_steps(args)


def test_step0_runs_on_default_config(tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    # 結果ディレクトリを tmp に逃がし、リポジトリを汚さない
    cfg.save(
        {
            "output.results_dir": str(tmp_path / "results"),
            "feedforward.model_path": str(tmp_path / "results" / "models" / "ff.pkl"),
        }
    )
    assert mainmod.main(["--only", "0", "--config", str(path)]) == 0
    out = capsys.readouterr().out
    assert "設定の検証: OK" in out
    assert "手順 0 完了" in out
    assert (tmp_path / "results" / "models").is_dir()


def test_step4_stops_as_not_implemented(capsys: pytest.CaptureFixture[str]) -> None:
    assert mainmod.main(["--only", "4"]) == 3
    assert "未実装" in capsys.readouterr().out


def test_dry_run_does_not_touch_config(tmp_path: Path) -> None:
    missing = tmp_path / "cfg.yaml"
    assert mainmod.main(["--upto", "3", "--dry-run", "--config", str(missing)]) == 0
    assert not missing.exists()


def test_invalid_config_returns_exit_code_2(
    tmp_path: Path, capsys: pytest.CaptureFixture[str]
) -> None:
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    cfg.save({"vehicle.max_decel_g": 2.5})
    assert mainmod.main(["--only", "0", "--config", str(path)]) == 2
    assert "max_decel_g" in capsys.readouterr().out
