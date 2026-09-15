"""settings.load_settings のユニットテスト（[model]・[learning]・[actuator] セクション中心）。"""

import tempfile
from pathlib import Path

import pytest

from src.infra.settings import (
    ActuatorAxisSettings,
    LearningSettings,
    ModelSettings,
    load_settings,
)


def _write_toml(content: str) -> Path:
    tmp = tempfile.NamedTemporaryFile(  # noqa: SIM115
        mode="w", suffix=".toml", delete=False, encoding="utf-8"
    )
    tmp.write(content)
    tmp.close()
    return Path(tmp.name)


class TestModelSettingsDefaults:
    def test_missing_model_section_uses_defaults(self) -> None:
        path = _write_toml('[serial]\naccel_port = "/dev/ttyUSB0"\n')
        settings = load_settings(path)
        assert settings.model == ModelSettings()

    def test_default_matches_current_nine_features(self) -> None:
        defaults = ModelSettings()
        assert defaults.lookahead_horizons_s == (0.5, 1.0, 2.0, 3.0)
        assert defaults.past_horizons_s == (0.5, 1.0)
        assert defaults.regime_horizon_s == 1.0
        assert defaults.include_v0_sq is True
        assert defaults.include_dv_regime_x_v0 is True
        assert defaults.accel_horizons_s == ()


class TestModelSettingsParsing:
    def test_custom_horizons_parsed_as_tuples(self) -> None:
        path = _write_toml(
            "[model]\n"
            "lookahead_horizons_s = [0.1, 0.2, 0.3]\n"
            "past_horizons_s = [0.1, 0.2]\n"
            "regime_horizon_s = 0.2\n"
            "accel_horizons_s = [0.2]\n"
        )
        settings = load_settings(path)
        assert settings.model.lookahead_horizons_s == (0.1, 0.2, 0.3)
        assert isinstance(settings.model.lookahead_horizons_s, tuple)
        assert settings.model.past_horizons_s == (0.1, 0.2)
        assert settings.model.regime_horizon_s == 0.2
        assert settings.model.accel_horizons_s == (0.2,)

    def test_partial_override_keeps_other_defaults(self) -> None:
        path = _write_toml("[model]\nregime_horizon_s = 2.0\n")
        settings = load_settings(path)
        assert settings.model.regime_horizon_s == 2.0
        assert settings.model.lookahead_horizons_s == (0.5, 1.0, 2.0, 3.0)

    def test_include_flags_parsed(self) -> None:
        path = _write_toml("[model]\ninclude_v0_sq = false\ninclude_dv_regime_x_v0 = false\n")
        settings = load_settings(path)
        assert settings.model.include_v0_sq is False
        assert settings.model.include_dv_regime_x_v0 is False


class TestLearningSettings:
    def test_missing_section_uses_defaults(self) -> None:
        path = _write_toml('[serial]\naccel_port = "/dev/ttyUSB0"\n')
        settings = load_settings(path)
        assert settings.learning == LearningSettings()
        assert settings.learning.refine_runs_stage1 == 2
        assert settings.learning.verify_runs == 0
        assert settings.learning.plan_learn_runs_max == 4
        assert settings.learning.refine_final_runs == 0

    def test_custom_values_parsed(self) -> None:
        path = _write_toml(
            "[learning]\n"
            "refine_runs_stage1 = 2\n"
            "plan_learn_runs_max = 4\n"
            "learning_timeout_s = 120.0\n"
        )
        settings = load_settings(path)
        assert settings.learning.refine_runs_stage1 == 2
        assert settings.learning.plan_learn_runs_max == 4
        assert settings.learning.learning_timeout_s == 120.0

    def test_removed_fields_are_ignored(self) -> None:
        """廃止フィールド（refine_runs_stage2 / tuning_on_target_mode / verify_runs_max /
        verify_min_runs）が残る既存 config でも起動を止めず、既知フィールドのみ取り込む。"""
        path = _write_toml(
            "[learning]\n"
            "refine_runs_stage1 = 4\n"
            "refine_runs_stage2 = 9\n"
            "tuning_on_target_mode = true\n"
            "verify_runs_max = 5\n"
            "verify_min_runs = 2\n"
        )
        settings = load_settings(path)
        assert settings.learning.refine_runs_stage1 == 4
        assert settings.learning.verify_runs == 0  # 新フィールドは既定値のまま


class TestActuatorSettings:
    """[actuator.accel] / [actuator.brake]（RCP6-ROD の機体仕様）。"""

    def test_missing_section_uses_defaults(self) -> None:
        """型式未記入の現場でも起動できること（リード長 0 = 未記入）。"""
        path = _write_toml('[serial]\naccel_port = "/dev/ttyUSB0"\n')
        settings = load_settings(path)
        assert settings.actuator.accel == ActuatorAxisSettings()
        assert settings.actuator.accel.min_speed_mm_s is None

    def test_parses_both_axes(self) -> None:
        path = _write_toml(
            "[actuator.accel]\n"
            'model = "RCP6-RA6R-WA-42P-6-100-P3-M-MT"\n'
            "lead_mm = 6.0\n"
            "stroke_mm = 100.0\n"
            "max_speed_mm_s = 250.0\n"
            "max_accel_g = 0.3\n"
            "[actuator.brake]\n"
            'model = "RCP6-RA7R-WA-56P-8-100-P3-M-ML"\n'
            "lead_mm = 8.0\n"
        )
        settings = load_settings(path)
        assert settings.actuator.accel.model.startswith("RCP6-RA6R")
        assert settings.actuator.accel.lead_mm == 6.0
        assert settings.actuator.accel.max_speed_mm_s == 250.0
        assert settings.actuator.brake.lead_mm == 8.0

    def test_min_speed_derived_from_lead(self) -> None:
        """RCP6-ROD 1.2.1「最低速度 = リード長 ÷ 0.8」。実機は accel 7.5 / brake 10.0。"""
        assert ActuatorAxisSettings(lead_mm=6.0).min_speed_mm_s == pytest.approx(7.5)
        assert ActuatorAxisSettings(lead_mm=8.0).min_speed_mm_s == pytest.approx(10.0)

    def test_unknown_keys_are_ignored(self) -> None:
        """将来フィールドを増やした config でも古いコードが起動できること。"""
        path = _write_toml("[actuator.accel]\nlead_mm = 6.0\nfuture_key = 1\n")
        settings = load_settings(path)
        assert settings.actuator.accel.lead_mm == 6.0

    def test_real_config_matches_hardware_doc(self) -> None:
        """リポジトリの config/settings.toml.example が docs/hardware.md と一致すること。"""
        settings = load_settings(Path("config/settings.toml.example"))
        assert settings.actuator.accel.lead_mm == 6.0
        assert settings.actuator.brake.lead_mm == 8.0
        assert settings.actuator.accel.min_speed_mm_s == pytest.approx(7.5)
        assert settings.actuator.brake.min_speed_mm_s == pytest.approx(10.0)
