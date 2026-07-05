"""settings.load_settings のユニットテスト（[model]・[learning] セクション中心）。"""

import tempfile
from pathlib import Path

from src.infra.settings import LearningSettings, ModelSettings, load_settings


def _write_toml(content: str) -> Path:
    tmp = tempfile.NamedTemporaryFile(  # noqa: SIM115
        mode="w", suffix=".toml", delete=False, encoding="utf-8"
    )
    tmp.write(content)
    tmp.close()
    return Path(tmp.name)


class TestModelSettingsDefaults:
    def test_missing_model_section_uses_defaults(self) -> None:
        path = _write_toml("[serial]\naccel_port = \"/dev/ttyUSB0\"\n")
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
        path = _write_toml(
            "[model]\ninclude_v0_sq = false\ninclude_dv_regime_x_v0 = false\n"
        )
        settings = load_settings(path)
        assert settings.model.include_v0_sq is False
        assert settings.model.include_dv_regime_x_v0 is False


class TestLearningSettings:
    def test_missing_section_uses_defaults(self) -> None:
        path = _write_toml("[serial]\naccel_port = \"/dev/ttyUSB0\"\n")
        settings = load_settings(path)
        assert settings.learning == LearningSettings()
        assert settings.learning.refine_runs_stage1 == 14
        assert settings.learning.refine_runs_stage2 == 5

    def test_custom_values_parsed(self) -> None:
        path = _write_toml(
            "[learning]\n"
            "refine_runs_stage1 = 2\n"
            "refine_runs_stage2 = 1\n"
            "learning_timeout_s = 120.0\n"
        )
        settings = load_settings(path)
        assert settings.learning.refine_runs_stage1 == 2
        assert settings.learning.refine_runs_stage2 == 1
        assert settings.learning.learning_timeout_s == 120.0
