"""tests.research.relearn（段2.5。既存 CSV からのオフライン再学習）のユニットテスト。

実機不要で `pattern_drive.build_ff_model` を呼ぶだけの薄いエントリポイントなので、ここでは
`--dry-run` が設定ファイルを書き換えないこと・指定しなければ書き換わることだけを確かめる。
CSV とスタブ車両のログは test_research_pattern_drive.py のヘルパーを再利用する（車両物理は
そちらの `_synthetic_samples` の流儀に合わせ、config_testVehicle.yaml の値を決め打ちしない）。
"""

from __future__ import annotations

from pathlib import Path

import pytest

from tests.research import drive_log as dlmod
from tests.research import relearn as relearnmod
from tests.research.test_research_pattern_drive import _synthetic_samples, _tmp_cfg


def test_relearn_dry_run_does_not_touch_config(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cfg = _tmp_cfg(tmp_path)
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg), csv_path)
    before = cfg.source_path.read_text(encoding="utf-8")
    monkeypatch.setattr(relearnmod, "DEFAULT_CONFIG_PATH", cfg.source_path)

    exit_code = relearnmod.main([str(csv_path), "--dry-run"])

    assert exit_code == 0
    assert cfg.source_path.read_text(encoding="utf-8") == before


def test_relearn_without_dry_run_saves_config(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cfg = _tmp_cfg(tmp_path)
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg), csv_path)
    before = cfg.source_path.read_text(encoding="utf-8")
    monkeypatch.setattr(relearnmod, "DEFAULT_CONFIG_PATH", cfg.source_path)

    exit_code = relearnmod.main([str(csv_path)])

    assert exit_code == 0
    assert cfg.source_path.read_text(encoding="utf-8") != before


def test_relearn_does_not_overwrite_measured_keys(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """relearn は pedal=None で呼ぶため、不感帯・停車保持開度（2-0 の実測）は書き換わらない。"""
    cfg = _tmp_cfg(tmp_path)
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg), csv_path)
    monkeypatch.setattr(relearnmod, "DEFAULT_CONFIG_PATH", cfg.source_path)

    before_ff = cfg.feedforward
    relearnmod.main([str(csv_path)])

    from tests.research import config as cfgmod

    after_cfg = cfgmod.load_config(cfg.source_path)
    assert after_cfg.feedforward.accel_deadband_pct == before_ff.accel_deadband_pct
    assert after_cfg.feedforward.brake_deadband_pct == before_ff.brake_deadband_pct
    assert after_cfg.feedforward.stop_brake_opening_pct == before_ff.stop_brake_opening_pct


def test_relearn_raises_exit_code_6_on_too_few_samples(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cfg = _tmp_cfg(tmp_path)
    csv_path = tmp_path / "drive.csv"
    dlmod.write_csv(_synthetic_samples(cfg)[:10], csv_path)
    monkeypatch.setattr(relearnmod, "DEFAULT_CONFIG_PATH", cfg.source_path)

    assert relearnmod.main([str(csv_path), "--dry-run"]) == 6
