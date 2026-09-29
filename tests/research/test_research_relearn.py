"""tests.research.relearn（段2.5。既存 CSV からのオフライン再学習）のユニットテスト。

実機不要で `pattern_drive.build_ff_model` を呼ぶだけの薄いエントリポイントなので、ここでは
`--dry-run` が設定ファイルを書き換えないこと・指定しなければ書き換わることだけを確かめる。
CSV とスタブ車両のログは test_research_pattern_drive.py のヘルパーを再利用する（車両物理は
そちらの `_synthetic_samples` の流儀に合わせ、config_testVehicle.yaml の値を決め打ちしない）。
"""

from __future__ import annotations

from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import pytest

from tests.research import drive_log as dlmod
from tests.research import relearn as relearnmod
from tests.research.research_types import DrivingMode, SpeedPoint
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


# ─────────────────────────────────────────────────────────────────────
# 段2（ProblemReport_20260925）: --ref-csv / --weight の引数解析
# `build_ff_model` はモックして呼び出し引数だけを確かめる（実機・DB には触らない）
# ─────────────────────────────────────────────────────────────────────


def _ref_csv(tmp_path: Path) -> Path:
    path = tmp_path / "ref.csv"
    path.write_text(
        "mode_time_s,ref_speed_kmh\n0.0,0.0\n1.0,10.0\n2.0,20.0\n", encoding="utf-8"
    )
    return path


def _capture_build_ff_model(monkeypatch: pytest.MonkeyPatch) -> dict[str, Any]:
    """`build_ff_model` を差し替え、呼び出し引数を記録するだけのモックにする。"""
    calls: dict[str, Any] = {}

    def fake_build_ff_model(cfg: Any, csv_path: Any, **kwargs: Any) -> Any:  # noqa: ANN401
        calls["cfg"] = cfg
        calls["csv_path"] = csv_path
        calls.update(kwargs)
        return None

    monkeypatch.setattr(relearnmod, "build_ff_model", fake_build_ff_model)
    return calls


def test_relearn_weight_config_default_does_not_read_wltp(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """既定（--weight config、--ref-csv 省略）かつ config が無効なら WLTP は読まない（DB 不要）。"""
    cfg = _tmp_cfg(tmp_path)
    csv_path = tmp_path / "drive.csv"
    csv_path.write_text("dummy\n", encoding="utf-8")
    monkeypatch.setattr(relearnmod, "DEFAULT_CONFIG_PATH", cfg.source_path)
    calls = _capture_build_ff_model(monkeypatch)

    def fail_load_mode(*_args: Any, **_kwargs: Any) -> Any:  # noqa: ANN401
        raise AssertionError("config が無効なのに DB の load_mode を呼んではいけない")

    monkeypatch.setattr(relearnmod, "load_mode", fail_load_mode)

    assert relearnmod.main([str(csv_path), "--dry-run"]) == 0
    assert calls["wltp_mode"] is None
    assert calls["sample_weight_enabled"] is None


def test_relearn_weight_on_without_ref_csv_reads_db_mode(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """--weight on かつ --ref-csv 省略なら DB の load_mode を読む（ここではモックで差し替え）。"""
    cfg = _tmp_cfg(tmp_path)
    csv_path = tmp_path / "drive.csv"
    csv_path.write_text("dummy\n", encoding="utf-8")
    monkeypatch.setattr(relearnmod, "DEFAULT_CONFIG_PATH", cfg.source_path)
    calls = _capture_build_ff_model(monkeypatch)

    db_mode = DrivingMode(
        id="db", name="wltp", description="", total_duration=2.0, max_speed=20.0,
        created_at=datetime.now(tz=UTC), is_system=False,
        reference_speed=[
            SpeedPoint(time_s=0.0, speed_kmh=0.0), SpeedPoint(time_s=2.0, speed_kmh=20.0),
        ],
    )

    async def fake_load_mode(_cfg: Any, _name: str) -> DrivingMode:
        return db_mode

    monkeypatch.setattr(relearnmod, "load_mode", fake_load_mode)

    assert relearnmod.main([str(csv_path), "--dry-run", "--weight", "on"]) == 0
    assert calls["wltp_mode"] is db_mode
    assert calls["sample_weight_enabled"] is True


def test_relearn_weight_off_with_ref_csv_still_reads_wltp_for_comparison(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """--weight off でも --ref-csv があれば WLTP を読む（mae_wltp で比較できるように）。"""
    cfg = _tmp_cfg(tmp_path)
    csv_path = tmp_path / "drive.csv"
    csv_path.write_text("dummy\n", encoding="utf-8")
    ref_csv = _ref_csv(tmp_path)
    monkeypatch.setattr(relearnmod, "DEFAULT_CONFIG_PATH", cfg.source_path)
    calls = _capture_build_ff_model(monkeypatch)

    def fail_load_mode(*_args: Any, **_kwargs: Any) -> Any:  # noqa: ANN401
        raise AssertionError("--ref-csv 指定時は DB の load_mode を呼んではいけない")

    monkeypatch.setattr(relearnmod, "load_mode", fail_load_mode)

    exit_code = relearnmod.main(
        [str(csv_path), "--dry-run", "--ref-csv", str(ref_csv), "--weight", "off"]
    )
    assert exit_code == 0
    assert calls["sample_weight_enabled"] is False
    assert calls["wltp_mode"] is not None
    assert [(p.time_s, p.speed_kmh) for p in calls["wltp_mode"].reference_speed] == [
        (0.0, 0.0), (1.0, 10.0), (2.0, 20.0),
    ]
