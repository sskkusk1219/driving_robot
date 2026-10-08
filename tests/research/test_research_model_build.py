"""段2k 段D: `build_ff_model` が candidate ごとに正しい種類のモデル（pkl）を作ることのテスト。

小さな合成 CSV（`test_research_pattern_drive._synthetic_samples`）だけを使う。C8 は学習を
ごく小さく（max_iter 3・乱数 1 本・workers 1）して、種類（表／モデル）が合うことだけを見る。
"""

from __future__ import annotations

from dataclasses import replace
from datetime import timedelta
from pathlib import Path

import pytest

from tests.research import config as cfgmod
from tests.research import drive_log as dlmod
from tests.research import fwd_tool
from tests.research import hardware as hwmod
from tests.research import pattern_drive as pdmod
from tests.research.fwd_cv import SMOKE_MAX_ITER
from tests.research.fwd_map import KIND_MODEL, KIND_TABLE, load_forward_map
from tests.research.test_research_pattern_drive import PEDAL, _synthetic_samples, _tmp_cfg

REPEAT = 6
SMALL_C8 = {"forward_workers": 1, "forward_seeds": (0,), "forward_max_iter": SMOKE_MAX_ITER}


def _cfg_with(tmp_path: Path, candidate: str) -> cfgmod.ResearchConfig:
    cfg = _tmp_cfg(tmp_path)
    cfg.save({"feedforward.candidate": candidate})
    return cfgmod.load_config(cfg.source_path)


def _csv(tmp_path: Path, cfg: cfgmod.ResearchConfig, *, ramp_groups: bool = False) -> Path:
    base = _synthetic_samples(cfg)
    # 表は 1 マスに 1 秒以上の実測が要るので、同じ走りを繰り返して秒数を稼ぐ
    span = base[-1].elapsed_s + 0.1
    samples = [
        replace(
            s,
            elapsed_s=s.elapsed_s + k * span,
            timestamp=s.timestamp + timedelta(seconds=k * span),
        )
        for k in range(REPEAT)
        for s in base
    ]
    if ramp_groups:  # G ランプ走行と同じ「群の種類が少ない」pattern 列（段C 以降の CSV の形）
        names = ["1:COAST_DOWN", "2:G_RAMP", "3:G_RAMP", "4:CREEP_BRAKE_HOLD"]
        n = len(samples)
        samples = [
            replace(s, pattern=names[min(i * len(names) // n, len(names) - 1)])
            for i, s in enumerate(samples)
        ]
    path = tmp_path / "drive.csv"
    dlmod.write_csv(samples, path)
    return path


@pytest.mark.parametrize("candidate", ["C1", "C2", "C3", "C4", "C5"])
def test_polynomial_candidates_are_rejected_before_reading_the_csv(
    tmp_path: Path, candidate: str
) -> None:
    """段2l-6: C1〜C5（多項式）は手順2 の対象外。モデルを作らず ConfigError。"""
    cfg = _cfg_with(tmp_path, candidate)
    before = cfg.source_path.read_text(encoding="utf-8")
    with pytest.raises(cfgmod.ConfigError, match="手順 2 の対象外"):
        pdmod.build_ff_model(cfg, _csv(tmp_path, cfg), hw_mode=hwmod.HW_REAL, pedal=PEDAL)
    assert cfg.source_path.read_text(encoding="utf-8") == before  # yaml は書き換えない


def test_c7_makes_table_pkl(tmp_path: Path) -> None:
    cfg = _cfg_with(tmp_path, "C7")
    before = cfg.source_path.read_text(encoding="utf-8")
    result = pdmod.build_ff_model(
        cfg, _csv(tmp_path, cfg), hw_mode=hwmod.HW_REAL, pedal=PEDAL, write_config=True
    )
    table = load_forward_map(Path(result.model_path))
    assert table.kind == KIND_TABLE
    assert result.metrics == {}
    saved = cfgmod.load_config(cfg.source_path)
    assert saved.feedforward.model_path == result.model_path
    assert saved.feedforward.candidate == "C7"
    assert before != cfg.source_path.read_text(encoding="utf-8")
    # 多項式専用の参照キーは書かない
    assert saved.feedforward.model_accel_horizons_s == cfg.feedforward.model_accel_horizons_s


def test_c7_uses_min_as_the_origin_of_u(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """C7 の u の原点は yaml の vehicle.min_*_opening_pct（2-0 の実測ではない。段2l-6）。"""
    cfg = _cfg_with(tmp_path, "C7")
    seen: list[tuple[float, float]] = []
    orig = fwd_tool.read_logs

    def spy(paths, st, **kw):
        seen.append((st.accel_deadband_pct, st.brake_deadband_pct))
        return orig(paths, st, **kw)

    monkeypatch.setattr(fwd_tool, "read_logs", spy)
    pdmod.build_ff_model(cfg, _csv(tmp_path, cfg), hw_mode=hwmod.HW_STUB, pedal=PEDAL)
    assert seen == [(cfg.vehicle.min_accel_opening_pct, cfg.vehicle.min_brake_opening_pct)]


def test_c8_makes_model_pkl_and_stub_names_differ(tmp_path: Path) -> None:
    cfg = _cfg_with(tmp_path, "C8")
    before = cfg.source_path.read_text(encoding="utf-8")
    result = pdmod.build_ff_model(
        cfg, _csv(tmp_path, cfg), hw_mode=hwmod.HW_STUB, pedal=PEDAL, **SMALL_C8
    )
    table = load_forward_map(Path(result.model_path))
    assert table.kind == KIND_MODEL
    assert "_stub" in Path(result.model_path).name
    assert cfg.source_path.read_text(encoding="utf-8") == before  # スタブは config を触らない
    # 先に作った C7 の表（格子と設定の元）も同じ場所に残る
    tables = list(Path(result.model_path).parent.glob("fwd_table_*_stub.pkl"))
    assert tables and load_forward_map(tables[0]).kind == KIND_TABLE


def test_c8_real_writes_model_path_and_keeps_candidate(tmp_path: Path) -> None:
    cfg = _cfg_with(tmp_path, "C8")
    result = pdmod.build_ff_model(
        cfg, _csv(tmp_path, cfg), hw_mode=hwmod.HW_REAL, pedal=PEDAL, **SMALL_C8
    )
    saved = cfgmod.load_config(cfg.source_path)
    assert saved.feedforward.model_path == result.model_path
    assert saved.feedforward.candidate == "C8"
    assert "_stub" not in Path(result.model_path).name
    assert load_forward_map(Path(result.model_path)).kind == KIND_MODEL
