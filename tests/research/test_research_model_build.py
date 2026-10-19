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


_REAL_CHECKS = pdmod._run_step2_checks  # fixture が差し替える前の本物


@pytest.fixture(autouse=True)
def _no_step2_checks(monkeypatch: pytest.MonkeyPatch) -> None:
    """ここで見るのはモデル（pkl）の作り方。合成データは学習結果のチェックに通らないことがあるので、
    チェックは空にする（チェックそのものは test_research_step2_check.py・
    test_research_pattern_drive.py で見る）。"""
    monkeypatch.setattr(pdmod, "_run_step2_checks", lambda **kwargs: [])


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
    # 2026-10-20 段4-3: 発進側・停止側の表の開度の元は設定（既定 actual ＝ 0ee7c1b より前と同じ）。
    # 停止側の戻しの層は既定で作らない（stop_release_* は None）。アクセルの戻し・ブレーキの戻しの
    # 7.5〜20 km/h の秒数は設定に関係なく meta にある
    assert table.meta["side_table_opening_source"] == "actual"
    assert table.meta["opening_source"] == "cmd"  # 本体（ニューラルネット）は指令のまま
    assert table.meta["stop_release_layer_enabled"] is False
    assert table.stop_release_grid is None and table.stop_release_accel is None
    assert "stop_release_layer_s" not in table.meta
    assert table.meta["accel_release_rows_s"] >= 0.0 and "accel_release_max_pct" in table.meta
    assert table.meta["brake_release_band_s"] >= 0.0
    assert table.meta["brake_release_band_hi_kmh"] == 20.0
    assert "_stub" in Path(result.model_path).name
    assert cfg.source_path.read_text(encoding="utf-8") == before  # スタブは config を触らない
    # 先に作った C7 の表（格子と設定の元）も同じ場所に残る
    tables = list(Path(result.model_path).parent.glob("fwd_table_*_stub.pkl"))
    assert tables and load_forward_map(tables[0]).kind == KIND_TABLE


def test_c8_side_table_settings_switch_opening_source_and_stop_release_layer(
    tmp_path: Path,
) -> None:
    """2026-10-20 段4-3: learning.side_table_opening_source=cmd・stop_release_layer_enabled=true で
    指令の開度の表と停止側の戻しの層ができる。既定（actual・false）の発進・停止側の本体の表は、
    層を除けば同じ作り方（開度の元 cmd と actual で表の値が変わりうるだけ）。"""
    cfg = _cfg_with(tmp_path, "C8")
    cfg.save({
        "learning.side_table_opening_source": "cmd",
        "learning.stop_release_layer_enabled": True,
    })
    cfg = cfgmod.load_config(cfg.source_path)
    assert cfg.learning.side_table_opening_source == "cmd"
    assert cfg.learning.stop_release_layer_enabled is True
    result = pdmod.build_ff_model(
        cfg, _csv(tmp_path, cfg), hw_mode=hwmod.HW_STUB, pedal=PEDAL, **SMALL_C8
    )
    table = load_forward_map(Path(result.model_path))
    assert table.meta["side_table_opening_source"] == "cmd"
    assert table.meta["stop_release_layer_enabled"] is True
    assert len(table.meta["stop_release_layer_s"]) == 5


def test_c8_stub_runs_every_check_and_shows_c8_seed_mae(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """本物のチェックをスタブで通す（NG があっても止まらない）。C1 は SKIP。"""
    monkeypatch.setattr(pdmod, "_run_step2_checks", _REAL_CHECKS)
    cfg = _cfg_with(tmp_path, "C8")
    result = pdmod.build_ff_model(
        cfg, _csv(tmp_path, cfg), hw_mode=hwmod.HW_STUB, pedal=PEDAL, **SMALL_C8
    )
    assert [c.id for c in result.checks] == [
        "B1", "B3", "B4", "B5", "B6", "C1", "C2", "D1", "I1",
    ]
    by_id = {c.id: c for c in result.checks}
    assert by_id["C1"].status == "SKIP" and by_id["D1"].status == "SKIP"  # モード無し
    assert by_id["B1"].metrics["c8_mae"] > 0.0 and "ratio_mae" in by_id["B1"].metrics
    assert "seed_mae" in load_forward_map(Path(result.model_path)).meta


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


# ── 2026-10-13 段2o-2: C8 は relu、遅れを表に入れる ────────────────────────────


def test_c8_builds_relu_two_layers_of_50_with_stop_approach_dropped(tmp_path: Path) -> None:
    """手順2 の C8 は relu（2 層 50 個・停車まぎわ除外あり・追加学習なし）。fwd_final の CLI の
    既定（1-1 絶対誤差）は変わらない。"""
    from tests.research import fwd_final

    cfg = _cfg_with(tmp_path, "C8")
    result = pdmod.build_ff_model(
        cfg, _csv(tmp_path, cfg), hw_mode=hwmod.HW_STUB, pedal=PEDAL, **SMALL_C8
    )
    meta = load_forward_map(Path(result.model_path)).meta
    assert tuple(meta["hidden"]) == (50, 50)
    assert meta["activation"] == "relu"
    assert meta["drop_stop_approach"] is True
    assert meta["rollout_s"] == 0
    default = fwd_final.final_config()  # CLI の既定は 1-1（変えていない）
    assert (default.name, default.hidden, default.activation) == (
        "1-1 絶対誤差", (30, 30), "logistic"
    )


def test_c7_without_hold_step_keeps_default_lags(tmp_path: Path) -> None:
    """HOLD_STEP の無い CSV（旧形式・スタブ）は遅れを測れず、表の遅れは既定値のまま。"""
    cfg = _cfg_with(tmp_path, "C7")
    result = pdmod.build_ff_model(cfg, _csv(tmp_path, cfg), hw_mode=hwmod.HW_STUB, pedal=PEDAL)
    meta = load_forward_map(Path(result.model_path)).meta
    assert meta["accel_lag_s"] == pytest.approx(fwd_tool.DEFAULT_ACCEL_LAG_S)
    assert meta["brake_lag_s"] == pytest.approx(fwd_tool.DEFAULT_BRAKE_LAG_S)
    lags = pdmod.measure_pedal_lags(_csv(tmp_path, cfg))
    assert lags.accel_s is None and lags.brake_s is None


@pytest.mark.parametrize("candidate", ["C7", "C8"])
def test_measured_lags_go_into_the_table_meta(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, candidate: str
) -> None:
    """測れたペダルの遅れが accel_lag_s・brake_lag_s に入り、惰行の遅れはアクセルと同じ値。
    片方だけ測れたときは、測れなかったほうは既定値。"""
    cfg = _cfg_with(tmp_path, candidate)
    monkeypatch.setattr(
        pdmod, "measure_pedal_lags",
        lambda _p, **_kw: pdmod.MeasuredLags(accel_s=0.21, brake_s=None),
    )
    kw = SMALL_C8 if candidate == "C8" else {}
    result = pdmod.build_ff_model(
        cfg, _csv(tmp_path, cfg), hw_mode=hwmod.HW_STUB, pedal=PEDAL, **kw
    )
    meta = load_forward_map(Path(result.model_path)).meta
    assert meta["accel_lag_s"] == pytest.approx(0.21)
    assert meta["coast_lag_s"] == pytest.approx(0.21)
    assert meta["brake_lag_s"] == pytest.approx(fwd_tool.DEFAULT_BRAKE_LAG_S)


def test_measure_pedal_lags_uses_per_pedal_medians(tmp_path: Path) -> None:
    """HOLD_STEP の 5 ステップ × 2 車速の合成 CSV から、ペダル別の遅れ 0.2 秒が測れる。"""
    import numpy as np

    from tests.research.test_research_pedal_lag import _synth_lag_steps

    rows = _synth_lag_steps((30.0, 60.0), lag_s=0.20)
    path = tmp_path / "lag.csv"
    with path.open("w", newline="", encoding="utf-8") as f:
        f.write("section,elapsed_s,actual_speed_kmh,accel_cmd_pct,brake_cmd_pct,phase\n")
        for t, v, a, b, ph in zip(
            rows.t_s, rows.speed_kmh, rows.accel_cmd_pct, rows.brake_cmd_pct, rows.phase,
            strict=True,
        ):
            f.write(f"{dlmod.SECTION_PATTERN_DRIVE},{t:.3f},{v:.3f},{a:.3f},{b:.3f},{ph}\n")
    lags = pdmod.measure_pedal_lags(path)
    assert lags.accel_s is not None and lags.brake_s is not None
    assert np.isclose(lags.accel_s, 0.20, atol=0.05) and np.isclose(lags.brake_s, 0.20, atol=0.05)
    # 件数が足りないペダルは採らない（アクセル 1 件・ブレーキ 0 件）
    assert pdmod._usable_lag((1, 0.2), 2) is None and pdmod._usable_lag((2, 3.0), 2) is None


@pytest.mark.parametrize("candidate", ["C7", "C8"])
def test_onsets_go_into_the_table_meta_as_amounts_above_min(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, candidate: str
) -> None:
    """2026-10-16 段2o-6: 効き始めは min からの量 u で meta に入る。求まらないペダルはキーを
    入れない。惰行の関数は表の coast_accel の符号を反転した正の減速量。"""
    from tests.research import play_onset

    cfg = _cfg_with(tmp_path, candidate)
    seen: dict[str, float] = {}

    def fake(path: Path, *, accel_min: float, brake_min: float, coast_fn, **_kw):  # type: ignore[no-untyped-def]
        seen["accel_min"], seen["brake_min"] = accel_min, brake_min
        seen["coast_30"] = float(coast_fn(30.0))
        return (
            play_onset.OnsetResult(accel_min + 5.0, 0.4, (), ""),
            play_onset.OnsetResult(None, None, (), "まとまりが足りない"),
        )

    monkeypatch.setattr(play_onset, "onsets_from_csv", fake)
    kw = SMALL_C8 if candidate == "C8" else {}
    result = pdmod.build_ff_model(
        cfg, _csv(tmp_path, cfg, ramp_groups=True), hw_mode=hwmod.HW_STUB, pedal=PEDAL, **kw
    )
    table = load_forward_map(Path(result.model_path))
    assert table.meta["accel_onset_u_pct"] == pytest.approx(5.0)
    assert "brake_onset_u_pct" not in table.meta
    assert seen["accel_min"] == pytest.approx(cfg.vehicle.min_accel_opening_pct)
    assert seen["coast_30"] == pytest.approx(-float(table.coast_accel(30.0)))


def test_table_without_onset_keys_still_loads_and_inverts(tmp_path: Path) -> None:
    """古い pkl（効き始めのキーなし）も読めて、meta にキーが無いだけ（必須ではない）。"""
    cfg = _cfg_with(tmp_path, "C7")
    result = pdmod.build_ff_model(cfg, _csv(tmp_path, cfg), hw_mode=hwmod.HW_STUB, pedal=PEDAL)
    table = load_forward_map(Path(result.model_path))
    table.meta.pop("accel_onset_u_pct", None)
    table.meta.pop("brake_onset_u_pct", None)
    path = table.save(tmp_path / "old.pkl")
    old = load_forward_map(path)
    assert "accel_onset_u_pct" not in old.meta and "brake_onset_u_pct" not in old.meta
    assert old.meta.get("accel_onset_u_pct") is None


@pytest.mark.parametrize("source", ["actual", "cmd"])
def test_side_tables_are_read_with_the_configured_opening_source(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, source: str
) -> None:
    """2026-10-20 段4-3: 発進側・停止側の行（keep_stopped=True で読む行）は、
    learning.side_table_opening_source の開度（既定 actual。cmd なら指令）で読む。動き出す開度も
    同じ元で測る。"""
    cfg = _cfg_with(tmp_path, "C8")
    cfg.save({"learning.side_table_opening_source": source})
    cfg = cfgmod.load_config(cfg.source_path)
    seen: list[str] = []
    orig = fwd_tool.read_logs

    def spy(paths, st, **kw):
        if kw.get("keep_stopped"):
            seen.append(str(kw.get("opening_source")))
        return orig(paths, st, **kw)

    sources: list[str] = []
    import tests.research.fwd_data as fdmod  # noqa: PLC0415

    orig_bk = fdmod.launch_breakaway

    def spy_bk(path, **kw):
        sources.append(str(kw.get("opening_source")))
        return orig_bk(path, **kw)

    monkeypatch.setattr(fwd_tool, "read_logs", spy)
    monkeypatch.setattr(fdmod, "launch_breakaway", spy_bk)
    pdmod.build_ff_model(
        cfg, _csv(tmp_path, cfg), hw_mode=hwmod.HW_STUB, pedal=PEDAL, **SMALL_C8
    )
    assert seen and set(seen) == {source}
    assert sources == [source]
