"""研究開発用ハーネス 手順 3（FF のみでモード走行 → CSV/レポート）のユニットテスト。

スタブ HW（StubVehicle 付き）で数秒の合成モードを走らせ、ループ・安全停止・CSV・レポートを確認する。
FF は手順 2 のモデルに依存しないよう、同じインターフェースの簡単な偽物を使う。
KPI の数え方は本番 KPIMonitor と突き合わせる。
"""

from __future__ import annotations

import csv
import random
from datetime import UTC, datetime, timedelta
from pathlib import Path

import numpy as np
import pytest

from tests.research import config as cfgmod
from tests.research import drive_log as dlmod
from tests.research import ff_candidate
from tests.research import hardware as hwmod
from tests.research import kpi as kpimod
from tests.research import main as mainmod
from tests.research import mode_drive as mdmod
from tests.research import mode_report as mrmod
from tests.research.axis_safety import ALARM_CHECK_INTERVAL_S
from tests.research.dob import PedalDisturbanceObserver
from tests.research.launch_hold import LaunchMoveHold, launch_hold_gain
from tests.research.research_types import (
    G_TO_KMHS,
    DriveLogData,
    DrivingMode,
    FeedforwardParams,
    SpeedPoint,
)
from tests.research.vehicle import feedforward_params, opening_to_pulse, pulse_to_opening

HOLD_PCT = 22.0


def _tmp_cfg(tmp_path: Path) -> cfgmod.ResearchConfig:
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    min_brake = cfg.vehicle.min_brake_opening_pct  # 今の yaml の値（変わっても追従する）
    cfg.save(
        {
            "output.results_dir": str(tmp_path / "results"),
            "output.plot": False,
            # 絶対 22.0%（min ＋ 量）。段2l-7
            "feedforward.stop_brake_offset_pct": HOLD_PCT - min_brake,
            # 走行後の緩減速を短くする（判定ロジックは既定と同じ）
            "decel_stop.step_mm": 1.0,
            "decel_stop.dwell_s": 0.2,
            "decel_stop.slope_window_s": 0.2,
        }
    )
    return cfgmod.load_config(path)


def _mode(points: list[tuple[float, float]]) -> DrivingMode:
    return DrivingMode(
        id="test",
        name="test_mode",
        description="",
        reference_speed=[SpeedPoint(t, v) for t, v in points],
        total_duration=points[-1][0],
        max_speed=max(v for _, v in points),
        created_at=datetime(2026, 9, 11, tzinfo=UTC),
    )


class FakeFF:
    """FeedforwardController と同じ呼び方の偽物: 停車は保持ブレーキ、それ以外は向きで ±。"""

    horizons = (0.5, 1.0, 2.0, 3.0)
    past_horizons = (0.5, 1.0)
    candidate = "C1"
    uses_actual_speed = False

    def __init__(self) -> None:
        self.calls = 0

    def predict_effort(
        self, v0: float, future: list[float], past: list[float], **_select: float
    ) -> float:
        self.calls += 1
        if v0 <= 0.02 and future[0] <= 0.02:
            return -HOLD_PCT
        return 30.0 if future[1] >= v0 else -12.0


async def _held_stub(cfg: cfgmod.ResearchConfig) -> hwmod.ResearchHardware:
    """初期化済みで、走行前チェックの終わりと同じく停車してブレーキを踏んでいるスタブ。"""
    hw = hwmod.build_hardware(cfg, hwmod.HW_STUB)
    await hwmod.run_initialize(hw)
    assert isinstance(hw.can, hwmod.StubCANReader)
    hw.can.speed_kmh = 0.0
    await hw.brake.move_to_position(opening_to_pulse(HOLD_PCT))
    return hw


# ── 部品 ──────────────────────────────────────────────────────────────


def test_reference_speed_interpolates_and_clamps() -> None:
    ref = mdmod.ReferenceSpeed(_mode([(0.0, 0.0), (1.0, 10.0), (3.0, 10.0), (4.0, 0.0)]))
    assert ref.at(-1.0) == 0.0
    assert ref.at(0.5) == pytest.approx(5.0)
    assert ref.at(2.0) == pytest.approx(10.0)
    assert ref.at(3.25) == pytest.approx(7.5)
    assert ref.at(99.0) == 0.0


def test_split_effort_is_exclusive_and_clamped() -> None:
    assert mdmod.split_effort(12.5, 80.0, 80.0) == (12.5, 0.0)
    assert mdmod.split_effort(-30.0, 80.0, 25.0) == (0.0, 25.0)
    assert mdmod.split_effort(95.0, 80.0, 80.0) == (80.0, 0.0)
    assert mdmod.split_effort(0.0, 80.0, 80.0) == (0.0, 0.0)


def test_governor_caps_brake_while_decel_exceeds_limit_and_releases() -> None:
    gov = mdmod.DecelGovernor(max_decel_g=0.4, reduce_step_pct=2.0)
    hard = gov.limit_kmhs * 1.5  # 上限を超える減速 [km/h/s]
    speed, t = 100.0, 0.0
    brake, active = gov.apply(t, speed, 20.0)
    assert (brake, active) == (20.0, False)
    for expected in (20.0, 18.0, 16.0):  # 超えた周期は前周期の値で頭打ち → 以降 2% ずつ下げる
        t += 0.05
        speed -= hard * 0.05
        brake, active = gov.apply(t, speed, 35.0)
        assert active
        assert brake == pytest.approx(expected)
    t += 0.45  # 減速が収まって（傾きの窓 0.4s を過ぎて）も頭打ちは保つ（FF が浅くするまで）
    brake, active = gov.apply(t, speed, 35.0)
    assert gov.decel_kmhs == 0.0
    assert (brake, active) == (pytest.approx(16.0), True)
    t += 0.05
    brake, active = gov.apply(t, speed, 10.0)  # FF 自身が頭打ちより浅くした → 解除
    assert (brake, active) == (10.0, False)


def test_governor_disabled_passes_through() -> None:
    gov = mdmod.DecelGovernor(max_decel_g=0.4, reduce_step_pct=2.0, enabled=False)
    for i in range(10):
        brake, active = gov.apply(i * 0.05, 100.0 - i * 3.0, 40.0)
        assert (brake, active) == (40.0, False)
    assert gov.decel_kmhs == pytest.approx(3.0 / 0.05)
    assert gov.limit_kmhs == pytest.approx(0.4 * G_TO_KMHS * 0.98)


def test_segment_at_uses_bounds() -> None:
    modes = cfgmod.ModesSection()
    assert [modes.segment_at(t) for t in (0.0, 588.9, 589.0, 1500.0, 1800.0)] == [
        "Low", "Low", "Mid", "ExHi", "ExHi"
    ]


def test_segment_config_is_validated() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.modes.segment_bounds_s = [589.0, 1022.0]  # 区間名 4 つに境界 2 つ
    assert any("segment_bounds_s" in p for p in cfgmod.validate_config(cfg))


def test_load_feedforward_requires_trained_model(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    cfg.feedforward.model_path = str(tmp_path / "none.pkl")
    with pytest.raises(cfgmod.ConfigError, match="手順 2"):
        mdmod.load_feedforward(cfg)


def test_load_feedforward_passes_coast_band_to_candidate(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """段2: `load_feedforward` が config の `coast_band_kmhs` を候補へ渡すこと。"""
    cfg = _tmp_cfg(tmp_path)
    cfg.feedforward.coast_band_kmhs = 0.5
    cfg.feedforward.model_path = str(tmp_path / "dummy.pkl")
    (tmp_path / "dummy.pkl").write_bytes(b"")  # is_model_trained は存在チェックのみ

    calls: dict[str, object] = {}

    class _FakeFF(ff_candidate.CandidateFeedforward):
        def set_research_params(self, research: object) -> None:
            calls["research"] = research
            super().set_research_params(research)  # type: ignore[arg-type]

        def load_model(self, model_path: str) -> None:
            calls["model_path"] = model_path  # 実ファイルは読まない（このテストの関心外）

    monkeypatch.setattr(mdmod, "make_candidate", lambda name: _FakeFF())  # noqa: ARG005

    ff = mdmod.load_feedforward(cfg)

    assert calls["model_path"] == cfg.feedforward.model_path
    assert calls["research"].coast_band_kmhs == pytest.approx(0.5)  # type: ignore[attr-defined]
    assert ff._research.coast_band_kmhs == pytest.approx(0.5)  # noqa: SLF001


def test_load_feedforward_rejects_reach_horizon_not_in_model(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """段3: `reach_horizons_s` にモデルの先読みホライズンに無い値があれば ConfigError。

    `_FakeFF.load_model` は実ファイルを読まないため `ff.horizons` は既定の
    `DEFAULT_FEATURE_SPEC.lookahead_horizons_s`（0.5, 1.0, 2.0, 3.0）のまま。5.0 はそこに
    無いので弾かれる。
    """
    cfg = _tmp_cfg(tmp_path)
    cfg.feedforward.reach_horizons_s = [0.5, 5.0]
    cfg.feedforward.model_path = str(tmp_path / "dummy.pkl")
    (tmp_path / "dummy.pkl").write_bytes(b"")

    class _FakeFF(ff_candidate.CandidateFeedforward):
        def load_model(self, model_path: str) -> None:
            pass  # 実ファイルは読まない（このテストの関心外）

    monkeypatch.setattr(mdmod, "make_candidate", lambda name: _FakeFF())  # noqa: ARG005

    with pytest.raises(cfgmod.ConfigError, match="reach_horizons_s"):
        mdmod.load_feedforward(cfg)


# ── 手順6: ペダル別ホライズン（ProblemReport_20260921） ─────────────


def _write_pedal_separated_pkl(path: Path) -> None:
    import pickle
    from dataclasses import asdict

    from tests.research.ff_model import MODEL_TYPE, FeatureSpec, make_estimator

    accel_spec = FeatureSpec(lookahead_horizons_s=(0.5, 1.0, 3.0))
    brake_spec = FeatureSpec(lookahead_horizons_s=(0.1, 1.0))
    control_spec = FeatureSpec(lookahead_horizons_s=(0.1, 0.5, 1.0, 3.0))
    accel_model, brake_model = make_estimator(), make_estimator()
    rng = np.random.default_rng(5)
    xa = rng.normal(size=(30, len(accel_spec.feature_names())))
    xb = rng.normal(size=(30, len(brake_spec.feature_names())))
    accel_model.fit(xa, xa[:, 0])
    brake_model.fit(xb, xb[:, 0])
    payload = {
        "model_type": MODEL_TYPE,
        "accel_model": accel_model,
        "brake_model": brake_model,
        "feature_spec": asdict(control_spec),
        "accel_feature_spec": asdict(accel_spec),
        "brake_feature_spec": asdict(brake_spec),
        "stop_horizon_s": 0.5,
        "speed_clip_max": 140.0,
    }
    with path.open("wb") as f:
        pickle.dump(payload, f)


async def test_prepare_mode_drive_shows_pedal_specific_horizons(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """ペダル別ホライズンの pkl を読んだとき、アクセル/ブレーキ別に先読みを表示すること。"""
    cfg = _tmp_cfg(tmp_path)
    cfg.feedforward.model_path = str(tmp_path / "separated.pkl")
    cfg.feedforward.candidate = "C1"  # 多項式 pkl を読む候補（yaml の C15 は不可）
    _write_pedal_separated_pkl(Path(cfg.feedforward.model_path))
    mode = _mode([(0.0, 0.0), (10.0, 20.0)])

    async def fake_load_mode(_cfg: cfgmod.ResearchConfig, _name: str) -> object:
        return mode

    monkeypatch.setattr(mdmod, "load_mode", fake_load_mode)

    setup = await mdmod.prepare_mode_drive(cfg, "dummy_mode")

    assert setup.ff.is_pedal_separated
    assert setup.ff.accel_spec.lookahead_horizons_s == (0.5, 1.0, 3.0)
    assert setup.ff.brake_spec.lookahead_horizons_s == (0.1, 1.0)
    out = capsys.readouterr().out
    assert "先読み アクセル" in out
    assert "先読み" in out and "ブレーキ" in out


async def test_prepare_mode_drive_warns_on_h0_h1_mismatch(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """pkl の h0/h1 が現在の設定と食い違うと警告を出す（走行は pkl のまま続ける）。"""
    cfg = _tmp_cfg(tmp_path)
    cfg.feedforward.model_path = str(tmp_path / "separated.pkl")
    cfg.feedforward.candidate = "C1"  # 多項式 pkl を読む候補（yaml の C15 は不可）
    _write_pedal_separated_pkl(Path(cfg.feedforward.model_path))
    cfg.features.h1_s = 2.0  # pkl の regime_horizon_s（1.0）と食い違わせる
    mode = _mode([(0.0, 0.0), (10.0, 20.0)])

    async def fake_load_mode(_cfg: cfgmod.ResearchConfig, _name: str) -> object:
        return mode

    monkeypatch.setattr(mdmod, "load_mode", fake_load_mode)

    await mdmod.prepare_mode_drive(cfg, "dummy_mode")

    out = capsys.readouterr().out
    assert "食い違っています" in out
    assert "h1_s" in out


# ── C7（順モデルの表） ────────────────────────────────────────────────


def _write_forward_table_pkl(
    path: Path, deadbands: tuple[float, float] = (4.0, 7.0), kind: str = "table"
) -> None:
    """小さな合成の順モデルの表（a = −2 − 0.01v + 0.3u（u≥0）/ 0.6u（u<0））を pkl にする。"""
    from tests.research.fwd_map import POINT_MEASURED, ForwardTable, save_table

    rep = np.array([0.0, 50.0, 100.0, 150.0])
    grid = np.arange(-30.0, 31.0, 1.0)
    accel = (-2.0 - 0.01 * rep[:, None]) + np.where(grid >= 0, 0.3 * grid, 0.6 * grid)[None, :]
    save_table(
        ForwardTable(
            speed_edges=np.array([-25.0, 25.0, 75.0, 125.0, 175.0]),
            speed_rep=rep,
            u_grid=grid,
            accel_grid=accel,
            seconds=np.full(accel.shape, 10.0),
            point_kind=np.full(accel.shape, POINT_MEASURED, dtype=int),
            meas_u_min=np.full(4, -30.0),
            meas_u_max=np.full(4, 30.0),
            kind=kind,
            meta={
                "accel_deadband_pct": deadbands[0],
                "brake_deadband_pct": deadbands[1],
                "accel_lag_s": 0.05,
                "brake_lag_s": 0.05,
            },
        ),
        path,
    )


def _c7_cfg(tmp_path: Path, deadbands: tuple[float, float] = (4.0, 7.0)) -> cfgmod.ResearchConfig:
    cfg = _tmp_cfg(tmp_path)
    cfg.feedforward.candidate = "C7"
    # 2026-10-08 段2l-6: u の原点は vehicle.min_*_opening_pct（表の meta と比べる）
    cfg.vehicle.min_accel_opening_pct = 4.0
    cfg.vehicle.min_brake_opening_pct = 7.0
    cfg.feedforward.stop_brake_offset_pct = HOLD_PCT - 7.0  # 絶対 HOLD_PCT（段2l-7）
    cfg.feedforward.model_path = str(tmp_path / "fwd_table.pkl")
    _write_forward_table_pkl(Path(cfg.feedforward.model_path), deadbands)
    return cfg


def test_load_feedforward_c7_reads_table_and_rejects_mismatch(tmp_path: Path) -> None:
    cfg = _c7_cfg(tmp_path)
    ff = mdmod.load_feedforward(cfg)
    assert isinstance(ff, ff_candidate.ForwardMapFeedforward) and ff.has_model
    # C7 に多項式モデルの pkl → ConfigError（走り出す前に止まる）
    poly = tmp_path / "poly.pkl"
    poly.write_bytes(b"\x80\x04N.")  # pickle(None): model_type を持たない
    cfg.feedforward.model_path = str(poly)
    with pytest.raises(cfgmod.ConfigError, match="順モデルの表"):
        mdmod.load_feedforward(cfg)
    # 逆向き: C1 に表の pkl → ConfigError
    cfg2 = _c7_cfg(tmp_path)
    cfg2.feedforward.candidate = "C1"
    with pytest.raises(cfgmod.ConfigError, match="model_type"):
        mdmod.load_feedforward(cfg2)


async def test_prepare_mode_drive_c7_builds_plan_and_shows_summary(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    cfg = _c7_cfg(tmp_path)
    mode = _mode([(0.0, 0.0), (2.0, 0.0), (8.0, 30.0), (12.0, 30.0), (16.0, 0.0), (18.0, 0.0)])

    async def fake_load_mode(_cfg: cfgmod.ResearchConfig, _name: str) -> object:
        return mode

    monkeypatch.setattr(mdmod, "load_mode", fake_load_mode)
    setup = await mdmod.prepare_mode_drive(cfg, "dummy_mode")

    assert isinstance(setup.ff, ff_candidate.ForwardMapFeedforward)
    assert setup.ff.plan.t_s[-1] == pytest.approx(mode.total_duration)
    out = capsys.readouterr().out
    for word in ("C7", "遅れ", "惰行の帯", "範囲外", "穴埋め", "アクセル", "停車保持"):
        assert word in out
    assert "ペダル切替" in out
    assert "⚠" not in out  # u の原点（min）は表と設定で一致
    assert "待機位置（min） アクセル 4.00% / ブレーキ 7.00%" in out


async def test_prepare_mode_drive_c7_warns_on_deadband_mismatch(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    cfg = _c7_cfg(tmp_path, deadbands=(4.0, 9.0))  # 表のブレーキの u の原点だけ設定とずらす
    mode = _mode([(0.0, 0.0), (10.0, 20.0)])

    async def fake_load_mode(_cfg: cfgmod.ResearchConfig, _name: str) -> object:
        return mode

    monkeypatch.setattr(mdmod, "load_mode", fake_load_mode)
    await mdmod.prepare_mode_drive(cfg, "dummy_mode")
    out = capsys.readouterr().out
    # 2026-10-08 段2l-7: 警告（⚠）ではなくお知らせ。表は u で持つのでそのまま使う
    assert "⚠" not in out and "ブレーキ pkl 9.00 / 設定 7.00" in out
    assert "u の原点（min）が表を作ったときと違います" in out
    assert "そのまま使います" in out and "手順2 をやり直してください" in out
    assert "アクセル pkl" not in out


async def test_c7_plan_shifts_exactly_with_min(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """2026-10-08 段2l-7: 同じ表・同じ offset で min だけ +3% の設定にすると、計画の開度
    （アクセル・ブレーキ・停車保持）がどれもちょうど +3%（絶対値側へ）ずれる。"""
    mode = _mode([(0.0, 0.0), (2.0, 0.0), (8.0, 30.0), (12.0, 30.0), (16.0, 0.0), (18.0, 0.0)])

    async def fake_load_mode(_cfg: cfgmod.ResearchConfig, _name: str) -> object:
        return mode

    monkeypatch.setattr(mdmod, "load_mode", fake_load_mode)
    base_cfg = _c7_cfg(tmp_path)
    shift_cfg = _c7_cfg(tmp_path)  # 同じ表（pkl の meta は min 4.0/7.0 のまま）
    shift_cfg.vehicle.min_accel_opening_pct += 3.0
    shift_cfg.vehicle.min_brake_opening_pct += 3.0
    shift_cfg.feedforward.stop_brake_offset_pct = base_cfg.feedforward.stop_brake_offset_pct
    base = (await mdmod.prepare_mode_drive(base_cfg, "dummy_mode")).ff.plan.effort_pct
    shifted = (await mdmod.prepare_mode_drive(shift_cfg, "dummy_mode")).ff.plan.effort_pct

    assert shift_cfg.stop_brake_opening_pct == pytest.approx(base_cfg.stop_brake_opening_pct + 3.0)
    accel, brake = base > 0, base < 0
    assert accel.any() and brake.any()
    assert np.allclose(shifted[accel], base[accel] + 3.0)
    assert np.allclose(shifted[brake], base[brake] - 3.0)  # 停車保持（負の定数）もここに含まれる
    assert np.array_equal(shifted == 0, base == 0)


async def test_run_mode_drive_c7_uses_plan_for_ff_output(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """走行ループが C7 のとき、ログの FF 出力（plan_effort）が plan.at(t) と一致する。"""
    cfg = _c7_cfg(tmp_path)
    cfg.mode_drive.pedal_standby = False
    mode = _mode([(0.0, 0.0), (0.6, 0.0), (2.0, 6.0), (3.0, 6.0), (3.6, 0.0), (4.0, 0.0)])

    async def fake_load_mode(_cfg: cfgmod.ResearchConfig, _name: str) -> object:
        return mode

    monkeypatch.setattr(mdmod, "load_mode", fake_load_mode)
    setup = await mdmod.prepare_mode_drive(cfg, "dummy_mode")
    hw = await _held_stub(cfg)
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")

    # モードより長く走っても（端で止まる）落ちない
    result = await mdmod.run_mode_drive(hw, cfg, setup, log=log)
    await log.close()

    assert result.completed and result.cycles >= 70
    plan = setup.ff.plan
    assert len(result.samples) > 50
    for sample in result.samples:
        assert sample.mode_time_s is not None
        assert sample.data.plan_effort_pct == pytest.approx(plan.at(sample.mode_time_s))
        assert sample.candidate == "C7"
    # 停車保持の区間（先頭）は保持ブレーキの開度
    assert result.samples[0].data.plan_effort_pct == pytest.approx(-HOLD_PCT)


# ── KPI ───────────────────────────────────────────────────────────────


# 本番 KPIMonitor（src/domain/control/kpi_monitor.py）に下の系列を通した結果。2026-09-25 に
# src を実行して採取した固定値（`tests/` だけで完結させるため src は呼ばない。
# ProblemReport_20260924）。
PROD_KPI_MAX_ABS_KMH = 3.1069116493567535
PROD_KPI_REVERSAL_MAX_PER_5S = 9
PROD_KPI_P95_KMH = 1.61  # 本番はビン幅 0.01 に丸めた値


def test_kpi_matches_production_monitor() -> None:
    """最大逸脱・符号反転は本番 KPIMonitor の値と一致、p95 はビン幅 0.01 以内。"""
    rng = random.Random(3)
    t, dev = [], []
    value = 0.0
    for i in range(3000):
        value = 0.9 * value + rng.gauss(0.0, 0.35)
        t.append(i * 0.1)
        dev.append(value)

    result = kpimod.compute_kpi(t, dev, cfgmod.KpiSection())
    assert result.max_abs_kmh == pytest.approx(PROD_KPI_MAX_ABS_KMH, abs=1e-9)
    assert result.reversal_max_per_window == PROD_KPI_REVERSAL_MAX_PER_5S
    assert result.reversal_max_per_window > 1  # 反転が数えられるデータになっていること
    assert 0.0 <= PROD_KPI_P95_KMH - result.p95_kmh <= 0.0101


def test_kpi_episodes_and_verdict() -> None:
    t = [i * 0.1 for i in range(100)]
    dev = [0.0] * 100
    dev[10:15] = [1.2, 1.5, -0.2, 1.1, 1.05]  # 10-11 と 13-14 の 2 区間
    dev[50] = -2.0
    result = kpimod.compute_kpi(t, dev, cfgmod.KpiSection())
    assert [(round(e.start_s, 1), round(e.end_s, 1)) for e in result.episodes] == [
        (1.0, 1.1), (1.3, 1.4), (5.0, 5.0)
    ]
    assert result.episodes[0].peak_kmh == 1.5
    assert result.max_abs_kmh == 2.0 and result.max_abs_t_s == pytest.approx(5.0)
    assert result.time_over_limit_s == pytest.approx(0.5)
    assert not result.max_ok and result.p95_ok
    assert result.passed_count == 2 and not result.passed


# ── 走行 ──────────────────────────────────────────────────────────────


async def test_run_mode_drive_records_rows_and_ends_in_stop_hold(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    cfg.mode_drive.pedal_standby = False  # 待機位置なし（使っていないペダルは 0%）
    hw = await _held_stub(cfg)
    mode = _mode([(0.0, 0.0), (0.6, 0.0), (2.0, 6.0), (3.0, 6.0), (3.6, 0.0), (4.0, 0.0)])
    ff = FakeFF()
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")

    result = await mdmod.run_mode_drive(hw, cfg, mdmod.ModeDriveSetup(mode, ff), log=log)  # type: ignore[arg-type]
    await log.close()

    assert result.completed and result.abort_reason == ""
    assert result.run_duration_s == pytest.approx(4.0, abs=0.1)
    assert result.cycles >= 70  # 50ms 周期で 4s
    assert ff.calls == result.cycles
    rows = mrmod.rows_from_samples(result.samples)
    assert 75 <= len(rows) <= 85  # 50ms ごと（段1b: csv_interval_s 0.1→0.05）
    assert rows[0].t_s == pytest.approx(0.0, abs=0.06)
    assert rows[0].brake_pct == HOLD_PCT and rows[0].accel_pct == 0.0  # 停車中は保持ブレーキ
    assert any(r.accel_pct == 30.0 for r in rows)  # 加速中はアクセル
    assert all(r.accel_pct == 0.0 or r.brake_pct == 0.0 for r in rows)
    assert {r.segment for r in rows} == {"Low"}
    # 終わりは停車保持（止まっていなければ緩減速で止めてから）
    assert pulse_to_opening(await hw.brake.read_position()) == pytest.approx(HOLD_PCT, abs=0.02)
    assert await hw.accel.read_position() == 0

    # CSV にモード経過秒・偏差・effort が入り、CSV から同じ行を読み直せる
    from_csv = mrmod.rows_from_csv(log.csv_path)
    assert len(from_csv) == len(rows)
    assert from_csv[5].deviation_kmh == pytest.approx(rows[5].deviation_kmh, abs=2e-3)
    assert from_csv[5].ff_effort_pct == pytest.approx(rows[5].ff_effort_pct, abs=1e-3)

    # A7: FF のペダル別・指令と実開度（スタブは実開度 = 指令）・ステータス・周期の処理時間
    with log.csv_path.open(newline="", encoding="utf-8") as f:
        mode_rows = [r for r in csv.DictReader(f) if r["section"] == dlmod.SECTION_MODE_DRIVE]
    assert len(mode_rows) == len(rows)
    assert any(float(r["accel_ff_pct"]) > 0.0 for r in mode_rows)
    assert all(r["accel_ff_pct"] and r["brake_ff_pct"] and r["cycle_ms"] for r in mode_rows)
    assert all(r["accel_actual_pct"] == r["accel_cmd_pct"] for r in mode_rows)
    assert all(r["brake_actual_mm"] == r["brake_cmd_mm"] for r in mode_rows)
    assert {r["brake_servo_on"] for r in mode_rows} == {"1"}
    assert len(run_cycle_ms := [float(r["cycle_ms"]) for r in mode_rows]) == len(rows)
    assert min(run_cycle_ms) >= 0.0


def test_cycle_time_text_counts_cycles_over_period() -> None:
    text = mdmod.cycle_time_text([10.0] * 18 + [40.0, 60.0], 0.05)
    assert text == "平均 14.0 / p95 40.0 / 最大 60.0 ms（周期 50ms を超えた 1 周期）"


async def test_run_mode_drive_limit_stops_early(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = await _held_stub(cfg)
    mode = _mode([(0.0, 0.0), (0.2, 0.0), (5.0, 8.0), (30.0, 8.0), (31.0, 0.0)])
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    result = await mdmod.run_mode_drive(
        hw, cfg, mdmod.ModeDriveSetup(mode, FakeFF()), log=log, limit_s=1.5  # type: ignore[arg-type]
    )
    await log.close()
    assert result.completed
    assert result.mode_duration_s == 1.5
    assert result.run_duration_s == pytest.approx(1.5, abs=0.1)
    # 走行中に打ち切ったので緩減速で止まっている
    assert await hw.can.read_speed() < 0.05
    assert any(s.section == dlmod.SECTION_DECEL_TO_STOP for s in log.samples)


async def test_run_mode_drive_releases_pedals_on_overcurrent(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = await _held_stub(cfg)
    assert isinstance(hw.brake, hwmod.StubActuator)
    calls = 0

    async def spiking_current() -> float:
        nonlocal calls
        calls += 1
        return 1e6 if calls > 10 else 0.0

    hw.brake.read_current = spiking_current  # type: ignore[method-assign]
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    mode = _mode([(0.0, 0.0), (5.0, 0.0)])
    result = await mdmod.run_mode_drive(hw, cfg, mdmod.ModeDriveSetup(mode, FakeFF()), log=log)  # type: ignore[arg-type]
    await log.close()
    assert not result.completed
    assert "過電流" in result.abort_reason
    assert hw.brake.position == 0 and hw.accel.position == 0  # ペダルを離している


async def test_run_mode_drive_stops_on_alarm(tmp_path: Path) -> None:
    """アクチュエータのアラームは 1.0s ごとの確認で検知し、DriveError で中断してペダルを離す。"""
    cfg = _tmp_cfg(tmp_path)
    hw = await _held_stub(cfg)
    assert isinstance(hw.accel, hwmod.StubActuator)
    calls = 0

    async def alarming() -> bool:
        nonlocal calls
        calls += 1
        return calls > 2  # 1・2 回目（t=0, 1.0s）は正常、3 回目（t=2.0s）でアラーム

    hw.accel.is_alarm_active = alarming  # type: ignore[method-assign]
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    mode = _mode([(0.0, 0.0), (5.0, 8.0), (30.0, 8.0)])
    result = await mdmod.run_mode_drive(hw, cfg, mdmod.ModeDriveSetup(mode, FakeFF()), log=log)  # type: ignore[arg-type]
    await log.close()
    assert not result.completed
    assert "アラーム" in result.abort_reason and "アクセル" in result.abort_reason
    assert result.run_duration_s == pytest.approx(2.0, abs=0.1)
    assert hw.brake.position == 0 and hw.accel.position == 0  # ペダルを離している


async def test_alarm_is_polled_once_per_second_not_every_cycle(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = await _held_stub(cfg)
    assert isinstance(hw.accel, hwmod.StubActuator)
    calls = 0

    async def counting_alarm() -> bool:
        nonlocal calls
        calls += 1
        return False

    hw.accel.is_alarm_active = counting_alarm  # type: ignore[method-assign]
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    mode = _mode([(0.0, 0.0), (3.0, 0.0)])
    result = await mdmod.run_mode_drive(hw, cfg, mdmod.ModeDriveSetup(mode, FakeFF()), log=log)  # type: ignore[arg-type]
    await log.close()
    assert result.completed
    every = round(ALARM_CHECK_INTERVAL_S / cfg.control.loop_interval_s)
    assert every == 20
    assert calls == len(range(0, result.cycles, every))
    assert 1 < calls < result.cycles  # 毎周期は呼んでいない


async def test_run_mode_drive_stops_on_zero_current_after_having_moved(tmp_path: Path) -> None:
    """指令位置>0 なのに、一度は流れていた電流が 1.0s 0mA のままなら中断する（脱落の安全網）。"""
    cfg = _tmp_cfg(tmp_path)
    hw = await _held_stub(cfg)
    assert isinstance(hw.accel, hwmod.StubActuator)
    calls = 0

    async def dropping_current() -> float:
        nonlocal calls
        calls += 1
        return 300.0 if calls <= 3 else 0.0

    hw.accel.read_current = dropping_current  # type: ignore[method-assign]
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    mode = _mode([(0.0, 0.0), (5.0, 8.0), (30.0, 8.0)])
    result = await mdmod.run_mode_drive(hw, cfg, mdmod.ModeDriveSetup(mode, FakeFF()), log=log)  # type: ignore[arg-type]
    await log.close()
    assert not result.completed
    assert "0mA" in result.abort_reason and "アクセル" in result.abort_reason
    assert hw.brake.position == 0 and hw.accel.position == 0  # ペダルを離している


async def test_zero_current_never_nonzero_does_not_abort(tmp_path: Path) -> None:
    """一度も電流>0 を返さない軸（スタブ相当。既定の StubActuator は常に 0mA）では中断しない。"""
    cfg = _tmp_cfg(tmp_path)
    hw = await _held_stub(cfg)
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    mode = _mode([(0.0, 0.0), (5.0, 8.0), (6.0, 8.0)])
    result = await mdmod.run_mode_drive(hw, cfg, mdmod.ModeDriveSetup(mode, FakeFF()), log=log)  # type: ignore[arg-type]
    await log.close()
    assert result.completed  # StubActuator.read_current() は常に 0.0 だが誤検知しない
    rows = mrmod.rows_from_samples(result.samples)
    assert any(r.accel_pct > 0.0 for r in rows)  # アクセルは実際に開いていた


async def test_unused_pedal_waits_at_standby_at_min(tmp_path: Path) -> None:
    """使っていないペダルは 0% ではなく min（触れた位置）で待つ（phase は FF の選択のまま）。"""
    cfg = _tmp_cfg(tmp_path)
    accel_sb, brake_sb = mdmod.standby_openings(cfg)
    assert accel_sb == pytest.approx(cfg.vehicle.min_accel_opening_pct)
    assert brake_sb == pytest.approx(cfg.vehicle.min_brake_opening_pct)
    assert 0.0 < accel_sb and 0.0 < brake_sb
    hw = await _held_stub(cfg)
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    mode = _mode([(0.0, 0.0), (0.6, 0.0), (2.0, 6.0), (3.0, 6.0), (3.6, 0.0), (4.0, 0.0)])
    result = await mdmod.run_mode_drive(hw, cfg, mdmod.ModeDriveSetup(mode, FakeFF()), log=log)  # type: ignore[arg-type]
    await log.close()
    assert result.completed
    samples = result.samples
    accel_rows = [s for s in samples if s.phase == mdmod.PHASE_ACCEL]
    brake_rows = [s for s in samples if s.phase == mdmod.PHASE_BRAKE]
    assert accel_rows and brake_rows
    for s in accel_rows:  # アクセル中のブレーキは待機位置
        assert s.data.accel_opening == 30.0
        assert s.data.brake_opening == pytest.approx(brake_sb)
        assert s.data.brake_pos == opening_to_pulse(brake_sb)
    for s in brake_rows:  # ブレーキ中（停車保持を含む）のアクセルは待機位置
        assert s.data.accel_opening == pytest.approx(accel_sb)
        assert s.data.accel_pos == opening_to_pulse(accel_sb)
    # FF が待機位置より浅いブレーキ（12%）を出したら、待機位置ではなく FF の値
    assert brake_sb < 12.0 and any(s.data.brake_opening == 12.0 for s in brake_rows)
    # 走行後は従来通り: アクセル 0・ブレーキ停車保持
    assert await hw.accel.read_position() == 0
    assert pulse_to_opening(await hw.brake.read_position()) == pytest.approx(HOLD_PCT, abs=0.02)


async def test_governor_cap_to_zero_stops_at_brake_standby(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """ガバナーが頭打ちを 0% まで下げても、ブレーキは原点まで戻さず待機位置で止める。"""
    cfg = _tmp_cfg(tmp_path)
    _, brake_sb = mdmod.standby_openings(cfg)

    def cap_to_zero(
        self: mdmod.DecelGovernor, now: float, speed: float, brake: float
    ) -> tuple[float, bool]:
        return (0.0, True) if brake > 0.0 else (brake, False)

    monkeypatch.setattr(mdmod.DecelGovernor, "apply", cap_to_zero)
    hw = await _held_stub(cfg)
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    mode = _mode([(0.0, 0.0), (1.0, 0.0)])  # 停車保持のブレーキ指令 → ガバナーが 0% に頭打ち
    result = await mdmod.run_mode_drive(
        hw, cfg, mdmod.ModeDriveSetup(mode, FakeFF()), log=log, limit_s=0.5  # type: ignore[arg-type]
    )
    await log.close()
    governed = [s for s in result.samples if s.phase == mdmod.PHASE_BRAKE_GOVERNED]
    assert governed and all(s.governor_active for s in governed)
    assert all(s.data.brake_opening == pytest.approx(brake_sb) for s in governed)
    assert all(s.data.brake_pos == opening_to_pulse(brake_sb) for s in governed)


async def test_abort_releases_pedals_to_zero_even_with_standby(tmp_path: Path) -> None:
    """中断時は待機位置ではなく両軸 0 に戻す（既存の安全動作を変えない）。"""
    cfg = _tmp_cfg(tmp_path)
    assert cfg.mode_drive.pedal_standby
    hw = await _held_stub(cfg)
    assert isinstance(hw.accel, hwmod.StubActuator)
    calls = 0

    async def alarming() -> bool:
        nonlocal calls
        calls += 1
        return calls > 1

    hw.accel.is_alarm_active = alarming  # type: ignore[method-assign]
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    mode = _mode([(0.0, 0.0), (5.0, 8.0), (30.0, 8.0)])
    result = await mdmod.run_mode_drive(hw, cfg, mdmod.ModeDriveSetup(mode, FakeFF()), log=log)  # type: ignore[arg-type]
    await log.close()
    assert not result.completed
    assert hw.brake.position == 0 and hw.accel.position == 0  # type: ignore[attr-defined]


def test_standby_label_and_disabled() -> None:
    cfg = cfgmod.load_config(Path("tests/research/config_testVehicle.yaml"))
    cfg.vehicle.min_accel_opening_pct, cfg.vehicle.min_brake_opening_pct = 8.0, 11.16
    assert mdmod.standby_label(cfg) == "8.00% / 11.16%（触れた位置 min）"
    assert mdmod.standby_openings(cfg) == (8.0, 11.16)
    cfg.vehicle.min_accel_opening_pct = 0.0  # min が 0 なら待機位置も 0%
    assert mdmod.standby_openings(cfg)[0] == 0.0
    cfg.mode_drive.pedal_standby = False
    assert mdmod.standby_openings(cfg) == (0.0, 0.0)
    assert mdmod.standby_label(cfg) == "なし（0%）"


def test_pedal_stats_use_phase_not_opening_with_standby() -> None:
    """待機位置で開度 > 0 のまま待つ行は「そのペダルを使った」に数えない。"""
    def row(t: float, phase: str, accel: float, brake: float) -> mrmod.ModeRow:
        return mrmod.ModeRow(
            t_s=t, ref_kmh=20.0, actual_kmh=20.0, accel_pct=accel, brake_pct=brake,
            ff_effort_pct=0.0, pid_effort_pct=0.0, effort_pct=0.0, segment="Low", phase=phase,
        )

    rows = [
        row(0.0, "ACCEL", 15.0, 11.16),
        row(0.1, "ACCEL", 15.0, 11.16),
        row(0.2, "BRAKE", 8.0, 16.0),
        row(0.3, "BRAKE_GOV", 8.0, 11.16),  # ガバナーで待機位置まで頭打ち → 不感帯より浅い指令
        row(0.4, "COAST", 8.0, 11.16),
    ]
    cfg = cfgmod.load_config(Path("tests/research/config_testVehicle.yaml"))
    st = mrmod.pedal_stats(rows, 10.0, 13.16, feedforward_params(cfg))
    assert st.accel_active_s == pytest.approx(0.2)
    assert st.accel_in_deadband_s == 0.0
    assert st.brake_active_moving_s == pytest.approx(0.2)
    assert st.brake_in_deadband_moving_s == pytest.approx(0.1)
    assert st.accel_max_pct == 15.0 and st.brake_max_moving_pct == 16.0
    assert st.switches == 1
    assert st.governor_s == pytest.approx(0.1)


# ── レポート ──────────────────────────────────────────────────────────


def _synthetic_rows(n: int = 1200) -> list[mrmod.ModeRow]:
    rows = []
    for i in range(n):
        t = i * 0.1
        ref = max(0.0, min(40.0, 4.0 * (t - 10.0))) if t < 80 else max(0.0, 40.0 - 3.0 * (t - 80))
        dev = -0.8 if 10 < t < 20 else 1.4 if 81 < t < 83 else 0.1
        effort = 15.0 if 10 < t < 20 else -14.0 if t > 80 else 5.0
        rows.append(mrmod.ModeRow(
            t_s=t, ref_kmh=ref, actual_kmh=ref + dev,
            accel_pct=max(0.0, effort), brake_pct=max(0.0, -effort),
            ff_effort_pct=effort, pid_effort_pct=0.0, effort_pct=effort,
            segment="Low" if t < 60 else "Mid", phase="ACCEL" if effort > 0 else "BRAKE",
        ))
    return rows


def test_write_mode_report_creates_markdown_and_figures(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    info = mrmod.RunInfo(
        label="FF", title="手順 3: FF のみでモード走行", controller="FF のみ",
        csv_path=tmp_path / "x.csv", hw_mode="stub", mode_name="test_mode",
        started_at=datetime(2026, 9, 11, 10, 0), completed=True, cycles=2400, overruns=0,
    )
    rows = _synthetic_rows()
    path = mrmod.write_mode_report(rows, cfg, info, tmp_path)
    assert path.name == "report20260911_RunFF.md"
    text = path.read_text(encoding="utf-8")
    assert "## 1. 結論（プライマリー KPI）" in text
    assert "最大逸脱 1.40 km/h" in text
    assert "| Mid |" in text and "| 加速 |" in text
    for name in ("overview", "deviation", "worst_zoom", "distribution"):
        fig = tmp_path / "report20260911_RunFF" / f"{name}.png"
        assert fig.stat().st_size > 1000
        assert f"report20260911_RunFF/{name}.png" in text
    # 同じ日の 2 本目は上書きしない
    assert mrmod.write_mode_report(rows, cfg, info, tmp_path).name == "report20260911_RunFF_2.md"


def test_driving_states_classify_by_reference_slope() -> None:
    rows = _synthetic_rows()
    states = mrmod.driving_states(rows)
    by_t = {round(r.t_s, 1): s for r, s in zip(rows, states, strict=True)}
    assert by_t[5.0] == mrmod.STATE_STOP
    assert by_t[15.0] == mrmod.STATE_ACCEL
    assert by_t[50.0] == mrmod.STATE_CRUISE
    assert by_t[85.0] == mrmod.STATE_DECEL


def test_report_can_be_rebuilt_from_csv(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    wall0 = datetime(2026, 9, 11, tzinfo=UTC)
    samples = [
        dlmod.DriveSample(
            elapsed_s=30.0 + r.t_s,
            timestamp=wall0 + timedelta(seconds=r.t_s),
            data=DriveLogData(
                ref_speed_kmh=r.ref_kmh, actual_speed_kmh=r.actual_kmh,
                accel_opening=r.accel_pct, brake_opening=r.brake_pct,
                accel_pos=opening_to_pulse(r.accel_pct), brake_pos=opening_to_pulse(r.brake_pct),
                accel_current=0.0, brake_current=0.0,
                plan_effort_pct=r.ff_effort_pct,
                trim_effort_pct=0.0,
                applied_effort_pct=r.effort_pct,
            ),
            section=dlmod.SECTION_MODE_DRIVE, phase=r.phase, pattern=r.segment, mode_time_s=r.t_s,
        )
        for r in _synthetic_rows(600)
    ]
    csv_path = tmp_path / "drive_log_stub_20260911_120000.csv"
    dlmod.write_csv(samples, csv_path)
    assert mrmod.main([str(csv_path), "--label", "FF", "--config", str(cfg.source_path)]) == 0
    assert (tmp_path / "report20260911_RunFF.md").exists()


# ── CLI 経由 ─────────────────────────────────────────────────────────


def test_step3_requires_step1_before_loading_mode(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    async def must_not_load(cfg: object, name: object) -> None:
        raise AssertionError("HW 未初期化なら DB・モデルを読みに行かない")

    monkeypatch.setattr(mainmod, "prepare_mode_drive", must_not_load)
    assert mainmod.main(["--only", "3"]) == 4
    assert "手順 1 を先に実行" in capsys.readouterr().out


def test_step3_prepare_error_stops_before_pre_drive_check(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    async def failing_prepare(cfg: object, name: object) -> None:
        raise cfgmod.ConfigError("走行モード 'x' が driving_modes にありません")

    async def must_not_check(hw: object, cfg: object, **kwargs: object) -> int:
        raise AssertionError("準備に失敗したら走行前チェックに進まない")

    cfg = _tmp_cfg(tmp_path)
    monkeypatch.setattr(mainmod, "prepare_mode_drive", failing_prepare)
    monkeypatch.setattr(mainmod, "run_pre_drive_check", must_not_check)
    assert mainmod.main(["--steps", "1,3", "--config", str(cfg.source_path)]) == 2
    out = capsys.readouterr().out
    assert "driving_modes にありません" in out
    assert "終了処理: 完了" in out


def test_step3_end_to_end_with_stub(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    mode = _mode([(0.0, 0.0), (0.5, 0.0), (2.0, 5.0), (3.0, 0.0), (3.5, 0.0)])

    async def fake_prepare(cfg: object, name: object) -> mdmod.ModeDriveSetup:
        return mdmod.ModeDriveSetup(mode, FakeFF())  # type: ignore[arg-type]

    async def fake_pre_check(hw: hwmod.ResearchHardware, cfg: object, **kwargs: object) -> int:
        """走行前チェックは test_research_pre_drive_check.py で確認する。停車させて返す。"""
        # 手順 3 の前なので stop_brake_opening_pct へ一気に踏む分岐（next_step != 2）で呼ばれる
        assert kwargs.get("next_step") == 3
        assert isinstance(hw.can, hwmod.StubCANReader)
        hw.can.speed_kmh = 0.0
        return 0

    cfg = _tmp_cfg(tmp_path)
    monkeypatch.setattr(mainmod, "prepare_mode_drive", fake_prepare)
    monkeypatch.setattr(mainmod, "run_pre_drive_check", fake_pre_check)
    assert mainmod.main(["--steps", "1,3", "--config", str(cfg.source_path)]) == 0
    out = capsys.readouterr().out
    assert "手順 3 完了" in out
    results = tmp_path / "results"
    reports = list(results.glob("report*_RunFF.md"))
    assert len(reports) == 1
    csvs = list(results.glob("drive_log_stub_*.csv"))
    assert len(csvs) == 1
    assert len(mrmod.rows_from_csv(csvs[0])) >= 30


def test_step3_abort_writes_report_and_returns_5(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    mode = _mode([(0.0, 0.0), (5.0, 0.0)])

    async def fake_prepare(cfg: object, name: object) -> mdmod.ModeDriveSetup:
        return mdmod.ModeDriveSetup(mode, FakeFF())  # type: ignore[arg-type]

    async def fake_pre_check(hw: hwmod.ResearchHardware, cfg: object, **kwargs: object) -> int:
        assert isinstance(hw.can, hwmod.StubCANReader)
        hw.can.speed_kmh = 0.0
        calls = 0

        async def failing_read() -> float:
            nonlocal calls
            calls += 1
            if calls > 15:
                raise OSError("CAN バスが止まった（テスト）")
            return 0.0

        hw.can.read_speed = failing_read  # type: ignore[method-assign]
        return 0

    cfg = _tmp_cfg(tmp_path)
    monkeypatch.setattr(mainmod, "prepare_mode_drive", fake_prepare)
    monkeypatch.setattr(mainmod, "run_pre_drive_check", fake_pre_check)
    assert mainmod.main(["--steps", "1,3", "--config", str(cfg.source_path)]) == 5
    out = capsys.readouterr().out
    assert "CAN 車速を読めません" in out
    assert "終了処理: 完了" in out
    report = next((tmp_path / "results").glob("report*_RunFF.md"))
    assert "中断: CAN 車速を読めません" in report.read_text(encoding="utf-8")


# ── V2: 実測車速の配線（ActualSpeedHistory・_ff_inputs） ────────────────


def test_actual_speed_history_interpolates_and_falls_back() -> None:
    hist = mdmod.ActualSpeedHistory(max_span_s=2.0)
    assert hist.at(5.0) == 0.0  # 空なら 0
    hist.record(0.0, 10.0)
    hist.record(1.0, 20.0)
    assert hist.at(-1.0) == 10.0  # 最初より前は最初の値
    assert hist.at(0.5) == pytest.approx(15.0)  # 線形補間
    assert hist.at(5.0) == 20.0  # 最後より後は最後の値


def test_actual_speed_history_drops_points_older_than_span() -> None:
    hist = mdmod.ActualSpeedHistory(max_span_s=1.0)
    for i in range(5):
        hist.record(float(i), float(i))
    assert hist.at(0.0) != 0.0 or len(hist._points) < 5  # noqa: SLF001 - 古い点は捨てている


class _FakeCandidate:
    candidate = "C1"
    uses_actual_speed = False
    horizons = (0.5, 1.0)
    past_horizons = (0.5,)


class _FakeC4:
    candidate = "C4"
    uses_actual_speed = True
    horizons = (0.5, 1.0)
    past_horizons = (0.5,)


class _FakeC5:
    candidate = "C5"
    uses_actual_speed = True
    horizons = (0.5, 1.0)
    past_horizons = (0.5,)


class _ConstModel:
    """与えた定数を返すだけの推定器（predict_effort の停車レジーム分岐だけを見るため）。

    tests/research/test_research_ff_candidate.py の `_Const` と同じ役割。
    """

    def __init__(self, value: float) -> None:
        self.value = value

    def predict(self, x: np.ndarray) -> np.ndarray:
        return np.full(len(x), self.value, dtype=float)


def _bare_mode_run(ff: object) -> mdmod._ModeRun:  # noqa: SLF001 - 単体テストで直接組み立てる
    mode = _mode([(0.0, 0.0), (10.0, 100.0)])
    run = object.__new__(mdmod._ModeRun)
    run.ff = ff  # type: ignore[attr-defined]
    run.ref = mdmod.ReferenceSpeed(mode)
    run.actual_history = mdmod.ActualSpeedHistory(max(ff.past_horizons, default=1.0) + 0.5)  # type: ignore[attr-defined]
    return run


def test_ff_inputs_uses_reference_speed_for_c1() -> None:
    run = _bare_mode_run(_FakeCandidate())
    t, ref = 4.0, 40.0
    v0, future, past = run._ff_inputs(t, ref, speed=55.0)  # noqa: SLF001
    assert v0 == ref
    assert future == [run.ref.at(t + h) for h in _FakeCandidate.horizons]
    assert past == [run.ref.at(t - h) for h in _FakeCandidate.past_horizons]


def test_ff_inputs_c4_rebases_onto_actual_speed_keeping_reference_delta() -> None:
    run = _bare_mode_run(_FakeC4())
    t, ref, speed = 4.0, 40.0, 55.0
    v0, future, past = run._ff_inputs(t, ref, speed)  # noqa: SLF001
    assert v0 == speed
    for h, f in zip(_FakeC4.horizons, future, strict=True):
        assert f == pytest.approx(speed + (run.ref.at(t + h) - ref))
    for h, p in zip(_FakeC4.past_horizons, past, strict=True):
        assert p == pytest.approx(speed + (run.ref.at(t - h) - ref))


def test_ff_inputs_c5_uses_absolute_reference_future_and_actual_past() -> None:
    run = _bare_mode_run(_FakeC5())
    run.actual_history.record(3.5, 33.0)
    run.actual_history.record(4.0, 55.0)
    t, ref, speed = 4.0, 40.0, 55.0
    v0, future, past = run._ff_inputs(t, ref, speed)  # noqa: SLF001
    assert v0 == speed
    assert future == [run.ref.at(t + h) for h in _FakeC5.horizons]  # 基準の絶対値
    assert past == [run.actual_history.at(t - h) for h in _FakeC5.past_horizons]  # 実測履歴
    assert past[0] == pytest.approx(33.0)


def _bare_mode_run_for(ff: object, mode: DrivingMode) -> mdmod._ModeRun:  # noqa: SLF001
    """`_bare_mode_run` の mode 差し替え版（停車区間を持つモードで確認したいテスト用）。"""
    run = object.__new__(mdmod._ModeRun)
    run.ff = ff  # type: ignore[attr-defined]
    run.ref = mdmod.ReferenceSpeed(mode)
    run.actual_history = mdmod.ActualSpeedHistory(max(ff.past_horizons, default=1.0) + 0.5)  # type: ignore[attr-defined]
    return run


def test_ff_inputs_stop_regime_forces_reference_speed_for_c4_and_c5() -> None:
    """基準が停車レジームなら C4・C5 も基準車速ベースの入力になる（A8 報告 5 章 #2）。

    停車指示中（t=0〜10s は基準 0 km/h）でも実車速がクリープ等で動いていることがある
    （ここでは speed=3.0）。v0 を実車速にする C4・C5 は「実車速が 0.02 km/h を下回る」
    条件が現実には成立せず停車保持に入れないため、基準車速が停車レジームのときは
    uses_actual_speed に関係なく C1 と同じ入力（基準車速そのもの）を返す。
    """
    stopped_mode = _mode([(0.0, 0.0), (10.0, 0.0), (20.0, 100.0)])
    t, speed = 5.0, 3.0
    ref = mdmod.ReferenceSpeed(stopped_mode).at(t)
    assert ref == 0.0  # 前提: t=5 は停車区間の途中

    for ff_cls in (_FakeCandidate, _FakeC4, _FakeC5):
        run = _bare_mode_run_for(ff_cls(), stopped_mode)
        v0, future, past = run._ff_inputs(t, ref, speed)  # noqa: SLF001
        assert v0 == 0.0, ff_cls.candidate
        assert future == [run.ref.at(t + h) for h in ff_cls.horizons], ff_cls.candidate
        assert future[0] == 0.0, ff_cls.candidate
        assert past == [run.ref.at(t - h) for h in ff_cls.past_horizons], ff_cls.candidate


def test_ff_inputs_stop_regime_feeds_into_stop_brake_hold() -> None:
    """停車レジームの入力を実際に predict_effort へ渡すと停車保持ブレーキが出る（結合確認）。

    _ff_inputs だけでなく CandidateFeedforward.predict_effort まで通して、A8 報告 5 章 #2 の
    症状（停車指示中に実車速が動いていて停車保持に入らない）が解消したことを確認する。
    """
    stopped_mode = _mode([(0.0, 0.0), (10.0, 0.0), (20.0, 100.0)])
    ff = ff_candidate.CandidateFeedforward()
    ff.set_params(FeedforwardParams(stop_brake_opening_pct=HOLD_PCT))
    ff._accel_model = _ConstModel(20.0)  # noqa: SLF001 - テスト用の差し込み
    ff._brake_model = _ConstModel(20.0)  # noqa: SLF001

    run = _bare_mode_run_for(ff, stopped_mode)
    t, speed = 5.0, 3.0  # 停車指示中だが実車速はクリープ等で動いている
    ref = run.ref.at(t)
    v0, future, past = run._ff_inputs(t, ref, speed)  # noqa: SLF001

    assert ff.predict_effort(v0, future, past) == pytest.approx(-HOLD_PCT)


def test_select_kwargs_point_is_empty_and_window_uses_L_and_H() -> None:
    """ペダル選択の方式（20260929 段3）: point は何も渡さず、window は G と ref(t+L)を渡す。"""
    from tests.research.pedal_select import window_slope_kmhs  # noqa: PLC0415

    run = _bare_mode_run(_FakeCandidate())
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    run.cfg = cfg  # type: ignore[attr-defined]
    cfg.feedforward.pedal_select_mode = "point"
    cfg.feedforward.reach_horizons_s = [0.5]  # 到達判定が選択を担う間は point でも何も渡さない
    assert run._select_kwargs(4.0, 40.0) == {}  # noqa: SLF001
    cfg.feedforward.reach_horizons_s = []

    cfg.feedforward.pedal_select_mode = "window"
    cfg.feedforward.pedal_select_brake_center_s = None  # yaml に L_b があっても外す
    cfg.feedforward.pedal_select_center_s = 0.5
    cfg.feedforward.pedal_select_width_s = 3.0
    kwargs = run._select_kwargs(4.0, 40.0)  # noqa: SLF001
    expected_g = window_slope_kmhs(run.ref.at, 4.0, 0.5, 3.0)
    assert kwargs["select_accel_kmhs"] == pytest.approx(expected_g)
    assert kwargs["select_speed_kmh"] == pytest.approx(run.ref.at(4.5))
    assert set(kwargs) == {"select_accel_kmhs", "select_speed_kmh"}  # L_b なしなら足さない

    # L_b（段2-2）: ブレーキ用に L_b の傾きと ref(t+L_b) を足す。L 側の 2 つは変わらない
    cfg.feedforward.pedal_select_brake_center_s = 0.1
    kwb = run._select_kwargs(4.0, 40.0)  # noqa: SLF001
    assert kwb["select_accel_kmhs"] == kwargs["select_accel_kmhs"]
    assert kwb["select_speed_kmh"] == kwargs["select_speed_kmh"]
    assert kwb["select_brake_accel_kmhs"] == pytest.approx(
        window_slope_kmhs(run.ref.at, 4.0, 0.1, 3.0)
    )
    assert kwb["select_brake_speed_kmh"] == pytest.approx(run.ref.at(4.1))


def test_select_kwargs_point_uses_pedal_select_point_s_independent_of_h1() -> None:
    """point の先読み（pedal_select_point_s）だけでペダル選択の傾きが決まる（20260930）。

    hp=1.0 は FF の入力の h1 点（future の 1.0s 先）から作る従来の desired_accel と完全一致。
    """
    run = _bare_mode_run(_FakeCandidate())
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    run.cfg = cfg  # type: ignore[attr-defined]
    cfg.feedforward.pedal_select_mode = "point"
    cfg.feedforward.reach_horizons_s = []
    t, speed = 4.0, 55.0
    ref = run.ref.at(t)
    v0, future, _ = run._ff_inputs(t, ref, speed)  # noqa: SLF001
    h1_point = future[list(_FakeCandidate.horizons).index(1.0)]
    assert h1_point == run.ref.at(t + 1.0)

    cfg.feedforward.pedal_select_point_s = 1.0
    kwargs = run._select_kwargs(t, v0)  # noqa: SLF001
    assert set(kwargs) == {"select_accel_kmhs"}  # 惰行加速度は v0 のまま（速度は渡さない）
    assert kwargs["select_accel_kmhs"] == (h1_point - v0) / 1.0  # 浮動小数まで一致

    cfg.feedforward.pedal_select_point_s = 2.0
    kwargs = run._select_kwargs(t, v0)  # noqa: SLF001
    assert kwargs["select_accel_kmhs"] == pytest.approx((run.ref.at(t + 2.0) - v0) / 2.0)


def test_select_kwargs_point_c4_uses_reference_delta() -> None:
    """C4 は v0 が実車速。FF の dv は ref(t+h) − ref(t) なので選択も同じ基準にする。"""
    run = _bare_mode_run(_FakeC4())
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    run.cfg = cfg  # type: ignore[attr-defined]
    cfg.feedforward.pedal_select_mode = "point"
    cfg.feedforward.reach_horizons_s = []
    cfg.feedforward.pedal_select_point_s = 2.0
    t, speed = 4.0, 55.0
    kwargs = run._select_kwargs(t, speed)  # noqa: SLF001
    assert kwargs["select_accel_kmhs"] == pytest.approx(
        (run.ref.at(t + 2.0) - run.ref.at(t)) / 2.0
    )


# ─── 実開度の安全網（2026-10-17 段2o-9） ───────────────────────────────────


def _interlock_run(cfg: cfgmod.ResearchConfig) -> mdmod._ModeRun:
    """_apply_interlock だけを試すための最小の _ModeRun（コンストラクタを通さない）。"""
    run = mdmod._ModeRun.__new__(mdmod._ModeRun)
    run.cfg = cfg
    run.interlock = (
        mdmod.InterlockStats(period_s=0.05) if cfg.mode_drive.pedal_interlock_enabled else None
    )
    run._last_accel_actual = None
    run._last_brake_actual = None
    run._accel_blocked = False
    run._brake_blocked = False
    return run


def test_interlock_blocks_next_pedal_until_other_actual_returns(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    amin, bmin = cfg.vehicle.min_accel_opening_pct, cfg.vehicle.min_brake_opening_pct
    run = _interlock_run(cfg)
    # 実開度が未取得: 止めない
    assert run._apply_interlock(30.0, 0.0) == (30.0, 0.0)
    # ブレーキの実開度が min を 0.2% 超えている: アクセルは止める（0 = 待機位置のまま）
    run._last_brake_actual = bmin + 0.2
    assert run._apply_interlock(30.0, 0.0) == (0.0, 0.0)
    assert run._apply_interlock(30.0, 0.0) == (0.0, 0.0)
    # 戻った（min + 揺れ 0.05% 以内）: 出す
    run._last_brake_actual = bmin + 0.05
    assert run._apply_interlock(30.0, 0.0) == (30.0, 0.0)
    st = run.interlock
    assert (st.accel_events, st.accel_cycles, st.brake_events, st.brake_cycles) == (1, 2, 0, 0)
    assert st.accel_s == pytest.approx(0.1)
    # 逆向き: アクセルの実開度が min を超えている間はブレーキを止める。止め始めは 1 回と数える
    run._last_accel_actual = amin + 3.0
    assert run._apply_interlock(0.0, 12.0) == (0.0, 0.0)
    assert run._apply_interlock(0.0, 12.0) == (0.0, 0.0)
    assert run.interlock.brake_events == 1 and run.interlock.brake_cycles == 2
    run._last_accel_actual = amin
    assert run._apply_interlock(0.0, 12.0) == (0.0, 12.0)
    # 止め終わったあとの次の止めは別の 1 回
    run._last_accel_actual = amin + 1.0
    run._apply_interlock(0.0, 12.0)
    assert run.interlock.brake_events == 2
    # 要求のないペダルは何もしない（惰行）
    assert run._apply_interlock(0.0, 0.0) == (0.0, 0.0)


def test_interlock_disabled_passes_through(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    cfg.mode_drive.pedal_interlock_enabled = False
    run = _interlock_run(cfg)
    run._last_brake_actual = 50.0
    run._last_accel_actual = 50.0
    assert run._apply_interlock(30.0, 12.0) == (30.0, 12.0)
    assert run.interlock is None


def test_interlock_config_default_is_enabled() -> None:
    assert cfgmod.ModeDriveSection().pedal_interlock_enabled is True


async def test_run_mode_drive_reports_interlock_stats(tmp_path: Path) -> None:
    """走行の結果に安全網の集計が入る（スタブは実開度が指令にすぐ追いつくので止まる回数は少ない）。"""
    cfg = _tmp_cfg(tmp_path)
    hw = await _held_stub(cfg)
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    mode = _mode([(0.0, 0.0), (0.6, 0.0), (2.0, 6.0), (3.0, 6.0), (3.6, 0.0), (4.0, 0.0)])
    result = await mdmod.run_mode_drive(hw, cfg, mdmod.ModeDriveSetup(mode, FakeFF()), log=log)  # type: ignore[arg-type]
    await log.close()
    assert result.interlock is not None
    assert result.interlock.accel_cycles >= result.interlock.accel_events >= 0
    assert result.interlock.period_s == cfg.control.loop_interval_s
    assert "止めた" in result.interlock.summary()


# ─── C10（偏差の上乗せ。2026-10-17 段2p） ────────────────────────────────


def _c10_cfg(
    tmp_path: Path, candidate: str = "C10", c: float = 100.0, row: str = "reference"
) -> cfgmod.ResearchConfig:
    """C8/C10 用の設定（kind=model の合成 pkl。表の遅れは両ペダル 0.05s）。"""
    cfg = _c7_cfg(tmp_path)
    cfg.feedforward.candidate = candidate
    cfg.forward_map.deviation_c = c
    cfg.forward_map.deviation_row = row
    cfg.forward_map.dob_tq_coef = 40.0  # テストの入力として固定（yaml の値 20 に依存しない）
    cfg.mode_drive.pedal_standby = False
    _write_forward_table_pkl(Path(cfg.feedforward.model_path), kind="model")
    return cfg


_C10_MODE_POINTS = [(0.0, 0.0), (2.0, 0.0), (8.0, 30.0), (12.0, 30.0), (16.0, 0.0), (18.0, 0.0)]


async def _c10_run_obj(
    cfg: cfgmod.ResearchConfig, monkeypatch: pytest.MonkeyPatch
) -> mdmod._ModeRun:
    """_c10_offset だけを試すための最小の _ModeRun（prepare_mode_drive で計画と τ を作る）。"""
    mode = _mode(_C10_MODE_POINTS)

    async def fake_load_mode(_cfg: cfgmod.ResearchConfig, _name: str) -> object:
        return mode

    monkeypatch.setattr(mdmod, "load_mode", fake_load_mode)
    setup = await mdmod.prepare_mode_drive(cfg, "dummy_mode")
    run = mdmod._ModeRun.__new__(mdmod._ModeRun)
    run.cfg = cfg
    run.ff = setup.ff
    is_c11 = cfg.feedforward.candidate == "C11"
    run.c10 = (
        mdmod.C10Stats(period_s=0.05) if cfg.feedforward.candidate in ("C10", "C11") else None
    )
    run._c10_prev_clamped = False
    # C11（DOB）: _ModeRun.__init__ と同じ持ち物
    run.dob = None
    run.dob_stats = None
    run._dob_effort = None
    if is_c11:
        ff = run.ff
        run.dob = PedalDisturbanceObserver(
            ff.table, tq_accel_s=ff.tq_accel_s, tq_brake_s=ff.tq_brake_s,
            lag_accel_s=ff.dob_lag_accel_s, lag_brake_s=ff.dob_lag_brake_s,
        )
        run.dob_stats = mdmod.DobStats(period_s=0.05)
    return run


async def test_c10_is_dispatched_like_c8_and_tau_is_c_times_lag(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    cfg = _c10_cfg(tmp_path, c=100.0)
    ff = mdmod.load_feedforward(cfg)
    assert isinstance(ff, ff_candidate.DeviationForwardFeedforward)
    assert isinstance(ff, ff_candidate.ForwardModelFeedforward) and ff.expected_kind == "model"
    run = await _c10_run_obj(cfg, monkeypatch)
    assert run.ff.tau_accel_s == pytest.approx(5.0) and run.ff.tau_brake_s == pytest.approx(5.0)
    out = capsys.readouterr().out
    assert "c=100・τ アクセル 5.0s / ブレーキ 5.0s・表の行 基準車速（案A）" in out
    # yaml の遅れの上書きがあれば、そちらが τ の元になる
    cfg.forward_map.accel_lag_s = 0.1
    run2 = await _c10_run_obj(cfg, monkeypatch)
    assert run2.ff.tau_accel_s == pytest.approx(10.0)


async def test_c10_without_measured_lag_does_not_correct_that_pedal(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    cfg = _c10_cfg(tmp_path)
    from tests.research.fwd_map import load_forward_map, save_table

    table = load_forward_map(Path(cfg.feedforward.model_path))
    del table.meta["brake_lag_s"]
    save_table(table, Path(cfg.feedforward.model_path))
    run = await _c10_run_obj(cfg, monkeypatch)
    assert run.ff.tau_brake_s is None and run.ff.tau_accel_s is not None
    assert "ブレーキ 上乗せなし（遅れが測れていない）" in capsys.readouterr().out
    plan = run.ff.plan
    ib = int(np.flatnonzero((plan.effort_pct < 0.0) & (plan.flag != 4))[0])  # FLAG_STOP 以外
    t = float(plan.t_s[ib])
    assert run._c10_offset(t, 20.0, 10.0, float(plan.effort_pct[ib])) == 0.0  # noqa: SLF001


async def test_c10_adds_throttle_when_behind_and_row_case_matters(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cfg = _c10_cfg(tmp_path, row="reference")
    run_a = await _c10_run_obj(cfg, monkeypatch)
    plan = run_a.ff.plan
    ia = int(np.flatnonzero(plan.effort_pct > 4.0 + 1.0)[0])
    t, e = float(plan.t_s[ia]), float(plan.effort_pct[ia])
    ref = 40.0
    # dev = ref − speed = +2 km/h（遅れている）→ 踏み増す。進んでいれば減る
    d_behind = run_a._c10_offset(t, ref, ref - 2.0, e)  # noqa: SLF001
    d_ahead = run_a._c10_offset(t, ref, ref + 2.0, e)  # noqa: SLF001
    assert d_behind > 0.0 and d_ahead < 0.0
    assert run_a.c10.accel_cycles == 2 and run_a.c10.brake_cycles == 0
    # 案B（実車速の行）: 低い車速の行のほうが上乗せが小さい（合成の表は速度が高いほど惰行が深い）
    cfg_b = _c10_cfg(tmp_path, row="actual")
    run_b = await _c10_run_obj(cfg_b, monkeypatch)
    assert run_b.ff.deviation_row == "actual"
    d_behind_b = run_b._c10_offset(t, ref, ref - 2.0, e)  # noqa: SLF001
    assert 0.0 < d_behind_b < d_behind


def _gain_entry(ratio: float) -> dict:
    """局所の効きの倍率の表（車速 30〜50・開度の列 −50〜+100 の全部のマスが同じ倍率）。"""
    return {
        "speed_rep_kmh": [30.0, 50.0],
        "u_from_onset_pct": [-50.0, 100.0],
        "ratio": [[ratio, ratio], [ratio, ratio]],
        "n": [[3, 3], [3, 3]],
    }


async def test_c10_local_gain_divides_the_offset_per_pedal_key(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cfg = _c10_cfg(tmp_path)
    run = await _c10_run_obj(cfg, monkeypatch)
    plan = run.ff.plan
    ia = int(np.flatnonzero(plan.effort_pct > 4.0 + 1.0)[0])
    ib = int(np.flatnonzero((plan.effort_pct < 0.0) & (plan.flag != 4))[0])
    ta, ea = float(plan.t_s[ia]), float(plan.effort_pct[ia])
    tb_, eb = float(plan.t_s[ib]), float(plan.effort_pct[ib])
    meta = run.ff.table.meta
    meta.update(accel_onset_u_pct=1.0, brake_onset_u_pct=1.0)
    ref = 40.0

    def offsets() -> tuple[float, float]:
        a = run._c10_offset(ta, ref, ref - 2.0, ea)  # noqa: SLF001
        b = run._c10_offset(tb_, ref, ref - 2.0, eb)  # noqa: SLF001
        return a, b

    base_a, base_b = offsets()
    assert base_a != 0.0 and base_b != 0.0
    assert run.c10.accel_gain_cycles == 0 and run.c10.brake_gain_cycles == 0
    # meta に倍率があってもキーが false なら今と同じ
    meta["local_gain_accel"] = _gain_entry(2.0)
    meta["local_gain_brake"] = _gain_entry(2.0)
    assert offsets() == (base_a, base_b)
    # ブレーキだけ true: ブレーキの差が半分（アクセルは変わらない）
    cfg.forward_map.local_gain_brake = True
    a, b = offsets()
    assert a == base_a and b == pytest.approx(base_b / 2.0)
    assert run.c10.brake_gain_cycles == 1 and run.c10.accel_gain_cycles == 0
    assert run.c10.brake_gain_cut_sum_pct == pytest.approx(abs(base_b) / 2.0)
    # アクセルだけ true
    cfg.forward_map.local_gain_brake = False
    cfg.forward_map.local_gain_accel = True
    a, b = offsets()
    assert a == pytest.approx(base_a / 2.0) and b == base_b
    assert run.c10.accel_gain_cycles == 1
    # 7.5 km/h 未満の行・meta なしは 1（今と同じ）。車速の範囲外は端の行の倍率
    low = run._c10_offset(ta, 5.0, 3.0, ea)  # noqa: SLF001
    cfg.forward_map.local_gain_accel = False
    assert low == run._c10_offset(ta, 5.0, 3.0, ea)  # noqa: SLF001
    plain = run._c10_offset(ta, 90.0, 88.0, ea)  # noqa: SLF001
    cfg.forward_map.local_gain_accel = True
    assert run._c10_offset(ta, 90.0, 88.0, ea) == pytest.approx(plain / 2.0)  # noqa: SLF001
    del meta["local_gain_accel"]
    assert run._c10_offset(ta, ref, ref - 2.0, ea) == base_a  # noqa: SLF001
    meta["local_gain_accel"] = _gain_entry(2.0)
    del meta["accel_onset_u_pct"]  # 効き始めが無ければ 1
    assert run._c10_offset(ta, ref, ref - 2.0, ea) == base_a  # noqa: SLF001
    assert "倍率で割った" in run.c10.summary()
    text = mdmod.c10_settings_text(run.ff, cfg.forward_map)
    assert "倍率 ブレーキ off（表あり） / アクセル on（表あり）" in text
    assert "倍率" not in mdmod.c10_settings_text(run.ff)  # fm なしは今までと同じ文


async def test_c10_local_gain_divides_before_the_min_clamp(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cfg = _c10_cfg(tmp_path)
    run = await _c10_run_obj(cfg, monkeypatch)
    plan = run.ff.plan
    amin = cfg.vehicle.min_accel_opening_pct
    ia = int(np.flatnonzero(plan.effort_pct > amin + 1.0)[0])
    ta = float(plan.t_s[ia])
    e = amin + 1.0  # min の 1% 上。減らす量が 1% を超えると min で止まる
    meta = run.ff.table.meta
    meta.update(accel_onset_u_pct=0.0, local_gain_accel=_gain_entry(4.0))
    ref = 40.0
    found = False
    for dev in (1.0, 2.0, 3.0, 5.0, 8.0, 12.0):
        cfg.forward_map.local_gain_accel = False
        off = run._c10_offset(ta, ref, ref + dev, e)  # noqa: SLF001
        cfg.forward_map.local_gain_accel = True
        on = run._c10_offset(ta, ref, ref + dev, e)  # noqa: SLF001
        if off == pytest.approx(-1.0) and on > -1.0 + 1e-9:
            found = True  # 割る前なら min で止まる量が、割ると止まらず 1/4 になる
            break
    assert found


async def test_c10_zero_c_is_identical_to_c8_and_leaves_ff_column(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cfg = _c10_cfg(tmp_path, c=0.0)
    run = await _c10_run_obj(cfg, monkeypatch)
    plan = run.ff.plan
    for i in range(0, len(plan.t_s), 7):
        t, e = float(plan.t_s[i]), float(plan.effort_pct[i])
        assert run._c10_offset(t, 30.0, 10.0, e) == 0.0  # noqa: SLF001
    # C8 の計画は C10 と同一（C10 は列を変えない）
    cfg8 = _c10_cfg(tmp_path, candidate="C8")
    run8 = await _c10_run_obj(cfg8, monkeypatch)
    assert run8.c10 is None
    assert np.array_equal(run8.ff.plan.effort_pct, plan.effort_pct)


async def test_c10_run_logs_offset_column_and_c8_leaves_it_blank(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    results = {}
    plans = {}
    for cand in ("C10", "C8"):
        cfg = _c10_cfg(tmp_path, candidate=cand)
        mode = _mode([(0.0, 0.0), (0.6, 0.0), (2.0, 6.0), (3.0, 6.0), (3.6, 0.0), (4.0, 0.0)])

        async def fake_load_mode(
            _cfg: cfgmod.ResearchConfig, _name: str, _m: object = mode
        ) -> object:
            return _m

        monkeypatch.setattr(mdmod, "load_mode", fake_load_mode)
        setup = await mdmod.prepare_mode_drive(cfg, "dummy_mode")
        hw = await _held_stub(cfg)
        log = dlmod.SessionLog(cfg, hw, has_ref=True)
        log.start(dlmod.SECTION_MODE_DRIVE, "")
        result = await mdmod.run_mode_drive(hw, cfg, setup, log=log)
        await log.close()
        results[cand] = (result, log.csv_path)
        plans[cand] = setup.ff.plan
    r10, csv10 = results["C10"]
    r8, csv8 = results["C8"]
    assert r8.c10 is None and r8.c10_settings is None
    assert r10.c10 is not None and r10.c10_settings is not None and "c=100" in r10.c10_settings
    with csv10.open(newline="", encoding="utf-8") as f:
        rows10 = [r for r in csv.DictReader(f) if r["section"] == dlmod.SECTION_MODE_DRIVE]
    assert any(r["c10_effort_pct"] not in ("", "0.000") for r in rows10)
    with csv8.open(newline="", encoding="utf-8") as f:
        assert all(r["c10_effort_pct"] == "" for r in csv.DictReader(f))
    # FF の列は上乗せで変わらない（ログの plan_effort は plan.at のまま）
    plan10 = plans["C10"]
    for sample in r10.samples:
        assert sample.data.plan_effort_pct == pytest.approx(plan10.at(sample.mode_time_s))


def test_forward_map_deviation_settings_are_validated(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    # 今の yaml の値（段6-1b で deviation_c 6.0）
    assert cfg.forward_map.deviation_c == 6.0 and cfg.forward_map.deviation_row == "reference"
    assert not any("deviation" in p for p in cfgmod.validate_config(cfg))
    cfg.forward_map.deviation_row = "bogus"
    assert any("deviation_row" in p for p in cfgmod.validate_config(cfg))
    cfg.forward_map.deviation_row = "actual"
    cfg.forward_map.deviation_c = -1.0
    assert any("deviation_c" in p for p in cfgmod.validate_config(cfg))


# ─── C11（DOB。2026-10-18 段2q） ─────────────────────────────────────────


async def test_c11_sets_tq_from_coef_and_lag_and_prints_settings(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    cfg = _c10_cfg(tmp_path, candidate="C11")
    run = await _c10_run_obj(cfg, monkeypatch)
    # 遅れ 0.05 s × 係数 40 ＝ 2.0 s。C10 の τ（c=100 → 5.0 s）はそのまま
    assert run.ff.tq_accel_s == pytest.approx(2.0) and run.ff.tq_brake_s == pytest.approx(2.0)
    assert run.ff.tau_accel_s == pytest.approx(5.0)
    out = capsys.readouterr().out
    assert "C11（DOB）: Tq 係数 40・Tq アクセル 2.0s / ブレーキ 2.0s" in out
    assert "C10（偏差の上乗せ）" in out
    assert run.dob is not None and run.dob_stats is not None


async def test_c11_dob_part_is_separated_and_signs_are_right(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cfg = _c10_cfg(tmp_path, candidate="C11")
    run = await _c10_run_obj(cfg, monkeypatch)
    plan = run.ff.plan
    ia = int(np.flatnonzero(plan.effort_pct > 4.0 + 1.0)[0])
    t, e = float(plan.t_s[ia]), float(plan.effort_pct[ia])
    ref = 40.0
    # 推定 0 のときは C10 と同じで、DOB の分は 0
    d0 = run._c10_offset(t, ref, ref, e)  # noqa: SLF001
    assert run._dob_effort == 0.0  # noqa: SLF001
    assert d0 == 0.0
    # 表より加速していない（d<0）→ 踏み増す。合計の上乗せ ＝ DOB の分（偏差 0 なので P は 0）
    run.dob.d_accel = -0.3
    d1 = run._c10_offset(t, ref, ref, e)  # noqa: SLF001
    assert d1 > 0.0 and run._dob_effort == pytest.approx(d1)  # noqa: SLF001
    assert run.dob_stats.accel_effort_cycles == 1
    # 偏差があるときは、合計 ＝ P ＋ DOB。DOB の分だけが _dob_effort に入る
    d2 = run._c10_offset(t, ref, ref - 2.0, e)  # noqa: SLF001
    assert d2 > d1 and 0.0 < run._dob_effort < d2  # noqa: SLF001
    # ブレーキの行（減速が浅い d>0）→ 深く踏む（負）
    ib = int(np.flatnonzero((plan.effort_pct < -7.0 - 1.0) & (plan.flag != 4))[0])
    tb, eb = float(plan.t_s[ib]), float(plan.effort_pct[ib])
    run.dob.d_brake = 0.3
    db = run._c10_offset(tb, ref, ref, eb)  # noqa: SLF001
    assert db < 0.0 and run._dob_effort == pytest.approx(db)  # noqa: SLF001
    # 段2q ③ の修正: 推定を更新しない低速（停車速度以下）では使わない
    run.dob.d_accel = -0.3
    run._c10_offset(t, 0.0, 0.0, e)  # noqa: SLF001
    assert run._dob_effort == 0.0  # noqa: SLF001


async def test_c11_dob_holds_while_stopped_and_updates_while_moving(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """段2q ① `121218` の修正: 停車中の深いブレーキでは推定を更新しない（実測 0・表は強い減速で
    ブレーキの推定が積み上がっていた）。走っている間のブレーキでは更新する。"""
    cfg = _c10_cfg(tmp_path, candidate="C11")
    run = await _c10_run_obj(cfg, monkeypatch)
    brake = cfg.vehicle.min_brake_opening_pct + 10.0
    for k in range(200):  # 10 秒、停車のまま深いブレーキ
        run._dob_update(k * 0.05, 0.0, 0.0, 0.0, brake, False)  # noqa: SLF001
    assert run.dob.d_brake == 0.0 and run.dob.n_brake == 0
    for k in range(200, 400):  # 40 km/h で一定のまま（表は減速を言う → d > 0）
        run._dob_update(k * 0.05, 40.0, 40.0, 0.0, brake, False)  # noqa: SLF001
    assert run.dob.n_brake > 0 and run.dob.d_brake > 0.0


async def test_c11_run_logs_dob_columns_and_c10_leaves_them_blank(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    results = {}
    for cand in ("C11", "C10"):
        cfg = _c10_cfg(tmp_path, candidate=cand)
        mode = _mode([(0.0, 0.0), (0.6, 0.0), (2.0, 6.0), (3.0, 6.0), (3.6, 0.0), (4.0, 0.0)])

        async def fake_load_mode(
            _cfg: cfgmod.ResearchConfig, _name: str, _m: object = mode
        ) -> object:
            return _m

        monkeypatch.setattr(mdmod, "load_mode", fake_load_mode)
        setup = await mdmod.prepare_mode_drive(cfg, "dummy_mode")
        hw = await _held_stub(cfg)
        log = dlmod.SessionLog(cfg, hw, has_ref=True)
        log.start(dlmod.SECTION_MODE_DRIVE, "")
        result = await mdmod.run_mode_drive(hw, cfg, setup, log=log)
        await log.close()
        results[cand] = (result, log.csv_path)
    r11, csv11 = results["C11"]
    r10, csv10 = results["C10"]
    assert r10.dob is None and r10.dob_settings is None
    assert r11.dob is not None and r11.dob_settings is not None
    assert "Tq 係数 40" in r11.dob_settings and "更新" in r11.dob.summary()
    with csv11.open(newline="", encoding="utf-8") as f:
        rows11 = [r for r in csv.DictReader(f) if r["section"] == dlmod.SECTION_MODE_DRIVE]
    assert all(r["dob_accel_kmhs"] != "" and r["dob_effort_pct"] != "" for r in rows11)
    with csv10.open(newline="", encoding="utf-8") as f:
        rows10 = list(csv.DictReader(f))
    assert all(
        r["dob_accel_kmhs"] == "" and r["dob_brake_kmhs"] == "" and r["dob_effort_pct"] == ""
        for r in rows10
    )


# ─── C12（惰行の補助。2026-10-18 段2r） ──────────────────────────────────


async def test_c12_assists_in_coast_rows_and_logs_coast_effort(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """表の惰行の加速度（≒ −2.3 km/h/s）で基準を下げる区間は計画が惰行になる。実車速を基準と
    ずれる 30 km/h に固定したスタブで、偏差が 0.5 を超えた周期に
    補助が入り、coast_effort_pct に出る。C11 の行は空欄。"""
    results = {}
    for cand in ("C12", "C11"):
        cfg = _c10_cfg(tmp_path, candidate=cand)
        mode = _mode([(0.0, 0.0), (2.0, 0.0), (6.0, 40.0), (8.0, 40.0), (14.0, 26.0), (16.0, 26.0)])

        async def fake_load_mode(
            _cfg: cfgmod.ResearchConfig, _name: str, _m: object = mode
        ) -> object:
            return _m

        monkeypatch.setattr(mdmod, "load_mode", fake_load_mode)
        setup = await mdmod.prepare_mode_drive(cfg, "dummy_mode")
        assert np.count_nonzero(setup.ff.plan.flag == 3) > 0  # 惰行の行がある（FLAG_COAST）
        hw = await _held_stub(cfg)
        hw.can.speed_kmh = 30.0  # 基準が 40 → 26 の間は偏差 +10〜−4（動かない車速）
        log = dlmod.SessionLog(cfg, hw, has_ref=True)
        log.start(dlmod.SECTION_MODE_DRIVE, "")
        result = await mdmod.run_mode_drive(hw, cfg, setup, log=log)
        await log.close()
        results[cand] = (result, log.csv_path)
    r12, csv12 = results["C12"]
    r11, csv11 = results["C11"]
    assert r11.coast_stats is None and r11.coast_settings is None
    assert r12.coast_stats is not None and "入る 0.2 km/h" in (r12.coast_settings or "")
    st = r12.coast_stats
    assert st.accel_entries + st.brake_entries >= 1 and st.accel_s + st.brake_s > 0.0
    out = capsys.readouterr().out
    assert "C12（惰行の補助）:" in out
    with csv12.open(newline="", encoding="utf-8") as f:
        rows12 = [r for r in csv.DictReader(f) if r["section"] == dlmod.SECTION_MODE_DRIVE]
    vals = [float(r["coast_effort_pct"]) for r in rows12]
    # 補助した周期は min 以上の絶対開度が出る。この惰行の区間では片方のペダルだけ
    assert any(abs(v) >= min(cfg.vehicle.min_accel_opening_pct, 4.0) for v in vals)
    assert not (any(v > 0.0 for v in vals) and any(v < 0.0 for v in vals))
    with csv11.open(newline="", encoding="utf-8") as f:
        rows11 = list(csv.DictReader(f))
    assert all(r["coast_effort_pct"] == "" for r in rows11)


# ─── C13（予測器・車速のフィルタ。2026-10-19 段2u） ──────────────────────


async def _c13_run(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, candidate: str, *, enabled: bool = True
) -> tuple[mdmod.ModeDriveResult, list[dict[str, str]]]:
    """40 → 26 km/h に下げる基準を、30 km/h に固定したスタブ車速で走る（C12 の試験と同じ）。"""
    cfg = _c10_cfg(tmp_path, candidate=candidate)
    cfg.forward_map.predictor_enabled = enabled
    mode = _mode([(0.0, 0.0), (2.0, 0.0), (6.0, 40.0), (8.0, 40.0), (14.0, 26.0), (16.0, 26.0)])

    async def fake_load_mode(_cfg: cfgmod.ResearchConfig, _name: str) -> object:
        return mode

    monkeypatch.setattr(mdmod, "load_mode", fake_load_mode)
    setup = await mdmod.prepare_mode_drive(cfg, "dummy_mode")
    hw = await _held_stub(cfg)
    hw.can.speed_kmh = 30.0
    log = dlmod.SessionLog(cfg, hw, has_ref=True)
    log.start(dlmod.SECTION_MODE_DRIVE, "")
    result = await mdmod.run_mode_drive(hw, cfg, setup, log=log)
    await log.close()
    with log.csv_path.open(newline="", encoding="utf-8") as f:
        rows = [r for r in csv.DictReader(f) if r["section"] == dlmod.SECTION_MODE_DRIVE]
    return result, rows


async def test_c13_logs_filter_and_prediction_columns_and_stats(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    result, rows = await _c13_run(tmp_path, monkeypatch, "C13")
    assert result.predictor is not None and result.predictor_settings is not None
    assert "σ_a" in result.predictor_settings and "R:" in result.predictor_settings
    assert result.predictor.est_n > 100 and result.predictor.pred_used_cycles > 0
    assert "予測なし" in result.predictor.summary()
    out = capsys.readouterr().out
    assert "C13（予測器・車速のフィルタ" in out
    assert all(r["speed_est_kmh"] != "" and r["pred_dev_kmh"] != "" for r in rows)
    # 30 km/h は低速の行ではないので、予測が出る周期がある。停車保持の行（先頭）では空欄
    assert any(r["speed_pred_kmh"] != "" for r in rows)
    assert rows[0]["speed_pred_kmh"] == ""
    # 低速の行などで予測を使わない周期の偏差は今の偏差（基準 − CAN 車速）
    r0 = rows[0]
    assert float(r0["pred_dev_kmh"]) == pytest.approx(
        float(r0["ref_speed_kmh"]) - float(r0["actual_speed_kmh"]), abs=1e-3
    )


async def test_c13_with_predictor_disabled_gives_the_same_commands_as_c12(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    r13, rows13 = await _c13_run(tmp_path, monkeypatch, "C13", enabled=False)
    r12, rows12 = await _c13_run(tmp_path, monkeypatch, "C12")
    assert r13.predictor is None and r13.predictor_settings is None
    assert len(rows13) == len(rows12) > 100
    # スタブは実時間で動くので、同じ C12 を 2 回走っても指令は最大 0.5% ほどずれる。
    # 予測器が混ざれば（偏差が基準の L 秒先に変わる）これを超えて違う
    for key in ("accel_cmd_pct", "brake_cmd_pct", "c10_effort_pct", "dob_effort_pct"):
        diff = max(abs(float(a[key] or 0.0) - float(b[key] or 0.0))
                   for a, b in zip(rows13, rows12, strict=True))
        assert diff < 1.0, (key, diff)
    assert all(r["dob_accel_kmhs"] != "" for r in rows13)
    assert all(
        r["speed_est_kmh"] == "" and r["speed_pred_kmh"] == "" and r["pred_dev_kmh"] == ""
        for r in rows13 + rows12
    )


async def test_c13_prepare_shows_predictor_config(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    cfg = _c10_cfg(tmp_path, candidate="C13")
    cfg.forward_map.speed_filter_r = 2e-4
    mode = _mode(_C10_MODE_POINTS)

    async def fake_load_mode(_cfg: cfgmod.ResearchConfig, _name: str) -> object:
        return mode

    monkeypatch.setattr(mdmod, "load_mode", fake_load_mode)
    setup = await mdmod.prepare_mode_drive(cfg, "dummy_mode")
    assert isinstance(setup.ff, ff_candidate.PredictorFeedforward)
    out = capsys.readouterr().out
    assert "C13（予測器・車速のフィルタ）: 有効・Q pkl の meta から" in out
    assert "R yaml の上書き 0.0002" in out


async def test_c10_offset_uses_predicted_deviation_only_when_given(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """C13: `_pred_dev` があれば P の上乗せはその偏差から。None なら今の偏差（基準 − CAN 車速）で、
    C10・C11・C12（および predictor_enabled が false の C13）と同じ出力。"""
    cfg = _c10_cfg(tmp_path)
    run = await _c10_run_obj(cfg, monkeypatch)
    plan = run.ff.plan
    ia = int(np.flatnonzero(plan.effort_pct > 4.0 + 1.0)[0])
    t, e = float(plan.t_s[ia]), float(plan.effort_pct[ia])
    ref = 40.0
    plain = run._c10_offset(t, ref, ref - 2.0, e)  # noqa: SLF001
    run._pred_dev = 2.0  # noqa: SLF001  今の偏差と同じ値なら出力も同じ
    assert run._c10_offset(t, ref, ref - 2.0, e) == pytest.approx(plain)  # noqa: SLF001
    run._pred_dev = -1.0  # noqa: SLF001  予測では追いつく見込み → 減らす向き
    assert run._c10_offset(t, ref, ref - 2.0, e) < plain  # noqa: SLF001
    assert run._c10_offset(t, ref, ref - 2.0, e) < 0.0  # noqa: SLF001


def test_cut_deeper_brake_fb() -> None:
    """段6 C: ブレーキを FF より深くする向きの FB だけ 0。浅くする向き・アクセル側は通す。"""
    from tests.research.mode_drive import cut_deeper_brake_fb

    assert cut_deeper_brake_fb(-8.0, -2.0) == 0.0  # 深くする
    assert cut_deeper_brake_fb(0.0, -1.5) == 0.0  # FF が惰行でもブレーキを足さない
    assert cut_deeper_brake_fb(-8.0, 2.0) == 2.0  # 浅くする向きは通る
    assert cut_deeper_brake_fb(5.0, -2.0) == -2.0  # アクセル側は触らない
    assert cut_deeper_brake_fb(5.0, 3.0) == 3.0


# ─── 段8-a（基準 0 で動いたら止めて待つ）・段8-b（FB から A2 を外す。2026-10-24） ──────


def _hold(ki: float = 2.0, bk: float = 8.0, bmin: float = 4.0) -> LaunchMoveHold:
    return LaunchMoveHold(ki=ki, bk_pct=bk, brake_min_pct=bmin, dt_s=0.05)


def _go(h: LaunchMoveHold, prep: bool, v: float, ff: float, midway: bool = False) -> float:
    return h.step(prep=prep, speed_kmh=v, ff_effort=ff, midway=midway)


def test_launch_hold_freezes_ff_and_integrates_only_while_moving_deeper_only() -> None:
    h = _hold()
    assert _go(h, True, 0.0, -10.0) == 0.0 and h.frozen is None  # 動いていない: 何もしない
    assert _go(h, True, 0.05, -10.0) == 0.0  # 0.05 ちょうどは動いたと見ない
    # 動いた周期: その周期の FF（-9）で止め、I を足す（Ki × v × dt ＝ 2 × 1 × 0.05 ＝ 0.1）
    assert _go(h, True, 1.0, -9.0) == pytest.approx(0.1)
    assert h.frozen == -9.0 and h.integral == pytest.approx(0.1)
    # 計画がさらに離れても（-8）止めた値 ＋ I で出す（指令 ＝ -9.2 → 足す深さ ＝ 1.2）
    assert _go(h, True, 1.0, -8.0) == pytest.approx(1.2)
    # 止まっている間は I を足さずその値で待つ
    assert _go(h, True, 0.0, -8.0) == pytest.approx(1.2)
    assert h.integral == pytest.approx(0.2)
    # 計画が止めた値より深いなら計画に I を足す。足す量は常に 0 以上（浅くしない）
    assert _go(h, True, 0.0, -12.0) == pytest.approx(0.2)
    assert all(_go(h, True, v, -8.0) >= 0.0 for v in (0.0, 0.5, 3.0))


def test_launch_hold_resets_at_next_launch_and_ignores_accel_rows() -> None:
    h = _hold()
    _go(h, True, 1.0, -9.0)
    _go(h, False, 1.0, -9.0)  # 発進の連なりが終わる
    assert h.integral > 0.0
    assert _go(h, True, 0.0, -9.0) == 0.0  # 次の発進: 戻る
    assert h.frozen is None and h.integral == 0.0
    # アクセルの行・惰行の行には何もしない
    assert _go(h, True, 1.0, 5.0) == 0.0 and _go(h, True, 1.0, 0.0) == 0.0 and h.frozen is None


def test_launch_hold_hand_over_offset_applies_then_drops_at_min() -> None:
    h = _hold(ki=2.0, bk=8.0, bmin=4.0)
    _go(h, True, 1.0, -9.5)  # 止めた 9.5・I ＝ 0.1
    for _ in range(9):
        _go(h, True, 1.0, -9.0)  # I ＝ 1.0
    assert h.integral == pytest.approx(1.0)
    # 終わった周期: ずれ ＝ 9.5 ＋ 1.0 − 8.0 ＝ 2.5。発進の途中のブレーキの行に足す
    assert _go(h, False, 1.0, -8.0, midway=True) == pytest.approx(2.5)
    assert _go(h, False, 1.0, -6.0, midway=True) == pytest.approx(2.5)
    # 計画が min に戻ったらやめ、その後は足さない
    assert _go(h, False, 1.0, -4.0, midway=True) == 0.0
    assert _go(h, False, 1.0, -8.0, midway=True) == 0.0
    # 発進の途中でない行でも足さない／止めた値が bk 以下ならずれなし
    h2 = _hold()
    _go(h2, True, 1.0, -7.0)
    assert _go(h2, False, 1.0, -8.0, midway=True) == 0.0
    h3 = _hold()
    _go(h3, True, 1.0, -9.5)
    assert _go(h3, False, 1.0, -8.0, midway=False) == 0.0


class _GainTable:
    """launch_hold_gain 用の最小の表（発進側の加速度は u の直線: 傾き 0.8 km/h/s/%）。"""

    def __init__(self, slope: float, bk: float | None = -4.0) -> None:
        self.slope = slope
        self.meta: dict[str, float] = {} if bk is None else {"launch_breakaway_u_pct": bk}
        self.floor = True

    def with_launch_breakaway_floor(self, enabled: bool) -> _GainTable:
        self.floor = enabled
        return self

    def accel(self, v: float, u: float, *, launch: bool = False) -> float:
        assert launch and not self.floor  # A2 なしの発進側の表を引く
        return self.slope * u + 1.0


def test_launch_hold_gain_is_one_over_tau_squared_times_k() -> None:
    ki, k, note = launch_hold_gain(_GainTable(0.8), tau_brake_s=1.5)
    assert k == pytest.approx(0.8) and ki == pytest.approx(1.0 / (1.5**2 * 0.8))
    assert "Ki" in note
    # k ≦ 0・τ なし・meta なしは働かない（Ki は None）
    assert launch_hold_gain(_GainTable(-0.1), tau_brake_s=1.5)[0] is None
    assert launch_hold_gain(_GainTable(0.8), tau_brake_s=None)[0] is None
    assert launch_hold_gain(_GainTable(0.8), tau_brake_s=0.0)[0] is None
    assert launch_hold_gain(_GainTable(0.8, bk=None), tau_brake_s=1.5)[0] is None


def _plan_stub(prep: list[bool], midway: list[bool] | None = None) -> object:
    from types import SimpleNamespace

    n = len(prep)
    mid = None if midway is None else np.array(midway)
    return SimpleNamespace(
        launch_prep=np.array(prep), launch_midway=mid, t_s=np.arange(n) * 0.05,
        index_at=lambda t: int(np.clip(round(t / 0.05), 0, n - 1)),
        is_launch_midway=lambda i: bool(mid[i]),
    )


def _hold_run(cfg: cfgmod.ResearchConfig, hold: LaunchMoveHold | None, plan: object):
    from types import SimpleNamespace

    run = mdmod._ModeRun.__new__(mdmod._ModeRun)  # noqa: SLF001
    run.cfg = cfg
    run.ff = SimpleNamespace(plan=plan)
    run.launch_hold = hold
    run._launch_hold_pct = None if hold is None else 0.0  # noqa: SLF001
    return run


def test_launch_hold_step_off_is_identical_and_on_logs_added_depth(tmp_path: Path) -> None:
    cfg = _c7_cfg(tmp_path)
    plan = _plan_stub([True, True, True, False, False], [False, False, False, True, True])
    off = _hold_run(cfg, None, plan)
    assert off._launch_hold_step(0.1, 1.0, -9.0) == 0.0  # noqa: SLF001
    assert off._launch_hold_pct is None  # noqa: SLF001  CSV は空欄
    on = _hold_run(cfg, _hold(), plan)
    assert on._launch_hold_step(0.0, 1.0, -9.0) < 0.0  # noqa: SLF001  深くする向き（負の effort）
    assert on._launch_hold_pct == pytest.approx(0.1)  # noqa: SLF001
    # ブレーキの上限（max_brake）を超えない
    on2 = _hold_run(cfg, _hold(ki=1e6), plan)
    d = on2._launch_hold_step(0.0, 1.0, -9.0)  # noqa: SLF001
    assert -9.0 + d == pytest.approx(-cfg.vehicle.max_brake_opening_pct)


def test_make_launch_hold_needs_ramp_and_flag(tmp_path: Path) -> None:
    from types import SimpleNamespace

    cfg = _c7_cfg(tmp_path)
    ff = SimpleNamespace(table=_GainTable(0.8))
    assert mdmod.make_launch_hold(cfg, ff, _plan_stub([True]))[0] is None  # off
    cfg.forward_map.launch_move_hold = True
    plan = _plan_stub([True])
    plan.launch_prep = None
    hold, text = mdmod.make_launch_hold(cfg, ff, plan)
    assert hold is None and "働かない" in text
    cfg.forward_map.brake_lag_s = 0.2
    cfg.forward_map.deviation_c = 5.0
    hold, text = mdmod.make_launch_hold(cfg, ff, _plan_stub([True]))
    assert hold is not None and hold.ki == pytest.approx(1.0 / (1.0**2 * 0.8))
    assert hold.bk_pct == pytest.approx(cfg.vehicle.min_brake_opening_pct + 4.0)
    assert "Ki" in text
    ff_bad = SimpleNamespace(table=_GainTable(-1.0))
    assert "働かない" in mdmod.make_launch_hold(cfg, ff_bad, _plan_stub([True]))[1]


async def test_fb_no_breakaway_floor_lets_p_go_deeper_than_bk(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    import dataclasses

    from tests.research.fwd_plan import FLAG_MEASURED, deviation_offset_detail

    cfg = _c10_cfg(tmp_path)
    run = await _c10_run_obj(cfg, monkeypatch)
    tb = run.ff.table
    from types import SimpleNamespace

    run.ff = SimpleNamespace(table=dataclasses.replace(
        tb, launch_accel=tb.accel_grid + 12.0, launch_breakaway_floor=True,
        meta={**tb.meta, "launch_breakaway_u_pct": -4.0},
    ))
    run.fb_no_floor = False
    assert run._fb_table() is run.ff.table  # noqa: SLF001  off ＝ 同じ表
    kw = dict(
        effort=-15.0, flag=FLAG_MEASURED, need_kmhs=1.0, ref_kmh=0.3, row_kmh=0.3, dev_kmh=-1.0,
        tau_accel_s=0.3, tau_brake_s=0.3, accel_min_pct=4.0, brake_min_pct=7.0, side=1,
    )
    off, _ = deviation_offset_detail(run._fb_table(), **kw)  # noqa: SLF001
    run.fb_no_floor = True
    fb = run._fb_table()  # noqa: SLF001
    assert fb.launch_breakaway_floor is False and run.ff.table.launch_breakaway_floor is True
    on, _ = deviation_offset_detail(fb, **kw)
    assert off == pytest.approx(-15.0)  # A2 あり: bk（4.0）より深い解は頭打ち → P は何も足さない
    assert on < -15.0 - 1.0  # A2 なし: ref ≦ 0.5 km/h でも bk より深く踏み増せる


# --- 2026-10-26 段11-3: PedalTrack は pkl の meta（手順2 で測った値）を優先、無ければ yaml ---


def _pt_meta(**over: float) -> dict[str, float]:
    from tests.research.pedal_lag import PEDAL_TRACK_META_KEYS

    base = dict(zip(PEDAL_TRACK_META_KEYS, (0.01, 105.0, 1200.0, 0.01, 105.0, 1200.0, 0.125, 0.1),
                    strict=True))
    base.update(over)
    return {f"pedal_track_{k}": v for k, v in base.items()}


def test_pedal_track_of_prefers_meta_and_falls_back_to_yaml() -> None:
    from tests.research.config import ForwardMapSection
    from tests.research.mode_drive import _pedal_track_of, pedal_track_source

    fm = ForwardMapSection()
    meta = _pt_meta()
    t = _pedal_track_of(fm, meta)
    assert (t.accel_delay_s, t.accel_max_rate_pct_s, t.accel_max_accel_pct_s2) == (
        0.01, 105.0, 1200.0)
    assert (t.accel_effect_lag_s, t.brake_effect_lag_s) == (0.125, 0.1)
    assert "meta" in pedal_track_source(meta)
    # meta なし（古い pkl）・None は yaml（config の値）
    for m in (None, {}, {"accel_deadband_pct": 4.2}):
        t = _pedal_track_of(fm, m)
        assert t.accel_max_rate_pct_s == fm.pedal_track_accel_max_rate_pct_s
        assert t.brake_effect_lag_s == fm.pedal_track_brake_effect_lag_s
        assert "yaml" in pedal_track_source(m)
    # 1 つ欠ける・非有限・範囲外（速さ 0 以下・遅れ負）は meta 全体を使わない
    bad_missing = _pt_meta()
    del bad_missing["pedal_track_brake_effect_lag_s"]
    for bad in (bad_missing, _pt_meta(accel_max_rate_pct_s=0.0),
                _pt_meta(accel_delay_s=float("nan")), _pt_meta(brake_effect_lag_s=-0.1)):
        assert _pedal_track_of(fm, bad).accel_max_rate_pct_s == fm.pedal_track_accel_max_rate_pct_s


# --- 2026-10-27 段3 P3・段6 P1・段7 P2: FB（C10 の P）まわりの 3 つの機能（既定 off） ---


def test_cut_fb_at_ref_zero_only_cuts_deeper_brake_when_ref_is_stopped() -> None:
    stop = 0.02
    # 基準 0・ブレーキの行・偏差 + で P が深くする向き（負）→ 0
    assert mdmod.cut_fb_at_ref_zero(-8.6, -9.6, 0.0, stop) == 0.0
    # 浅くする向き（正）は通す
    assert mdmod.cut_fb_at_ref_zero(-8.6, +0.5, 0.0, stop) == 0.5
    # 基準が動いている行は通す（深くする向きでも）
    assert mdmod.cut_fb_at_ref_zero(-8.6, -9.6, 0.5, stop) == -9.6
    # アクセルの行（FF > 0）は触らない
    assert mdmod.cut_fb_at_ref_zero(3.0, -1.0, 0.0, stop) == -1.0


def test_cap_p_in_breakaway_band_caps_only_inside_band() -> None:
    kw = dict(speed_top_kmh=7.5, brake_min_pct=4.2, hold_pct=18.09, cap_pct=0.25)
    cap = mdmod.cap_p_in_breakaway_band
    # 帯の中（低速・FF ブレーキ 8.57%）: 両方向とも |delta| ≦ cap
    assert cap(-1.0, -8.57, 1.0, **kw) == -0.25
    assert cap(+5.0, -8.57, 1.0, **kw) == 0.25
    assert cap(0.1, -8.57, 1.0, **kw) == 0.1  # cap 以下はそのまま
    # 帯の外: 車速が上限以上・FF がアクセル・ブレーキ開度が min 未満・停車保持以上
    assert cap(-1.0, -8.57, 7.5, **kw) == -1.0
    assert cap(-1.0, 8.57, 1.0, **kw) == -1.0
    assert cap(-1.0, -3.0, 1.0, **kw) == -1.0
    assert cap(-1.0, -18.09, 1.0, **kw) == -1.0


def test_stop_end_monotone_brake_never_lets_go_inside_section() -> None:
    f = mdmod.stop_end_monotone_brake
    prev = None
    out = []
    for e in (-8.0, -9.0, -7.0, -10.0, -6.0):  # 区間の中: 踏み増すだけ
        prev = f(e, prev)
        out.append(prev)
    assert out == [-8.0, -9.0, -9.0, -10.0, -10.0]
    # 前の周期が区間外（None）なら離してよい・アクセル側（≥ 0）は触らない
    assert f(-5.0, None) == -5.0
    assert f(2.0, -9.0) == 2.0


def test_band_cap_of_uses_meta_step_and_reports_missing(tmp_path: Path) -> None:
    from types import SimpleNamespace

    cfg = _c7_cfg(tmp_path)
    ff = SimpleNamespace(table=SimpleNamespace(meta={
        "launch_u_step_pct": 0.25, "stop_max_speed_kmh": 7.5}))
    assert mdmod.band_cap_of(cfg, ff)[0] is None  # 既定 off
    cfg.forward_map.fb_breakaway_band_cap = True
    cap, top, text = mdmod.band_cap_of(cfg, ff)
    assert cap == 0.25 and top == 7.5 and "あり" in text
    cap, top, text = mdmod.band_cap_of(cfg, SimpleNamespace(table=SimpleNamespace(meta={})))
    assert cap is None and top == 7.5 and "働かない" in text  # meta に刻みなし → 止めて知らせる


def test_new_fb_features_default_off_and_run_defaults_are_identity() -> None:
    fm = cfgmod.ForwardMapSection()
    assert (fm.fb_no_deeper_at_ref_zero, fm.fb_breakaway_band_cap,
            fm.stop_end_brake_monotone) == (False, False, False)
    run = mdmod._ModeRun.__new__(mdmod._ModeRun)  # noqa: SLF001
    assert run.fb_ref0_no_deeper is False and run.band_cap_pct is None
    assert run.stop_end_monotone is False
    # 3 つと launch_midway_fb_no_deeper は同時に true にできる（検査に引っかからない）
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.forward_map.fb_no_deeper_at_ref_zero = True
    cfg.forward_map.fb_breakaway_band_cap = True
    cfg.forward_map.stop_end_brake_monotone = True
    cfg.forward_map.launch_midway_fb_no_deeper = True
    assert cfgmod.validate_config(cfg) == []


# ─── 停C の先読みの偏差（2026-10-31 段3 ProblemReport_20261031_停止間際 FB） ─────────


class _LinearRef:
    """時刻 t の基準 ＝ 30 − 3 × (t − 12)（減速区間 12〜16 s の途中、傾き −3）。"""

    def at(self, t: float) -> float:
        return 30.0 - 3.0 * (t - 12.0)


async def _stop_pred_run(
    cfg: cfgmod.ResearchConfig, monkeypatch: pytest.MonkeyPatch, enabled: bool
) -> mdmod._ModeRun:
    cfg.forward_map.stop_end_accel_pred_enabled = enabled
    cfg.forward_map.hold_pi_enabled = False  # この試験は保持の場面を使わない
    cfg.forward_map.hold_accel_pred_enabled = False
    run = await _c10_run_obj(cfg, monkeypatch)
    run.ff.table.meta["stop_max_speed_kmh"] = 40.0
    mode = _mode(_C10_MODE_POINTS)
    run.ff.set_stop_end(lead_enabled=True, p_enabled=True, lead_s=5.0)
    run.stop_end = mdmod.make_stop_end(run.ff, cfg, mode)
    run.c10 = mdmod.C10Stats(period_s=0.05)
    run.ref = run.ref_t = _LinearRef()
    run._stop_end_tau_s = None
    run._stop_pred_dev = None
    run._stop_pred_tp_s = mdmod.stop_end_pred_tp_s(cfg, run.ff) if enabled else None
    run._stop_speed_win = mdmod.deque()
    return run


def _feed_window(run: mdmod._ModeRun, t_end: float, accel: float, speed_end: float) -> None:
    """窓へ、t_end までの 0.05 s 刻みの車速（傾き accel）を入れる（本物の周期の更新と同じ作り）。"""
    w = run.cfg.decel_stop.slope_window_s
    n = int(round(w / 0.05))
    for k in range(n, -1, -1):
        tk = t_end - 0.05 * k
        run._stop_speed_win.append((tk, speed_end - accel * 0.05 * k, run.ref_t.at(tk)))


async def test_stop_end_pred_tp_is_brake_lag_plus_half_window(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cfg = _c10_cfg(tmp_path, candidate="C15")
    run = await _stop_pred_run(cfg, monkeypatch, True)
    assert mdmod.stop_end_pred_tp_s(cfg, run.ff) == pytest.approx(
        0.05 + cfg.decel_stop.slope_window_s / 2.0
    )
    cfg.forward_map.brake_lag_s = 0.2  # yaml の上書きが優先
    assert mdmod.stop_end_pred_tp_s(cfg, run.ff) == pytest.approx(
        0.2 + cfg.decel_stop.slope_window_s / 2.0
    )


async def test_stop_end_pred_on_changes_only_inside_section(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    cfg = _c10_cfg(tmp_path, candidate="C15")
    off = await _stop_pred_run(cfg, monkeypatch, False)
    on = await _stop_pred_run(cfg, monkeypatch, True)
    t_in, t_out = 14.0, 8.0  # 14 s: 基準 24 で 5 s 以内に 0 になる / 8 s: 5 s 以内に 0 にならない
    eb = float(off.ff.plan.effort_pct[off.ff.plan.index_at(t_in)])
    assert eb < 0.0
    speed = 22.0
    # (a) off: 窓があっても今の偏差のまま（_stop_pred_dev は None）
    _feed_window(off, t_in, -3.0, speed)
    d_off = off._c10_offset(t_in, off.ref.at(t_in), speed, eb)  # noqa: SLF001
    assert off._stop_pred_dev is None  # noqa: SLF001
    # (b) on・加速度が基準どおり（−3）: 先読みの偏差 ＝ 今の偏差 ＝ 同じ指令
    _feed_window(on, t_in, -3.0, speed)
    d_on = on._c10_offset(t_in, on.ref.at(t_in), speed, eb)  # noqa: SLF001
    tp = on._stop_pred_tp_s  # noqa: SLF001
    assert tp is not None and on._stop_end_tau_s is not None  # noqa: SLF001
    assert on._stop_pred_dev == pytest.approx(on.ref.at(t_in) - speed)  # noqa: SLF001
    assert d_on == pytest.approx(d_off)
    # 実測の加速度が基準より急（−4）なら、先読みの偏差は今の偏差より小さい（速すぎ→深く）
    on2 = await _stop_pred_run(cfg, monkeypatch, True)
    _feed_window(on2, t_in, -4.0, speed)
    on2._c10_offset(t_in, on2.ref.at(t_in), speed, eb)  # noqa: SLF001
    assert on2._stop_pred_dev == pytest.approx(  # noqa: SLF001
        on2.ref.at(t_in) - speed + tp  # 基準 −3・実測 −4: 遅すぎ側（正）
    )
    # 2026-10-31 段3d: 予測車速が 0 以下になる入力（0.1 + (-3)*tp < 0）は先読みせず
    # 今の偏差（基準 ref_t(t) − 車速）
    on4 = await _stop_pred_run(cfg, monkeypatch, True)
    _feed_window(on4, t_in, -3.0, 0.1)
    on4._c10_offset(t_in, on4.ref.at(t_in), 0.1, eb)  # noqa: SLF001
    assert on4._stop_pred_dev == pytest.approx(on4.ref_t.at(t_in) - 0.1)  # noqa: SLF001
    # 窓が 3 点未満なら今の偏差
    on3 = await _stop_pred_run(cfg, monkeypatch, True)
    on3._stop_speed_win.append((t_in, speed, on3.ref_t.at(t_in)))  # noqa: SLF001
    assert on3._c10_offset(t_in, on3.ref.at(t_in), speed, eb) == pytest.approx(d_off)  # noqa: SLF001
    assert on3._stop_pred_dev is None  # noqa: SLF001
    # (c) 区間外（τ を引き直さない行）は今のまま・先読みを使わない
    eo = float(on.ff.plan.effort_pct[on.ff.plan.index_at(t_out)])
    on4 = await _stop_pred_run(cfg, monkeypatch, True)
    _feed_window(on4, t_out, -4.0, 30.0)
    on4._c10_offset(t_out, on4.ref.at(t_out), 30.0, eo)  # noqa: SLF001
    assert on4._stop_pred_dev is None  # noqa: SLF001
    assert on4._stop_end_tau_s is None  # noqa: SLF001
