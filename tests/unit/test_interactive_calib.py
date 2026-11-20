# ruff: noqa: E501
"""段11: キャリブレーションで max を測る（src/robot/interactive.py の calib 上限・save_max）と、ログの分析（src/web/analysis.py）。"""

from __future__ import annotations

import asyncio
import io
from pathlib import Path
from types import SimpleNamespace

import pytest

from src.robot import config_store as cs
from src.robot import events
from src.robot.core import hardware as core_hardware
from src.robot.interactive import (
    CALIB_STROKE_LIMIT_PCT,
    CommandError,
    InteractiveSession,
    Limits,
    parse_command,
    plan_move,
)
from src.web.analysis import build_analysis

LIM = Limits(min_accel=6.0, max_accel=60.0, min_brake=3.0, max_brake=40.0)


def test_parse_calib_flag_and_save_max() -> None:
    assert parse_command('{"cmd":"jog","axis":"accel","amount_pct":1}').calib is False
    assert parse_command('{"cmd":"jog","axis":"accel","amount_pct":1,"calib":true}').calib is True
    assert parse_command('{"cmd":"move","axis":"brake","target_pct":9,"calib":true}').calib is True
    c = parse_command('{"cmd":"save_max","axis":"brake"}')
    assert c.cmd == "save_max" and c.axis == "brake"
    with pytest.raises(CommandError):
        parse_command('{"cmd":"save_max","axis":"both"}')


def test_calib_limits_widen_max_only_for_calibration() -> None:
    cl = LIM.for_calib()
    assert cl.max_accel == cl.max_brake == CALIB_STROKE_LIMIT_PCT == 99.0
    assert (cl.min_accel, cl.min_brake) == (6.0, 3.0)
    assert plan_move("accel", 99.5, 6.0, 3.0, LIM).target_pct == 60.0  # 手動運転は今までどおり max
    p = plan_move("accel", 99.5, 6.0, 3.0, cl)
    assert p.target_pct == 99.0 and p.clamped
    # 片足ルールは calib でも同じ（ブレーキが min を超えていれば先に戻す）
    p = plan_move("accel", 70.0, 6.0, 15.0, cl)
    assert p.moves == (("brake", 3.0), ("accel", 70.0)) and p.released_other


YAML = """\
vehicle:
  name: car_a
  min_accel_opening_pct: 6.5
  max_accel_opening_pct: 60.0   # アクセルの max
  min_brake_opening_pct: 2.9
  max_brake_opening_pct: 40.0
"""


async def test_calib_jog_goes_beyond_max_and_save_max_writes_yaml(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    vdir = tmp_path / "vehicles" / "car_a"
    vdir.mkdir(parents=True)
    (vdir / "vehicle.yaml").write_text(YAML, encoding="utf-8")
    common = tmp_path / "c.yaml"
    common.write_text("modes:\n  wltp_mode_name: x\n", encoding="utf-8")
    cfg = cs.load_config(common)
    cfg.vehicle.min_accel_opening_pct, cfg.vehicle.max_accel_opening_pct = 6.5, 60.0
    cfg.vehicle.min_brake_opening_pct, cfg.vehicle.max_brake_opening_pct = 2.9, 40.0
    hw = core_hardware.build_hardware(cfg, core_hardware.HW_STUB)
    await asyncio.gather(hw.accel.connect(), hw.brake.connect(), hw.can.connect())
    sess = InteractiveSession(hw, cfg, vehicle_id="car_a", vehicles_dir=tmp_path / "vehicles")
    sink = io.StringIO()
    events.set_sink(events.EventSink(stream=sink))
    monkeypatch.setattr("src.robot.interactive.SETTLE_S", 0.0)
    try:
        # 手動運転（calib なし）は max 60 で頭打ち
        await sess.execute(parse_command('{"cmd":"move","axis":"accel","target_pct":90}'))
        assert round((await sess.read_openings())[0], 1) == 60.0
        # calib は 99% まで
        await sess.execute(parse_command('{"cmd":"move","axis":"accel","target_pct":150,"calib":true}'))
        assert round((await sess.read_openings())[0], 1) == 99.0
        # ブレーキを動かすと先にアクセルを min へ戻す（片足ルール）
        await sess.execute(parse_command('{"cmd":"move","axis":"brake","target_pct":45,"calib":true}'))
        a, b = await sess.read_openings()
        assert round(a, 1) == 6.5 and round(b, 1) == 45.0
        # save_max: 今の位置を vehicle.max_*_opening_pct へ（コメントは保つ）
        await sess.execute(parse_command('{"cmd":"save_max","axis":"brake"}'))
        text = (vdir / "vehicle.yaml").read_text(encoding="utf-8")
        assert "max_brake_opening_pct: 45.0" in text and "# アクセルの max" in text
        assert cfg.vehicle.max_brake_opening_pct == 45.0
        # min + 1% 未満（min の位置）は max にできない
        await sess.execute(parse_command('{"cmd":"home"}'))
        await sess.execute(parse_command('{"cmd":"move","axis":"accel","target_pct":6.5}'))
        await sess.execute(parse_command('{"cmd":"save_max","axis":"accel"}'))
        assert "max_accel_opening_pct: 60.0" in (vdir / "vehicle.yaml").read_text(encoding="utf-8")
    finally:
        events.set_sink(None)
    acks = [e for e in (events.decode_event_line(ln) for ln in sink.getvalue().splitlines()) if e and e["type"] == "ack"]
    assert acks[-1]["ok"] is False and "max にできません" in acks[-1]["message"]
    assert any("max_brake_opening_pct=45" in a["message"] for a in acks)
    capsys.readouterr()


# ── ログの分析 ────────────────────────────────────────────────


def _rows(n: int = 300, section: str = "MODE_DRIVE") -> list:
    out = []
    for i in range(n):
        t = i * 0.1
        ref = 30.0 + 10.0 * (1 if (i // 50) % 2 else -1) * 0.0 + (i % 50) * 0.1
        act = ref + (0.9 if i == 40 else 0.0) + (-0.1)
        seg = "Low" if i < 150 else "Mid"
        detail = {"section": section, "mode_time_s": str(t), "pattern": seg}
        out.append(SimpleNamespace(ref_speed_kmh=ref, actual_speed_kmh=act, detail=detail))
    return out


def test_build_analysis_kpi_sections_and_drive_index() -> None:
    a = build_analysis(_rows(), mass_kg=1500.0, a0=100.0, a1=0.5, a2=0.03, segment_names=["Low", "Mid", "High", "ExHi"])
    assert a["available"] is True
    k = a["kpi"]
    assert k["max"] == 0.8 and k["over_0_5_s"] == 0.1 and k["over_1_s"] == 0.0 and k["mean"] == pytest.approx(-0.097, abs=0.001)
    assert [s["name"] for s in a["sections"]] == ["Low", "Mid"]
    assert sum(s["share"] for s in a["sections"]) == pytest.approx(1.0)
    labels = [r["label"] for r in a["drive_index"]["rows"]]
    assert labels == ["Low", "Mid", "合計"]
    total = a["drive_index"]["rows"][-1]
    assert all(total[key] is not None for key in ("eer_pct", "iwr_pct", "ascr_pct"))
    assert "chart" not in a


def test_build_analysis_without_mass_has_no_eer_and_non_wltp_has_no_sections() -> None:
    a = build_analysis(_rows(), mass_kg=None, a0=None, a1=None, a2=None, segment_names=["High"])
    assert a["sections"] == []
    total = a["drive_index"]["rows"][-1]
    assert total["eer_pct"] is None and total["iwr_pct"] is not None


def test_build_analysis_unavailable_without_mode_drive_rows() -> None:
    assert build_analysis(_rows(section="PATTERN_DRIVE"), mass_kg=None, a0=None, a1=None, a2=None) == {"available": False}
    assert build_analysis([], mass_kg=None, a0=None, a1=None, a2=None) == {"available": False}
