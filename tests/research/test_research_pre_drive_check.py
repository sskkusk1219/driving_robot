"""研究開発用ハーネス 走行前チェック（手順 1 と手順 2 の間）のユニットテスト。

スタブ車両はクリープ中（約 5 km/h）から始まる。ブレーキを一気に踏まず 1 刻みずつ踏み、
停止を確認した位置で保持すること、本番 PreCheckRunner の NG で走行に進まないことを確かめる。
時間を縮めるため、刻みを 1mm・待ちを 0.3s にしている（判定ロジックは既定と同じ）。
"""

from __future__ import annotations

from pathlib import Path

import pytest

from src.domain.control.conversions import VEHICLE_STOP_SPEED_KMH
from tests.research import config as cfgmod
from tests.research import hardware as hwmod
from tests.research import main as mainmod
from tests.research import pedal_search as psmod
from tests.research import pre_drive_check as pcmod
from tests.research.vehicle import opening_to_pulse, pulse_to_opening

FAST = {
    "pedal_search.step_mm": 1.0,
    "pedal_search.dwell_s": 0.3,
    "pedal_search.onset_margin_kmh": 0.05,  # スタブは車速ノイズが無い
}


def _tmp_cfg(tmp_path: Path, **extra: float) -> cfgmod.ResearchConfig:
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    cfg.save(
        {
            "output.results_dir": str(tmp_path / "results"),
            "feedforward.model_path": str(tmp_path / "results" / "models" / "ff.pkl"),
            "output.plot": False,
            **FAST,
            **extra,
        }
    )
    return cfgmod.load_config(path)


async def _ready_stub(cfg: cfgmod.ResearchConfig) -> hwmod.ResearchHardware:
    hw = hwmod.build_hardware(cfg, hwmod.HW_STUB)
    await hwmod.run_initialize(hw)
    # 停車までを短くする（既定ゲインだとクリープの押しとほぼ釣り合い、止まるまで十数秒かかる）
    hw.can.vehicle.brake_gain_kmhs_per_pct = 1.5  # type: ignore[attr-defined]
    return hw


def _record_moves(axis: hwmod.StubActuator) -> list[int]:
    moves: list[int] = []
    original = axis.move_to_position

    async def record(pos: int, *, smooth_over_s: float | None = None) -> None:
        moves.append(pos)
        await original(pos, smooth_over_s=smooth_over_s)

    axis.move_to_position = record  # type: ignore[method-assign]
    return moves


def _record_timed_moves(axis: hwmod.StubActuator) -> list[tuple[int, int]]:
    moves: list[tuple[int, int]] = []
    original = axis.move_to_position_timed

    async def record(target_pos: int, current_pos: int, duration_s: float) -> None:
        moves.append((target_pos, current_pos))
        await original(target_pos, current_pos, duration_s)

    axis.move_to_position_timed = record  # type: ignore[method-assign]
    return moves


async def test_brakes_in_steps_until_stopped(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = await _ready_stub(cfg)
    assert await hw.can.read_speed() > 1.0  # クリープ中から始まる
    moves = _record_moves(hw.brake)  # type: ignore[arg-type]

    stop_pos = await pcmod.run_pre_drive_check(hw, cfg)

    # 一気に踏まず、1 刻みずつ停止確認の位置まで踏んでいる
    step = psmod.search_step_pulse(cfg)
    assert moves == list(range(step, stop_pos + 1, step))
    assert pulse_to_opening(stop_pos) > hwmod.STUB_BRAKE_PLAY_PCT  # 遊びを越えた位置で止まった
    assert stop_pos < opening_to_pulse(cfg.pedal_search.brake_max_pct)
    # その位置で保持したまま停車している
    assert hw.brake.position == stop_pos
    assert hw.accel.position == 0
    assert await hw.can.read_speed() < VEHICLE_STOP_SPEED_KMH
    await hwmod.shutdown(hw)


async def test_already_stopped_is_confirmed_without_pressing(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path, **{"feedforward.creep_rate_kmhs": 0.0})
    hw = await _ready_stub(cfg)
    hw.can.speed_kmh = 0.0  # type: ignore[attr-defined]
    moves = _record_moves(hw.brake)  # type: ignore[arg-type]

    assert await pcmod.run_pre_drive_check(hw, cfg) == 0
    assert moves == []
    await hwmod.shutdown(hw)


async def test_fails_when_brake_limit_reached_without_stopping(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path, **{"pedal_search.brake_max_pct": 5.0})  # スタブの遊び 8% より浅い
    hw = await _ready_stub(cfg)
    with pytest.raises(hwmod.PreDriveCheckError, match="ブレーキを 5% まで踏んでも停車しません"):
        await pcmod.run_pre_drive_check(hw, cfg)
    await hwmod.shutdown(hw)


async def test_brakes_directly_to_target_when_next_step_is_not_2(tmp_path: Path) -> None:
    """次の手順が 2 以外なら、停車ブレーキ位置(stop_brake_opening_pct)は判明済みとして扱い、
    段階的に探らず一気に踏む。"""
    cfg = _tmp_cfg(tmp_path)
    hw = await _ready_stub(cfg)
    assert await hw.can.read_speed() > 1.0  # クリープ中から始まる
    stepwise_moves = _record_moves(hw.brake)  # type: ignore[arg-type]
    timed_moves = _record_timed_moves(hw.brake)  # type: ignore[arg-type]
    target = opening_to_pulse(cfg.feedforward.stop_brake_opening_pct)

    stop_pos = await pcmod.run_pre_drive_check(hw, cfg, next_step=3)

    assert stop_pos == target
    assert stepwise_moves == []  # 段階踏み（手順 2 用の分岐）は使わない
    assert timed_moves == [(target, 0)]  # 一気に 1 回だけ
    assert hw.brake.position == stop_pos
    assert hw.accel.position == 0
    assert await hw.can.read_speed() < VEHICLE_STOP_SPEED_KMH
    await hwmod.shutdown(hw)


async def test_direct_press_fails_when_target_not_enough_to_stop(tmp_path: Path) -> None:
    """stop_brake_opening_pct まで踏んでも停車しなければ走行前チェックエラーにする
    （段階探索へフォールバックはしない。手順 2 をやり直すよう促す）。"""
    cfg = _tmp_cfg(tmp_path, **{"feedforward.stop_brake_opening_pct": 5.0})  # 遊び 8% より浅い
    hw = await _ready_stub(cfg)
    with pytest.raises(
        hwmod.PreDriveCheckError,
        match=r"stop_brake_opening_pct（5\.00%）まで踏んでも停車しません",
    ):
        await pcmod.run_pre_drive_check(hw, cfg, next_step=3)
    await hwmod.shutdown(hw)


async def test_pre_check_ng_stops_before_pressing_brake(tmp_path: Path) -> None:
    """踏込前チェック（本番 PreCheckRunner）が NG ならブレーキを踏まない。"""
    cfg = _tmp_cfg(tmp_path)
    hw = await _ready_stub(cfg)
    hw.accel.position = 500  # type: ignore[attr-defined]
    moves = _record_moves(hw.brake)  # type: ignore[arg-type]

    with pytest.raises(hwmod.PreDriveCheckError, match="踏込前チェック NG: アクチュエータ位置"):
        await pcmod.run_pre_drive_check(hw, cfg)
    assert moves == []
    await hwmod.shutdown(hw)


async def test_post_check_ng_when_ups_drops(tmp_path: Path) -> None:
    """踏込後チェックも本番 PreCheckRunner で判定する（停止後に UPS 残量が落ちた場合など）。

    既定 YAML は開発時に UPS を使わないため checks.init_ups/pre_ups が false になっている。
    この判定を確かめるにはどちらも true に戻す必要がある。
    """
    cfg = _tmp_cfg(tmp_path, **{"checks.init_ups": True, "checks.pre_ups": True})
    hw = await _ready_stub(cfg)
    original = pcmod.brake_until_stopped

    async def stop_then_ups_low(hw_: hwmod.ResearchHardware, cfg_: cfgmod.ResearchConfig) -> int:
        pos = await original(hw_, cfg_)
        hw_.ups.battery_pct = 10.0  # type: ignore[attr-defined]
        return pos

    pcmod.brake_until_stopped = stop_then_ups_low  # type: ignore[assignment]
    try:
        with pytest.raises(hwmod.PreDriveCheckError, match="踏込後チェック NG: UPS残量"):
            await pcmod.run_pre_drive_check(hw, cfg)
    finally:
        pcmod.brake_until_stopped = original  # type: ignore[assignment]
    await hwmod.shutdown(hw)


async def test_pre_ups_false_ignores_low_battery(tmp_path: Path) -> None:
    """checks.pre_ups: false（既定 YAML）なら UPS 残量が閾値未満でも走行前チェックを通過する。"""
    cfg = _tmp_cfg(tmp_path)
    hw = await _ready_stub(cfg)
    hw.ups.battery_pct = 0.0  # type: ignore[attr-defined]

    stop_pos = await pcmod.run_pre_drive_check(hw, cfg)

    assert hw.brake.position == stop_pos
    await hwmod.shutdown(hw)


async def test_pre_brake_stop_false_skips_pressing(tmp_path: Path) -> None:
    """checks.pre_brake_stop: false ならブレーキを踏まず、現在位置のまま踏込後チェックへ進む。"""
    cfg = _tmp_cfg(
        tmp_path, **{"checks.pre_brake_stop": False, "feedforward.creep_rate_kmhs": 0.0}
    )
    hw = await _ready_stub(cfg)
    hw.can.speed_kmh = 0.0  # type: ignore[attr-defined]  # 車速確認 NG にならないよう静止させておく
    moves = _record_moves(hw.brake)  # type: ignore[arg-type]

    stop_pos = await pcmod.run_pre_drive_check(hw, cfg)

    assert moves == []  # ブレーキ指令は一切出していない
    assert stop_pos == 0  # 原点のまま
    await hwmod.shutdown(hw)


# ── CLI 経由 ─────────────────────────────────────────────────────────


def test_pre_drive_check_runs_between_step1_and_step2(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    calls: list[object] = []

    async def fake_check(hw: object, cfg: object, **kwargs: object) -> int:
        calls.append(hw)
        assert kwargs.get("next_step") == 2  # 次が手順 2 なので段階踏みの分岐で呼ばれる
        return 0

    async def failing_search(hw: object, cfg: object, **kwargs: object) -> None:
        raise hwmod.DriveError("テスト用の探索失敗")

    cfg = _tmp_cfg(tmp_path)  # 走行ログを tmp に書く
    monkeypatch.setattr(mainmod, "run_pre_drive_check", fake_check)
    monkeypatch.setattr(mainmod, "run_pedal_search", failing_search)
    assert mainmod.main(["--steps", "1,2", "--config", str(cfg.source_path)]) == 5
    out = capsys.readouterr().out
    assert len(calls) == 1
    # 手順 1 → 走行前チェック → 手順 2 の順
    order = [
        out.index("手順 1 完了"),
        out.index("走行前チェック完了"),
        out.index("手順 2: ペダル探索"),
    ]
    assert order == sorted(order)


def test_pre_drive_check_error_exit_code_is_7(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    async def failing_check(hw: object, cfg: object, **kwargs: object) -> int:
        raise hwmod.PreDriveCheckError("テスト用の NG")

    cfg = _tmp_cfg(tmp_path)  # 走行ログを tmp に書く
    monkeypatch.setattr(mainmod, "run_pre_drive_check", failing_check)
    assert mainmod.main(["--steps", "1,2", "--config", str(cfg.source_path)]) == 7
    out = capsys.readouterr().out
    assert "走行前チェックエラー: テスト用の NG" in out
    assert "手順 2: ペダル探索" not in out  # 走行に進まない
    assert "終了処理: 完了" in out


def test_dry_run_lists_pre_drive_check_before_step2(capsys: pytest.CaptureFixture[str]) -> None:
    assert mainmod.main(["--steps", "1,2", "--dry-run"]) == 0
    out = capsys.readouterr().out
    assert out.index("手順 1:") < out.index("(dry-run) 走行前チェック") < out.index("手順 2:")
