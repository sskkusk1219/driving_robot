"""研究開発用ハーネス 手順 2-0（ペダル探索: クリープ車速 → 停車保持開度）のユニットテスト。

2026-10-08 段2l-6: 不感帯の探索（アクセル・ブレーキの反応判定）はやめた。両ペダルを min
（vehicle.min_*_opening_pct）に置き、クリープ車速が落ち着くのを待ち、ブレーキを min から刻んで
踏んで停止確認開度を測り、+ マージンで停車保持することを確かめる。
スタブ車両は「真の遊び」（アクセル 6%・ブレーキ 8%）を持つ（min はその手前の 2.9% を既定にする）。
時間を縮めるため、刻みを 1mm・待ちを 0.3s にしている（判定ロジックは既定と同じ）。
"""

from __future__ import annotations

import asyncio
from pathlib import Path

import pytest

from tests.research import config as cfgmod
from tests.research import hardware as hwmod
from tests.research import pedal_search as psmod
from tests.research.research_types import VEHICLE_STOP_SPEED_KMH
from tests.research.vehicle import build_vehicle_profile, opening_to_pulse, pulse_to_opening

WINDOW_S = 0.6
FAST = {
    "pedal_search.step_mm": 1.0,
    "pedal_search.brake_step_mm": 1.0,  # 既定 0.1mm だとテストが遅くなるため、粗く
    "pedal_search.dwell_s": 0.3,
    "pedal_search.onset_margin_kmh": 0.05,  # スタブは車速ノイズが無い
    "pedal_search.creep_settle_kmhs": 0.017,
    "pedal_search.creep_settle_min_s": 0.0,  # 実時間で待たない（FAST の趣旨）
    "pedal_search.creep_timeout_s": 10.0,
    "pedal_search.stop_hold_margin_pct": 3.0,
    "feedforward.creep_rate_kmhs": 3.0,  # クリープ速度まで早く上げる
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
    return hw


async def test_search_starts_from_min_finds_stop_and_holds(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = await _ready_stub(cfg)
    # 停車までを短くする（既定ゲインだとクリープの押しとほぼ釣り合い、止まるまで十数秒かかる）
    hw.can.vehicle.brake_gain_kmhs_per_pct = 1.5  # type: ignore[attr-defined]
    yaml_before = cfg.source_path.read_text(encoding="utf-8")
    visited_accel: list[int] = []
    visited_brake: list[int] = []
    for axis, visited in ((hw.accel, visited_accel), (hw.brake, visited_brake)):
        original = axis.move_to_position

        async def record(pos, *, smooth_over_s=None, _o=original, _v=visited):  # type: ignore[no-untyped-def]
            _v.append(pos)
            await _o(pos, smooth_over_s=smooth_over_s)

        axis.move_to_position = record  # type: ignore[method-assign]

    result = await psmod.run_pedal_search(hw, cfg, window_s=WINDOW_S)

    # 両ペダルはまず min（待機位置）へ。アクセルはその後動かさない
    assert visited_accel == [opening_to_pulse(cfg.vehicle.min_accel_opening_pct)]
    assert visited_brake[0] == opening_to_pulse(cfg.vehicle.min_brake_opening_pct)
    assert all(p >= visited_brake[0] for p in visited_brake)  # ブレーキは min から上へ
    # 停止確認は遊び（8%）を越えた位置。刻みは粗い（1mm ≒ 1.05%）ので数刻み遅れうる
    step_pct = pulse_to_opening(psmod.search_step_pulse(cfg))
    assert hwmod.STUB_BRAKE_PLAY_PCT < result.stop_confirm_pct
    assert result.stop_confirm_pct <= hwmod.STUB_BRAKE_PLAY_PCT + 12 * step_pct
    # 停車保持 = 停止確認 + マージン。その位置まで踏んで停車している
    assert result.stop_brake_opening_pct == pytest.approx(result.stop_confirm_pct + 3.0, abs=0.01)
    assert hw.brake.position == opening_to_pulse(result.stop_brake_opening_pct)
    assert hw.accel.position == opening_to_pulse(cfg.vehicle.min_accel_opening_pct)
    assert await hw.can.read_speed() < VEHICLE_STOP_SPEED_KMH
    # 結果は不感帯を持たない（クリープ車速・停止確認・停車保持開度＋書き戻し用のブレーキ min）
    assert set(result.__dataclass_fields__) == {
        "creep_speed_kmh", "stop_confirm_pct", "stop_brake_opening_pct", "brake_min_pct"
    }
    # スタブは YAML を書き換えない
    assert cfg.source_path.read_text(encoding="utf-8") == yaml_before
    await hwmod.shutdown(hw)


async def test_search_fails_without_creep(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path, **{"feedforward.creep_rate_kmhs": 0.0,
                                "pedal_search.creep_timeout_s": 1.0})
    hw = await _ready_stub(cfg)
    hw.can.speed_kmh = 0.0  # type: ignore[attr-defined]  # 停車から始める（クリープしない車）
    with pytest.raises(hwmod.DriveError, match="クリープ"):
        await psmod.run_pedal_search(hw, cfg, window_s=WINDOW_S)
    await hwmod.shutdown(hw)


async def test_search_fails_when_brake_does_not_stop_within_limit(tmp_path: Path) -> None:
    """vehicle.max_brake_opening_pct まで踏んでも停車しなければ DriveError（スタブの遊びは 8%）。"""
    cfg = _tmp_cfg(tmp_path, **{"vehicle.max_brake_opening_pct": 6.0})
    hw = await _ready_stub(cfg)
    with pytest.raises(
        hwmod.DriveError,
        match="ブレーキを開度上限 6%（vehicle.max_brake_opening_pct）まで踏んでも停車しません",
    ):
        await psmod.run_pedal_search(hw, cfg, window_s=WINDOW_S)
    await hwmod.shutdown(hw)


class _ScriptedCAN:
    """あらかじめ用意した車速の列を、呼ばれた順に1つずつ返すスタブ CAN。"""

    def __init__(self, speeds: list[float]) -> None:
        self._speeds = list(speeds)

    async def read_speed(self) -> float:
        return self._speeds.pop(0)


def _scripted_hw(speeds: list[float]) -> hwmod.ResearchHardware:
    return hwmod.ResearchHardware(
        accel=hwmod.StubActuator("accel", connected=True),
        brake=hwmod.StubActuator("brake", connected=True),
        can=_ScriptedCAN(speeds),  # type: ignore[arg-type]
        ups=None,  # type: ignore[arg-type]
        hw_mode=hwmod.HW_STUB,
    )


# ── クリープ安定判定（傾き＋最短時間。2026-09-20） ──────────────────────
#
# 実機ログ（drive_log_real_20260920_042349.csv）で、3s 平均が
# 4.79 → 4.88 → 4.93 → 4.96 → 4.98 km/h と 17.9 秒かけて漸近する途中の 4.88 km/h で
# 「連続する2窓平均の差 < 閾値」判定が確定していた（真の平衡 5.00）。傾き＋最短時間に
# 変えたことで、確定を遅らせられることを確かめる。


async def test_wait_creep_stable_uses_config_window_s(monkeypatch: pytest.MonkeyPatch) -> None:
    """window_s を渡さなければ cfg.pedal_search.creep_window_s が使われること。"""
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.pedal_search.creep_window_s = 0.2
    cfg.pedal_search.creep_settle_min_s = 0.0
    cfg.pedal_search.creep_settle_kmhs = 0.06
    # 窓 0.2s は SPEED_SAMPLE_INTERVAL_S=0.1s 間隔で3サンプル/窓（既存テストの dwell_s=0.2 と同じ）
    speeds = [5.0, 5.0, 5.0, 5.01, 5.01, 5.01]
    hw = _scripted_hw(speeds)
    seen: list[float] = []
    orig_mean_speed = psmod.mean_speed

    async def spy_mean_speed(hw_: hwmod.ResearchHardware, duration_s: float) -> float:
        seen.append(duration_s)
        return await orig_mean_speed(hw_, duration_s)

    monkeypatch.setattr(psmod, "mean_speed", spy_mean_speed)

    mean = await psmod.wait_creep_stable(hw, cfg)

    assert seen[0] == pytest.approx(0.2)
    assert mean == pytest.approx(5.01)


async def test_wait_creep_stable_waits_for_settle_min_s() -> None:
    """傾きが最初から閾値未満でも、creep_settle_min_s に達するまで確定しないこと。"""
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.pedal_search.creep_window_s = 0.2
    cfg.pedal_search.creep_settle_kmhs = 0.5  # 緩め: 傾き0はすぐ条件を満たす
    cfg.pedal_search.creep_settle_min_s = 0.5  # 最短時間ガード
    cfg.pedal_search.creep_timeout_s = 5.0
    # 傾き0（一定車速）を3窓分用意。1・2窓目は傾き条件を満たしても最短時間未達で確定しない
    speeds = [5.0] * 9
    hw = _scripted_hw(speeds)

    loop = asyncio.get_running_loop()
    started = loop.time()
    mean = await psmod.wait_creep_stable(hw, cfg)
    elapsed = loop.time() - started

    assert mean == pytest.approx(5.0)
    assert elapsed >= 0.5


async def test_wait_creep_stable_settles_on_slope() -> None:
    """最短時間を過ぎて傾きがしきい値を割ったら確定し、そのときの窓平均を返すこと。"""
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.pedal_search.creep_window_s = 0.2
    cfg.pedal_search.creep_settle_kmhs = 0.05
    cfg.pedal_search.creep_settle_min_s = 0.05  # 最短時間はすぐ満たす（傾きだけで確定を見る）
    cfg.pedal_search.creep_timeout_s = 5.0
    speeds = [
        4.00, 4.00, 4.00,      # 窓1（基準。傾き未算出）
        4.90, 4.90, 4.90,      # 窓2: 傾き (4.90-4.00)/0.2=4.5 → 閾値超え、確定しない
        4.92, 4.92, 4.92,      # 窓3: 傾き (4.92-4.90)/0.2=0.1 → まだ閾値超え、確定しない
        4.921, 4.921, 4.921,   # 窓4: 傾き (4.921-4.92)/0.2=0.005 → 閾値未満、確定
    ]
    hw = _scripted_hw(speeds)

    mean = await psmod.wait_creep_stable(hw, cfg)

    assert mean == pytest.approx(4.921)


class _RampingCAN:
    """呼ばれるたびに大きく増え続ける車速を返すスタブ CAN（傾きが閾値を割らないようにする）。

    交互に2値を返す実装だと、1窓のサンプル数が偶数のとき窓平均が毎回同じ値に丸まって
    傾きが0になってしまう（サンプルの取り方に依存して意図せず安定判定してしまう）ため、
    単調増加にして窓の切り方によらず必ず大きい傾きが出るようにする。
    """

    def __init__(self, step: float) -> None:
        self._step = step
        self._value = 0.0

    async def read_speed(self) -> float:
        self._value += self._step
        return self._value


def _hw_with_can(can: object) -> hwmod.ResearchHardware:
    return hwmod.ResearchHardware(
        accel=hwmod.StubActuator("accel", connected=True),
        brake=hwmod.StubActuator("brake", connected=True),
        can=can,  # type: ignore[arg-type]
        ups=None,  # type: ignore[arg-type]
        hw_mode=hwmod.HW_STUB,
    )


async def test_wait_creep_stable_raises_on_timeout() -> None:
    """傾きが閾値を割らないまま creep_timeout_s に達したら DriveError。"""
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.pedal_search.creep_window_s = 0.1
    cfg.pedal_search.creep_settle_min_s = 0.0
    cfg.pedal_search.creep_timeout_s = 0.35
    hw = _hw_with_can(_RampingCAN(step=1.0))

    with pytest.raises(hwmod.DriveError, match="安定しません"):
        await psmod.wait_creep_stable(hw, cfg)


async def test_search_brake_stops_at_first_step_below_stop_speed() -> None:
    """min から 1 刻みずつ踏み、平均車速が停止しきい値未満になった刻みの位置を返す。"""
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.pedal_search.dwell_s = 0.2  # 0.1s 間隔で 3 サンプル/刻み
    speeds = [
        5.0, 5.0, 5.0,     # 1 刻み目: 変化なし
        4.9, 4.9, 4.9,     # 2 刻み目: 少し下がる（onset_margin 未満なので続けて踏む）
        0.01, 0.01, 0.01,  # 3 刻み目: 停止
    ]
    hw = _scripted_hw(speeds)
    start, step = 100, 50

    stop_pos = await psmod._search_brake(hw, cfg, base=5.0, start_pos=start, step=step)

    assert stop_pos == start + 3 * step


async def test_search_brake_does_not_press_further_while_slowing() -> None:
    """車速が onset_margin_kmh 以上下がった直後の刻みは踏み増さない（踏み過ぎ防止）。"""
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.pedal_search.dwell_s = 0.2
    cfg.pedal_search.onset_margin_kmh = 0.3
    speeds = [
        5.0, 5.0, 5.0,     # 1 刻み目
        4.5, 4.5, 4.5,     # 2 刻み目: -0.5（margin 超え）
        4.5, 4.5, 4.5,     # 3 刻み目: 減速中なので保持（踏み増さない）。ここでは下がらない
        0.01, 0.01, 0.01,  # 4 刻み目: 停止
    ]
    hw = _scripted_hw(speeds)
    moved: list[int] = []
    original = hw.brake.move_to_position

    async def record(pos: int, *, smooth_over_s: float | None = None) -> None:
        moved.append(pos)
        await original(pos, smooth_over_s=smooth_over_s)

    hw.brake.move_to_position = record  # type: ignore[method-assign]

    stop_pos = await psmod._search_brake(hw, cfg, base=5.0, start_pos=0, step=50)

    assert moved == [50, 100, 150]  # 3 刻み目は動かさない（保持）
    assert stop_pos == 150


async def test_search_brake_raises_when_over_limit() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.pedal_search.dwell_s = 0.1
    cfg.vehicle.max_brake_opening_pct = 1.0  # 約 95 pulse（検証を通さず直接書き換える）
    hw = _scripted_hw([5.0] * 10)
    with pytest.raises(
        hwmod.DriveError,
        match="ブレーキを開度上限 1%（vehicle.max_brake_opening_pct）まで踏んでも停車しません",
    ):
        await psmod._search_brake(hw, cfg, base=5.0, start_pos=0, step=50)


def test_brake_step_mm_is_independent_of_step_mm(tmp_path: Path) -> None:
    """2-0 のブレーキ踏み込みの刻み（brake_step_mm）と停車保持への刻み送り（step_mm）を分離。"""
    cfg = _tmp_cfg(tmp_path, **{"pedal_search.step_mm": 0.5, "pedal_search.brake_step_mm": 0.1})
    assert psmod.search_step_pulse(cfg) == 50  # step_mm 基準（停車保持の刻み送り）
    assert psmod.brake_step_pulse(cfg) == 10  # brake_step_mm 基準（ブレーキ踏み込みの刻み）


def test_removed_deadband_search_api() -> None:
    """不感帯探索の部品は無い（段2l-6）。"""
    for name in ("OnsetDetector", "OnsetUpdate", "_search_accel", "end_speed",
                 "deadband_search_step_pulse", "PHASE_ACCEL_SEARCH"):
        assert not hasattr(psmod, name), name


async def test_step_to_position_moves_in_steps_both_ways() -> None:
    axis = hwmod.StubActuator("brake", connected=True)
    visited: list[int] = []
    original = axis.move_to_position

    async def record(pos: int, *, smooth_over_s: float | None = None) -> None:
        visited.append(pos)
        await original(pos, smooth_over_s=smooth_over_s)

    axis.move_to_position = record  # type: ignore[method-assign]
    assert await psmod.step_to_position(axis, 0, 250, step_pulse=100, dwell_s=0.0) == 250
    assert await psmod.step_to_position(axis, 250, 30, step_pulse=100, dwell_s=0.0) == 30
    assert visited == [100, 200, 250, 150, 50, 30]


def test_save_to_config_writes_measured_values(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    result = psmod.PedalSearchResult(
        creep_speed_kmh=4.9,
        stop_confirm_pct=14.74,
        stop_brake_opening_pct=24.74,
        brake_min_pct=2.9,
    )
    psmod.save_to_config(cfg, result)
    saved = cfgmod.load_config(cfg.source_path).feedforward
    # 2026-10-08 段2l-7: yaml へは「停車保持 − ブレーキの min」で書く
    assert saved.stop_brake_offset_pct == pytest.approx(24.74 - 2.9)
    assert saved.creep_speed_kmh == pytest.approx(4.9)


def test_config_updates_has_only_stop_hold_and_creep() -> None:
    result = psmod.PedalSearchResult(5.0, 14.0, 24.0, brake_min_pct=3.0)
    assert result.config_updates() == {
        "feedforward.stop_brake_offset_pct": 21.0,
        "feedforward.creep_speed_kmh": 5.0,
    }


def test_apply_to_profile_overrides_measured_values() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    result = psmod.PedalSearchResult(5.0, 14.0, 24.0)
    base = build_vehicle_profile(cfg)
    ffp = result.apply_to_profile(base).feedforward_params
    assert (ffp.stop_brake_opening_pct, ffp.creep_speed_kmh) == (24.0, 5.0)
    # u の原点（名前は不感帯のまま）は 2-0 で上書きしない。yaml の min のまま
    assert ffp.accel_deadband_pct == cfg.vehicle.min_accel_opening_pct
    assert ffp.brake_deadband_pct == cfg.vehicle.min_brake_opening_pct
