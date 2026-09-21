"""研究開発用ハーネス 手順 2-0（ペダル探索）のユニットテスト。

スタブ車両は設定の不感帯とは独立した「真の遊び」（アクセル 6%・ブレーキ 8%）を持つ。
探索がそれを当て、停止確認開度 + マージンで停車保持することを確かめる。
時間を縮めるため、刻みを 1mm・待ちを 0.3s にしている（判定ロジックは既定と同じ）。
"""

from __future__ import annotations

import asyncio
from pathlib import Path

import pytest

from src.domain.control.conversions import VEHICLE_STOP_SPEED_KMH
from tests.research import config as cfgmod
from tests.research import hardware as hwmod
from tests.research import pedal_search as psmod
from tests.research.vehicle import build_vehicle_profile, opening_to_pulse, pulse_to_opening

WINDOW_S = 0.6
FAST = {
    "pedal_search.step_mm": 1.0,
    "pedal_search.search_step_mm": 1.0,  # 既定 0.1mm だとテストが遅くなるため、旧来どおり粗く
    "pedal_search.dwell_s": 0.3,
    "pedal_search.onset_margin_kmh": 0.05,  # スタブは車速ノイズが無い
    # スタブは不感帯未満で傾きが厳密に0（ノイズ無し）なので浅くできるが、実時間の sleep に
    # 依存するテストのため、遊びを越えた直後の小さな傾き（実測で約0.05〜0.1）との差を
    # 十分に取れる値にする（0.05 ちょうどだとタイミング次第で誤判定した）
    "pedal_search.onset_accel_kmhs": 0.02,
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


async def test_search_finds_stub_play_and_holds_stop(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = await _ready_stub(cfg)
    # 停車までを短くする（既定ゲインだとクリープの押しとほぼ釣り合い、止まるまで十数秒かかる）
    hw.can.vehicle.brake_gain_kmhs_per_pct = 1.5  # type: ignore[attr-defined]
    yaml_before = cfg.source_path.read_text(encoding="utf-8")

    result = await psmod.run_pedal_search(hw, cfg, window_s=WINDOW_S)

    step_pct = pulse_to_opening(psmod.search_step_pulse(cfg))
    # 検出は応答を見てからなので真値より浅くは出ない。深い側は立ち上がり遅れぶん（数刻み）まで許す
    assert hwmod.STUB_ACCEL_PLAY_PCT <= result.accel_deadband_pct
    assert result.accel_deadband_pct <= hwmod.STUB_ACCEL_PLAY_PCT + 3 * step_pct
    assert hwmod.STUB_BRAKE_PLAY_PCT <= result.brake_deadband_pct
    assert result.brake_deadband_pct <= hwmod.STUB_BRAKE_PLAY_PCT + 3 * step_pct
    assert result.stop_confirm_pct >= result.brake_deadband_pct
    # 停車保持 = 停止確認 + マージン。その位置まで踏んで停車している
    assert result.stop_brake_opening_pct == pytest.approx(result.stop_confirm_pct + 3.0, abs=0.01)
    assert hw.brake.position == opening_to_pulse(result.stop_brake_opening_pct)
    assert hw.accel.position == 0
    assert await hw.can.read_speed() < VEHICLE_STOP_SPEED_KMH
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


async def test_search_fails_when_accel_has_no_effect(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path, **{"pedal_search.accel_max_pct": 3.0})
    hw = await _ready_stub(cfg)
    with pytest.raises(hwmod.DriveError, match="アクセルを 3% まで踏んでも"):
        await psmod.run_pedal_search(hw, cfg, window_s=WINDOW_S)
    await hwmod.shutdown(hw)


async def test_search_fails_when_brake_deadband_not_found_within_limit(tmp_path: Path) -> None:
    """段1b: 不感帯検出専用の上限 deadband_max_pct を超えても反応が無ければ DriveError。

    停止確認の上限 brake_max_pct とは別物であることを確かめる（スタブの遊びは 8%）。
    """
    cfg = _tmp_cfg(tmp_path, **{"pedal_search.deadband_max_pct": 3.0})
    hw = await _ready_stub(cfg)
    with pytest.raises(hwmod.DriveError, match="ブレーキを 3% まで踏んでも車速が下がりません"):
        await psmod.run_pedal_search(hw, cfg, window_s=WINDOW_S)
    await hwmod.shutdown(hw)


# ── 傾き算出（段1b。ProblemReport_20260916） ────────────────────────────


def test_speed_slope_is_zero_for_constant_speed() -> None:
    samples = [(t, 5.0) for t in (0.0, 0.1, 0.2, 0.3, 0.4, 0.5)]
    assert psmod.speed_slope(samples) == pytest.approx(0.0)


def test_speed_slope_recovers_ramp_rate() -> None:
    rate = 1.7
    samples = [(t, 5.0 + rate * t) for t in (0.0, 0.1, 0.2, 0.3, 0.4, 0.5)]
    assert psmod.speed_slope(samples) == pytest.approx(rate)


def test_speed_slope_uses_only_tail_half() -> None:
    """前半（下降）と後半（上昇）が混じっていても、後半だけの傾きが返ること。

    踏んでから車速が動くまでの遅れ（むだ時間）が前半に乗るのを避けるため（SLOPE_TAIL_FRACTION）。
    全体の最小二乗なら傾きはほぼ 0 だが、後半だけなら明確に正になる。
    """
    samples = [(0.0, 10.0), (0.1, 9.0), (0.2, 8.0), (0.3, 8.2), (0.4, 8.4), (0.5, 8.6)]
    assert psmod.speed_slope(samples) == pytest.approx(2.0)  # 後半3点: (8.6-8.2)/(0.5-0.3)


def test_speed_slope_returns_zero_for_fewer_than_two_samples() -> None:
    assert psmod.speed_slope([]) == 0.0
    assert psmod.speed_slope([(0.0, 5.0)]) == 0.0


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


async def test_search_accel_detects_true_onset_via_slope() -> None:
    """実機ログ（drive_log_real_20260915_052432.csv）相当の合成応答を模した回帰テスト。

    8.42%相当で -0.4、8.95%相当で +0.21、9.47%相当で +1.0 km/h/s という実測どおりの傾きを与えると、
    傾き判定は最初に反応した 8.95%相当の刻みを返す（confirm_count=2 は 9.47%相当で満たす）。
    旧来の「待ち時間の平均車速が基準+0.3km/hを超えたか」判定では、この3刻みのどの平均も
    基準を超えないため、実際には 10.00%相当までさらに検出が遅れていた。
    """
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.pedal_search.dwell_s = 0.2  # 0.1s間隔で3サンプル/刻み（後半2点で正確に傾きが再現できる）
    speeds = [
        5.00, 4.96, 4.92,       # 8.42%相当: -0.4 km/h/s（反応なし）
        4.92, 4.941, 4.962,     # 8.95%相当: +0.21 km/h/s ← 真の効き始め
        4.962, 5.062, 5.162,    # 9.47%相当: +1.0 km/h/s（2刻み連続で確定）
    ]
    hw = _scripted_hw(speeds)
    step = 50  # pulse。値自体は判定に無関係（位置の表示にのみ使う）

    onset = await psmod._search_accel(hw, cfg, base=4.98, step=step)

    assert onset == 2 * step  # 8.95%相当（最初に反応した刻み）


async def test_search_brake_detects_true_onset_via_slope_then_holds_and_stops() -> None:
    """`_search_brake` も傾きで反応を検出し、最初に反応した刻みを不感帯として返す。

    確定後は従来どおり平均車速で「減速中は踏み増さない（holding）」・停止確認を行う
    （this パスは変更していないので、ここで一緒に確かめる）。
    """
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.pedal_search.dwell_s = 0.2
    speeds = [
        5.0, 5.0, 5.0,     # 1刻み目: 傾き 0（反応なし）
        5.0, 4.9, 4.8,     # 2刻み目: 傾き -1.0（反応。first）
        4.9, 4.7, 4.5,     # 3刻み目: 傾き -2.0（2刻み連続 → 不感帯確定）
        4.7, 0.01, 0.01,   # 4刻み目: 平均車速が大きく下がる（last_drop が margin 超え）
        0.01, 0.01, 0.01,  # 5刻み目: holding で踏み増さず、平均車速が停止確認を満たす
    ]
    hw = _scripted_hw(speeds)
    step = 50

    onset, stop_pos = await psmod._search_brake(hw, cfg, base=5.0, step=step)

    assert onset == 2 * step  # 2刻み目（最初に反応した刻み）
    assert stop_pos == 4 * step  # 5刻み目は holding で踏み増していない


def test_search_step_mm_is_independent_of_step_mm(tmp_path: Path) -> None:
    """段1b: 不感帯探索の刻み（search_step_mm）と停車保持への刻み送り（step_mm）を分離。"""
    cfg = _tmp_cfg(tmp_path, **{"pedal_search.step_mm": 0.5, "pedal_search.search_step_mm": 0.1})
    assert psmod.search_step_pulse(cfg) == 50           # step_mm 基準（停車保持の刻み送り）
    assert psmod.deadband_search_step_pulse(cfg) == 10  # search_step_mm 基準（不感帯探索の刻み）


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
        accel_deadband_pct=6.32,
        brake_deadband_pct=8.42,
        stop_confirm_pct=14.74,
        stop_brake_opening_pct=24.74,
    )
    psmod.save_to_config(cfg, result)
    saved = cfgmod.load_config(cfg.source_path).feedforward
    assert saved.accel_deadband_pct == pytest.approx(6.32)
    assert saved.brake_deadband_pct == pytest.approx(8.42)
    assert saved.stop_brake_opening_pct == pytest.approx(24.74)
    assert saved.creep_speed_kmh == pytest.approx(4.9)


def test_apply_to_profile_overrides_measured_values() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    result = psmod.PedalSearchResult(5.0, 6.0, 8.0, 14.0, 24.0)
    ffp = result.apply_to_profile(build_vehicle_profile(cfg)).feedforward_params
    assert (
        ffp.accel_deadband_pct,
        ffp.brake_deadband_pct,
        ffp.stop_brake_opening_pct,
        ffp.creep_speed_kmh,
    ) == (6.0, 8.0, 24.0, 5.0)


def test_probe_list_must_be_ascending() -> None:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    cfg.learning.accel_deadband_probe_offsets_pct = [5.0, 1.0]
    assert any("accel_deadband_probe_offsets_pct" in p for p in cfgmod.validate_config(cfg))
