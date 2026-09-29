"""加振走行（ペダル→車速の周波数応答を開ループで測る。`tests/research/excite.py`）のユニットテスト。

`tests/research/test_research_mode_drive.py` の流儀に合わせる: 純関数（正弦波指令・ブロック長の
整数周期切り上げ・到達フェーズの積分・トリムのローパス）は直接叩き、走行そのものはスタブ HW
（`StubVehicle` 付き）で短い設定を完走させて確認する。
"""

from __future__ import annotations

import asyncio
import csv
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from tests.research import config as cfgmod
from tests.research import excite as excitemod
from tests.research import hardware as hwmod
from tests.research.drive_log import SECTION_EXCITE, SessionLog


def _tmp_cfg(tmp_path: Path) -> cfgmod.ResearchConfig:
    path = tmp_path / "cfg.yaml"
    cfg = cfgmod.load_config(path)
    cfg.save({"output.results_dir": str(tmp_path / "results"), "output.plot": False})
    return cfgmod.load_config(path)


# ─────────────────────────────────────────────────────────────────────
# 純関数: 正弦波の指令・クランプ
# ─────────────────────────────────────────────────────────────────────


def test_sine_opening_matches_amplitude_and_frequency() -> None:
    base, amplitude, freq_hz = 10.0, 0.3, 1.0
    assert excitemod._sine_opening(base, amplitude, freq_hz, 0.0) == pytest.approx(base)
    # 1/4 周期で振幅ぶん上、3/4 周期で振幅ぶん下（sin の定義どおり）
    assert excitemod._sine_opening(base, amplitude, freq_hz, 0.25) == pytest.approx(
        base + amplitude
    )
    assert excitemod._sine_opening(base, amplitude, freq_hz, 0.75) == pytest.approx(
        base - amplitude
    )
    # 周波数を変えても、対応する位相では同じ振れ幅になる
    assert excitemod._sine_opening(base, amplitude, 2.0, 0.125) == pytest.approx(base + amplitude)


def test_clamp_floors_and_ceils_to_deadband_and_max_opening() -> None:
    floor_pct, ceil_pct = 0.5, 80.0
    assert excitemod._clamp(150.0, floor_pct, ceil_pct) == ceil_pct
    assert excitemod._clamp(-5.0, floor_pct, ceil_pct) == floor_pct
    assert excitemod._clamp(10.0, floor_pct, ceil_pct) == 10.0


# ─────────────────────────────────────────────────────────────────────
# 純関数: ブロック長（整数周期への切り上げ）
# ─────────────────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    ("hold_s", "freq_hz", "expected_n"),
    [
        (12.0, 1.4, 17),  # ceil(12.0*1.4) = ceil(16.8) = 17
        (10.0, 2.0, 20),  # ちょうど割り切れる場合はそのまま
        (2.0, 0.05, 1),  # 1 周期に満たなくても最低 1 周期
    ],
)
def test_block_duration_rounds_up_to_whole_periods(
    hold_s: float, freq_hz: float, expected_n: int
) -> None:
    n_periods, duration_s = excitemod._block_duration_s(hold_s, freq_hz)
    assert n_periods == expected_n
    assert duration_s == pytest.approx(expected_n / freq_hz)
    assert duration_s >= hold_s - 1e-9  # 切り上げなので元の長さを下回らない


# ─────────────────────────────────────────────────────────────────────
# 純関数: 遅いトリムが加振周波数を通さないこと
# ─────────────────────────────────────────────────────────────────────


def test_trim_gain_attenuates_1_4hz_far_more_than_naive_proportional() -> None:
    """遅いトリムの 1.4Hz 成分は、同じゲインの値をそのまま比例ゲインとして掛けた場合の 1/50 以下。

    トリムは「積分（1周期ごとに trim_gain・偏差・dt を足し込む）」かつ「LPF を通した車速」を
    使うため、同じ trim_gain の値を素朴な比例ゲイン（`trim_gain・偏差` を毎周期そのまま開度に
    使う場合）と比べると、積分による 1/(2π f) の減衰と LPF 自身の減衰が両方乗る。
    """
    dt = 0.05  # control.loop_interval_s の既定
    tau_s = 1.0  # excite.speed_lpf_tau_s の既定
    alpha = dt / (tau_s + dt)
    trim_gain = 0.05  # excite.trim_gain_pct_per_kmh_s の既定
    v_star = 60.0
    amp_v = 2.0  # 車速の振れ幅 [km/h]（比は amp_v に依らないので値は任意）
    freq_hz = 1.4
    n = round(20.0 / dt)
    t = np.arange(n) * dt
    v = v_star + amp_v * np.sin(2.0 * np.pi * freq_hz * t)

    base = 0.0
    v_lpf: float | None = None
    bases = np.empty(n)
    for i in range(n):
        v_lpf = excitemod._lpf_step(v_lpf, float(v[i]), alpha)
        base += excitemod._trim_delta(trim_gain, v_star, v_lpf, dt)
        bases[i] = base

    skip = round(5.0 / dt)  # LPF の立ち上がり（過渡）を捨てる
    tt = t[skip:]
    b = bases[skip:] - bases[skip:].mean()
    amp_b = 2.0 * abs(excitemod._dft_component(tt, b, freq_hz)) / len(tt)

    naive_amp = trim_gain * amp_v  # 同じ trim_gain を積分せずそのまま比例ゲインとして掛けた場合
    assert amp_b > 0.0
    assert amp_b <= naive_amp / 50.0


# ─────────────────────────────────────────────────────────────────────
# 安全: 加振中だけ abort_band_kmh を見る
# ─────────────────────────────────────────────────────────────────────


def test_abort_band_checked_only_during_excite_phase(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    hw = hwmod.build_hardware(cfg, hwmod.HW_STUB)
    log = SessionLog(cfg, hw, has_ref=False)
    run = excitemod._ExciteRun(hw, cfg, log)
    assert cfg.excite.abort_band_kmh < 20.0  # 既定 8.0km/h。この差なら必ず超える

    # 到達フェーズ（check_abort_band=False）: 目標から大きく外れていても例外にしない
    run._check_safety(
        1.0, v_star=60.0, speed=40.0, accel_current=0.0, brake_current=0.0,
        accel_pos=0, brake_pos=0, alarm_accel=False, alarm_brake=False,
        check_abort_band=False,
    )

    # 加振フェーズ（check_abort_band=True）: abort_band_kmh を超えたら DriveError
    with pytest.raises(hwmod.DriveError, match="目標から"):
        run._check_safety(
            1.0, v_star=60.0, speed=40.0, accel_current=0.0, brake_current=0.0,
            accel_pos=0, brake_pos=0, alarm_accel=False, alarm_brake=False,
            check_abort_band=True,
        )

    # 帯の中なら加振中でも例外にしない
    run._check_safety(
        1.0, v_star=60.0, speed=60.0 + cfg.excite.abort_band_kmh - 0.1,
        accel_current=0.0, brake_current=0.0, accel_pos=0, brake_pos=0,
        alarm_accel=False, alarm_brake=False, check_abort_band=True,
    )


# ─────────────────────────────────────────────────────────────────────
# analyze(): 合成データからゲイン・位相を復元できること
# ─────────────────────────────────────────────────────────────────────


def test_analyze_recovers_gain_and_phase_from_synthetic_signal(tmp_path: Path) -> None:
    """y(t) = G・u(t − τ) + 直線トレンド から、gain ≈ G・phase ≈ −2π f τ を復元する。"""
    freq_hz = 1.0
    v_star = 60.0
    gain = 0.65
    tau_s = 0.1
    dt = 0.05
    n = round(12.0 / dt)  # 12 周期ぶん（skip_s=2.0 を引いても 10 周期残る）
    t = np.arange(n) * dt
    u = 1.0 * np.sin(2.0 * np.pi * freq_hz * t)  # 指令（振幅 1.0、平均は後で引かれる）
    y = (
        gain * np.sin(2.0 * np.pi * freq_hz * (t - tau_s))  # 応答（G 倍・τ 遅れ）
        + 50.0 + 0.1 * t  # 直線トレンド（analyze が外す）
    )

    csv_path = tmp_path / "excite.csv"
    pattern = f"{v_star:.0f}kmh_{freq_hz:.2f}Hz"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["section", "pattern", "mode_time_s", "actual_speed_kmh", "accel_cmd_pct"])
        for i in range(n):
            writer.writerow(
                [SECTION_EXCITE, pattern, f"{t[i]:.3f}", f"{y[i]:.5f}", f"{u[i]:.5f}"]
            )
        # EXCITE 以外・到達フェーズの行は無視されることも確かめる
        writer.writerow(["PRE_DRIVE_CHECK", "", "0.0", "0.0", "0.0"])
        writer.writerow([SECTION_EXCITE, f"{v_star:.0f}kmh_approach", "0.0", "60.0", "1.0"])

    blocks = excitemod.analyze(csv_path, skip_s=2.0)

    assert len(blocks) == 1  # 到達フェーズ・他区間の行は集計に入らない
    b = blocks[0]
    assert b.v_star_kmh == pytest.approx(v_star)
    assert b.freq_hz == pytest.approx(freq_hz)
    assert b.gain_kmh_per_pct == pytest.approx(gain, rel=0.05)
    expected_phase_deg = math.degrees(-2.0 * math.pi * freq_hz * tau_s)
    assert b.phase_deg == pytest.approx(expected_phase_deg, abs=5.0)


# ─────────────────────────────────────────────────────────────────────
# analyze(): 3 次トレンド除去 — 純積分器の理論値を、曲がったトレンド越しに復元できること
#
# 背景（`docs/Problem/ProblemReport_20260921.md` 手順2 段A）: 加振ブロック中の平均車速は
# 「到達フェーズの整定の尾」＋「加振中の遅いトリム」で曲がった軌跡を描き、1 次の直線トレンド
# 除去ではこの曲率が正弦波成分に漏れ込んでゲイン・位相に大きなバイアスが乗る
# （スタブ車両の実測で 1.4Hz のゲインが +45% ずれた）。3 次トレンド（`_TREND_DEGREE`）なら
# 理論値に一致することをスタブ実走ログで確認済みなので、ここでは合成信号で回帰させる。
# ─────────────────────────────────────────────────────────────────────


def _write_excite_csv(
    csv_path: Path, pattern: str, t: np.ndarray, y: np.ndarray, u: np.ndarray
) -> None:
    """`analyze` が読む最小限の列だけの加振走行 CSV を書き出す（テスト専用の小さなヘルパー）。"""
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["section", "pattern", "mode_time_s", "actual_speed_kmh", "accel_cmd_pct"])
        for i in range(len(t)):
            writer.writerow([SECTION_EXCITE, pattern, f"{t[i]:.3f}", f"{y[i]:.5f}", f"{u[i]:.5f}"])


def _integrator_block_signals(
    t: np.ndarray,
    *,
    freq_hz: float,
    g: float,
    amp_pct: float,
    v_star: float,
    curve_c1: float,
    curve_c2: float,
    curve_c3: float,
    noise_sigma: float,
    seed: int,
) -> tuple[np.ndarray, np.ndarray]:
    """純積分器（dy/dt = g・u、`StubVehicle` と同じ形）に正弦波指令を与えたときの (u, y) を作る。

    y には「整定の尾＋遅いトリム」を模した 3 次の曲がったトレンド（正規化時刻 tc_norm の
    1〜3 次式）と微小な白色ノイズを重ねる。理論値: 振幅 = g・amp_pct / (2π・freq_hz)、
    位相 = −90°（`u = amp_pct・sin(2π f t)` を基準にしたとき）。
    """
    omega = 2.0 * np.pi * freq_hz
    u = amp_pct * np.sin(omega * t)
    # dy/dt = g・u(t) の解析解。-cos(θ) = sin(θ - 90°) なので振幅 gA/ω・位相 -90° になる。
    y_ideal = -(g * amp_pct / omega) * np.cos(omega * t)

    tc_norm = (t - t.mean()) / (np.ptp(t) / 2.0)
    trend = v_star + curve_c1 * tc_norm + curve_c2 * tc_norm**2 + curve_c3 * tc_norm**3

    rng = np.random.default_rng(seed)
    noise = rng.normal(0.0, noise_sigma, size=t.shape)

    return u, y_ideal + trend + noise


def test_analyze_recovers_theoretical_gain_and_phase_for_pure_integrator(tmp_path: Path) -> None:
    """純積分器 + 曲がった3次トレンド + 微小ノイズから理論値（gain=g/(2πf)、phase=-90°）を復元する。

    3 次の計画行列（トレンド 4 列＋cos/sin 2 列）が、整定の尾＋遅いトリムに相当する曲率を
    ゲイン・位相の推定から正しく分離できることを確かめる。
    """
    freq_hz = 1.4  # 段A で問題になった周波数
    g = 0.25  # StubVehicle の理論加速度ゲイン [km/h/s per %]
    amp_pct = 20.0  # 指令振幅（gain は比なので値自体は任意）
    v_star = 60.0
    dt = 0.05
    n = round(14.0 / dt)  # skip_s=2.0 を引いても 12s ぶん（16.8 周期）残る
    t = np.arange(n) * dt

    u, y = _integrator_block_signals(
        t, freq_hz=freq_hz, g=g, amp_pct=amp_pct, v_star=v_star,
        curve_c1=2.0, curve_c2=3.0, curve_c3=4.0, noise_sigma=0.01, seed=0,
    )

    csv_path = tmp_path / "excite.csv"
    pattern = f"{v_star:.0f}kmh_{freq_hz:.2f}Hz"
    _write_excite_csv(csv_path, pattern, t, y, u)

    blocks = excitemod.analyze(csv_path, skip_s=2.0)
    assert len(blocks) == 1
    b = blocks[0]

    expected_gain = g / (2.0 * math.pi * freq_hz)
    assert b.gain_kmh_per_pct == pytest.approx(expected_gain, rel=0.05)
    assert b.phase_deg == pytest.approx(-90.0, abs=10.0)
    assert b.snr > 5.0  # 曲率をきちんと分離できていれば SNR は高いはず


def test_analyze_stays_accurate_as_trend_curvature_grows(tmp_path: Path) -> None:
    """トレンドの曲率をさらに大きくしても（1次の直線では確実に破綻する大きさ）、3次なら理論値に一致する。

    1次実装との比較は行わない（現行実装はすでに 3 次）。「曲率が大きくなっても推定が理論値に
    一致し続けること」を回帰的に確かめる。
    """
    freq_hz = 1.4
    g = 0.25
    amp_pct = 20.0
    v_star = 60.0
    dt = 0.05
    n = round(14.0 / dt)
    t = np.arange(n) * dt

    # 曲率を上のテストよりさらに強める
    u, y = _integrator_block_signals(
        t, freq_hz=freq_hz, g=g, amp_pct=amp_pct, v_star=v_star,
        curve_c1=10.0, curve_c2=15.0, curve_c3=20.0, noise_sigma=0.01, seed=1,
    )

    csv_path = tmp_path / "excite.csv"
    pattern = f"{v_star:.0f}kmh_{freq_hz:.2f}Hz"
    _write_excite_csv(csv_path, pattern, t, y, u)

    blocks = excitemod.analyze(csv_path, skip_s=2.0)
    assert len(blocks) == 1
    b = blocks[0]

    expected_gain = g / (2.0 * math.pi * freq_hz)
    assert b.gain_kmh_per_pct == pytest.approx(expected_gain, rel=0.05)
    assert b.phase_deg == pytest.approx(-90.0, abs=10.0)


# ─────────────────────────────────────────────────────────────────────
# 走行: スタブ HW で短い設定を完走させ、EXCITE の行が残ること
# ─────────────────────────────────────────────────────────────────────


async def test_run_excite_completes_with_stub_hw_and_records_excite_rows(
    tmp_path: Path,
) -> None:
    cfg = _tmp_cfg(tmp_path)
    # 速度 1 点・周波数 2 点・hold_s 2s 程度（動作確認用の短い設定）
    cfg.excite.speeds_kmh = [8.0]
    cfg.excite.frequencies_hz = [1.0, 2.0]
    cfg.excite.amplitude_pct = 1.0
    cfg.excite.hold_s = 2.0
    cfg.excite.analysis_skip_s = 0.2
    cfg.excite.approach_timeout_s = 20.0
    cfg.excite.approach_band_kmh = 1.0
    cfg.excite.approach_settle_s = 0.5
    # 走行後の緩減速を短くする（判定ロジックは既定と同じ）
    cfg.decel_stop.step_mm = 1.0
    cfg.decel_stop.dwell_s = 0.2
    cfg.decel_stop.slope_window_s = 0.2

    hw = hwmod.build_hardware(cfg, hwmod.HW_STUB)
    await hwmod.run_initialize(hw)
    assert isinstance(hw.can, hwmod.StubCANReader)
    # 目標速度で始めておく（到達フェーズをすぐ終わらせ、本テストの関心＝完走とログの形に絞る）
    hw.can.speed_kmh = cfg.excite.speeds_kmh[0]
    await hw.brake.move_to_position(0)

    log = SessionLog(cfg, hw, has_ref=False)
    log.start(SECTION_EXCITE, "")
    result = await excitemod.run_excite(hw, cfg, log=log)
    await log.close()

    assert result.completed and result.abort_reason == ""
    assert len(result.blocks) == 2  # 周波数 2 点ぶん
    assert result.samples

    with log.csv_path.open(newline="", encoding="utf-8") as f:
        excite_rows = [r for r in csv.DictReader(f) if r["section"] == SECTION_EXCITE]
    assert len(excite_rows) == len(result.samples)
    assert any(r["pattern"].endswith("Hz") for r in excite_rows)
    assert any(r["pattern"].endswith("approach") for r in excite_rows)
    # 待機位置に固定したブレーキは動いていないこと（惰行のみ。減速は最後の緩減速だけ）
    brake_pcts = {r["brake_cmd_pct"] for r in excite_rows}
    assert len(brake_pcts) == 1

    # ログの CSV から周波数応答が求まること（結線の確認。数値自体はスタブ理想モデルなので見ない）
    blocks = excitemod.analyze(log.csv_path, skip_s=cfg.excite.analysis_skip_s)
    assert len(blocks) == 2


async def test_run_excite_aborts_on_overcurrent_and_releases_pedals(tmp_path: Path) -> None:
    cfg = _tmp_cfg(tmp_path)
    cfg.excite.speeds_kmh = [8.0]
    cfg.excite.frequencies_hz = [1.0]
    cfg.excite.hold_s = 5.0
    cfg.excite.approach_timeout_s = 20.0

    hw = hwmod.build_hardware(cfg, hwmod.HW_STUB)
    await hwmod.run_initialize(hw)
    assert isinstance(hw.accel, hwmod.StubActuator)
    hw.can.speed_kmh = cfg.excite.speeds_kmh[0]

    calls = 0

    async def spiking_current() -> float:
        nonlocal calls
        calls += 1
        return 1e6 if calls > 5 else 0.0

    hw.accel.read_current = spiking_current  # type: ignore[method-assign]

    log = SessionLog(cfg, hw, has_ref=False)
    log.start(SECTION_EXCITE, "")
    result = await excitemod.run_excite(hw, cfg, log=log)
    await log.close()

    assert not result.completed
    assert "過電流" in result.abort_reason
    assert hw.accel.position == 0 and hw.brake.position == 0  # ペダルを離している


# ─────────────────────────────────────────────────────────────────────
# 到達フェーズ: 整定時の base（瞬時値ではなく整定窓の指令の平均）・
# タイムアウトの中断種別（_ControlAbort = 制御側。ハードは正常なので緩減速で止める）
# ─────────────────────────────────────────────────────────────────────


def _bare_run(
    hw: hwmod.ResearchHardware, cfg: cfgmod.ResearchConfig, log: SessionLog
) -> excitemod._ExciteRun:
    """`_approach` を単独で呼ぶための `_ExciteRun`。

    `run()` 冒頭がやるループ状態の初期化だけを真似る。
    """
    run = excitemod._ExciteRun(hw, cfg, log)
    loop = asyncio.get_running_loop()
    run._loop = loop
    run._started = loop.time()
    run._next_tick = run._started
    run.t = 0.0
    run.limit_s = None
    return run


async def test_approach_settles_base_to_mean_of_window_commands(tmp_path: Path) -> None:
    """整定時の `self.base` は瞬時の PI 出力ではなく、整定窓中に実際に送った指令の平均になる。

    実機の到達フェーズ PI は目標近傍でハンチングし指令の瞬時値が振れる
    （`ExciteSection.approach_band_kmh` のコメント参照）。帯内で常に振動する速度列を与え、
    整定後の `base` が窓内の指令（CSV の `accel_cmd_pct`。到達フェーズは常に
    `command = base_at_cycle` なのでこの列がそのまま毎周期の指令）の平均に一致し、
    かつ瞬時値（最後の指令）とは異なることを確かめる。
    """
    cfg = _tmp_cfg(tmp_path)
    cfg.control.loop_interval_ms = 10  # テストを速くする（既定 50ms → 10ms）
    cfg.control.log_interval_ms = 10  # 毎周期ログする（既定だと間引かれ窓の指令が復元できない）
    cfg.excite.speeds_kmh = [60.0]
    cfg.excite.approach_band_kmh = 1.0
    cfg.excite.approach_settle_s = 0.05  # 10ms刻みで最低 5 周期は窓に入る
    cfg.excite.approach_timeout_s = 5.0

    hw = hwmod.build_hardware(cfg, hwmod.HW_STUB)
    await hwmod.run_initialize(hw)
    hw.can.vehicle = None  # 車両モデルを介さず、速度は毎回こちらが与えた値をそのまま返す
    v_star = 60.0
    # 常に帯内（±1.0km/h）だが交互に振れる速度列 → 到達フェーズの指令（PI出力）が毎周期変動する
    speeds = [60.5, 59.5, 60.6, 59.4, 60.5, 59.5, 60.6, 59.4, 60.5, 59.5, 60.6, 59.4]
    speed_iter = iter(speeds + [60.0] * 50)  # 想定より周期数が増えても尽きないよう余裕を持たせる

    async def fake_read_speed() -> float:
        hw.can.speed_kmh = next(speed_iter)
        return hw.can.speed_kmh

    hw.can.read_speed = fake_read_speed  # type: ignore[method-assign]

    log = SessionLog(cfg, hw, has_ref=False)
    log.start(SECTION_EXCITE, "")
    run = _bare_run(hw, cfg, log)

    await run._approach(v_star)
    await log.close()

    with log.csv_path.open(newline="", encoding="utf-8") as f:
        rows = [
            r
            for r in csv.DictReader(f)
            if r["section"] == SECTION_EXCITE and r["pattern"] == "60kmh_approach"
        ]
    commands = [float(r["accel_cmd_pct"]) for r in rows]
    assert len(commands) >= 5  # 窓内に複数周期あること（変動を確かめるのに十分な数）
    # 実際に指令が変動したこと（変動が無ければ「凍結の弊害」自体を測れないので前提として確認）
    assert max(commands) - min(commands) > 1e-6

    expected_mean = sum(commands) / len(commands)
    assert run.base == pytest.approx(expected_mean, abs=1e-9)
    assert run.base != pytest.approx(commands[-1], abs=1e-6)  # 瞬時値（最後の指令）とは異なる


async def test_approach_timeout_raises_control_abort(tmp_path: Path) -> None:
    """到達フェーズが `approach_timeout_s` 以内に整定しなければ `_ControlAbort` になる。

    `_ControlAbort` は `DriveError` のサブクラス。
    """
    cfg = _tmp_cfg(tmp_path)
    cfg.control.loop_interval_ms = 10
    cfg.control.log_interval_ms = 10
    cfg.excite.speeds_kmh = [60.0]
    cfg.excite.approach_timeout_s = 0.05  # ほぼ即座にタイムアウトさせる

    hw = hwmod.build_hardware(cfg, hwmod.HW_STUB)
    await hwmod.run_initialize(hw)
    hw.can.vehicle = None
    hw.can.speed_kmh = 0.0  # 目標 60km/h から大きく外れたまま = 絶対に整定しない

    log = SessionLog(cfg, hw, has_ref=False)
    log.start(SECTION_EXCITE, "")
    run = _bare_run(hw, cfg, log)

    with pytest.raises(excitemod._ControlAbort) as exc_info:
        await run._approach(60.0)
    assert isinstance(exc_info.value, hwmod.DriveError)  # _ControlAbort は DriveError のサブクラス
    await log.close()


async def test_control_abort_decelerates_to_stop_instead_of_releasing_pedals(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch,
) -> None:
    """制御側の中断（到達フェーズタイムアウト）は `_release_pedals` ではなく `decelerate_to_stop` で
    止まること（ハードは正常なので緩減速。過電流など従来の中断はペダル解放のまま
    ＝ 既存テスト `test_run_excite_aborts_on_overcurrent_and_releases_pedals` で確認済み）。
    """
    cfg = _tmp_cfg(tmp_path)
    cfg.control.loop_interval_ms = 10
    cfg.control.log_interval_ms = 10
    cfg.excite.speeds_kmh = [60.0]
    cfg.excite.frequencies_hz = [1.0]
    cfg.excite.approach_timeout_s = 0.05

    hw = hwmod.build_hardware(cfg, hwmod.HW_STUB)
    await hwmod.run_initialize(hw)
    hw.can.vehicle = None
    hw.can.speed_kmh = 0.0  # 絶対に整定しない → approach_timeout で _ControlAbort

    release_calls = 0
    decel_calls = 0

    async def fake_release(hw_arg: object) -> None:
        nonlocal release_calls
        release_calls += 1

    async def fake_decelerate_to_stop(hw_arg, cfg_arg, profile_arg, *, log=None):
        nonlocal decel_calls
        decel_calls += 1
        return SimpleNamespace(hold_pct=1.23)

    monkeypatch.setattr(excitemod, "_release_pedals", fake_release)
    monkeypatch.setattr(excitemod, "decelerate_to_stop", fake_decelerate_to_stop)

    log = SessionLog(cfg, hw, has_ref=False)
    log.start(SECTION_EXCITE, "")
    result = await excitemod.run_excite(hw, cfg, log=log)
    await log.close()

    assert not result.completed  # completed は制御側中断でも False のまま
    assert "approach_timeout_s" in result.abort_reason
    assert decel_calls == 1  # 緩減速で止めた
    assert release_calls == 0  # ペダルは離していない
