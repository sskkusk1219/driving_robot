"""G ランプ走行をスタブ HW で通しで走らせる動作確認 CLI（段2k 段B）。

走る列: `pattern_drive.build_patterns`（手順2 と同じ。コーストダウン ×1 → `ramp_planner.plan_ramps`
の `execution_order()` の段。2026-10-08 段2l-6 でクリープ域ブレーキ保持は列から外した）。
--full なしのときだけ段を絞る。PatternLoopConfig も `pattern_drive.loop_config_from_config` を
共有する。

2-0 のペダル探索は飛ばし、スタブ車両の遊びを「ペダルに触れた位置 min」（メモリ上だけで
vehicle.min_*_opening_pct を上書き。yaml は書き換えない）と止まった開度にした
`PedalSearchResult` を使う（旧 grid_stub_run の骨組みを移したもの。あちらは段E で消した）。

スタブ車両は実車と別物（応答は実車の約 1/5）。ここで見るのは「流れ」だけで、制御性能・秒数は実車の
値ではない: 全段が走る／狙いの G に届く／上端で減速へ切り替わる／停車復帰して次へ進む。
`--gain-scale 5` はスタブ車両のアクセル・ブレーキゲインを 5 倍にして実車の応答に近づける
（旧 grid_stub_run と同じ意味。`ramp_rate_gain` には掛けない）。

使い方:
    .venv/bin/python -m tests.research.ramp_stub_run --full --gain-scale 5
    .venv/bin/python -m tests.research.ramp_stub_run --levels 0.08,0.25 --gain-scale 5
        # --full なしは短い確認用（既定の段は 0.08・0.25。--levels で変えられる）
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import math
import sys
from pathlib import Path

from tests.research import hardware as hwmod
from tests.research.app_settings import SafetySettings
from tests.research.axis_monitor import AxisMonitor
from tests.research.config import DEFAULT_CONFIG_PATH, ConfigError, ResearchConfig, load_config
from tests.research.mode_drive import load_mode
from tests.research.pattern_drive import build_patterns, loop_config_from_config, result_table
from tests.research.pattern_loop import PatternLoop
from tests.research.pedal_search import PedalSearchResult
from tests.research.ramp_planner import RampPlan, count_bands_for_modes, plan_ramps
from tests.research.research_types import VEHICLE_STOP_SPEED_KMH, DriveLogData
from tests.research.term import say
from tests.research.vehicle import build_vehicle_profile, feedforward_params, opening_to_pulse

STUB_STOP_HOLD_OFFSET_PCT = 8.0  # 停車保持ブレーキ = スタブのブレーキの遊び + これ
SHORT_LEVELS = (0.08, 0.25)  # --full なし・--levels なしのときの段（短い確認用）
STUB_NOTE = "注意: スタブの秒数は実車の値ではありません（スタブ車両は実車と応答が違う）。"


def _stub_pedal(cfg: ResearchConfig) -> PedalSearchResult:
    """スタブ車両の遊びを止まった開度・停車保持開度の基準にした 2-0 の結果（2-0 を飛ばすため）。"""
    return PedalSearchResult(
        creep_speed_kmh=feedforward_params(cfg).creep_speed_kmh,
        stop_confirm_pct=hwmod.STUB_BRAKE_PLAY_PCT + 4.0,
        stop_brake_opening_pct=hwmod.STUB_BRAKE_PLAY_PCT + STUB_STOP_HOLD_OFFSET_PCT,
    )


async def _wait_stopped(hw: hwmod.ResearchHardware, timeout_s: float = 60.0) -> None:
    """停車保持ブレーキで止まるまで待つ（スタブはクリープ車速から始まる）。"""
    waited = 0.0
    while await hw.can.read_speed() >= VEHICLE_STOP_SPEED_KMH:
        await asyncio.sleep(0.1)
        waited += 0.1
        if waited > timeout_s:
            raise hwmod.DriveError("スタブが停車しません")


def select_order(
    plan: RampPlan, *, full: bool, levels: list[float] | None
) -> list[tuple[float, float]]:
    """走る段（G, 上端の車速）。--full は全段、無ければ levels（既定 SHORT_LEVELS）だけ。"""
    order = plan.execution_order()
    if full:
        return order
    wanted = levels if levels else list(SHORT_LEVELS)
    picked = [(g, top) for g, top in order if any(math.isclose(g, w) for w in wanted)]
    return list(dict.fromkeys(picked))  # 短い確認用は 1 周目だけ（2 周目の重複は除く）


async def _run(args: argparse.Namespace) -> int:
    cfg = load_config(args.config)
    cfg.learning.timeout_s = max(cfg.learning.timeout_s, args.timeout_s)  # 保存しない
    # スタブ車両の遊びを「触れた位置 min」にする（メモリ上だけ。保存しない）
    cfg.vehicle.min_accel_opening_pct = hwmod.STUB_ACCEL_PLAY_PCT
    cfg.vehicle.min_brake_opening_pct = hwmod.STUB_BRAKE_PLAY_PCT
    lr = cfg.learning
    names = cfg.modes.coverage_mode_names
    modes = [await load_mode(cfg, n) for n in names]
    counts = count_bands_for_modes(modes, lr.ramp_g_levels)
    plan = plan_ramps(
        counts, mode_min_s=lr.ramp_mode_min_s, max_speed_kmh=cfg.vehicle.max_speed_kmh,
        budget_s=lr.ramp_time_budget_s,
    )
    levels = [float(v) for v in args.levels.split(",")] if args.levels else None
    order = select_order(plan, full=args.full, levels=levels)
    if not order:
        say("走る段が 1 つもありません（--levels の指定を確認）")
        return 2
    say(f"モード: {', '.join(names)}")
    say("走る段 (G, 上端 km/h): " + ", ".join(f"({g:g}, {t:g})" for g, t in order))

    hw = hwmod.build_hardware(cfg, hwmod.HW_STUB)
    vehicle = hw.can.vehicle  # type: ignore[attr-defined]
    vehicle.accel_gain_kmhs_per_pct *= args.gain_scale
    vehicle.brake_gain_kmhs_per_pct *= args.gain_scale
    say(f"スタブ車両の感度: アクセル {vehicle.accel_gain_kmhs_per_pct:.3f} / "
        f"ブレーキ {vehicle.brake_gain_kmhs_per_pct:.3f} km/h/s per %（×{args.gain_scale:g}）")
    await hwmod.run_initialize(hw)
    pedal = _stub_pedal(cfg)
    profile = pedal.apply_to_profile(build_vehicle_profile(cfg))
    patterns = build_patterns(cfg, profile, order)

    finished = asyncio.Event()
    state = {"emergency": False, "idx": -1}
    rows: list[tuple[float, int, str, float, float, float, bool]] = []  # --csv 用の生データ

    def on_sample(
        data: DriveLogData, idx: int, phase: str, governed: bool,
        alarm_accel: bool | None, alarm_brake: bool | None,
        monitor_accel: AxisMonitor, monitor_brake: AxisMonitor, cycle_ms: float,
    ) -> None:
        if args.csv is not None:
            rows.append((asyncio.get_running_loop().time(), idx, phase, data.actual_speed_kmh,
                         data.accel_opening, data.brake_opening, governed))
        if idx != state["idx"]:  # パターンが変わったとき 1 行（進み具合の確認用）
            state["idx"] = idx
            kind = patterns[idx].kind.value if idx < len(patterns) else "-"
            say(f"  パターン {idx + 1}/{len(patterns)}: {kind}"
                f"（車速 {data.actual_speed_kmh:.1f} km/h）")

    async def on_complete() -> None:
        finished.set()

    async def on_emergency() -> None:
        state["emergency"] = True
        finished.set()

    loop = PatternLoop(
        accel_driver=hw.accel, brake_driver=hw.brake, can_reader=hw.can, profile=profile,
        patterns=patterns, overcurrent_limit_ma=(
            hw.settings.safety.overcurrent_limit_ma if hw.settings is not None
            else SafetySettings().overcurrent_limit_ma
        ),
        on_complete=on_complete, on_emergency=on_emergency, on_sample=on_sample,
        config=loop_config_from_config(cfg, stop_confirm_pct=pedal.stop_confirm_pct),
    )
    exit_code = 0
    started = asyncio.get_running_loop().time()
    try:
        await hw.brake.move_to_position(opening_to_pulse(pedal.stop_brake_opening_pct))
        await _wait_stopped(hw)
        started = asyncio.get_running_loop().time()
        loop.start()
        try:
            await asyncio.wait_for(finished.wait(), timeout=args.timeout_s)
        except TimeoutError:
            say(f"打ち切り: {args.timeout_s:g}s 以内に完走しませんでした")
            exit_code = 3
        if state["emergency"]:
            say(f"走行が中断しました: {loop.abort_reason}")
            exit_code = 1
    except hwmod.DriveError as exc:
        say(f"走行が中断しました: {exc}")
        exit_code = 1
    finally:
        await loop.stop_and_join()
        if exit_code != 0:
            await asyncio.gather(hw.accel.home_return(), hw.brake.home_return(),
                                 return_exceptions=True)
        await hwmod.shutdown(hw)

    total_s = asyncio.get_running_loop().time() - started
    if args.csv is not None:
        with args.csv.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["elapsed_s", "pattern_idx", "phase", "speed_kmh",
                             "accel_opening", "brake_opening", "governed"])
            writer.writerows((f"{t - started:.3f}", *rest) for t, *rest in rows)
        say(f"生データ: {args.csv}（{len(rows)} 行）")
    say("\n── G ランプの結果 ──")
    say(result_table(order, loop.ramp_results, total_s))
    say(STUB_NOTE)
    say(f"走った段 {len(loop.ramp_results)} / {len(order)}、exit={exit_code}")
    return exit_code


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="G ランプ走行をスタブ HW で走らせる")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--full", action="store_true", help="全段を走る（無ければ短い確認用）")
    parser.add_argument("--levels", default="", help="走る段の G を絞る（例 0.08,0.25）")
    parser.add_argument("--gain-scale", type=float, default=1.0,
                        help="スタブのアクセル・ブレーキゲインの倍率（実車の約 1/5 → 5 を推奨）")
    parser.add_argument("--csv", type=Path, default=None,
                        help="毎周期の生データ（経過秒・フェーズ・車速・開度）を書き出す CSV")
    parser.add_argument("--timeout-s", type=float, default=3600.0,
                        help="パターン走行の打ち切り [s]")
    args = parser.parse_args(argv)
    try:
        return asyncio.run(_run(args))
    except ConfigError as exc:
        print(f"設定エラー: {exc}")
        return 2


if __name__ == "__main__":
    sys.exit(main())
