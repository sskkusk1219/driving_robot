"""格子ステップ走行をスタブ HW で直接走らせる動作確認 CLI（ProblemReport_20260925 段3）。

スタブの手順2 は 2-0 のペダル探索で必ず止まる（傾き判定に届かない）ため、`main --steps 1,2` では
パターン列を検証できない。そこで 2-0 を飛ばし、スタブ車両の遊びをそのまま不感帯にした
`PedalSearchResult` を作って `run_pattern_drive` に格子ステップ走行のパターン列を直接渡す。

スタブ車両は実車と別物（アクセルゲインは実車の約 1/5）。ここで見るのは「流れ」だけで、制御性能は
判断しない: 全ステーションを通る／感度 g がスタブの真値へ数回で寄る／打ち切りが働く／網羅表で
狙ったセルに学習データが入る／所要時間。

段4（手順2 のパターン列の置き換え）から `--full` を足した: 手順2 の新しい全体構成
（`pattern_drive.build_patterns` = コーストダウン → 格子ステップ → クリープ発進 → クリープ域
ブレーキ保持）で走り、終わったら同じ CSV でモデル作成（2-2。YAML は更新しない）まで通す。

使い方:
    .venv/bin/python -m tests.research.grid_stub_run --ref-csv <基準車速を持つCSV>
    .venv/bin/python -m tests.research.grid_stub_run --ref-csv <CSV> --stations 15,55
    .venv/bin/python -m tests.research.grid_stub_run --ref-csv <CSV> --stations 15,55 --gain-scale 5
        # --gain-scale: スタブのアクセル・ブレーキゲインを倍率変更（感度違いの確認）
        # --ref-csv を省くと DB の modes.coverage_mode_names（全モード）を読んで合成する
    .venv/bin/python -m tests.research.grid_stub_run --ref-csv <CSV> --full --stations 15
        # --full: 手順2 の全体構成で走り、モデル作成まで通す（--stations で格子ステーションを絞る）
"""

from __future__ import annotations

import argparse
import asyncio
import csv
import math
import sys
from pathlib import Path

from tests.research import hardware as hwmod
from tests.research.config import DEFAULT_CONFIG_PATH, ConfigError, ResearchConfig, load_config
from tests.research.drive_log import SECTION_PATTERN_DRIVE, read_drive_logs
from tests.research.grid_patterns import build_grid_patterns
from tests.research.grid_planner import StepKind, StepResult
from tests.research.learning_patterns import LearningDataError
from tests.research.mode_drive import load_mode
from tests.research.pattern_drive import (
    PatternDriveResult,
    build_ff_model,
    build_patterns,
    run_pattern_drive,
)
from tests.research.pattern_loop import GridLaunchPattern, GridStationPattern
from tests.research.pedal_search import PedalSearchResult
from tests.research.research_types import VEHICLE_STOP_SPEED_KMH, LearningPattern
from tests.research.term import say
from tests.research.vehicle import build_vehicle_profile, feedforward_params, opening_to_pulse
from tests.research.wltp_grid import (
    WltpCellStats,
    coverage_table,
    data_cells,
    edges_from_config,
    find_holes,
    holes_text,
    load_coverage_stats,
    mode_from_csv,
    outside_seconds,
    outside_text,
    summary_text,
    wltp_cell_stats,
)

STUB_STOP_HOLD_OFFSET_PCT = 8.0  # 停車保持ブレーキ = スタブのブレーキの遊び + これ


def _stub_pedal(cfg: ResearchConfig) -> PedalSearchResult:
    """スタブ車両の遊びをそのまま不感帯にした 2-0 の結果（2-0 を飛ばすため）。"""
    return PedalSearchResult(
        creep_speed_kmh=feedforward_params(cfg).creep_speed_kmh,
        accel_deadband_pct=hwmod.STUB_ACCEL_PLAY_PCT,
        brake_deadband_pct=hwmod.STUB_BRAKE_PLAY_PCT,
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


def _step_phases(csv_path: Path) -> list[str]:
    """パターン走行の行の phase 列（read_drive_logs の DriveLog.id と同じ順）。"""
    with csv_path.open(newline="", encoding="utf-8") as f:
        return [
            row["phase"] for row in csv.DictReader(f) if row["section"] == SECTION_PATTERN_DRIVE
        ]


def _result_summary(results: list[StepResult], true_accel: float, true_brake: float) -> str:
    verdicts: dict[str, int] = {}
    for r in results:
        verdicts[r.verdict] = verdicts.get(r.verdict, 0) + 1
    accel_g = [r.gain_after for r in results
               if r.kind in (StepKind.ACCEL, StepKind.LAUNCH) and not math.isnan(r.gain_after)]
    brake_g = [r.gain_after for r in results
               if r.kind in (StepKind.DECEL_BRAKE, StepKind.STOP) and not math.isnan(r.gain_after)]

    def trace(values: list[float]) -> str:
        return " → ".join(f"{v:.2f}" for v in values) if values else "（測定なし）"

    return "\n".join([
        f"ステップ {len(results)} 本: " + " / ".join(f"{k} {n}" for k, n in verdicts.items()),
        f"アクセルの感度 g の推移: {trace(accel_g)}（スタブの真値 {true_accel:.2f}）",
        f"ブレーキの感度 g の推移: {trace(brake_g)}（スタブの真値 {true_brake:.2f}）",
    ])


def _print_coverage(
    cfg: ResearchConfig, stats: WltpCellStats, result: PatternDriveResult, pedal: PedalSearchResult
) -> None:
    lr = cfg.learning
    min_s = lr.grid_target_min_s
    edges = edges_from_config(lr.grid_speed_edges_kmh, lr.grid_accel_edges_kmhs)
    logs = read_drive_logs(result.csv_path)
    phases = _step_phases(result.csv_path)
    for title, keep in (
        ("開度固定のステップの行だけ（HOLD_STEP）", lambda log: phases[log.id] == "HOLD_STEP"),
        ("全行（PI 追従中の行を含む。学習に渡る行）", None),
    ):
        data = data_cells(
            logs, pedal.accel_deadband_pct, pedal.brake_deadband_pct, edges, keep=keep
        )
        holes = find_holes(stats.seconds, data, edges, min_s, lr.grid_hole_data_max_s)
        say(f"\n── 網羅マップ: {title} ──")
        say("セル = 「モードが要る秒 / 学習データの秒」、* = 穴")
        say(coverage_table(stats.seconds, data, edges, min_s, lr.grid_hole_data_max_s))
        say(summary_text(stats.seconds, data, holes, min_s, lr.grid_hole_data_max_s))
        say(holes_text(holes))


def _full_patterns(
    cfg: ResearchConfig, pedal: PedalSearchResult, wltp_stats: object,
    stations: list[float] | None, *, with_launch: bool,
) -> list[LearningPattern]:
    """手順2 の全体構成。`--stations`・`--no-launch` は格子ステップ走行の部分だけを絞る。"""
    profile = pedal.apply_to_profile(build_vehicle_profile(cfg))
    patterns = build_patterns(cfg, profile, wltp_stats)  # type: ignore[arg-type]
    wanted = None if stations is None else set(stations)
    return [
        p for p in patterns
        if not (isinstance(p, GridStationPattern) and wanted is not None
                and (p.plan is None or p.plan.speed_kmh not in wanted))
        and not (isinstance(p, GridLaunchPattern) and not with_launch)
    ]


async def _run(args: argparse.Namespace) -> int:
    cfg = load_config(args.config)
    cfg.learning.timeout_s = max(cfg.learning.timeout_s, args.timeout_s)  # 保存しない
    cfg.learning.grid_settle_timeout_s = args.settle_timeout_s  # スタブは応答が遅いので長めに
    cfg.output.plot = False
    if args.sweep_max_passes is not None:  # 保存しない（スタブは遅いので回数を絞る用）
        cfg.learning.grid_sweep_max_passes = args.sweep_max_passes
    edges = edges_from_config(cfg.learning.grid_speed_edges_kmh, cfg.learning.grid_accel_edges_kmhs)
    if args.ref_csv is not None:
        mae_mode = mode_from_csv(args.ref_csv)  # 2-2 の MAE_WLTP 用（DB を使わない）
        wltp_stats = wltp_cell_stats(mae_mode, edges)
        outside = [(mae_mode.name, outside_seconds(mae_mode, edges))]
    else:  # 手順2 と同じ: modes.coverage_mode_names の全モードを DB から読んで合成
        wltp_stats, outside = await load_coverage_stats(cfg, edges)
        mae_mode = await load_mode(cfg, cfg.modes.wltp_mode_name)
    say("狙うモード（格子の外＝狙えない走りの秒数）:")
    say(outside_text(outside))
    stations = [float(v) for v in args.stations.split(",")] if args.stations else None
    patterns: list[LearningPattern] = []
    if not args.full:
        patterns = build_grid_patterns(
            cfg, wltp_stats, stations_kmh=stations, with_launch=not args.no_launch
        )
    if not args.full and not patterns:
        say("格子ステップ走行のパターンが 1 本もありません（--stations の指定を確認）")
        return 2

    hw = hwmod.build_hardware(cfg, hwmod.HW_STUB)
    vehicle = hw.can.vehicle  # type: ignore[attr-defined]
    vehicle.accel_gain_kmhs_per_pct *= args.gain_scale
    vehicle.brake_gain_kmhs_per_pct *= args.gain_scale
    say(f"スタブ車両の感度: アクセル {vehicle.accel_gain_kmhs_per_pct:.3f} / "
        f"ブレーキ {vehicle.brake_gain_kmhs_per_pct:.3f} km/h/s per %（×{args.gain_scale:g}）")
    await hwmod.run_initialize(hw)
    pedal = _stub_pedal(cfg)
    if args.full:
        patterns = _full_patterns(cfg, pedal, wltp_stats, stations, with_launch=not args.no_launch)
    try:
        await hw.brake.move_to_position(opening_to_pulse(pedal.stop_brake_opening_pct))
        await _wait_stopped(hw)
        result = await run_pattern_drive(hw, cfg, pedal=pedal, patterns=patterns)
    except hwmod.DriveError as exc:
        say(f"走行が中断しました: {exc}")
        return 1
    finally:
        await hwmod.shutdown(hw)

    say(f"\nパターン走行 {result.duration_s:.0f}s（{result.duration_s / 60:.1f} 分）")
    say(f"CSV: {result.csv_path}")
    say(_result_summary(result.grid_results, vehicle.accel_gain_kmhs_per_pct,
                        vehicle.brake_gain_kmhs_per_pct))
    _print_coverage(cfg, wltp_stats, result, pedal)
    if args.full:
        say("\n── 2-2 モデル作成（スタブ。YAML は更新しない） ──")
        try:
            build_ff_model(
                cfg, result.csv_path, hw_mode=hwmod.HW_STUB, pedal=pedal,
                wltp_mode=mae_mode, write_config=False,
            )
        except LearningDataError as exc:
            say(f"モデル作成エラー: {exc}（--stations を絞ると学習サンプルが足りないことがある）")
            return 1
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="格子ステップ走行をスタブ HW で走らせる")
    parser.add_argument("--ref-csv", type=Path, default=None,
                        help="基準車速を持つ CSV（省略時は DB の modes.coverage_mode_names）")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--stations", default="", help="ステーションの中心車速を絞る（例 15,55）")
    parser.add_argument("--no-launch", action="store_true", help="発進・停車セルを走らない")
    parser.add_argument("--full", action="store_true",
                        help="手順2 の全体構成（コースト→格子→クリープ）で走り、モデル作成まで通す")
    parser.add_argument("--sweep-max-passes", type=int, default=None,
                        help="通し掃引 1 本の最大回数（省略時は config。0 で掃引なし）")
    parser.add_argument("--gain-scale", type=float, default=1.0,
                        help="スタブのアクセル・ブレーキゲインの倍率")
    parser.add_argument("--settle-timeout-s", type=float, default=240.0,
                        help="落ち着くのを待つ上限（スタブは最初の接近が遅い）")
    parser.add_argument("--timeout-s", type=float, default=3600.0,
                        help="パターン走行の打ち切り（learning.timeout_s より短くはしない）")
    args = parser.parse_args(argv)
    try:
        return asyncio.run(_run(args))
    except ConfigError as exc:
        print(f"設定エラー: {exc}")
        return 2


if __name__ == "__main__":
    sys.exit(main())
