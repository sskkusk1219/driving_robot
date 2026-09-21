"""研究開発用ハーネスのエントリポイント（ProblemReport_20260910 手順 0〜10）。

本番の制御スタックは層が多く、どの層が KPI を壊しているのか切り分けられない。
そこで `tests/` 側に「FF → PID → 調停 → ペダル」だけの極端に単純な系を組み、
手順を 1 つずつ足しながらプライマリー KPI の達成度を測る。

    目標車速 → Feedforward ─┐
                            ＋ → effort → PedalArbiter → Accel/Brake → 車両
                 PID ───────┘                                          │
                  ↑                                                    │
                  └──────────────── 車速（Feedback） ───────────────────┘

使い方:
    .venv/bin/python -m tests.research.main --list          # 手順一覧
    .venv/bin/python -m tests.research.main --upto 0        # 手順 0 まで実行
    .venv/bin/python -m tests.research.main --only 3        # 手順 3 のみ
    .venv/bin/python -m tests.research.main --steps 0,1     # 手順 0 と 1
    .venv/bin/python -m tests.research.main --upto 1 --hw real   # 実機で初期化まで
    .venv/bin/python -m tests.research.main --steps 1,2 --hw real  # 実機でパターン走行→FF モデル
    .venv/bin/python -m tests.research.main --steps 1,3 --hw real  # 実機で FF のみ WLTP モード走行
    .venv/bin/python -m tests.research.main --steps 1,3 --limit-s 60  # スタブで先頭 60s だけ

実行順:
    手順 1（初期化）→ 走行前チェック（通信確認・車速 0 km/h 確認）→ 手順 2 以降（走行）
    走行前チェックは、走行する手順の最初の 1 つの前に 1 回だけ行う。

終了コード:
    0 成功 / 2 設定エラー / 3 未実装の手順に到達 / 4 初期化エラー / 5 走行エラー
    / 6 モデル作成エラー / 7 走行前チェックエラー

安全上の注意:
    `--hw real` は実アクチュエータを物理的に動かす。**ユーザーが自分で指定する**もので、
    既定は `stub`（本番と同じスタブ HW）。
"""

from __future__ import annotations

import argparse
import asyncio
import sys
import time
from collections.abc import Awaitable, Callable
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from src.domain.learning_drive import LearningDataError
from tests.research.config import (
    ConfigError,
    ResearchConfig,
    load_config,
    validate_config,
)
from tests.research.drive_log import SECTION_MODE_DRIVE, SECTION_PRE_DRIVE_CHECK, SessionLog
from tests.research.hardware import (
    HW_REAL,
    HW_STUB,
    InitializationError,
    PreDriveCheckError,
    ResearchHardware,
    build_hardware,
    run_initialize,
    shutdown,
)
from tests.research.mode_drive import ModeDriveSetup, prepare_mode_drive, run_mode_drive
from tests.research.mode_report import RunInfo, rows_from_samples, write_mode_report
from tests.research.pattern_drive import DriveError, build_ff_model, run_pattern_drive
from tests.research.pedal_search import run_pedal_search
from tests.research.pre_drive_check import PHASE_PRE_CHECK, run_pre_drive_check
from tests.research.term import banner, display_width, say
from tests.research.vehicle import STROKE_LIMIT_PULSE

DEFAULT_CONFIG = Path("tests/research/config_testVehicle.yaml")


class StepNotImplementedError(Exception):
    """まだ実装していない手順。1 手順ずつ進める方針のため意図的に投げる。"""


@dataclass
class RunContext:
    """1 回の実行を通して手順間で共有する状態。"""

    config: ResearchConfig
    hw_mode: str
    started_at: float
    # 手順 1 で構築・初期化し、以降の手順（走行）が使い回す。main が終了時に必ず片付ける。
    hardware: ResearchHardware | None = None
    # 走行前チェック（停車確認まで）を済ませたか。走行する手順の前に 1 回だけ行う
    pre_drive_checked: bool = False
    # 走行前チェック〜停車保持を 1 本に残す走行ログ。走行前チェックの直前に作り、手順 2 の停車保持の
    # 後（異常終了なら main の finally で原点復帰の後）に保存する
    session_log: SessionLog | None = None
    # モード走行（手順 3）の先頭だけ走る秒数（--limit-s。動作確認用）。None ならモード全体
    limit_s: float | None = None
    # モード走行の準備（走行モードと FF モデル）。走行前チェックより前に読み込む
    mode_setup: ModeDriveSetup | None = None
    # 設定検証を済ませたか。手順0 を含まない実行でも run_steps の冒頭で 1 回だけ行う
    config_validated: bool = False

    @property
    def is_real_hw(self) -> bool:
        return self.hw_mode == HW_REAL

    def require_hardware(self) -> ResearchHardware:
        """初期化済み HW を返す。未初期化なら手順 1 を先に走らせるよう促す。"""
        if self.hardware is None or not self.hardware.initialized:
            raise InitializationError(
                "ハードウェアが初期化されていません。手順 1 を先に実行してください"
                "（例: --upto 3 のように手順 1 を含めて実行する）"
            )
        return self.hardware


@dataclass(frozen=True)
class Step:
    number: int
    title: str
    handler: Callable[[RunContext], Awaitable[None]]
    drives: bool = False  # 車両を走らせる手順（前に走行前チェックを行う）
    # 走行前チェックより前に行う準備（DB・モデルの読み込みなど。失敗しても HW に触らずに止まる）
    prepare: Callable[[RunContext], Awaitable[None]] | None = None
    has_ref: bool = False  # 基準車速に沿って走る手順（走行グラフに基準車速を描く）


# ─────────────────────────────────────────────────────────────────────
# 手順 0: 設定ファイルの作成・検証
# ─────────────────────────────────────────────────────────────────────


def validate_config_once(ctx: RunContext) -> None:
    """設定を検証する（1 回だけ）。問題があれば ConfigError を送出する。

    手順0 を通らない実行（--only 2 / --only 3 など）でも設定検証が素通りしないよう、
    run_steps の冒頭で必ず呼ぶ。手順0 からも呼ばれるため、二重表示を防ぐために
    ctx.config_validated で 1 回だけに制限する。
    """
    if ctx.config_validated:
        return
    cfg = ctx.config
    problems = validate_config(cfg)
    if problems:
        say(f"設定の検証で {len(problems)} 件の問題が見つかりました:")
        for problem in problems:
            say(f"  ✗ {problem}")
        raise ConfigError("設定ファイルを修正してから再実行してください")
    say("設定の検証: OK（値域・相互整合）")
    ctx.config_validated = True


async def step0_setup(ctx: RunContext) -> None:
    """テスト用車両プロファイルを読み込み、値域を検証して要約を表示する。

    以降の手順が参照するディレクトリ（results / models）もここで作る。
    """
    cfg = ctx.config
    say(f"設定ファイル: {cfg.source_path}")

    validate_config_once(ctx)

    for label, directory in (
        ("結果", cfg.results_path),
        ("モデル", Path(cfg.feedforward.model_path).parent),
    ):
        directory.mkdir(parents=True, exist_ok=True)
        say(f"{label}ディレクトリ: {directory}")

    settings_path = Path(cfg.hardware.settings_path)
    if settings_path.exists():
        say(f"本番ハードウェア設定: {settings_path}（存在確認 OK）")
    else:
        say(f"警告: 本番ハードウェア設定が見つかりません: {settings_path}")
        say("      手順 1（初期化）で必要になります。")
        say("      config/settings.toml.example からコピーしてください")

    say()
    _print_summary(cfg)
    say()
    say("書き戻しテスト（コメント保持の確認）:")
    changed = cfg.save({"pid.kp": cfg.pid.kp})
    say(f"  {changed[0] if changed else '変更なし'}  ← 同値で上書きし、読み直して一致を確認")

    say()
    say("手順 0 完了。次は手順 1（初期化）です。")


def _print_summary(cfg: ResearchConfig) -> None:
    """ユーザーが「今どの値で走ろうとしているか」を 1 画面で確認できる要約。"""
    ff = cfg.feedforward
    ps = cfg.pedal_search
    rows: list[tuple[str, str]] = [
        ("車両名", cfg.vehicle.name),
        ("最高速", f"{cfg.vehicle.max_speed_kmh:.1f} km/h"),
        ("最大減速G", f"{cfg.vehicle.max_decel_g:.2f} G"),
        ("開度上限 (アクセル/ブレーキ)",
         f"{cfg.vehicle.max_accel_opening_pct:.0f} / {cfg.vehicle.max_brake_opening_pct:.0f} %"),
        ("開度 0%→100%", f"原点 0 → {STROKE_LIMIT_PULSE} pulse（キャリブレーション不使用）"),
        ("ペダル探索",
         f"{ps.step_mm:g}mm 刻み・待ち {ps.dwell_s:g}s・判定 ±{ps.onset_margin_kmh:g} km/h × "
         f"{ps.confirm_count}・停車保持 +{ps.stop_hold_margin_pct:g}%"),
        ("クリープ安定判定",
         f"{ps.creep_window_s:g}s 平均の傾き < {ps.creep_settle_kmhs:g} km/h/s・"
         f"最短 {ps.creep_settle_min_s:g}s・最大 {ps.creep_timeout_s:g}s"),
        ("走行後の緩減速",
         f"目標 {cfg.decel_stop.target_decel_g:g}G（踏み増し < "
         f"{cfg.decel_stop.target_decel_g - cfg.decel_stop.press_margin_g:g}G・戻し > "
         f"{cfg.decel_stop.release_above_g:g}G）・{cfg.decel_stop.step_mm:g}mm 刻み・"
         f"待ち {cfg.decel_stop.dwell_s:g}s"),
        ("FF モデル", ff.model_path),
        ("FF モデル学習済み", "はい" if ff.is_model_trained else "いいえ（手順 2 で作成）"),
        ("FF カーブ同定済み", "はい" if ff.is_curves_identified else "いいえ（手順 2 で同定）"),
        ("不感帯 (アクセル/ブレーキ)",
         f"{ff.accel_deadband_pct:.1f} / {ff.brake_deadband_pct:.1f} %"),
        ("PID ゲイン", f"Kp={cfg.pid.kp:g}  Ki={cfg.pid.ki:g}  Kd={cfg.pid.kd:g}"),
        ("PID 出力上限", f"{cfg.pid.output_limit_pct:.0f} %"),
        ("調停オプション", _arbiter_summary(cfg)),
        ("減速G ガバナー（モード走行）",
         f"{'有効' if cfg.mode_drive.decel_governor else '無効'}"
         f"（{cfg.vehicle.max_decel_g:g}G × 0.98 で頭打ち・"
         f"{cfg.mode_drive.governor_reduce_step_pct:g}%/周期ずつ下げる）"),
        ("制御周期 / ログ周期",
         f"{cfg.control.loop_interval_ms} ms / {cfg.control.log_interval_ms} ms"
         f"（{cfg.control.log_every_n_cycles} サイクルごと）"),
        ("モード走行", cfg.modes.wltp_mode_name),
        ("PID 適合パターン", cfg.modes.tuning_mode_name),
        ("適合の最大走行本数", f"{cfg.tuning.max_runs} 本（採用基準 {cfg.tuning.select_metric}）"),
        ("KPI 最大逸脱", f"|dev| <= {cfg.kpi.max_abs_deviation_kmh:.1f} km/h（例外なし）"),
        ("KPI p95", f"p95 <= {cfg.kpi.p95_deviation_kmh:.1f} km/h"),
        ("KPI 振動抑制",
         f"±{cfg.kpi.reversal_band_kmh:.1f} km/h の符号反転 <= "
         f"{cfg.kpi.reversal_limit_per_window:g} 回 / {cfg.kpi.reversal_window_s:g} s"),
        ("出力先", f"{cfg.output.results_dir}（CSV {cfg.output.csv_interval_s:g}s 刻み）"),
    ]
    width = max(display_width(label) for label, _ in rows)
    say("── テスト用車両プロファイル 要約 " + "─" * 34)
    for label, value in rows:
        pad = " " * (width - display_width(label))
        say(f"  {label}{pad} : {value}")
    say("─" * 68)


def _arbiter_summary(cfg: ResearchConfig) -> str:
    enabled = [
        name
        for name, on in (
            ("不感帯補償", cfg.arbiter.enable_deadband_compensation),
            ("レートリミット", cfg.arbiter.enable_rate_limit),
            ("ヒステリシス", cfg.arbiter.enable_hysteresis),
        )
        if on
    ]
    return " + ".join(enabled) if enabled else "すべて無効（第1段階）"


# ─────────────────────────────────────────────────────────────────────
# 手順 1: 初期化（本番の初期化シーケンスを引用）
# ─────────────────────────────────────────────────────────────────────


async def step1_initialize(ctx: RunContext) -> None:
    """ハードウェアを構築し、本番と同じ順で初期化して合否を表示する。

    サーボ通信チェック → エラー消去 → サーボON → CAN 通信チェック → UPS 通信チェック
    → アクチュエータ初期位置（原点復帰）。1 項目でも NG なら例外で止める。
    """
    if ctx.hardware is not None and ctx.hardware.initialized:
        say("すでに初期化済みです（スキップ）")
        return

    hw = build_hardware(ctx.config, ctx.hw_mode)
    # 構築できた時点で ctx に持たせる。以降で失敗しても main の finally が片付けられるように。
    ctx.hardware = hw
    if hw.is_real:
        say("実機モード: アクチュエータが物理的に動きます。周囲の安全を確認してください")

    report = await run_initialize(hw, ctx.config.checks)
    say(f"初期化 OK {report.ok_count} 項目 / SKIP {report.skip_count} 項目")
    say("手順 1 完了。次は走行前チェック（通信確認・車速 0 km/h 確認）→ 手順 2 です。")


# ─────────────────────────────────────────────────────────────────────
# 走行前チェック（手順 1 と手順 2 の間。本番の走行前チェックを引用）
# ─────────────────────────────────────────────────────────────────────

PRE_DRIVE_CHECK_TITLE = (
    "走行前チェック（通信確認・車速 0 km/h 確認。ブレーキは次の手順が 2 なら小刻みに、"
    "それ以外は stop_brake_opening_pct まで一気に踏む）"
)


async def pre_drive_check(ctx: RunContext, *, has_ref: bool = False, next_step: int) -> None:
    """初期化済み HW で走行前チェックを行い、停車保持の状態にする。"""
    hw = ctx.require_hardware()
    if ctx.session_log is None:
        log = SessionLog(ctx.config, hw, has_ref=has_ref)
        log.start(SECTION_PRE_DRIVE_CHECK, PHASE_PRE_CHECK)
        ctx.session_log = log
    await run_pre_drive_check(hw, ctx.config, log=ctx.session_log, next_step=next_step)
    ctx.pre_drive_checked = True
    say("走行前チェック完了。")


# ─────────────────────────────────────────────────────────────────────
# 手順 2: 閉ループパターン走行 → 2次多項式 FF モデル作成
# ─────────────────────────────────────────────────────────────────────


async def step2_pattern_drive(ctx: RunContext) -> None:
    """ペダル探索 → 本番の学習運転パターンで走行 → その CSV から 2次多項式 FF モデルを作る。"""
    hw = ctx.require_hardware()
    log = ctx.session_log
    say("2-0. ペダル探索（不感帯と停車保持開度を車速応答で測り、停車保持する）")
    pedal = await run_pedal_search(hw, ctx.config, log=log)
    say()
    say("2-1. パターン走行（本番の学習運転と同じパターン列・PatternLoop）→ 緩減速で停車保持")
    drive = await run_pattern_drive(hw, ctx.config, pedal=pedal, log=log)
    if log is not None:
        await log.close()  # 停車保持まで記録したら保存し、その CSV（パターン走行の行）で学習する
    say()
    say("2-2. 走行結果から 2次多項式 FF モデルを作成")
    await asyncio.to_thread(
        build_ff_model, ctx.config, drive.csv_path, hw_mode=hw.hw_mode, pedal=pedal
    )
    if hw.is_real:
        ctx.config = load_config(ctx.config.source_path)  # 以降の手順は書き戻した値で走る
    say()
    say("手順 2 完了。次は手順 3（FF のみで WLTP モード走行）です。")


# ─────────────────────────────────────────────────────────────────────
# 手順 3: FF のみで WLTP モード走行 → CSV / 図 / レポート
# ─────────────────────────────────────────────────────────────────────


async def prepare_step3(ctx: RunContext) -> None:
    """走行前チェックの前に、走行モード（DB）と FF モデルを読み込んでおく。"""
    ctx.mode_setup = await prepare_mode_drive(ctx.config, ctx.config.modes.wltp_mode_name)


def _session_log_for_drive(ctx: RunContext, hw: ResearchHardware) -> SessionLog:
    """走行前チェックから続くログがあればそれを、手順 2 で閉じていれば新しく作って使う。"""
    log = ctx.session_log
    if log is None or log.closed:
        log = SessionLog(ctx.config, hw, has_ref=True)
        log.start(SECTION_MODE_DRIVE, "")
        ctx.session_log = log
    return log


async def step3_mode_drive_ff(ctx: RunContext) -> None:
    """停車保持の状態から WLTP を FF だけで走り、CSV（0.1s 刻み）とレポートを残す。"""
    hw = ctx.require_hardware()
    setup = ctx.mode_setup
    if setup is None:  # prepare_step3 が走行前チェックの前に作る
        raise ConfigError("走行モードと FF モデルが読み込まれていません")
    log = _session_log_for_drive(ctx, hw)
    started_at = datetime.now()
    result = await run_mode_drive(hw, ctx.config, setup, log=log, limit_s=ctx.limit_s)
    await log.close()  # 停車保持まで記録したら保存し、その行でレポートを作る

    limited = result.mode_duration_s < setup.mode.total_duration
    notes = []
    if not hw.is_real:
        notes.append("スタブ HW の結果（車両モデルは実車の同定値ではない。流れの確認用）。")
    if limited:
        notes.append(
            f"--limit-s でモードの先頭 {result.mode_duration_s:.0f}s だけ走った（KPI は参考値）。"
        )
    info = RunInfo(
        label="FF",
        title="手順 3: FF のみでモード走行",
        controller="FF のみ（Kp=Ki=Kd=0）→ effort の符号でアクセル/ブレーキに振り分け",
        csv_path=log.csv_path,
        hw_mode=hw.hw_mode,
        mode_name=result.mode_name,
        started_at=started_at,
        mode_duration_s=result.mode_duration_s,
        run_duration_s=result.run_duration_s,
        completed=result.completed,
        limited=limited,
        abort_reason=result.abort_reason,
        cycles=result.cycles,
        overruns=result.overruns,
        notes=notes,
    )
    rows = rows_from_samples(result.samples)
    say()
    if rows:
        try:
            await asyncio.to_thread(
                write_mode_report, rows, ctx.config, info, ctx.config.results_path
            )
        except Exception as exc:  # 走行ログは保存済み。レポートは CSV から作り直せる
            say(f"警告: レポートを作れませんでした（{type(exc).__name__}: {exc}）。作り直し:")
            say(f"  .venv/bin/python -m tests.research.mode_report {log.csv_path} --label FF")
    else:
        say("モード走行の行が無いため、レポートは作りません")
    if not result.completed:
        raise DriveError(result.abort_reason)
    say()
    say("手順 3 完了。次は手順 4（Kp のみ適合）です。")


# ─────────────────────────────────────────────────────────────────────
# 手順 4 以降（1 手順ずつ実装する方針のため、未実装は明示的に停止する）
# ─────────────────────────────────────────────────────────────────────


def _pending(number: int) -> Callable[[RunContext], Awaitable[None]]:
    async def handler(ctx: RunContext) -> None:  # noqa: RUF029
        raise StepNotImplementedError(
            f"手順 {number} は未実装です。"
            "手順は 1 つずつ実装し、ユーザーの動作確認を経て次へ進みます"
            "（docs/Problem/ProblemReport_20260910.md「遵守事項」）。"
        )

    return handler


STEPS: tuple[Step, ...] = (
    Step(0, "設定作成・検証（config_testVehicle.yaml）", step0_setup),
    Step(1, "初期化（サーボ通信・エラー消去・CAN・UPS・初期位置）", step1_initialize),
    Step(
        2,
        "ペダル探索 → 閉ループパターン走行 → 2次多項式 FF モデル作成",
        step2_pattern_drive,
        drives=True,
    ),
    Step(
        3,
        "FF のみで WLTP モード走行 → CSV/図/レポート",
        step3_mode_drive_ff,
        drives=True,
        prepare=prepare_step3,
        has_ref=True,
    ),
    Step(4, "Kp のみ適合（最大 10 本）", _pending(4)),
    Step(5, "FF + Kp で WLTP モード走行", _pending(5)),
    Step(6, "Kp + Ki 適合", _pending(6)),
    Step(7, "FF + Kp + Ki で WLTP モード走行", _pending(7)),
    Step(8, "Kp + Ki + Kd 適合", _pending(8)),
    Step(9, "FF + Kp + Ki + Kd で WLTP モード走行", _pending(9)),
    Step(10, "制約の追加（不感帯 / レートリミット / 先読み / ILC）", _pending(10)),
)
_STEP_BY_NUMBER = {step.number: step for step in STEPS}
MAX_STEP = max(_STEP_BY_NUMBER)


# ─────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m tests.research.main",
        description="研究開発用ハーネス: FF/PID を 1 段ずつ足して KPI 達成度を測る",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=_steps_help(),
    )
    target = parser.add_mutually_exclusive_group()
    target.add_argument(
        "--upto",
        type=int,
        metavar="N",
        help=f"手順 0 から N までを順に実行する（0〜{MAX_STEP}、既定 0）",
    )
    target.add_argument("--only", type=int, metavar="N", help="手順 N だけを実行する")
    target.add_argument(
        "--steps", metavar="LIST", help="実行する手順をカンマ区切りで指定する（例 0,2,3）"
    )
    parser.add_argument(
        "--config", type=Path, default=DEFAULT_CONFIG, help=f"設定ファイル（既定 {DEFAULT_CONFIG}）"
    )
    parser.add_argument(
        "--hw",
        choices=(HW_STUB, HW_REAL),
        default=HW_STUB,
        help=f"ハードウェアモード。{HW_REAL} は実アクチュエータが動く（既定 {HW_STUB}）",
    )
    parser.add_argument(
        "--limit-s",
        type=float,
        metavar="S",
        help="モード走行（手順 3）をモードの先頭 S 秒で打ち切る（動作確認用。KPI は参考値）",
    )
    parser.add_argument(
        "--retrain",
        type=Path,
        metavar="CSV",
        help="走行せずに、既存の走行 CSV から FF モデルだけを作り直す"
        "（手順 2-2 のモデル作成だけを実行する。--hw real なら設定ファイルへ書き戻す）",
    )
    parser.add_argument("--list", action="store_true", help="手順一覧を表示して終了する")
    parser.add_argument(
        "--dry-run", action="store_true", help="実行する手順を表示するだけで走らせない"
    )
    return parser


def _steps_help() -> str:
    lines = ["手順:"]
    for step in STEPS:
        if step.drives and not any(s.drives for s in STEPS if s.number < step.number):
            lines.append(f"   -  {PRE_DRIVE_CHECK_TITLE}（走行する手順の前に 1 回）")
        lines.append(f"  {step.number:>2}  {step.title}")
    return "\n".join(lines)


def resolve_steps(args: argparse.Namespace) -> list[Step]:
    """CLI 引数から実行対象の手順を決める（既定は手順 0 のみ）。"""
    if args.only is not None:
        numbers = [args.only]
    elif args.steps is not None:
        try:
            numbers = [int(token) for token in args.steps.split(",") if token.strip()]
        except ValueError as exc:
            raise SystemExit(f"--steps の書式が不正です: {args.steps!r}") from exc
        if not numbers:
            raise SystemExit("--steps に手順番号がありません")
    else:
        upto = 0 if args.upto is None else args.upto
        numbers = list(range(0, upto + 1))

    unknown = sorted({n for n in numbers if n not in _STEP_BY_NUMBER})
    if unknown:
        raise SystemExit(
            f"手順 {', '.join(map(str, unknown))} は存在しません（0〜{MAX_STEP}）"
        )
    return [_STEP_BY_NUMBER[n] for n in numbers]


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    if args.list:
        print(_steps_help())
        return 0

    if args.retrain is not None:
        return retrain_only(args)

    steps = resolve_steps(args)
    if args.limit_s is not None and args.limit_s <= 0.0:
        raise SystemExit("--limit-s は正の秒数で指定してください")
    banner("研究開発用ハーネス（ProblemReport_20260910）")
    say(f"ハードウェアモード: {args.hw}")
    if args.hw == HW_REAL:
        say("*** 実機モード: アクセル/ブレーキアクチュエータが物理的に動きます ***")
    say("実行する手順: " + ", ".join(str(step.number) for step in steps))
    if args.limit_s is not None:
        say(f"モード走行は先頭 {args.limit_s:g}s で打ち切ります（動作確認用）")
    if args.dry_run:
        checked = False
        for step in steps:
            if step.drives and not checked:
                say(f"  (dry-run) {PRE_DRIVE_CHECK_TITLE}")
                checked = True
            say(f"  (dry-run) 手順 {step.number}: {step.title}")
        return 0

    try:
        config = load_config(args.config)
    except ConfigError as exc:
        say(f"設定の読み込みに失敗しました: {exc}")
        return 2

    ctx = RunContext(
        config=config, hw_mode=args.hw, started_at=time.monotonic(), limit_s=args.limit_s
    )
    # 全手順を 1 つのイベントループで回す。手順 1 で開いた HW 接続（pymodbus/python-can の
    # クライアントはループに紐づく）を手順 2 以降で使い回すため、手順ごとに asyncio.run
    # しないこと。
    return asyncio.run(run_steps(ctx, steps))


def retrain_only(args: argparse.Namespace) -> int:
    """走行せずに、既存の走行 CSV から FF モデルだけを作り直す（手順 2-2 だけ）。

    レポート 表 5-5 の関門 2（A1 学習行の選別）を、実機を動かさずに確かめるための入口。
    """
    banner("FF モデルの作り直し（走行なし・手順 2-2 のみ）")
    csv_path = args.retrain
    if not csv_path.exists():
        say(f"走行 CSV がありません: {csv_path}")
        return 2
    try:
        config = load_config(args.config)
    except ConfigError as exc:
        say(f"設定の読み込みに失敗しました: {exc}")
        return 2
    say(f"ハードウェアモード: {args.hw}（走行はしません）")
    try:
        build_ff_model(config, csv_path, hw_mode=args.hw, pedal=None)
    except LearningDataError as exc:
        say(f"モデル作成エラー: {exc}")
        return 6
    say()
    say("完了（FF モデルの作り直し）")
    return 0


async def run_steps(ctx: RunContext, steps: list[Step]) -> int:
    """手順を順に実行する。終了時は成否にかかわらず必ず HW を片付ける。"""
    exit_code = 0
    try:
        try:
            # 手順0 を含まない実行（--only 2 / --only 3 など）でも、手順を 1 つも動かす前に
            # 設定検証を通す。ループ内の except ConfigError では拾えない位置のため個別に囲む
            validate_config_once(ctx)
        except ConfigError as exc:
            say(f"設定エラー: {exc}")
            exit_code = 2
            return exit_code
        for step in steps:
            try:
                # 手順 1（初期化）→ [準備] → 走行前チェック → 走行する手順
                # （本番も走行開始前に必ず行う）
                if step.drives:
                    ctx.require_hardware()
                    if step.prepare is not None:
                        say()
                        banner(f"手順 {step.number} の準備（走行前チェックの前に読み込み）")
                        await step.prepare(ctx)
                    if not ctx.pre_drive_checked:
                        say()
                        banner(PRE_DRIVE_CHECK_TITLE)
                        await pre_drive_check(ctx, has_ref=step.has_ref, next_step=step.number)
                say()
                banner(f"手順 {step.number}: {step.title}")
                await step.handler(ctx)
            except StepNotImplementedError as exc:
                say(f"停止: {exc}")
                exit_code = 3
                break
            except ConfigError as exc:
                say(f"設定エラー: {exc}")
                exit_code = 2
                break
            except InitializationError as exc:
                say(f"初期化エラー: {exc}")
                exit_code = 4
                break
            except PreDriveCheckError as exc:
                say(f"走行前チェックエラー: {exc}")
                exit_code = 7
                break
            except DriveError as exc:
                say(f"走行エラー: {exc}")
                exit_code = 5
                break
            except LearningDataError as exc:
                say(f"モデル作成エラー: {exc}")
                exit_code = 6
                break
    finally:
        log = ctx.session_log
        if log is not None:
            await log.stop_sampler()  # 終了処理（原点復帰・切断）中に読みに行かない
        # 途中で失敗しても、ペダルを踏んだままプロセスを終わらせない
        if ctx.hardware is not None:
            say()
            await shutdown(ctx.hardware)
        if log is not None:
            await log.close()  # 異常終了でもそこまでのログを残す（ペダルを離した後に保存）
    if exit_code == 0:
        say()
        say(f"完了（手順 {', '.join(str(s.number) for s in steps)}）")
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
