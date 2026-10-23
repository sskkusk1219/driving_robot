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
    .venv/bin/python -m tests.research.main --steps 1,12 --hw real  # FEL の補正表だけ付け直す

実行順:
    手順 1（初期化）→ 走行前チェック（通信確認・車速 0 km/h 確認）→ 手順 2 以降（走行）
    走行前チェックは、走行する手順の最初の 1 つの前に 1 回だけ行う。

終了コード:
    0 成功 / 2 設定エラー / 3 未実装の手順に到達 / 4 初期化エラー / 5 走行エラー
    / 6 モデル作成エラー / 7 走行前チェックエラー
    / 8 手順2 の学習結果のチェックが NG（yaml は書き換えていない。後の手順には進まない）

安全上の注意:
    `--hw real` は実アクチュエータを物理的に動かす。既定は `stub`（本番と同じスタブ HW）。
    Claude が無人で実行してよい条件は `.claude/skills/bench-verification/SKILL.md` に従う。
"""

from __future__ import annotations

import argparse
import asyncio
import sys
import time
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, replace
from datetime import datetime
from pathlib import Path

from tests.research.config import (
    ConfigError,
    ResearchConfig,
    keys_missing_in_yaml,
    load_config,
    validate_config,
)
from tests.research.drive_log import (
    SECTION_EXCITE,
    SECTION_FEL_DRIVE,
    SECTION_MODE_DRIVE,
    SECTION_PRE_DRIVE_CHECK,
    SessionLog,
)
from tests.research.excite import _format_table as excite_format_table
from tests.research.excite import analyze as excite_analyze
from tests.research.excite import run_excite
from tests.research.fel import (
    FelParams,
    attach_fel,
    fel_rows,
    format_coverage,
    wltp_coverage,
)
from tests.research.fel_pattern import FelPattern, format_pattern, make_pattern
from tests.research.fwd_map import load_forward_map, save_table
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
from tests.research.learning_patterns import LearningDataError
from tests.research.mode_drive import (
    ModeDriveSetup,
    fel_text,
    load_mode,
    prepare_mode_drive,
    prepare_mode_drive_from,
    run_mode_drive,
)
from tests.research.mode_report import RunInfo, rows_from_samples, write_mode_report
from tests.research.pattern_drive import (
    DriveError,
    build_ff_model,
    require_forward_candidate,
    run_pattern_drive,
)
from tests.research.pedal_arbiter import enabled_arbiter_features
from tests.research.pedal_search import run_pedal_search
from tests.research.pre_drive_check import PHASE_PRE_CHECK, run_pre_drive_check
from tests.research.ramp_planner import (
    count_bands_for_modes,
    cruise_kwargs,
    format_plan_table,
    plan_ramps,
    top_speed_cap_kmh,
)
from tests.research.research_types import DrivingMode
from tests.research.step2_check import Step2CheckError, check_fel_coverage, format_checks
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
    # 手順 12（FEL の学習の走りだけ）の準備（走行前チェックより前に読み込む）
    fel_setup: FelRunSetup | None = None

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
    missing = keys_missing_in_yaml(cfg.source_path)
    if missing:
        say(f"yaml に無く既定値が効いている設定: {', '.join(missing)}")
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
         f"{ps.step_mm:g}mm 刻み（ブレーキ踏み込み {ps.brake_step_mm:g}mm）・"
         f"待ち {ps.dwell_s:g}s・停車保持 +{ps.stop_hold_margin_pct:g}%"),
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
        ("触れた位置 min (アクセル/ブレーキ)",
         f"{cfg.vehicle.min_accel_opening_pct:.1f} / {cfg.vehicle.min_brake_opening_pct:.1f} %"
         "（待機位置・u の原点）"),
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
    enabled = enabled_arbiter_features(cfg.arbiter)
    return " + ".join(enabled) if enabled else "すべて無効（符号で振り分けのみ）"


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
    "それ以外は停車保持開度（min ＋ stop_brake_offset_pct）まで一気に踏む）"
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


# candidate ごとの、手順2 が作るモデルの種類（表示用）
_CANDIDATE_KIND: dict[str, str] = {
    **dict.fromkeys(("C1", "C2", "C3", "C4", "C5"), "2次多項式 Ridge 逆モデル"),
    "C7": "順モデルの表",
    "C8": "順モデル",
    "C10": "順モデル（C8 と同じ pkl。走行中に偏差の分を上乗せする）",  # 2026-10-17 段2p
    "C11": "順モデル（C8 と同じ pkl。C10 に外乱オブザーバ DOB の分を足す）",  # 2026-10-18 段2q
    "C12": "順モデル（C8 と同じ pkl。C11 に惰行の行の補助を足す）",  # 2026-10-18 段2r
    "C13": "順モデル（C8 と同じ pkl。C12 に遅れの予測器と車速のフィルタを足す）",  # 2026-10-19 段2u
    "C14": "順モデル（C8 と同じ pkl。C12 に停車の最後のブレーキの下限を足す）",  # 段3a
    # 段4-2
    "C15": "順モデル（C8 と同じ pkl。C12 に停A（停車保持を早める）・停C（停車まぎわの P）を足す）",
}


async def prepare_step2(ctx: RunContext) -> None:
    """走行前チェックの前（ハードを動かす前）に、手順2 の対象外の candidate を止める。"""
    require_forward_candidate(ctx.config)


async def step2_pattern_drive(ctx: RunContext) -> None:
    """ペダル探索 → G ランプのパターン走行 → その CSV から順モデル（C7・C8）を作る。

    2026-10-08 段2l-6: C1〜C5（多項式）は対象外。走る前に ConfigError で止める。
    """
    require_forward_candidate(ctx.config)
    hw = ctx.require_hardware()
    log = ctx.session_log
    # 段2k（ProblemReport_20261001）: G ランプの段と上端の車速は選んだモード
    # （modes.coverage_mode_names）の車速 × 加速度の秒数から作るので、走る前（2-0 の前）に
    # 全モードの基準車速を読む
    # （DB が読めないならここで止まる）。wltp_mode は MAE_WLTP（2-2）用
    cfg = ctx.config
    wltp_mode = await load_mode(cfg, cfg.modes.wltp_mode_name)
    cover_modes = [
        wltp_mode if name == cfg.modes.wltp_mode_name else await load_mode(cfg, name)
        for name in cfg.modes.coverage_mode_names
    ]
    lr = cfg.learning
    # 2026-10-07 段2l-4: 上端 = min(モードの最高車速 + 余裕, 車両の最高速)。ランプと惰行で共通
    top_cap = top_speed_cap_kmh(cover_modes, lr.ramp_top_margin_kmh, cfg.vehicle.max_speed_kmh)
    plan = plan_ramps(
        count_bands_for_modes(
            cover_modes, lr.ramp_g_levels, cruise_kwargs(cfg)["cruise_min_kmh"]
        ),  # 2026-10-14 段2o-5: 戻しの数えは クリープ ＋ 5 km/h 以上で終わる減速
        mode_min_s=lr.ramp_mode_min_s, max_speed_kmh=top_cap,
        budget_s=lr.ramp_time_budget_s,
        max_passes=lr.ramp_max_passes,  # 2026-10-17 段2o-8
        hold_s=lr.ramp_hold_time_s, turn_s=lr.ramp_turn_time_s,
        min_at_g_s=lr.ramp_min_at_g_s, launch_s=lr.ramp_launch_time_s, edge_g=lr.ramp_edge_g,
        **cruise_kwargs(cfg),  # 2026-10-13 段2o-2: いちばん弱い段の途中の定速保持
    )
    say(f"G ランプの段（モード: {', '.join(cfg.modes.coverage_mode_names)}、"
        f"最高速 {top_cap:.1f} km/h）:")
    say(format_plan_table(plan))
    if not plan.levels:
        say("警告: 走る段が 1 つもありません（ramp_g_levels・ramp_mode_min_s を確認）")
    candidate = cfg.feedforward.candidate
    say(f"feedforward.candidate = {candidate}: "
        f"この種類のモデルを作ります（{_CANDIDATE_KIND[candidate]}）")
    say("2-0. ペダル探索（クリープ車速と停車保持開度を車速応答で測り、停車保持する）")
    pedal = await run_pedal_search(hw, ctx.config, log=log)
    say()
    say("2-1. パターン走行（コーストダウン → G ランプ。PatternLoop）→ 緩減速で停車保持")
    drive = await run_pattern_drive(hw, ctx.config, pedal=pedal, log=log,
                                   ramp_order=plan.execution_order(),
                                   top_speed_cap_kmh=top_cap)
    if log is not None:
        await log.close()  # 停車保持まで記録したら保存し、その CSV（パターン走行の行）で学習する
    say()
    say(f"2-2. 物理定数の推定とモデル作成（candidate={candidate}: {_CANDIDATE_KIND[candidate]}）")
    # 重み付けは SAMPLE_WEIGHT_ENABLED のときだけ有効（無効でも wltp_mode を渡すと MAE_WLTP が出る）
    built = await asyncio.to_thread(
        build_ff_model,
        ctx.config, drive.csv_path, hw_mode=hw.hw_mode, pedal=pedal, wltp_mode=wltp_mode,
        ramp_results=drive.ramp_results,  # チェック B5① 用（走行中の最後の効き始め）
    )
    if hw.is_real:
        ctx.config = load_config(ctx.config.source_path)  # 以降の手順は書き戻した値で走る
    # 2026-10-19 段2s-2b: 2-2 のチェックが NG なら build_ff_model が Step2CheckError を出すので
    # ここには来ない（学習の走りは走らない）。スタブは yaml を書かないので、2-2 が作った pkl の
    # パスを直接渡す
    say()
    if not ctx.config.learning.fel_pattern_enabled:
        # 2026-10-20 段4-3: 設定で 2-3 を走らない（ユーザー決定。約 43 → 32 分。pkl は 2-2 のまま）
        say("2-3. 学習用パターン（FEL）: 走らない（learning.fel_pattern_enabled: false）。"
            "補正表は付けません（手順12 で後から走れます）")
        say()
        say("手順 2 完了。次は手順 3（FF のみで WLTP モード走行）です。")
        return
    say(f"2-3. 学習用パターン（FEL。手順3 と同じ候補 {candidate} で走り、"
        f"補正表を {built.model_path} に付ける）")
    fel_run = _prepare_fel_run(ctx.config, built.model_path, cover_modes, top_cap)
    await _run_fel_learning(ctx, hw, fel_run, wltp_mode, checks=built.checks)
    say()
    say("手順 2 完了。次は手順 3（FF のみで WLTP モード走行）です。")


# ─────────────────────────────────────────────────────────────────────
# 学習用パターン（FEL）の走り: 手順2 の 2-3 と手順 12 が共有する
# ─────────────────────────────────────────────────────────────────────


@dataclass
class FelRunSetup:
    """学習の走りの準備: 走る設定（FEL なし・表は model_path）・パターン・走行の準備・表。"""

    cfg: ResearchConfig
    model_path: str
    pattern: FelPattern
    setup: ModeDriveSetup
    wltp_mode: DrivingMode


def _prepare_fel_run(
    cfg: ResearchConfig, model_path: str, cover_modes: list[DrivingMode], top_cap: float,
    wltp_mode: DrivingMode | None = None,
) -> FelRunSetup:
    """パターンを作り、手順3 と同じ候補（cfg.feedforward.candidate）で走る準備をする。

    走る設定は model_path の表を使い、FEL は fel.enabled に関係なく使わない（この走りは
    補正表を作る側。新しい pkl にはまだ補正表が無い）。ここで失敗しても HW には触れない。
    """
    table = load_forward_map(Path(model_path))
    pattern = make_pattern(cfg, cover_modes, top_cap_kmh=top_cap, table=table)
    say(format_pattern(pattern))
    run_cfg = replace(
        cfg,
        feedforward=replace(cfg.feedforward, model_path=str(model_path)),
        fel=replace(cfg.fel, enabled=False),
    )
    say(f"走行の準備（候補 {run_cfg.feedforward.candidate}・FEL なし）:")
    setup = prepare_mode_drive_from(run_cfg, pattern.mode)
    return FelRunSetup(
        cfg=run_cfg, model_path=str(model_path), pattern=pattern, setup=setup,
        wltp_mode=wltp_mode if wltp_mode is not None else cover_modes[0],
    )


async def _run_fel_learning(
    ctx: RunContext,
    hw: ResearchHardware,
    run: FelRunSetup,
    wltp_mode: DrivingMode,
    *,
    checks: list | None = None,
    limit_s: float | None = None,
) -> None:
    """学習用パターンを走り、その CSV から補正表（cell・nn）を作って pkl に付ける。

    走りが止まった・補正表を作れなかったときは、pkl を補正表なしのまま残して DriveError にする
    （手順2 の 2-2 の結果は消さない）。埋まり方（WLTP が通るマスの割合）を出して、お知らせとして
    step2_check の項目 F1 に足す（NG にはしない）。
    """
    try:
        log = _session_log_for_drive(ctx, hw, section=SECTION_FEL_DRIVE, has_ref=True)
        result = await run_mode_drive(
            hw, run.cfg, run.setup, log=log, limit_s=limit_s, section=SECTION_FEL_DRIVE
        )
        await log.close()  # 停車保持まで記録したら保存し、その CSV で補正表を作る
        if not result.completed:
            raise DriveError(f"学習の走りが中断しました: {result.abort_reason}")
        params = FelParams.from_config(run.cfg.fel)
        table = load_forward_map(Path(run.model_path))
        rows = await asyncio.to_thread(fel_rows, log.csv_path, table, params)
        if len(rows) == 0:
            raise DriveError("学習の走りの行がありません（補正表を作れません）")
        new = await asyncio.to_thread(
            attach_fel, table, rows, params, source=log.csv_path.name
        )
        path = save_table(new, Path(run.model_path))
        info = new.meta["fel"]
        say(f"補正表を付けて保存: {path}")
        say(f"  学習 {info['source_csv']}・行 {info['rows']}（{info['total_seconds']:.0f} 秒）・"
            f"マス {info['occupied_cells']}・補正あり(cell) "
            f"{info['cells_with_correction']['cell']}・offset {info['offset_kmhs']}")
        cov = wltp_coverage(new, wltp_mode, params)
        say()
        if cov is not None:
            say(format_coverage(cov))
        item = check_fel_coverage(cov)
        say()
        say("手順2 の学習結果のチェック（お知らせ）:")
        say(format_checks([item]))
        if checks is not None:
            checks.append(item)
    except DriveError as exc:
        say(f"学習の走りを完了できませんでした: {exc}")
        say(f"手順2 の結果（pkl {run.model_path}）は補正表なしのまま残してあります。")
        raise
    except Exception as exc:  # noqa: BLE001 - どんな失敗でも 2-2 の結果は残して走行エラーで終える
        say(f"学習の走りを完了できませんでした（{type(exc).__name__}: {exc}）")
        say(f"手順2 の結果（pkl {run.model_path}）は補正表なしのまま残してあります。")
        raise DriveError(f"学習の走り（FEL）: {type(exc).__name__}: {exc}") from exc


async def prepare_step12(ctx: RunContext) -> None:
    """走行前チェックの前に、モード（DB）と今の pkl からパターンと走行の準備を作っておく。"""
    cfg = ctx.config
    require_forward_candidate(cfg)
    wltp_mode = await load_mode(cfg, cfg.modes.wltp_mode_name)
    cover_modes = [
        wltp_mode if name == cfg.modes.wltp_mode_name else await load_mode(cfg, name)
        for name in cfg.modes.coverage_mode_names
    ]
    top_cap = top_speed_cap_kmh(
        cover_modes, cfg.learning.ramp_top_margin_kmh, cfg.vehicle.max_speed_kmh
    )
    model_path = cfg.feedforward.model_path
    if not Path(model_path).exists():
        raise ConfigError(f"feedforward.model_path の pkl がありません: {model_path}")
    ctx.fel_setup = _prepare_fel_run(cfg, model_path, cover_modes, top_cap, wltp_mode)


async def step12_fel_learning(ctx: RunContext) -> None:
    """学習用パターンだけを走り、今の pkl（yaml の model_path）に補正表を付け直す。"""
    hw = ctx.require_hardware()
    run = ctx.fel_setup
    if run is None:  # prepare_step12 が走行前チェックの前に作る
        raise ConfigError("学習用パターンと FF モデルが読み込まれていません")
    await _run_fel_learning(ctx, hw, run, run.wltp_mode, limit_s=ctx.limit_s)
    say()
    say("手順 12 完了。補正表を付け直しました（次は手順 3 で fel.enabled を切り替えて比べます）。")


# ─────────────────────────────────────────────────────────────────────
# 手順 3: FF のみで WLTP モード走行 → CSV / 図 / レポート
# ─────────────────────────────────────────────────────────────────────


async def prepare_step3(ctx: RunContext) -> None:
    """走行前チェックの前に、走行モード（DB）と FF モデルを読み込んでおく。"""
    ctx.mode_setup = await prepare_mode_drive(ctx.config, ctx.config.modes.wltp_mode_name)


def _session_log_for_drive(
    ctx: RunContext, hw: ResearchHardware, *, section: str = SECTION_MODE_DRIVE,
    has_ref: bool = True,
) -> SessionLog:
    """走行前チェックから続くログがあればそれを、前の手順で閉じていれば新しく作って使う。

    `has_ref`/`section` は手順 11（加振走行）向け: 基準車速が無い走行（`has_ref=False`）は
    グラフに基準線を描かない。
    """
    log = ctx.session_log
    if log is None or log.closed:
        log = SessionLog(ctx.config, hw, has_ref=has_ref)
        log.start(section, "")
        ctx.session_log = log
    return log


def _fel_text_of(cfg: ResearchConfig, setup: ModeDriveSetup) -> str | None:
    """レポート 5 章の FEL の 1 行。表の候補（補正表を持てる FF）でなければ None。"""
    table = getattr(setup.ff, "table", None)
    return None if table is None else fel_text(cfg, table)


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
        controller=f"FF のみ（Kp=Ki=Kd=0）→ 調停: {_arbiter_summary(ctx.config)}",
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
        interlock=(
            None if result.interlock is None else (
                result.interlock.accel_events, result.interlock.accel_s,
                result.interlock.brake_events, result.interlock.brake_s,
            )
        ),
        c10_settings=result.c10_settings,
        c10_summary=None if result.c10 is None else result.c10.summary(),
        dob_settings=result.dob_settings,
        dob_summary=None if result.dob is None else result.dob.summary(),
        coast_settings=result.coast_settings,
        coast_summary=None if result.coast_stats is None else result.coast_stats.summary(),
        predictor_settings=result.predictor_settings,
        predictor_summary=None if result.predictor is None else result.predictor.summary(),
        creep_floor_settings=result.creep_floor_settings,
        creep_floor_summary=(
            None if result.creep_floor_stats is None else result.creep_floor_stats.summary()
        ),
        stop_end_settings=result.stop_end_settings,
        stop_lead_summary=(
            None if result.stop_lead_stats is None else result.stop_lead_stats.summary()
        ),
        stop_end_summary=(
            None if result.stop_end_stats is None else result.stop_end_stats.summary()
        ),
        ref_smooth_sigma_s=ctx.config.feedforward.ref_smooth_sigma_s,
        low_speed_side_rule=ctx.config.forward_map.low_speed_side_rule,
        switch_lead_enabled=ctx.config.forward_map.switch_lead_enabled,
        fel_text=_fel_text_of(ctx.config, setup),
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
# 手順 11: 加振走行（ペダル→車速の周波数応答。ProblemReport_20260921 の 1.4Hz ばたつき対策）
# ─────────────────────────────────────────────────────────────────────


async def step11_excite(ctx: RunContext) -> None:
    """停車保持の状態から加振走行を行い、CSV と周波数応答の表を残す。

    FF・PID は使わない開ループ走行なので、手順 2/3 のような FF モデル・走行モード（DB）の
    読み込みは不要（`prepare` なし）。基準車速を持たないので `has_ref=False`。
    """
    hw = ctx.require_hardware()
    log = _session_log_for_drive(ctx, hw, section=SECTION_EXCITE, has_ref=False)
    result = await run_excite(hw, ctx.config, log=log, limit_s=ctx.limit_s)
    await log.close()  # 停車保持まで記録したら保存し、その CSV で周波数応答を求める

    if result.samples:
        blocks = excite_analyze(log.csv_path, skip_s=ctx.config.excite.analysis_skip_s)
        say()
        if blocks:
            say(excite_format_table(blocks, None))
        else:
            say("加振ブロックの行が skip_s で全て捨てられたため、周波数応答は出しません"
                f"（.venv/bin/python -m tests.research.excite --analyze {log.csv_path} で"
                "後から確認できます）")
    else:
        say("加振走行の行が無いため、周波数応答は出しません")
    if not result.completed:
        raise DriveError(result.abort_reason)
    say()
    say("手順 11 完了。")


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
        prepare=prepare_step2,
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
    Step(
        11,
        "加振走行（ペダル→車速の周波数応答）",
        step11_excite,
        drives=True,
    ),
    Step(
        12,
        "学習用パターンだけを走り、今の pkl（yaml の model_path）に補正表を付け直す",
        step12_fel_learning,
        drives=True,
        prepare=prepare_step12,
        has_ref=True,
    ),
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
    # WLTP の基準車速は DB から読む（学習結果のチェック D1・B3 と重み付けに使う）。読めなければ
    # None（D1 は SKIP）。retrain_only は asyncio ループの外（同期関数）から呼ばれるので asyncio.run
    wltp_mode = None
    try:
        wltp_mode = asyncio.run(load_mode(config, config.modes.wltp_mode_name))
    except Exception as exc:  # noqa: BLE001 - DB が無くても作り直しは続ける
        say(f"注: WLTP のモードを DB から読めません（{type(exc).__name__}）→ チェック D1 は SKIP")
    try:
        # 走行の材料（ramp_results・pedal）は無い。それを使う項目（B5①・C1）は SKIP
        build_ff_model(config, csv_path, hw_mode=args.hw, pedal=None, wltp_mode=wltp_mode)
    except Step2CheckError as exc:
        say(f"手順2 の学習結果のチェックエラー: {exc}")
        return 8
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
            except Step2CheckError as exc:
                say(f"手順2 の学習結果のチェックエラー: {exc}")
                say("yaml は書き換えていません。後の手順には進みません。")
                exit_code = 8
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
