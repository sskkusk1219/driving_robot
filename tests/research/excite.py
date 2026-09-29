"""加振走行: 一定速度に小さな正弦波を重ね、FF・PID を使わずにペダル→車速の周波数応答 P(f) を測る。

背景（`docs/Problem/ProblemReport_20260921.md` 手順2 段A）:
    手順 3（FF のみ・C5）でアクセルが約 1.4Hz でばたつく。原因は C5 の `dv`（偏差）経路が
    持つ実質 Kp（0.5〜1.7 %/(km/h)。`tests/research/model_gain.py`）だと分かったが、
    **1.4Hz でペダル 1% が車速を何 km/h 動かすか（プラント応答 P）が未測定**で、対策を
    「Kp を一律に下げる」にするか「その周波数だけ落とす」にするか決められない。
    既存ログでは測れない（開度一定の区間にその周波数の入力が無く、走行中は閉ループで
    入出力を分離できない）。そこで**一定速度で基準開度に小さな正弦波を重ね、フィードバック
    を使わずに** P(f) を直接測る。段A の予測は P(1.4Hz) ≈ 0.65 km/h per %。

引用元（`mode_drive.py::_ModeRun` を手本にする。同じ順序・同じ安全網を使うが、FF・PID・
調停は一切使わない — これが「開ループで測る」という要点）:
    tests/research/mode_drive.py   … 1 周期の順序（CAN 車速 → 指令の計算 → `drive_axis` で
                                      変化した軸だけ送信 → 電流 → 安全確認 → ログ）、
                                      絶対時刻ドリフト補正型の周期、AXIS_SMOOTH_DUTY 等の定数
    tests/research/axis_safety.py  … アラーム確認・電流ゼロ継続の安全網（AxisSafetyNet）
    tests/research/pattern_drive.py … 過電流しきい値・異常終了時のペダル解放
    tests/research/stop_decel.py   … 走行終わりの緩減速 → 停車保持（decelerate_to_stop）

走行シーケンス:
    for v_star in excite.speeds_kmh:
        到達フェーズ: 定速保持の PI で v_star に入れる
                      （旧定速階段の PI＝段4 で削除、と同じ構造・
                      同じ既定値 = 旧 `learning.cruise_hold_*`。同じ車で 9.6〜141km/h の定速保持の
                      実績がある。純積分では偏差が残る限り開度が伸び続けてオーバーシュートする
                      ため、P 項＋条件付き積分（アンチワインドアップ）＋レート制限にしている）
        for f in excite.frequencies_hz:
            加振フェーズ: 指令 = base + A*sin(2π f (t − block_start))
                          base の更新は「遅いトリム」だけ（1.4Hz を通さない弱いゲイン）。
                          到達フェーズ→加振フェーズの移行はバンプレス（P 項を積分へ畳み込む）
    最後に既存の decelerate_to_stop で停車する

安全条件（`mode_drive._ModeRun._check_safety` と同じものを流用）:
    - 過電流: `pattern_drive._overcurrent_limit_ma` を超えたら中止
    - `AxisSafetyNet`（アラーム確認・電流ゼロ継続）
    - 車速が `vehicle.max_speed_kmh` を超えたら中止
    - 車速が目標 `v_star` を `excite.abort_band_kmh` より超えて速い側に外れたら**両フェーズで**
      中止する（実機で目標より大幅に速く走らせないための安全網）。遅い側は**加振中だけ**見る
      （到達フェーズは整定前で目標との差が大きいのが前提のため）
    - 中断時は `pattern_drive._release_pedals` でペダルを解放してから結果を返す
    - ブレーキは待機位置（`mode_drive.standby_openings`）で固定し、一切踏まない（減速は惰行のみ）

CLI（解析のみ。車両には触らない）:
    .venv/bin/python -m tests.research.excite --analyze <CSV> [--pkl <pkl>] [--report]
"""

from __future__ import annotations

import argparse
import asyncio
import cmath
import csv
import math
import re
import sys
import time
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import numpy as np

from tests.research.axis_monitor import AxisMonitor
from tests.research.axis_safety import AxisSafetyNet
from tests.research.config import DEFAULT_CONFIG_PATH, ResearchConfig, load_config
from tests.research.debug_process23 import md_table
from tests.research.drive_log import (
    SECTION_DECEL_TO_STOP,
    SECTION_EXCITE,
    DriveSample,
    SessionLog,
    cmd_opening,
)
from tests.research.hardware import ActuatorProtocol, DriveError, ResearchHardware
from tests.research.live_plot import FONT_FAMILY
from tests.research.mode_drive import (
    AXIS_PRE_READ_DELAY_S,
    AXIS_SMOOTH_DUTY,
    PHASE_ACCEL,
    standby_label,
    standby_openings,
)
from tests.research.model_gain import _load_pkl, level_gain
from tests.research.pattern_drive import _overcurrent_limit_ma, _release_pedals
from tests.research.research_types import DriveLogData
from tests.research.stop_decel import PHASE_APPROACH, decelerate_to_stop
from tests.research.term import drive_status_line, say
from tests.research.vehicle import build_vehicle_profile, opening_to_pulse


class _LimitReached(Exception):  # noqa: N818 - 異常ではなく --limit-s による正常な打ち切り
    """`--limit-s`（スタブ動作確認用）に達したので走行を打ち切る。"""


class _ControlAbort(DriveError):
    """制御側の中断（到達フェーズの整定タイムアウト・abort_band 逸脱）。

    ハード自体は正常（過電流・アラーム・通信断ではない）なので、`_release_pedals` で惰行に
    投げ出すのではなく、走行終わりと同じ `decelerate_to_stop` の緩減速で止める。過電流・
    アラーム・CAN 読み取り失敗・最高速超えなど、ハード側の異常は従来どおり素の `DriveError`
    のまま（ペダル解放）。
    """


# ─────────────────────────────────────────────────────────────────────
# 純関数（制御則の計算だけを切り出す。ユニットテストはこれらを直接叩く）
# ─────────────────────────────────────────────────────────────────────


def _clamp(value: float, lo: float, hi: float) -> float:
    return min(max(value, lo), hi)


def _sine_opening(base: float, amplitude_pct: float, freq_hz: float, t_in_block: float) -> float:
    """加振フェーズの指令（クランプ前）: `base + amplitude_pct・sin(2π・freq_hz・t_in_block)`。

    `t_in_block` はブロック開始からの経過秒（`_excite_block` は `t - block_start` を渡す）。
    """
    return base + amplitude_pct * math.sin(2.0 * math.pi * freq_hz * t_in_block)


def _block_duration_s(hold_s: float, freq_hz: float) -> tuple[int, float]:
    """加振 1 ブロックの (周期数, 長さ[s])。DFT のビンを合わせるため整数周期に切り上げる。"""
    n_periods = max(1, math.ceil(hold_s * freq_hz))
    return n_periods, n_periods / freq_hz


def _pi_hold_step(
    base: float, integral: float, v_star: float, v_meas: float, *,
    kp: float, ki: float, min_pct: float, max_pct: float, max_rate_pct_s: float, dt: float,
) -> tuple[float, float]:
    """到達フェーズ（定速保持）の PI 1 周期ぶんの更新。戻り値は `(新しい base, 新しい integral)`。

    旧定速階段の PI（段4 で削除） と同じ P + 条件付き積分 + レート制限の構造・同じ入力
    （同じ車で 9.6〜141km/h の定速保持の実績がある）を土台にするが、アンチワインドアップは
    出力クランプ [min_pct, max_pct] だけでなく**レート制限も飽和源として扱う**（back-calculation:
    実際に反映できた `base` の変化量に合わせて積分を巻き戻す）。

    旧定速階段の素朴な条件付き積分（min/max だけを見る）は、CRUISE_HOLD が
    毎回 10km/h 程度の隣接する段しか狙わない前提では十分だが、加振走行の到達フェーズは
    0→60km/h のような大きな段を一気に狙うため、`max_rate_pct_s` で頭打ちのまま何十秒も
    積分だけが伸び続け、目標を大きく超えてから戻すのに同じだけ時間がかかる致命的な
    オーバーシュートを起こす（スタブでの実測。ProblemReport_20260921 対応・実機投入前に
    必須の修正）。レート制限も飽和源に含めることで、`desired` は常に `base` が実際に
    到達できる範囲の近くに留まり、誤差の符号が反転した瞬間から即座に逆方向へ動ける。

    `v_meas` は**実車速そのもの**（`pattern_loop` と同じ。加振フェーズのトリムと違い、まだ
    加振成分が乗っていないのでローパスを挟む理由が無く、挟むと収束が鈍ってオーバーシュートが
    悪化する）。
    """
    error = v_star - v_meas
    step_limit = max_rate_pct_s * dt
    candidate_integral = integral + ki * error * dt
    unsaturated = kp * error + candidate_integral
    desired = _clamp(unsaturated, min_pct, max_pct)
    new_base = base + _clamp(desired - base, -step_limit, step_limit)
    if abs(new_base - unsaturated) < 1e-9:
        new_integral = candidate_integral  # 頭打ちなし: 通常どおり積分を確定する
    else:
        # 出力クランプ・レート制限のどちらかで頭打ち: 実際に反映できた分まで積分を巻き戻す
        new_integral = new_base - kp * error
    return new_base, new_integral


def _lpf_step(v_lpf: float | None, speed: float, alpha: float) -> float:
    """車速の1次ローパス更新（`alpha = dt/(tau+dt)`）。初回は実測値でそのまま初期化する。"""
    return speed if v_lpf is None else v_lpf + alpha * (speed - v_lpf)


def _trim_delta(trim_gain_pct_per_kmh_s: float, v_star: float, v_lpf: float, dt: float) -> float:
    """加振フェーズの `base` の 1 周期ぶんの変化量（遅いトリムのみ。クランプ・頭打ちなし）。"""
    return trim_gain_pct_per_kmh_s * (v_star - v_lpf) * dt


# ─────────────────────────────────────────────────────────────────────
# 走行
# ─────────────────────────────────────────────────────────────────────


@dataclass(frozen=True)
class ExciteResult:
    completed: bool
    abort_reason: str
    run_duration_s: float
    cycles: int
    overruns: int
    blocks: tuple[tuple[float, float, float, float], ...]  # (v_star, freq_hz, 開始t, 終了t)
    samples: list[DriveSample]


class _ExciteRun:
    """1 本の加振走行の状態。`run()` が全速度×全周波数を順に回す。"""

    def __init__(self, hw: ResearchHardware, cfg: ResearchConfig, log: SessionLog) -> None:
        self.hw = hw
        self.cfg = cfg
        self.ex = cfg.excite
        self.log = log
        self.interval_s = cfg.control.loop_interval_s
        self.overcurrent_ma = _overcurrent_limit_ma(hw)
        self.safety = AxisSafetyNet(self.interval_s)
        # ブレーキは待機位置に固定（不感帯 − standby_margin_pct。使わなければ 0%）
        _accel_standby, self.brake_standby = standby_openings(cfg)
        self.floor_pct = cfg.feedforward.accel_deadband_pct
        self.ceil_pct = cfg.vehicle.max_accel_opening_pct
        self.amplitude_pct = min(self.ex.amplitude_pct, 1.0)  # ハード上限
        self.lpf_alpha = self.interval_s / (self.ex.speed_lpf_tau_s + self.interval_s)
        # 状態（フェーズをまたいで持ち越す。base は最初の到達フェーズが初期化する）
        self.base = self.floor_pct  # 基準開度（到達フェーズの PI・加振フェーズのトリムで動く）
        self.integral: float | None = None  # 到達フェーズ PI の積分項。最初の到達フェーズで初期化
        self.v_lpf: float | None = None  # トリムに使う車速の1次ローパス（1.4Hz を通さない）
        self.t = 0.0
        self.limit_s: float | None = None
        self.cycles = 0
        self.overruns = 0
        self.samples: list[DriveSample] = []
        self.blocks: list[tuple[float, float, float, float]] = []
        self.control_abort = False  # run()が_ControlAbortで止まったか（run_exciteが緩減速に使う）
        self._accel_cmd: int | None = None
        self._brake_cmd: int | None = None
        self._next_print = 0.0
        # run() の冒頭で実値を入れる（asyncio.get_running_loop() が要るため __init__ では作れない）
        self._loop: asyncio.AbstractEventLoop | None = None
        self._started = 0.0
        self._next_tick = 0.0

    async def run(self, limit_s: float | None) -> tuple[bool, str, float]:
        """全速度×全周波数を走り、(完了したか, 中断理由, 走った秒数) を返す。

        `_ControlAbort`（制御側の中断）で止まった場合はペダルを解放せず、`self.control_abort`
        を立てて呼び出し側（`run_excite`）に緩減速で止めさせる。それ以外の `DriveError`
        （ハード側の異常）は従来どおりここでペダルを解放する。
        """
        loop = asyncio.get_running_loop()
        self._loop = loop
        self._started = loop.time()
        self._next_tick = self._started
        self.t = 0.0
        self.limit_s = limit_s
        try:
            for v_star in self.ex.speeds_kmh:
                await self._approach(v_star)
                for freq_hz in self.ex.frequencies_hz:
                    await self._excite_block(v_star, freq_hz)
        except _LimitReached:
            return True, "", self.t
        except _ControlAbort as exc:
            say(f"加振走行を中断しました（制御側・ハードは正常）: {exc}")
            self.control_abort = True
            return False, str(exc), self.t
        except DriveError as exc:
            say(f"加振走行を中断しました: {exc}")
            await _release_pedals(self.hw)
            return False, str(exc), self.t
        return True, "", self.t

    async def _approach(self, v_star: float) -> None:
        """定速保持の PI（旧定速階段の PI（段4 で削除） と同じ構造・同じ入力）で `v_star` へ
        入れる。

        誤差は**実車速そのもの**（旧定速階段の PI（段4 で削除） と同じ）で見る。加振中の
        トリムと違い、到達フェーズにはまだ 1.4Hz 等の加振成分が乗っていないので、`v_lpf`
        （1 次ローパス）を挟む理由が無く、むしろ最大 `speed_lpf_tau_s` ぶんの無駄な遅れが
        フィードバックに入って収束を鈍らせ、オーバーシュートを助長する。整定するまで正弦波は
        重ねない。`base`・`integral` は最初の到達フェーズでだけ初期化し、以降は速度・フェーズを
        またいで持ち越す（段の変わり目・加振フェーズとの往復で指令を跳躍させない）。

        整定した瞬間に `self.base` へ入れる値は、その周期の PI 出力（瞬時値）ではなく**連続して
        帯内に入っていた窓の間に実際に送った指令の平均**にする（実機: 到達フェーズの PI は
        約 0.34Hz でハンチングし、指令の瞬時値が ±0.14% 程度振れる。定常状態の感度は 1% あたり
        約 50km/h と大きいため、瞬時値をそのまま加振フェーズの基準開度に凍結すると平均車速が
        目標から数 km/h ずれる。窓内の平均を使うことでこのハンチングを均す）。
        """
        ex = self.ex
        say(f"到達フェーズ: {v_star:.1f} km/h へ（PI Kp={ex.approach_kp_pct_per_kmh:g}%/(km/h) "
            f"Ki={ex.approach_ki_pct_per_kmh_s:g}%/(km/h・s)、レート上限 "
            f"{ex.approach_max_rate_pct_s:g}%/s）")
        if self.integral is None:
            self.base = _clamp(
                self.floor_pct + ex.approach_initial_offset_pct, self.floor_pct, self.ceil_pct
            )
            self.integral = self.base
        pattern = f"{v_star:.0f}kmh_approach"
        started_t = self.t
        settle_since: float | None = None
        window_commands: list[float] = []  # 連続して帯内だった窓で実際に送った指令（settle判定用）
        while True:
            base_at_cycle = self.base
            speed = await self._cycle(
                v_star=v_star, pattern=pattern, command=lambda _t: base_at_cycle,
                check_abort_band=False,
            )
            self.base, self.integral = _pi_hold_step(
                self.base, self.integral, v_star, speed,
                kp=ex.approach_kp_pct_per_kmh, ki=ex.approach_ki_pct_per_kmh_s,
                min_pct=self.floor_pct, max_pct=self.ceil_pct,
                max_rate_pct_s=ex.approach_max_rate_pct_s, dt=self.interval_s,
            )

            within = abs(speed - v_star) <= ex.approach_band_kmh
            if within:
                if settle_since is None:
                    settle_since = self.t
                    window_commands = []
                window_commands.append(base_at_cycle)
                if self.t - settle_since >= ex.approach_settle_s:
                    mean_base = _clamp(
                        sum(window_commands) / len(window_commands), self.floor_pct, self.ceil_pct
                    )
                    say(f"  整定: {speed:.2f} km/h（{self.t - started_t:.1f}s）"
                        f" base = 窓平均 {mean_base:.3f}%（瞬時値 {self.base:.3f}%）に固定")
                    self.base = mean_base
                    # バンプレス: 加振フェーズ（P 項なし）へ移る前に、次にこの速度へ戻ってきたときの
                    # ために integral を base に合わせておく。目標帯内＝誤差はほぼ 0 なので P 項も
                    # ほぼ 0、つまり `_pi_hold_step` の unsaturated ≒ integral。次の到達フェーズの
                    # 初回呼び出しで unsaturated ≒ base になるよう integral = base としておけば、
                    # 上書きした base（窓平均、瞬時値とはズレている）からも跳躍しない
                    self.integral = self.base
                    return
            else:
                settle_since = None
                window_commands = []
            if self.t - started_t >= ex.approach_timeout_s:
                raise _ControlAbort(
                    f"到達フェーズが approach_timeout_s={ex.approach_timeout_s:g}s 以内に"
                    f" {v_star:.1f} km/h へ入りませんでした（実車速 {speed:.2f} km/h、"
                    f"t={self.t:.1f}s）"
                )

    async def _excite_block(self, v_star: float, freq_hz: float) -> None:
        """`v_star` を保ったまま `freq_hz` で正弦波を重ねる（整数周期ぶんちょうど）。"""
        ex = self.ex
        n_periods, duration_s = _block_duration_s(ex.hold_s, freq_hz)
        pattern = f"{v_star:.0f}kmh_{freq_hz:.2f}Hz"
        say(f"加振フェーズ: {v_star:.1f} km/h × {freq_hz:g} Hz "
            f"（振幅 ±{self.amplitude_pct:g}%・{duration_s:.2f}s = {n_periods} 周期）")
        block_start = self.t

        def command(t: float) -> float:
            return _sine_opening(self.base, self.amplitude_pct, freq_hz, t - block_start)

        while self.t - block_start < duration_s:
            await self._cycle(
                v_star=v_star, pattern=pattern, command=command, check_abort_band=True
            )
            assert self.v_lpf is not None
            # 加振中の base 更新は「遅いトリム」のみ（1.4Hz などの加振周波数は通さない弱さ）。
            # 長時間の加振で伸び続けないよう、トリム後も必ずクランプする
            self.base += _trim_delta(
                ex.trim_gain_pct_per_kmh_s, v_star, self.v_lpf, self.interval_s
            )
            self.base = _clamp(self.base, self.floor_pct, self.ceil_pct)
        self.blocks.append((v_star, freq_hz, block_start, self.t))

    async def _cycle(
        self,
        *,
        v_star: float,
        pattern: str,
        command: Callable[[float], float],
        check_abort_band: bool,
    ) -> float:
        """1 周期進める（速度読み取り → 指令 → 安全確認 → ログ → 次周期まで待つ）。実車速を返す。"""
        if self.limit_s is not None and self.t >= self.limit_s:
            raise _LimitReached
        cfg, hw = self.cfg, self.hw
        cycle_started = time.perf_counter()
        t = self._loop.time() - self._started
        self.t = t
        try:
            speed = await hw.can.read_speed()
        except Exception as exc:
            raise DriveError(f"CAN 車速を読めません（{type(exc).__name__}: {exc}）") from exc
        self.v_lpf = _lpf_step(self.v_lpf, speed, self.lpf_alpha)

        accel = _clamp(command(t), self.floor_pct, self.ceil_pct)
        accel_pos = opening_to_pulse(accel)
        brake_pos = opening_to_pulse(self.brake_standby)
        try:
            monitor_accel, monitor_brake = await asyncio.gather(
                self._drive_axis(hw.accel, accel_pos, is_accel=True),
                self._drive_axis(hw.brake, brake_pos, is_accel=False),
            )
        except Exception as exc:
            raise DriveError(
                f"アクチュエータ通信に失敗しました（{type(exc).__name__}: {exc}）"
            ) from exc
        cycle_ms = 1000.0 * (time.perf_counter() - cycle_started)
        accel_current, brake_current = monitor_accel.current_ma, monitor_brake.current_ma
        alarm_accel, alarm_brake = await self.safety.poll_alarms(self.cycles, hw.accel, hw.brake)
        self._check_safety(
            t, v_star, speed, accel_current, brake_current, accel_pos, brake_pos,
            alarm_accel, alarm_brake, check_abort_band=check_abort_band,
        )

        if self.cycles % cfg.control.log_every_n_cycles == 0:
            data = DriveLogData(
                ref_speed_kmh=v_star,
                actual_speed_kmh=speed,
                accel_opening=accel,
                brake_opening=self.brake_standby,
                accel_pos=accel_pos,
                brake_pos=brake_pos,
                accel_current=accel_current,
                brake_current=brake_current,
                plan_effort_pct=accel,  # 指令開度（base + 正弦波成分。クランプ後）
                trim_effort_pct=self.base,  # base だけ（あとで正弦波成分を引き算で分離できる）
                applied_effort_pct=accel,
                phase=PHASE_ACCEL,
            )
            self.samples.append(
                self.log.record(
                    data, section=SECTION_EXCITE, phase=PHASE_ACCEL, pattern=pattern,
                    mode_time_s=t, alarm_accel=alarm_accel, alarm_brake=alarm_brake,
                    monitor_accel=monitor_accel, monitor_brake=monitor_brake, cycle_ms=cycle_ms,
                )
            )
        self.cycles += 1
        if t >= self._next_print:
            interval = cfg.output.print_interval_s
            self._next_print = (math.floor(t / interval) + 1) * interval
            say(drive_status_line(
                t_s=t, ref_kmh=v_star, actual_kmh=speed, kp=0.0, ki=0.0, kd=0.0,
                accel_pct=accel, brake_pct=self.brake_standby, extra=pattern,
            ))

        # 周期の絶対時刻ドリフト補正（mode_drive._ModeRun.run と同じ書き方）
        self._next_tick += self.interval_s
        now = self._loop.time()
        if now - self._next_tick > self.interval_s:
            self.overruns += 1
            self._next_tick = now
        await asyncio.sleep(max(0.0, self._next_tick - now))
        return speed

    async def _drive_axis(
        self, axis: ActuatorProtocol, pos: int, *, is_accel: bool
    ) -> AxisMonitor:
        """位置が変わった軸だけ指令し（`mode_drive._ModeRun.drive_axis` と同じ）まとめ読みする。"""
        last = self._accel_cmd if is_accel else self._brake_cmd
        if pos != last:
            await axis.move_to_position(pos, smooth_over_s=self.interval_s * AXIS_SMOOTH_DUTY)
            if is_accel:
                self._accel_cmd = pos
            else:
                self._brake_cmd = pos
            await asyncio.sleep(AXIS_PRE_READ_DELAY_S)
        return await axis.read_monitor()

    def _check_safety(
        self, t: float, v_star: float, speed: float, accel_current: float, brake_current: float,
        accel_pos: int, brake_pos: int, alarm_accel: bool | None, alarm_brake: bool | None,
        *, check_abort_band: bool,
    ) -> None:
        v = self.cfg.vehicle
        for label, current in (("アクセル", accel_current), ("ブレーキ", brake_current)):
            if current > self.overcurrent_ma:
                raise DriveError(
                    f"{label}軸が過電流です"
                    f"（{current:.0f} mA > {self.overcurrent_ma:.0f} mA、t={t:.1f}s）"
                )
        # アラーム確認・電流ゼロ継続（アラームが出ないサーボ脱落を拾う安全網。axis_safety.py）
        reason = self.safety.check(
            t=t, accel_pos=accel_pos, brake_pos=brake_pos,
            accel_current=accel_current, brake_current=brake_current,
            alarm_accel=alarm_accel, alarm_brake=alarm_brake,
        )
        if reason is not None:
            raise DriveError(reason)
        if speed > v.max_speed_kmh:
            raise DriveError(
                f"車速 {speed:.1f} km/h が最高速 {v.max_speed_kmh:.1f} km/h を超えました"
            )
        # 過速（目標より速い側）は両フェーズで見る。実機で目標を大幅に超える速度を出さないため。
        # 制御側の逸脱（ハードは正常）なので _ControlAbort → 呼び出し側は緩減速で止める
        if speed - v_star > self.ex.abort_band_kmh:
            raise _ControlAbort(
                f"車速が目標 {v_star:.1f} km/h を {speed - v_star:.2f} km/h 超えました"
                f"（しきい値 {self.ex.abort_band_kmh:g} km/h、t={t:.1f}s）"
            )
        # 遅い側は加振中だけ見る（到達フェーズは整定前で目標との差が大きいのが前提のため）
        if check_abort_band and v_star - speed > self.ex.abort_band_kmh:
            raise _ControlAbort(
                f"加振中に車速が目標から {v_star - speed:.2f} km/h 低く外れました"
                f"（目標 {v_star:.1f} km/h、しきい値 {self.ex.abort_band_kmh:g} km/h、t={t:.1f}s）"
            )


async def run_excite(
    hw: ResearchHardware,
    cfg: ResearchConfig,
    *,
    log: SessionLog,
    limit_s: float | None = None,
) -> ExciteResult:
    """停車保持の状態から加振走行を行い、緩減速で停車保持にして終える。

    異常（CAN・通信・過電流・最高速超え・逸脱）は例外にせず、abort_reason 付きの結果を返す
    （`mode_drive.run_mode_drive` と同じ扱い方）。ハード側の異常（過電流・アラーム・通信断）は
    ペダルを離して終える。到達フェーズのタイムアウトや abort_band 逸脱のような**制御側**の
    中断（`_ControlAbort`）はハードが正常なので、完走時と同じ緩減速（`decelerate_to_stop`）で
    止める（`_ExciteRun.run` 参照）。`limit_s` はスタブ動作確認用（合計がこの秒数を超えたら
    打ち切って正常終了扱いにする）。
    """
    ex = cfg.excite
    say(f"加振走行: 速度 {ex.speeds_kmh} km/h × 周波数 {ex.frequencies_hz} Hz"
        f"（振幅 ±{min(ex.amplitude_pct, 1.0):g}%・1 周波数あたり約 {ex.hold_s:g}s）")
    say("制御構成: FF・PID は使いません（開ループでペダル→車速の応答 P(f) を直接測ります）")
    say(f"ペダルの待機位置（使わない側=ブレーキ）: {standby_label(cfg)}")
    say(f"中止条件: 過電流・アラーム・最高速 {cfg.vehicle.max_speed_kmh:g}km/h 超え・"
        f"目標より {ex.abort_band_kmh:g}km/h 速い（両フェーズ）・加振中に目標より "
        f"{ex.abort_band_kmh:g}km/h 遅い")
    if hw.is_real:
        say(f"*** 実機モード: 車両が最高 {max(ex.speeds_kmh):.0f} km/h まで走り、最高 "
            f"{max(ex.frequencies_hz):g} Hz でアクセルを振幅 ±{min(ex.amplitude_pct, 1.0):g}% "
            "揺らします。シャシダイナモ上で実施してください ***")

    run = _ExciteRun(hw, cfg, log)
    await log.stop_sampler()  # 制御ループと Modbus を取り合わない（記録はループから）
    log.mark(SECTION_EXCITE, "")
    completed, abort_reason, run_s = await run.run(limit_s)
    say(f"加振走行 {'完了' if completed else '中断'}: {run_s:.1f}s・{run.cycles} 周期"
        f"（1 周期以上の遅れ {run.overruns} 回・加振ブロック {len(run.blocks)} 本）")

    if completed or run.control_abort:
        if run.control_abort:
            say("制御側の中断なので緩減速で停車させます …")
        else:
            say("緩減速で停車させ、停車保持ブレーキをかけます …")
        log.mark(SECTION_DECEL_TO_STOP, PHASE_APPROACH)
        log.start_sampler()
        profile = build_vehicle_profile(cfg)
        stop = await decelerate_to_stop(hw, cfg, profile, log=log)
        say(f"停車保持: ブレーキ {stop.hold_pct:.2f}%")

    return ExciteResult(
        completed=completed,
        abort_reason=abort_reason,
        run_duration_s=run_s,
        cycles=run.cycles,
        overruns=run.overruns,
        blocks=tuple(run.blocks),
        samples=run.samples,
    )


# ─────────────────────────────────────────────────────────────────────
# 解析（CSV → ブロックごとの周波数応答）
# ─────────────────────────────────────────────────────────────────────

_BLOCK_PATTERN_RE = re.compile(r"^(?P<v>\d+(?:\.\d+)?)kmh_(?P<f>\d+(?:\.\d+)?)Hz$")


@dataclass(frozen=True)
class BlockResponse:
    v_star_kmh: float
    freq_hz: float
    n_rows: int
    cmd_amp_pct: float  # 指令の振幅（その周波数成分）[%]
    speed_amp_kmh: float  # 車速の振幅（その周波数成分）[km/h]
    gain_kmh_per_pct: float  # speed_amp / cmd_amp（= |Y|/|U|）[km/h / %]
    phase_deg: float  # 車速が指令に対して遅れる向きを負で表す [deg]
    snr: float  # その周波数成分の振幅 ÷ 残差（3次トレンドと当該成分を同時に除いた後）の RMS


# トレンド除去の多項式次数。
#
# 加振ブロック中の平均車速は「到達フェーズの整定の尾」＋「加振中の遅いトリム」で曲がった
# （1次の直線では外れない）軌跡を描く。その曲率が正弦波成分と直交しないため、1次のまま
# 当てはめるとゲイン・位相に大きなバイアスが乗る（スタブ車両の実測で 1.4Hz のゲインが
# 理論値 0.0284 に対し 0.0413 と +45% ずれ、0.7Hz の位相が 30° ずれた。
# `docs/Problem/ProblemReport_20260921.md` 参照）。3次多項式にすると理論値に一致する。
_TREND_DEGREE = 3


def _normalized_time(t: np.ndarray) -> np.ndarray:
    """ブロック内の時刻を ±1 程度に正規化する（多項式の高次項の数値安定のため）。"""
    span = float(np.ptp(t))
    if span <= 0.0:
        return np.zeros_like(t)
    return (t - t.mean()) / (span / 2.0)


def _design_matrix(t: np.ndarray, freq_hz: float, trend_degree: int = _TREND_DEGREE) -> np.ndarray:
    """トレンド（0〜trend_degree 次の多項式）と当該周波数の cos/sin を 1 つの計画行列にまとめる。

    トレンドを先に抜いてから正弦波を当てる 2 段構えだと、トレンドの当てはめ誤差が正弦波側の
    推定にそのまま漏れ込む。1 回の最小二乗で同時に解くことでこのバイアスを避ける。
    """
    tt = _normalized_time(t)
    trend_cols = [tt**k for k in range(trend_degree + 1)]
    cos_col = np.cos(2.0 * np.pi * freq_hz * t)
    sin_col = np.sin(2.0 * np.pi * freq_hz * t)
    return np.column_stack([*trend_cols, cos_col, sin_col])


def _fit_sinusoid(
    t: np.ndarray, x: np.ndarray, freq_hz: float, *, trend_degree: int = _TREND_DEGREE
) -> tuple[complex, np.ndarray]:
    """トレンド＋当該周波数の正弦波を最小二乗で同時に当てはめる。

    戻り値は (当該周波数成分の複素表現, 残差)。複素表現は離散フーリエ和
    `Σ x_n·exp(-2πj·freq_hz·t_n)` と同じ規約（`2*abs(big)/n` が振幅、`angle(big)` が位相）。
    残差はトレンド・正弦波の両方を差し引いた後の `x - フィット値`。
    """
    design = _design_matrix(t, freq_hz, trend_degree)
    coeffs, *_ = np.linalg.lstsq(design, x, rcond=None)
    cos_coeff, sin_coeff = coeffs[-2], coeffs[-1]
    n = len(t)
    big = 0.5 * n * complex(cos_coeff, -sin_coeff)
    residual = x - design @ coeffs
    return big, residual


def _dft_component(t: np.ndarray, x: np.ndarray, freq_hz: float) -> complex:
    """当該周波数の離散フーリエ成分（`_TREND_DEGREE` 次のトレンドを同時に除いた最小二乗あてはめ）。

    不等間隔の時刻列でもよい。トレンド除去の要点は `_design_matrix` を参照。
    """
    big, _residual = _fit_sinusoid(t, x, freq_hz)
    return big


def analyze(csv_path: Path, *, skip_s: float = 2.0) -> list[BlockResponse]:
    """加振走行 CSV（`section == EXCITE`）から、ブロック（速度×周波数）ごとの周波数応答を求める。

    `pattern` 列が `<速度>kmh_<周波数>Hz` の行だけを使う（到達フェーズの `..._approach` は
    対象外）。各ブロックの先頭 `skip_s` は過渡応答として捨てる。残りの区間で
        車速 y(t)・指令 u(t) それぞれに対して `_TREND_DEGREE` 次のトレンドと当該周波数の
        正弦波を 1 回の最小二乗で同時に当てはめる（`_fit_sinusoid`）
    ことで、その周波数ちょうどの複素成分 U（指令）・Y（車速）を求め、
        gain  = |Y| / |U|                      … 車速振幅 ÷ 指令振幅 [km/h / %]
        phase = angle(Y/U) を度で（負が車速の遅れる向き）
        snr   = 車速振幅 ÷ 残差（トレンドと当該周波数成分を同時に除いた後）の RMS
    を求める。`snr` が低い点は、その周波数成分が測れていない（他の変動に埋もれている）ことを表す。
    """
    with csv_path.open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))

    order: list[str] = []
    groups: dict[str, list[Mapping[str, str]]] = {}
    for row in rows:
        if row.get("section") != SECTION_EXCITE:
            continue
        pattern = row.get("pattern", "")
        if _BLOCK_PATTERN_RE.match(pattern) is None:
            continue
        if pattern not in groups:
            groups[pattern] = []
            order.append(pattern)
        groups[pattern].append(row)

    results: list[BlockResponse] = []
    for pattern in order:
        group_rows = groups[pattern]
        m = _BLOCK_PATTERN_RE.match(pattern)
        assert m is not None  # order は match 済みの pattern だけ
        v_star, freq_hz = float(m.group("v")), float(m.group("f"))

        t_all = np.array([float(r["mode_time_s"]) for r in group_rows], dtype=float)
        mask = (t_all - t_all[0]) >= skip_s
        n = int(np.count_nonzero(mask))
        # 計画行列の列数（トレンド _TREND_DEGREE+1 列 + cos/sin 2 列）に満たないブロックは
        # 最小二乗が解けない（過小決定）ため捨てる。
        min_rows = _TREND_DEGREE + 1 + 2
        if n < min_rows:
            continue
        t = t_all[mask]
        y = np.array([float(r["actual_speed_kmh"]) for r in group_rows], dtype=float)[mask]
        u = np.array([cmd_opening(r, "accel") for r in group_rows], dtype=float)[mask]

        big_u, _u_residual = _fit_sinusoid(t, u, freq_hz)
        big_y, y_residual = _fit_sinusoid(t, y, freq_hz)
        cmd_amp = 2.0 * abs(big_u) / n
        speed_amp = 2.0 * abs(big_y) / n
        gain = abs(big_y) / abs(big_u) if abs(big_u) > 0.0 else float("nan")
        phase_deg = math.degrees(cmath.phase(big_y / big_u)) if abs(big_u) > 0.0 else float("nan")
        rms_residual = float(np.sqrt(np.mean(y_residual**2)))
        snr = speed_amp / rms_residual if rms_residual > 0.0 else float("inf")

        results.append(
            BlockResponse(
                v_star_kmh=v_star, freq_hz=freq_hz, n_rows=n,
                cmd_amp_pct=cmd_amp, speed_amp_kmh=speed_amp,
                gain_kmh_per_pct=gain, phase_deg=phase_deg, snr=snr,
            )
        )
    return results


# ─────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────


def _format_table(blocks: Sequence[BlockResponse], gains: Mapping[float, float] | None) -> str:
    header = [
        "速度 [km/h]", "周波数 [Hz]", "行数", "指令振幅 [%]", "車速振幅 [km/h]",
        "ゲイン P [km/h / %]", "位相 [deg]", "SNR",
    ]
    if gains is not None:
        header.append("ループゲイン P×Kp")
    table_rows: list[list[str]] = []
    for b in blocks:
        row = [
            f"{b.v_star_kmh:.0f}", f"{b.freq_hz:g}", str(b.n_rows),
            f"{b.cmd_amp_pct:.3f}", f"{b.speed_amp_kmh:.3f}",
            f"{b.gain_kmh_per_pct:.3f}", f"{b.phase_deg:+.1f}", f"{b.snr:.1f}",
        ]
        if gains is not None:
            kp = gains.get(b.v_star_kmh)
            row.append("—" if kp is None else f"{b.gain_kmh_per_pct * kp:.3f}")
        table_rows.append(row)
    return md_table(header, table_rows)


def plot_freq_response(blocks: Sequence[BlockResponse], path: Path) -> Path:
    """速度ごとに、ゲイン（上段）・位相（下段）を対数周波数軸で描く。"""
    import matplotlib.pyplot as plt  # noqa: PLC0415

    fig, (ax_gain, ax_phase) = plt.subplots(2, 1, figsize=(8.0, 7.0), sharex=True)
    for v_star in sorted({b.v_star_kmh for b in blocks}):
        series = sorted((b for b in blocks if b.v_star_kmh == v_star), key=lambda b: b.freq_hz)
        freqs = [b.freq_hz for b in series]
        label = f"{v_star:.0f} km/h"
        ax_gain.plot(freqs, [b.gain_kmh_per_pct for b in series], marker="o", label=label)
        ax_phase.plot(freqs, [b.phase_deg for b in series], marker="o", label=label)
    ax_gain.set_xscale("log")
    ax_gain.set_ylabel("ゲイン P [km/h / %]")
    ax_gain.grid(True, which="both", alpha=0.3)
    ax_gain.legend(loc="best")
    ax_phase.set_xscale("log")
    ax_phase.set_ylabel("位相 [deg]")
    ax_phase.set_xlabel("周波数 [Hz]")
    ax_phase.grid(True, which="both", alpha=0.3)
    fig.savefig(path)
    plt.close(fig)
    return path


def write_report(
    blocks: Sequence[BlockResponse], results_dir: Path, *, gains: Mapping[float, float] | None,
) -> Path:
    """report*_RunFreqResp[_N].md に表と図を書く（`mode_report.unique_report_path` で採番する）。"""
    import matplotlib  # noqa: PLC0415

    from tests.research.mode_report import unique_report_path  # noqa: PLC0415

    results_dir.mkdir(parents=True, exist_ok=True)
    started_at = datetime.now()
    path = unique_report_path(results_dir, "FreqResp", started_at)
    fig_path = path.with_suffix(".png")
    with matplotlib.rc_context({"font.family": FONT_FAMILY}):
        plot_freq_response(blocks, fig_path)
    table = _format_table(blocks, gains)
    text = (
        f"# 加振走行 周波数応答（{started_at:%Y-%m-%d %H:%M}）\n\n"
        f"{table}\n\n"
        f"![周波数応答（ゲイン・位相）]({fig_path.name})\n"
    )
    path.write_text(text, encoding="utf-8")
    return path


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="python -m tests.research.excite",
        description=__doc__,
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "--analyze", type=Path, metavar="CSV", required=True, help="加振走行の CSV（EXCITE 区間）"
    )
    parser.add_argument(
        "--pkl", type=Path, default=None,
        help="指定すると model_gain.level_gain でその速度の実質 Kp を求め、"
             "ループゲイン P×Kp の列を足す（段A の予測 P(1.4Hz)≈0.65 と比較できる）",
    )
    parser.add_argument(
        "--skip-s", type=float, default=2.0, help="各ブロック先頭の過渡応答を捨てる秒数（既定 2.0）"
    )
    parser.add_argument(
        "--report", action="store_true",
        help="results/reportYYYYMMDD_RunFreqResp[_N].md に表と図（ゲイン・位相）を書く",
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    args = parser.parse_args(argv)

    try:
        blocks = analyze(args.analyze, skip_s=args.skip_s)
    except (OSError, ValueError) as exc:
        print(f"エラー: {exc}", file=sys.stderr)
        return 2
    if not blocks:
        print(f"EXCITE の行が見つかりません（または全ブロックが skip_s で空）: {args.analyze}",
              file=sys.stderr)
        return 1

    gains: dict[float, float] | None = None
    if args.pkl is not None:
        try:
            model, spec, _meta = _load_pkl(args.pkl)
        except ValueError as exc:
            print(f"エラー: {exc}", file=sys.stderr)
            return 2
        gains = {v: level_gain(model, spec, v) for v in sorted({b.v_star_kmh for b in blocks})}

    print(_format_table(blocks, gains))

    if args.report:
        config = load_config(args.config)
        path = write_report(blocks, config.results_path, gains=gains)
        print(f"\nレポート: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
