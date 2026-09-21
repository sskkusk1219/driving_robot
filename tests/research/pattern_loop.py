"""手順 2-1 の閉ループパターン走行を実行する、研究用の自前状態機械。

ProblemReport_20260910 の遵守事項「`/src` の本番環境のコードを実行しないこと。実行環境は
すべて `tests/` に記載すること」に対応する。以前は `src.domain.control.learning_loop.LearningLoop`
をサブクラス化してそのまま実行していたが、それは「引用」ではなく本番コードの実行に当たるため、
アルゴリズムだけを移植した自前実装に置き換えた。

引用元（アルゴリズムを移植。クラス自体は import・実行しない）:
    src/domain/control/learning_loop.py … フェーズ遷移（MEASURE/DRIVE_ACCEL/COAST/DRIVE_BRAKE/
        BRAKE_HOLD/CRUISE_TRIM）、包絡線ガバナ（上限G厳守）、ランプ、オーバースピード回復、
        非常停止判定（CAN取得失敗・アクチュエータ通信失敗・過電流）のロジック。
    src/domain/safety_monitor.py … `check_overcurrent`（`current_ma > limit` の比較のみ）。

本番との違い（研究用にシンプルにしたところ）:
    - `CycleLoopBase`（本番の共通基盤）が持つ絶対時刻グリッドスケジューリング・
      ウォッチドッグ（サイクル未完了の連続監視）・DBログ書き込みバックログ管理は持たない。
      研究ハーネスは単発の短い走行のみを行うため、固定周期の単純な asyncio ループで足りる。
    - `SafetyMonitor` クラスは使わず、過電流しきい値との比較を直接行う（同じロジック）。
    - `RealtimeSnapshot`（WebSocket 配信用のキャッシュ）は持たない。研究ハーネスは配信しない。
    - サンプルごとのログフックは `on_sample` コールバックとして最初から持ち、
      本番の `_enqueue_log_write` をサブクラスでオーバーライドする形は取らない。

2026-09-13 A6（KAIZEN 表5-5 順3。report20260912_KAIZEN_process2,3.md）で本番から変えたところ:
    - ガバナーの解除: 本番は頭打ちをフェーズが変わるまで戻さず、実機では DRIVE_BRAKE が 8〜10%
      （不感帯未満）・DRIVE_ACCEL が 19〜22% に張り付いて「打ち切りに張り付く」原因になっていた。
      加減速が上限 × gov_release_frac を下回ったら gov_raise_step_pct ずつ戻し、
      指令に届いたら解除する。
    - 停車復帰を全運転パターンの後に置く: ACCEL_SWEEP だけでなく BRAKE_HOLD・COAST_DOWN・
      CRUISE_TRIM も、終わって停車していなければ DRIVE_BRAKE で停車させてから次へ進む
      （次の段を必ず停車から始める）。目標は ACCEL_SWEEP がリセットブレーキ、他は停車保持開度。
    - DRIVE_BRAKE の打ち切りは「過ぎたら次へ進む」ではなく
      「過ぎても停車しなければ中断する安全上限」。
    - ブレーキのランプはフェーズに入ったときの開度から始める
      （BRAKE_HOLD の保持開度から 0% に抜けない）。
    - 走行中の安全網（axis_safety.py: アラーム確認・電流ゼロの継続）を入れ、
      中断理由を abort_reason に残す。

2026-09-13 A2・A5（KAIZEN 表5-5 順4）で足したところ:
    - 目標車速つきパターン `SpeedTargetPattern`: 加速を cap ではなく accel_target_kmh で終える
      （BRAKE_HOLD を 60 km/h から保持する段に使う。src の LearningPattern は変えない）。

2026-09-13 A3・A4（KAIZEN 表5-5 順5）で足したところ:
    - トリム階段 `TrimStairPattern`（kind は CRUISE_TRIM のまま）: accel_target_kmh まで加速したら、
      trim_steps_pct をこの順に step_hold_s ずつ保持し、終わったら停車復帰する。
      cap に達したら次の（低い）段へ下げ、最後の段は cap に達しても保持を続ける
      （最高速を超えたら今と同じ回復に入る）。通常の CRUISE_TRIM は今と同じく cap で終える。
    - A4（20 km/h から高ブレーキで停車）は SpeedTargetPattern の BRAKE_HOLD をそのまま使う。

2026-09-13 A7（KAIZEN 表5-5 順6）で変えたところ:
    - 各軸の電流の読み取り（CNOW）を、0x9000 から 14 レジスタのまとめ読み（`read_monitor`）に
      替えた。実位置・ステータスも同じ 1 回で取れる。過電流・安全網の判定は電流だけを見る（中身は
      変えない）。
    - `on_sample` に両軸のモニターと、その周期の処理時間 cycle_ms（車速を読む直前から
      まとめ読みの後まで）を足して渡す。

2026-09-14 定速階段（段2）で足したところ:
    背景（docs/memo.md「応答遅れの実測 → 高速域の偏りの原因 → 定速階段＋走行抵抗式（C6）の計画」）:
    手順2に「定速保持」の状態が 50 km/h 以上で 1 行も無く、FF モデルの定速開度の予測が
    60〜80 km/h で浅く（遅れ）・130 km/h で深く（出すぎ）なる原因になっていた。手順3では
    指令→実開度の遅れがほぼ無い（|実−指令| p95 0.06〜0.07%）と実測済みなので、階段の開度は
    低ゲイン・レート制限の緩い PI で「実開度 ≈ 指令」を保ったまま定常開度を記録すればよい。
    - トリム階段と同じ流儀の `CruiseStairPattern`（kind は CRUISE_TRIM のまま）を追加:
      `hold_speeds_kmh`（昇順。例 30〜130 km/h を 10 km/h 刻み）へ、専用フェーズ
      `_Phase.CRUISE_HOLD` で 1 本ずつ弱い PI 保持する。DRIVE_ACCEL は最初の車速
      （`hold_speeds_kmh[0]`）で終える（`_accel_target_kmh`）。
    - CRUISE_HOLD の開度は速度偏差 [km/h] → 開度 [%] の PI（`_cruise_hold_opening`）。
      アンチワインドアップ（[不感帯, max_accel_opening] で頭打ちの方向へはこれ以上積分しない）・
      1 周期あたりの変化量を `max_rate_pct_per_s × interval_s` に制限（実開度が指令に追従できる
      速さに保つ）。初期開度は「アクセル不感帯 + initial_offset_pct」（DRIVE_ACCEL の加速用開度
      は定速に対して大きすぎ、そこから始めるとレート制限で最初の車速に間に合わないため）。
      積分・開度は車速をまたいでも引き継ぎ、段の変わり目で跳躍させない。
    - 前進判定（`_advance_cruise_hold`）: |車速−目標| ≤ settle_tol_kmh が settle_s 続いたら
      保持タイマーを開始し、hold_s 経過で次の車速へ（積分は引き継ぐ・settle/hold タイマーだけ
      リセット）。step_timeout_s を過ぎたら保持できていなくても次へ進む。最後の車速の後は
      `_finish_pattern`（停車復帰）。最高速超え・低速側（coast_down_stop_speed_kmh 以下）は
      トリム階段と同じ回復（前者は DRIVE_BRAKE の速度超過回復、後者は停車復帰）。
    - `_move_duration` は CRUISE_HOLD も DRIVE_ACCEL と同じ短い一手（pedal_step_time_s）にし、
      実開度が指令に追従できるようにする。G ガバナー（`_update_governor`）は CRUISE_HOLD に
      適用しない（緩い PI・レート制限そのものが穏やかなので、ガバナーによる頭打ちは不要と判断）。

2026-09-17 クリープ発進・クリープ域ブレーキ保持（段1。ProblemReport_20260916 課題#2）で足したところ:
    背景（docs/Problem/ProblemReport_20260916.md）: 本番のクリープ加速率 creep_rate_kmhs=0.19 が
    実測クリープ加速（中央値 2.39 km/h/s）と 12 倍ずれていた。原因は「ペダルオフ・低速・加速中」の
    全サンプルの中央値を取る既存推定（手順2がクリープ安定まで待つため母集団が定常側に偏る）。
    - `CreepLaunchPattern`（kind は CREEP_SETTLE を流用）を追加: 専用フェーズ `_Phase.CREEP_LAUNCH`
      で両ペダル 0%（完全解放）を指令し、`target_kmh` 到達または `timeout_s` の打ち切りまでクリープ
      のみで自走させる（`_advance_creep_launch`）。到達後は `hold_after` で分岐:
      False（クリープ発進）は通常の停車復帰（`_finish_pattern` → `_Phase.DRIVE_BRAKE`）、
      True（クリープ域ブレーキ保持）は `brake_opening`（不感帯 + offset）を保持する
      既存の `_Phase.BRAKE_HOLD` へ直接入る。
    - `_move_duration` は変更しない（CREEP_LAUNCH は COAST と同じく既定の pedal_release_time_s）。
      G ガバナー（`_update_governor`）も適用しない（両ペダル 0% で開度指令が無いため不要）。

2026-09-19 低開度階段（段3-1。ProblemReport_20260919 候補(c)）で足したところ:
    背景（docs/Problem/ProblemReport_20260919.md 6章）: 手順2 に「不感帯の直上を細かく刻んで
    一定保持する」パターンが 1 本も無く、5〜24 km/h × 不感帯 +0〜2% の学習行が 5.4 秒しか
    無かった（既存の ACCEL_DEADBAND_PROBE は無ランプ・昇順で停車に戻らずつながっており、
    開度と車速が一体で動くため定常判定をほとんど通らない）。
    - `LowOpenStairPattern`（`TrimStairPattern` を継承。kind は CRUISE_TRIM のまま）を追加:
      停車から `accel_target_kmh`（= 1 段目と同じ開度で終える低い車速）まで加速し、
      `trim_steps_pct` を `step_hold_s` ずつ一定保持して、その開度固有の平衡車速へ収束させる。
      `TrimStairPattern` の機構（`_command_openings` の CRUISE_TRIM 分岐・
      `_phase_after_accel`・`_finish_pattern` 等）をそのまま使うが、低速域が運転域の
      真ん中に来るため、低速終了の判定だけ `min_speed_kmh`（既定 2.0）に差し替える
      （`_advance_trim_stair`。トリム階段の `coast_down_stop_speed_kmh`=5.0 のままだと
      1 段目で即終了してしまう）。
"""

from __future__ import annotations

import asyncio
import time
from collections import deque
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Protocol

from src.domain.control.conversions import (
    G_TO_KMHS,
    VEHICLE_STOP_SPEED_KMH,
    clamp_opening,
    opening_to_position,
)
from src.domain.control.pedal_safety import enforce_pedal_exclusion
from src.models.drive_log import DriveLogData
from src.models.learning_drive import LearningPattern, PatternKind
from src.models.profile import VehicleProfile
from tests.research.axis_monitor import AxisMonitor
from tests.research.axis_safety import AxisSafetyNet

PATTERN_LOOP_INTERVAL_S: float = 0.1  # 100ms 周期（drive_logs の記録間隔に一致。本番と同じ）
STOP_SPEED_KMH: float = VEHICLE_STOP_SPEED_KMH


@dataclass(frozen=True)
class SpeedTargetPattern(LearningPattern):
    """加速を cap ではなく `accel_target_kmh` に達したところで終えるパターン（研究側の追加）。

    cap の先読み（overspeed_lead_s）は最高速の行き過ぎを防ぐためのものなので使わない。
    """

    accel_target_kmh: float = 0.0


@dataclass(frozen=True)
class TrimStairPattern(LearningPattern):
    """トリム階段（A3。研究側の追加）: `accel_target_kmh` まで加速 → `trim_steps_pct` を順に保持。

    kind は CRUISE_TRIM を使う（CSV の pattern・phase 列の形式を変えない。段は開度で見分ける）。
    """

    accel_target_kmh: float = 0.0
    trim_steps_pct: tuple[float, ...] = ()
    step_hold_s: float = 8.0


@dataclass(frozen=True)
class CruiseStairPattern(LearningPattern):
    """定速階段（段2。研究側の追加）: `hold_speeds_kmh` を昇順に、弱い PI で 1 本ずつ保持する。

    kind は CRUISE_TRIM を使う（CSV の pattern・phase 列の形式を変えない。段は phase 列の
    CRUISE_HOLD と開度で見分ける）。トリム階段（`TrimStairPattern`）と違い、開度は固定値の
    列ではなく速度フィードバックの PI で決める（事実5・6の「定速保持の状態が無い」対策）。

    Attributes:
        hold_speeds_kmh: 保持する車速 [km/h]（昇順）。DRIVE_ACCEL はこの最初の値で終える。
        settle_tol_kmh: 「保持できた」とみなす速度偏差の許容幅 [km/h]。
        settle_s: 許容幅の中に連続でこの秒数いたら保持タイマーを開始する。
        hold_s: 保持タイマー開始後、この秒数記録したら次の車速へ進む。
        step_timeout_s: 1 車速あたりの打ち切り（保持できていなくても次へ進む）[s]。
        kp: 速度偏差 [km/h] → 開度 [%] の比例ゲイン [%/(km/h)]。
        ki: 積分ゲイン [%/(km/h·s)]。
        max_rate_pct_per_s: 1 周期あたりの開度変化量の上限 [%/s]（実開度が追従できる速さに保つ）。
        initial_offset_pct: CRUISE_HOLD に入った直後の初期開度 = アクセル不感帯 + この値 [%]。
    """

    hold_speeds_kmh: tuple[float, ...] = ()
    settle_tol_kmh: float = 1.0
    settle_s: float = 3.0
    hold_s: float = 8.0
    step_timeout_s: float = 30.0
    kp: float = 0.3
    ki: float = 0.05
    max_rate_pct_per_s: float = 1.0
    initial_offset_pct: float = 7.0


@dataclass(frozen=True)
class LowOpenStairPattern(TrimStairPattern):
    """低開度階段（段3-1）: 不感帯の直上を細かく刻み、1 段ずつ一定保持して平衡車速へ収束させる。

    kind は CRUISE_TRIM（TrimStairPattern と同じ。CSV の pattern/phase 列の形式を変えない）。
    トリム階段との違いは運転域だけ: トリム階段が 50〜120 km/h から開度を下げていくのに対し、
    こちらは停車から始めて 5〜23 km/h を上り下りする。そのため `_advance_trim_stair` の
    低速終了判定（coast_down_stop_speed_kmh = 5.0 km/h）が運転域の真ん中に来てしまい、
    1 段目で即終了する。この 1 点だけをパターン側の `min_speed_kmh` で差し替える。
    """

    min_speed_kmh: float = 2.0


@dataclass(frozen=True)
class CreepLaunchPattern(LearningPattern):
    """クリープ発進・クリープ域ブレーキ保持（段1。ProblemReport_20260916 課題#2。研究側の追加）。

    両ペダル 0%（完全解放）でクリープ自走し、専用フェーズ `_Phase.CREEP_LAUNCH` で `target_kmh`
    到達または `timeout_s` の打ち切りを待つ。到達後の扱いは `hold_after`:
      - False（クリープ発進）: 通常の停車復帰（`_Phase.DRIVE_BRAKE`、停車保持開度）。
      - True（クリープ域ブレーキ保持）: `brake_opening`（不感帯 + offset。呼び出し側で
        max_brake_opening にクランプ済み）を保持して停車させる（既存の `_Phase.BRAKE_HOLD`）。

    kind は CREEP_SETTLE を流用する（両ペダル解放でクリープを測る、という意味は共通）。CSV の
    pattern 列表示にのみ影響し、`_advance` の CREEP_SETTLE 専用分岐は `_Phase.MEASURE` でしか
    見ない（本パターンの初期フェーズは CREEP_LAUNCH）ため衝突しない。
    """

    target_kmh: float = 0.0
    timeout_s: float = 20.0
    hold_after: bool = False


def _accel_target_kmh(pattern: LearningPattern) -> float | None:
    """加速を終える目標車速。None なら cap − 先読み。"""
    if isinstance(pattern, SpeedTargetPattern | TrimStairPattern):
        return pattern.accel_target_kmh
    if isinstance(pattern, CruiseStairPattern) and pattern.hold_speeds_kmh:
        return pattern.hold_speeds_kmh[0]
    return None


class _Phase(Enum):
    MEASURE = auto()
    DRIVE_ACCEL = auto()
    COAST = auto()
    DRIVE_BRAKE = auto()
    BRAKE_HOLD = auto()
    CRUISE_TRIM = auto()
    CRUISE_HOLD = auto()  # 定速階段（段2）。CRUISE_TRIM とは別フェーズにして CSV で見分ける
    CREEP_LAUNCH = auto()  # クリープ発進・クリープ域ブレーキ保持（段1）。両ペダル 0% で自走
    DONE = auto()


@dataclass
class PatternLoopConfig:
    """開ループ実行のタイミング・ランプ。`LearningLoopConfig` と同じフィールド名。

    既定値も本番と同じ。ただし A6 で accel_full_range_timeout_s（20 → 30s）・brake_stop_timeout_s
    （10 → 60s、意味も「停車しなければ中断する安全上限」）を変え、gov_release_frac・
    gov_raise_step_pct（ガバナーの解除）を足した。
    """

    brake_stop_timeout_s: float = field(default=60.0)
    brake_hold_timeout_s: float = field(default=20.0)
    accel_ramp_time_s: float = field(default=1.5)
    brake_ramp_time_s: float = field(default=1.5)
    pedal_release_time_s: float = field(default=0.4)
    pedal_step_time_s: float = field(default=0.2)
    skip_consecutive_required: int = field(default=2)
    g_smoothing_window_s: float = field(default=0.4)
    g_limit_frac: float = field(default=0.98)
    gov_reduce_step_pct: float = field(default=2.0)
    gov_release_frac: float = field(default=0.7)  # 加減速が上限のこの割合を下回ったら頭打ちを戻す
    gov_raise_step_pct: float = field(default=0.5)  # 戻すときの 1 周期あたりの量 [%]
    accel_speed_cap_frac: float = field(default=0.98)
    accel_full_range_timeout_s: float = field(default=30.0)
    overspeed_lead_s: float = field(default=1.2)
    overspeed_recovery_brake_pct: float = field(default=30.0)
    coast_down_stop_speed_kmh: float = field(default=5.0)
    coast_timeout_s: float = field(default=90.0)
    creep_settle_min_s: float = field(default=13.0)
    creep_settle_stable_tol_kmhs: float = field(default=0.3)
    creep_settle_stable_duration_s: float = field(default=2.0)
    creep_settle_timeout_s: float = field(default=25.0)
    # クリープ発進（段1b。CreepLaunchPattern）の平衡到達判定。learning.creep_launch_settle_* から
    # pattern_drive.py が渡す（既定値はここと config.py の LearningSection とで揃えている）
    creep_launch_settle_kmhs: float = field(default=0.1)
    creep_launch_settle_s: float = field(default=2.0)
    # 2026-09-17（誤判定修正）: 停車保持解放直後は車速0・傾き0で
    # abs(accel_kmhs) < creep_launch_settle_kmhs が最初から成立してしまい、車が動き出す前に
    # 「平衡到達」と誤判定していた。車速がこの値以上になるまで安定カウントを積まない
    # （本質的なガード）。learning.creep_launch_min_speed_kmh から渡す
    creep_launch_min_speed_kmh: float = field(default=1.0)
    # elapsed がこの値以上になるまで平衡到達で終了しない（_advance_creep_settle の
    # creep_settle_min_s と同じ流儀の最短時間ガード）。learning.creep_launch_settle_min_s から渡す
    creep_launch_settle_min_s: float = field(default=5.0)


class ActuatorDriverProtocol(Protocol):
    async def move_to_position_timed(
        self, target_pos: int, current_pos: int, duration_s: float
    ) -> None: ...

    async def read_monitor(self) -> AxisMonitor: ...

    async def is_alarm_active(self) -> bool: ...


class CANReaderProtocol(Protocol):
    async def read_speed(self) -> float: ...


# on_sample(データ, パターン番号, 段, ガバナー作動, アラーム(アクセル), アラーム(ブレーキ),
#           モニター(アクセル), モニター(ブレーキ), 周期の処理時間 [ms])
OnSample = Callable[
    [DriveLogData, int, str, bool, bool | None, bool | None, AxisMonitor, AxisMonitor, float],
    None,
]


class PatternLoop:
    """開度パターンを開ループ実行し、1 サイクルごとに `on_sample` でログへ渡す研究用ループ。

    `src/domain/control/learning_loop.py::LearningLoop` のフェーズ遷移・ガバナ・非常停止判定を
    移植したもの（アルゴリズムは変更しない）。
    """

    def __init__(
        self,
        *,
        accel_driver: ActuatorDriverProtocol,
        brake_driver: ActuatorDriverProtocol,
        can_reader: CANReaderProtocol,
        profile: VehicleProfile,
        patterns: list[LearningPattern],
        overcurrent_limit_ma: float,
        on_complete: Callable[[], Awaitable[None]],
        on_emergency: Callable[[], Awaitable[None]],
        on_sample: OnSample,
        config: PatternLoopConfig | None = None,
        interval_s: float = PATTERN_LOOP_INTERVAL_S,
    ) -> None:
        self._accel_driver = accel_driver
        self._brake_driver = brake_driver
        self._can_reader = can_reader
        self._profile = profile
        self._patterns = patterns
        self._overcurrent_limit_ma = overcurrent_limit_ma
        self._on_complete = on_complete
        self._on_emergency = on_emergency
        self._on_sample = on_sample
        self._config = config if config is not None else PatternLoopConfig()
        self._interval_s = interval_s

        self._max_decel_kmhs = max(profile.max_decel_g * G_TO_KMHS, 0.0)
        self._g_limit_kmhs = self._max_decel_kmhs * self._config.g_limit_frac
        self._accel_speed_cap = profile.max_speed * self._config.accel_speed_cap_frac

        self._running = False
        self._cycle_task: asyncio.Task[None] | None = None
        self.abort_reason: str | None = None  # 非常停止した理由（正常終了なら None）
        # 状態機械（同期）が決めた中断。周期の終わりで処理する
        self._pending_abort: str | None = None
        self._safety = AxisSafetyNet(interval_s)
        self._cycles = 0
        self._started_at: float | None = None

        self._pattern_idx = 0
        self._phase = self._initial_phase(0)
        self._phase_started_at: float | None = None
        self._prev_speed: float | None = None
        self._prev_time: float | None = None
        self._skip_count = 0
        self._stable_count = 0
        self._trim_step = 0  # トリム階段・定速階段で共通の段番号
        self._trim_step_started_at = 0.0
        self._accel_gov_cap: float | None = None
        self._brake_gov_cap: float | None = None
        self._overspeed_recovery = False
        self._speed_hist: deque[tuple[float, float]] = deque()
        self._last_speed = 0.0  # 定速階段の PI が使う直近の車速（_execute_one_cycle で更新）
        # 定速階段（CRUISE_HOLD）の PI 状態。フェーズに入るたび _enter_phase でリセットする
        self._cruise_opening: float | None = None  # None なら未初期化（次回に初期開度を設定）
        self._cruise_integral = 0.0
        self._cruise_settle_since: float | None = None  # 許容幅に連続で入り始めた時刻
        self._cruise_hold_start: float | None = None  # 保持タイマーを開始した時刻
        self._accel_pos_cmd = 0
        self._brake_pos_cmd = 0
        self._current_accel_opening = 0.0
        self._current_brake_opening = 0.0
        self._accel_request = 0.0  # ガバナーをかける前の指令開度（解除の判定に使う）
        self._brake_request = 0.0
        self._brake_ramp_from = 0.0  # ブレーキのランプを始める開度（フェーズに入ったときの開度）
        # 停車復帰（ACCEL_SWEEP 以外）の目標開度 = 停車保持開度
        self._stop_return_brake_pct = min(
            self._profile.feedforward_params.stop_brake_opening_pct,
            self._profile.max_brake_opening,
        )

        calib = self._profile.calibration
        if calib is not None:
            self._accel_pos_cmd = calib.accel_zero_pos
            self._brake_pos_cmd = opening_to_position(
                self._stop_return_brake_pct, calib.brake_zero_pos, calib.brake_full_pos
            )

    # ── ライフサイクル ─────────────────────────────────────────────
    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._cycle_task = asyncio.ensure_future(self._run())

    def stop(self) -> None:
        self._running = False

    async def stop_and_join(self, timeout_s: float = 2.0) -> None:
        self.stop()
        task = self._cycle_task
        if task is None or task.done() or task is asyncio.current_task():
            return
        try:
            await asyncio.wait_for(asyncio.shield(task), timeout_s)
        except TimeoutError:
            task.cancel()
        except Exception:
            pass

    async def _run(self) -> None:
        loop = asyncio.get_running_loop()
        next_tick = loop.time()
        try:
            while self._running:
                await self._execute_one_cycle()
                if not self._running:
                    return
                next_tick += self._interval_s
                delay = max(0.0, next_tick - loop.time())
                await asyncio.sleep(delay)
        except Exception as exc:
            await self._abort_emergency(f"パターン走行ループで例外（{type(exc).__name__}: {exc}）")

    # ── 1 サイクル ─────────────────────────────────────────────────
    async def _execute_one_cycle(self) -> None:
        if not self._running:
            return

        loop = asyncio.get_running_loop()
        now = loop.time()
        cycle_started = time.perf_counter()

        if self._pattern_idx >= len(self._patterns):
            self._running = False
            await self._on_complete()
            return

        pattern = self._patterns[self._pattern_idx]
        if self._phase_started_at is None:
            self._phase_started_at = now
        if self._started_at is None:
            self._started_at = now

        try:
            speed = await self._can_reader.read_speed()
        except Exception as exc:
            await self._abort_emergency(f"CAN 車速を読めません（{type(exc).__name__}: {exc}）")
            return

        self._last_speed = speed
        self._speed_hist.append((now, speed))
        accel_kmhs = self._smoothed_accel(now)

        self._update_governor(accel_kmhs)
        accel_opening, brake_opening = self._command_openings(pattern, now)
        accel_opening, brake_opening = enforce_pedal_exclusion(accel_opening, brake_opening)
        self._current_accel_opening = accel_opening
        self._current_brake_opening = brake_opening

        calib = self._profile.calibration
        if calib is None:
            await self._abort_emergency("車両プロファイルにキャリブレーションがありません")
            return

        accel_pos = opening_to_position(accel_opening, calib.accel_zero_pos, calib.accel_full_pos)
        brake_pos = opening_to_position(brake_opening, calib.brake_zero_pos, calib.brake_full_pos)

        if not self._running:
            return

        try:
            monitor_accel, monitor_brake = await self._drive_or_sense(accel_pos, brake_pos)
        except Exception as exc:
            await self._abort_emergency(
                f"アクチュエータ通信に失敗しました（{type(exc).__name__}: {exc}）"
            )
            return

        cycle_ms = 1000.0 * (time.perf_counter() - cycle_started)
        accel_current, brake_current = monitor_accel.current_ma, monitor_brake.current_ma
        t = now - self._started_at
        for label, current in (("アクセル", accel_current), ("ブレーキ", brake_current)):
            if current > self._overcurrent_limit_ma:
                await self._abort_emergency(
                    f"{label}軸が過電流です"
                    f"（{current:.0f} mA > {self._overcurrent_limit_ma:.0f} mA、t={t:.1f}s）"
                )
                return
        alarm_accel, alarm_brake = await self._safety.poll_alarms(
            self._cycles, self._accel_driver, self._brake_driver
        )
        self._cycles += 1
        reason = self._safety.check(
            t=t, accel_pos=accel_pos, brake_pos=brake_pos,
            accel_current=accel_current, brake_current=brake_current,
            alarm_accel=alarm_accel, alarm_brake=alarm_brake,
        )
        if reason is not None:
            await self._abort_emergency(reason)
            return

        self._on_sample(
            DriveLogData(
                ref_speed_kmh=None,
                actual_speed_kmh=speed,
                accel_opening=accel_opening,
                brake_opening=brake_opening,
                accel_pos=accel_pos,
                brake_pos=brake_pos,
                accel_current=accel_current,
                brake_current=brake_current,
            ),
            self._pattern_idx,
            self._phase.name,
            self._governor_limiting(),
            alarm_accel,
            alarm_brake,
            monitor_accel,
            monitor_brake,
            cycle_ms,
        )

        self._advance(pattern, speed, accel_kmhs, now)
        if self._pending_abort is not None:
            await self._abort_emergency(self._pending_abort)
            return

        self._prev_speed = speed
        self._prev_time = now

    # ── 状態機械 ───────────────────────────────────────────────────
    def _initial_phase(self, idx: int) -> _Phase:
        if idx >= len(self._patterns):
            return _Phase.DONE
        if isinstance(self._patterns[idx], CreepLaunchPattern):
            return _Phase.CREEP_LAUNCH
        if self._patterns[idx].kind in (
            PatternKind.ACCEL_SWEEP,
            PatternKind.BRAKE_HOLD,
            PatternKind.COAST_DOWN,
            PatternKind.CRUISE_TRIM,
        ):
            return _Phase.DRIVE_ACCEL
        return _Phase.MEASURE

    def _command_openings(self, pattern: LearningPattern, now: float) -> tuple[float, float]:
        assert self._phase_started_at is not None
        elapsed = max(0.0, now - self._phase_started_at)
        if self._phase is _Phase.DRIVE_ACCEL:
            ramped = pattern.accel_opening * self._ramp_fraction(
                elapsed, self._config.accel_ramp_time_s
            )
            self._accel_request = ramped
            if self._accel_gov_cap is not None:
                ramped = min(ramped, self._accel_gov_cap)
            return clamp_opening(ramped, self._profile.max_accel_opening), 0.0
        if self._phase is _Phase.COAST:
            return 0.0, 0.0
        if self._phase is _Phase.CREEP_LAUNCH:
            return 0.0, 0.0  # 両ペダル完全解放（クリープのみで自走）
        if self._phase is _Phase.CRUISE_TRIM:
            opening = pattern.trim_opening
            if isinstance(pattern, TrimStairPattern):
                opening = pattern.trim_steps_pct[self._trim_step]
            trim = clamp_opening(opening, self._profile.max_accel_opening)
            return trim, 0.0
        if self._phase is _Phase.CRUISE_HOLD:
            assert isinstance(pattern, CruiseStairPattern)
            return self._cruise_hold_opening(pattern), 0.0
        if self._phase in (_Phase.DRIVE_BRAKE, _Phase.BRAKE_HOLD):
            target_brake = pattern.brake_opening
            if self._phase is _Phase.DRIVE_BRAKE:
                if pattern.kind is not PatternKind.ACCEL_SWEEP:
                    target_brake = self._stop_return_brake_pct  # 停車復帰（A6）
                if self._overspeed_recovery:
                    target_brake = max(target_brake, self._config.overspeed_recovery_brake_pct)
            start = self._brake_ramp_from
            frac = self._ramp_fraction(elapsed, self._config.brake_ramp_time_s)
            ramped = start + (target_brake - start) * frac
            self._brake_request = ramped
            if self._brake_gov_cap is not None:
                ramped = min(ramped, self._brake_gov_cap)
            return 0.0, clamp_opening(ramped, self._profile.max_brake_opening)
        return pattern.accel_opening, pattern.brake_opening

    @staticmethod
    def _ramp_fraction(elapsed: float, ramp_time_s: float) -> float:
        if ramp_time_s <= 0.0:
            return 1.0
        return min(1.0, max(0.0, elapsed) / ramp_time_s)

    def _advance(
        self, pattern: LearningPattern, speed: float, accel_kmhs: float, now: float
    ) -> None:
        assert self._phase_started_at is not None
        elapsed = now - self._phase_started_at

        if self._phase is _Phase.DRIVE_ACCEL:
            self._advance_drive_accel(pattern, speed, accel_kmhs, elapsed, now)
            return
        if self._phase is _Phase.COAST:
            self._advance_coast(speed, elapsed, now)
            return
        if self._phase is _Phase.CREEP_LAUNCH:
            self._advance_creep_launch(pattern, speed, accel_kmhs, elapsed, now)
            return
        if self._phase is _Phase.DRIVE_BRAKE:
            if speed <= STOP_SPEED_KMH:
                self._advance_pattern(now)
            elif elapsed >= self._config.brake_stop_timeout_s:
                self._pending_abort = (
                    f"停車復帰（DRIVE_BRAKE）で brake_stop_timeout_s="
                    f"{self._config.brake_stop_timeout_s:g}s 以内に停車しません"
                    f"（パターン {self._pattern_idx + 1}:{pattern.kind.name}、"
                    f"車速 {speed:.1f} km/h、"
                    f"ブレーキ {self._current_brake_opening:.2f}%）"
                )
            return
        if self._phase is _Phase.BRAKE_HOLD:
            if speed <= STOP_SPEED_KMH:
                self._advance_pattern(now)
            elif elapsed >= self._config.brake_hold_timeout_s:
                self._finish_pattern(speed, now)
            return
        if self._phase is _Phase.CRUISE_TRIM:
            self._advance_cruise_trim(speed, elapsed, now)
            return
        if self._phase is _Phase.CRUISE_HOLD:
            self._advance_cruise_hold(speed, now)
            return
        if self._phase is _Phase.MEASURE and pattern.kind is PatternKind.CREEP_SETTLE:
            self._advance_creep_settle(elapsed, accel_kmhs, now)
            return
        if self._phase is _Phase.MEASURE:
            if elapsed >= pattern.hold_duration_s:
                self._advance_pattern(now)
            return

    def _advance_drive_accel(
        self, pattern: LearningPattern, speed: float, accel_kmhs: float, elapsed: float, now: float
    ) -> None:
        cfg = self._config
        if speed > self._profile.max_speed:
            self._skip_count += 1
        else:
            self._skip_count = 0
        if self._skip_count >= cfg.skip_consecutive_required:
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return

        target = _accel_target_kmh(pattern)
        if target is not None:
            exit_speed = target
        else:
            exit_speed = self._accel_speed_cap - max(0.0, accel_kmhs) * cfg.overspeed_lead_s
        if speed >= exit_speed or elapsed >= cfg.accel_full_range_timeout_s:
            self._enter_phase(self._phase_after_accel(pattern), now)

    @staticmethod
    def _phase_after_accel(pattern: LearningPattern) -> _Phase:
        if pattern.kind is PatternKind.ACCEL_SWEEP:
            return _Phase.DRIVE_BRAKE
        if pattern.kind is PatternKind.BRAKE_HOLD:
            return _Phase.BRAKE_HOLD
        if isinstance(pattern, CruiseStairPattern):  # TrimStairPattern と同じ kind なので先に判定
            return _Phase.CRUISE_HOLD
        if pattern.kind is PatternKind.CRUISE_TRIM:
            return _Phase.CRUISE_TRIM
        return _Phase.COAST

    def _advance_coast(self, speed: float, elapsed: float, now: float) -> None:
        cfg = self._config
        if speed > self._profile.max_speed:
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return
        if speed <= cfg.coast_down_stop_speed_kmh or elapsed >= cfg.coast_timeout_s:
            self._finish_pattern(speed, now)

    def _advance_creep_launch(
        self, pattern: LearningPattern, speed: float, accel_kmhs: float, elapsed: float, now: float
    ) -> None:
        """クリープ発進・クリープ域ブレーキ保持（段1b）の前進判定。

        2026-09-17（ProblemReport_20260916 ユーザー決定）: 終了条件を「target_kmh 到達」から
        「平衡到達（加速が止まった）」に変えた。車速の傾きが creep_launch_settle_kmhs 未満の
        状態が creep_launch_settle_s 続いたら平衡到達とみなす（_advance_creep_settle と同じ
        流儀）。target_kmh は安全上限として残す（クリープでこれ以上の車速にはならないはずなので、
        達したら平衡を待たず打ち切る）。timeout_s は従来どおりの打ち切り。

        2026-09-17（誤判定修正）: 停車保持を解放した直後は車速 0・傾き 0 で
        abs(accel_kmhs) < creep_launch_settle_kmhs が最初から成立してしまい、車が動き出す前に
        「平衡到達」と誤判定して即終了していた（実機では解放から動き出しまで約 0.3〜0.4s、その
        間の傾きは 0.17 km/h/s 程度でしきい値 0.1 に近く、確実に誤判定する）。
        _advance_creep_settle の settled 判定（creep_settle_min_s による最短時間ガード）に
        揃えて、2 つのガードを足す:
          - 車速ゲート: speed が creep_launch_min_speed_kmh 以上になるまで安定カウントを
            積まない（動き出す前を「平衡」に含めない、本質的なガード）。
          - 最短時間ガード: elapsed が creep_launch_settle_min_s 以上になるまで終了しない。
        到達後は hold_after=True なら BRAKE_HOLD（brake_opening を保持して停車）、
        False なら通常の停車復帰（_finish_pattern）へ進む。
        """
        assert isinstance(pattern, CreepLaunchPattern)
        cfg = self._config
        if speed > self._profile.max_speed:
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return
        moving = speed >= cfg.creep_launch_min_speed_kmh
        if moving and abs(accel_kmhs) < cfg.creep_launch_settle_kmhs:
            self._stable_count += 1
        else:
            self._stable_count = 0
        stable_long = self._stable_count * self._interval_s >= cfg.creep_launch_settle_s
        settled = elapsed >= cfg.creep_launch_settle_min_s and stable_long
        if settled or speed >= pattern.target_kmh or elapsed >= pattern.timeout_s:
            self._stable_count = 0
            if pattern.hold_after:
                self._enter_phase(_Phase.BRAKE_HOLD, now)
            else:
                self._finish_pattern(speed, now)

    def _advance_cruise_trim(self, speed: float, elapsed: float, now: float) -> None:
        cfg = self._config
        if speed > self._profile.max_speed:
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return
        pattern = self._patterns[self._pattern_idx]
        if isinstance(pattern, TrimStairPattern):
            self._advance_trim_stair(pattern, speed, now)
            return
        if speed >= self._accel_speed_cap:
            self._finish_pattern(speed, now)
            return
        if elapsed >= pattern.hold_duration_s or speed <= cfg.coast_down_stop_speed_kmh:
            self._finish_pattern(speed, now)

    def _advance_trim_stair(self, pattern: TrimStairPattern, speed: float, now: float) -> None:
        """最高速超えの判定は呼び出し元（_advance_cruise_trim）が済ませている。

        低速終了の判定値は `LowOpenStairPattern` だけ `min_speed_kmh` に差し替える
        （運転域が低速に来るため、トリム階段の coast_down_stop_speed_kmh=5.0 のままだと
        1 段目で即終了してしまう。クラスの docstring 参照）。
        """
        floor = (
            pattern.min_speed_kmh
            if isinstance(pattern, LowOpenStairPattern)
            else self._config.coast_down_stop_speed_kmh
        )
        if speed <= floor:
            self._finish_pattern(speed, now)
            return
        last = self._trim_step >= len(pattern.trim_steps_pct) - 1
        if speed >= self._accel_speed_cap:
            if not last:  # cap の安全策: 低い段へ下げる。最後の段は保持を続ける
                self._next_trim_step(now)
            return
        if now - self._trim_step_started_at >= pattern.step_hold_s:
            if last:
                self._finish_pattern(speed, now)
            else:
                self._next_trim_step(now)

    def _next_trim_step(self, now: float) -> None:
        self._trim_step += 1
        self._trim_step_started_at = now

    def _advance_cruise_hold(self, speed: float, now: float) -> None:
        """定速階段（段2）の前進判定。

        settle_tol_kmh 以内に settle_s 続けて入ったら保持タイマーを開始し、hold_s 経過で
        次の車速へ（積分は引き継ぎ、settle/hold タイマーだけリセット）。step_timeout_s を
        過ぎたら保持できていなくても次へ進む。最高速超え・低速側の回復はトリム階段と同じ。
        """
        cfg = self._config
        if speed > self._profile.max_speed:
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return
        if speed <= cfg.coast_down_stop_speed_kmh:
            self._finish_pattern(speed, now)
            return
        pattern = self._patterns[self._pattern_idx]
        assert isinstance(pattern, CruiseStairPattern)
        target = pattern.hold_speeds_kmh[self._trim_step]
        last = self._trim_step >= len(pattern.hold_speeds_kmh) - 1

        if abs(speed - target) <= pattern.settle_tol_kmh:
            if self._cruise_settle_since is None:
                self._cruise_settle_since = now
            if (
                self._cruise_hold_start is None
                and now - self._cruise_settle_since >= pattern.settle_s
            ):
                self._cruise_hold_start = now
        elif self._cruise_hold_start is None:
            self._cruise_settle_since = None  # 保持タイマー開始前は連続条件をやり直す

        held_enough = (
            self._cruise_hold_start is not None
            and now - self._cruise_hold_start >= pattern.hold_s
        )
        timed_out = now - self._trim_step_started_at >= pattern.step_timeout_s
        if held_enough or timed_out:
            if last:
                self._finish_pattern(speed, now)
            else:
                self._next_cruise_step(now)

    def _next_cruise_step(self, now: float) -> None:
        self._trim_step += 1
        self._trim_step_started_at = now
        self._cruise_settle_since = None
        self._cruise_hold_start = None

    def _cruise_hold_opening(self, pattern: CruiseStairPattern) -> float:
        """CRUISE_HOLD の開度 = 目標車速への PI（速度偏差 [km/h] → 開度 [%]）。

        手順3では指令→実開度の遅れがほぼ無いと実測済み（docs/memo.md 事実1）なので、低ゲイン・
        レート制限で「実開度 ≈ 指令」を保ったまま定常開度を記録する。アンチワインドアップは
        [アクセル不感帯, max_accel_opening] で頭打ちの方向へ誤差が続くときだけ積分を止める
        （まだ積分していない値で判定する。素朴な条件付き積分）。開度・積分は車速の段をまたいでも
        引き継ぎ、段の変わり目で跳躍させない（初期化は CRUISE_HOLD に入った最初の 1 回だけ、
        `_enter_phase` が `_cruise_opening = None` にした直後に行う）。
        """
        ff = self._profile.feedforward_params
        min_pct = ff.accel_deadband_pct
        max_pct = self._profile.max_accel_opening
        dt = self._interval_s
        if self._cruise_opening is None:
            # 初期開度 = 不感帯 + initial_offset_pct。DRIVE_ACCEL の加速用開度（BRAKE_HOLD_ACCEL_PCT
            # 相当）は定速には大きすぎ、そこから始めるとレート制限で最初の車速に間に合わない。
            # 積分の初期値は「この初期開度」自体にする（0 から始めると、P 項だけでは初期開度に
            # 遠く及ばず、最初の周期でいきなり大きく下げる向きに動いてしまう）。
            self._cruise_opening = min(max(min_pct + pattern.initial_offset_pct, min_pct), max_pct)
            self._cruise_integral = self._cruise_opening
        target = pattern.hold_speeds_kmh[self._trim_step]
        error = target - self._last_speed
        pre_integration_output = pattern.kp * error + self._cruise_integral
        saturated_high = pre_integration_output >= max_pct and error > 0.0
        saturated_low = pre_integration_output <= min_pct and error < 0.0
        if not (saturated_high or saturated_low):
            self._cruise_integral += pattern.ki * error * dt
        desired = pattern.kp * error + self._cruise_integral
        desired = min(max_pct, max(min_pct, desired))
        step_limit = pattern.max_rate_pct_per_s * dt
        delta = min(step_limit, max(-step_limit, desired - self._cruise_opening))
        self._cruise_opening += delta
        return self._cruise_opening

    def _advance_creep_settle(self, elapsed: float, accel_kmhs: float, now: float) -> None:
        cfg = self._config
        if abs(accel_kmhs) < cfg.creep_settle_stable_tol_kmhs:
            self._stable_count += 1
        else:
            self._stable_count = 0
        stable_long = self._stable_count * self._interval_s >= cfg.creep_settle_stable_duration_s
        settled = elapsed >= cfg.creep_settle_min_s and stable_long
        if settled or elapsed >= cfg.creep_settle_timeout_s:
            self._stable_count = 0
            self._advance_pattern(now)

    def _finish_pattern(self, speed: float, now: float) -> None:
        """運転パターンの計測が終わった。停車していなければ停車復帰（DRIVE_BRAKE）に入る（A6）。"""
        if speed > STOP_SPEED_KMH:
            self._enter_phase(_Phase.DRIVE_BRAKE, now)
        else:
            self._advance_pattern(now)

    def _advance_pattern(self, now: float) -> None:
        self._pattern_idx += 1
        self._enter_phase(self._initial_phase(self._pattern_idx), now)

    def _enter_phase(self, phase: _Phase, now: float, *, overspeed_recovery: bool = False) -> None:
        self._phase = phase
        self._phase_started_at = now
        self._skip_count = 0
        self._stable_count = 0
        self._trim_step = 0
        self._trim_step_started_at = now
        self._accel_gov_cap = None
        self._brake_gov_cap = None
        self._overspeed_recovery = overspeed_recovery
        self._brake_ramp_from = self._current_brake_opening
        # 定速階段の PI 状態。CRUISE_HOLD はパターンにつき一度しか入らないので、ここでリセット
        # しておけば「初期化は最初の 1 回だけ」（_cruise_hold_opening）の前提になる
        self._cruise_opening = None
        self._cruise_integral = 0.0
        self._cruise_settle_since = None
        self._cruise_hold_start = None

    def _update_governor(self, accel_kmhs: float) -> None:
        """G ガバナー。CRUISE_HOLD には適用しない（段2）: PI 自体が低ゲイン・レート制限で
        穏やかなので、頭打ちの必要が無いと判断した（実測前の設計判断。過大な加減速が実際に
        出るようなら段4のユーザー確認で気づける）。
        """
        if self._g_limit_kmhs <= 0.0:
            return
        if self._phase is _Phase.DRIVE_ACCEL:
            self._accel_gov_cap = self._next_gov_cap(
                self._accel_gov_cap, accel_kmhs, self._current_accel_opening, self._accel_request
            )
        elif self._phase in (_Phase.DRIVE_BRAKE, _Phase.BRAKE_HOLD):
            self._brake_gov_cap = self._next_gov_cap(
                self._brake_gov_cap, -accel_kmhs, self._current_brake_opening, self._brake_request
            )

    def _next_gov_cap(
        self, cap: float | None, pedal_accel_kmhs: float, current: float, request: float
    ) -> float | None:
        """頭打ちの次の値。`pedal_accel_kmhs` はそのペダルが出す向きの加速度（ブレーキなら減速度）。

        上限以上 … 本番と同じ。最初は現在開度で頭打ち、以降 1 周期ごとに gov_reduce_step_pct 下げる
        上限 × gov_release_frac 未満 … gov_raise_step_pct ずつ戻し、
                                        指令（request）に届いたら解除（A6）
        その間 … 保持
        """
        cfg = self._config
        limit = self._g_limit_kmhs
        if pedal_accel_kmhs >= limit:
            if cap is None:
                return current
            return max(0.0, cap - cfg.gov_reduce_step_pct)
        if cap is None:
            return None
        if pedal_accel_kmhs < limit * cfg.gov_release_frac:
            raised = cap + cfg.gov_raise_step_pct
            return None if raised >= request else raised
        return cap

    def _governor_limiting(self) -> bool:
        """この周期の指令がガバナーで削られたか（CSV の governor_active 列）。"""
        if self._phase is _Phase.DRIVE_ACCEL and self._accel_gov_cap is not None:
            return self._accel_gov_cap < self._accel_request
        braking = self._phase in (_Phase.DRIVE_BRAKE, _Phase.BRAKE_HOLD)
        if braking and self._brake_gov_cap is not None:
            return self._brake_gov_cap < self._brake_request
        return False

    def _smoothed_accel(self, now: float) -> float:
        hist = self._speed_hist
        win = self._config.g_smoothing_window_s
        while len(hist) >= 2 and now - hist[0][0] > win:
            hist.popleft()
        if len(hist) >= 2:
            t0, v0 = hist[0]
            t1, v1 = hist[-1]
            if t1 - t0 > 0.0:
                return (v1 - v0) / (t1 - t0)
        return self._measured_accel()

    def _measured_accel(self) -> float:
        if self._prev_speed is None or self._prev_time is None:
            return 0.0
        cur = self._speed_hist[-1][1] if self._speed_hist else self._prev_speed
        dt = self._speed_hist[-1][0] - self._prev_time if self._speed_hist else 0.0
        if dt <= 0.0:
            return 0.0
        return (cur - self._prev_speed) / dt

    # ── 補助 ───────────────────────────────────────────────────────
    def _move_duration(self) -> float:
        if self._phase in (
            _Phase.DRIVE_ACCEL, _Phase.DRIVE_BRAKE, _Phase.BRAKE_HOLD, _Phase.CRUISE_HOLD,
        ):
            return self._config.pedal_step_time_s
        return self._config.pedal_release_time_s

    async def _abort_emergency(self, reason: str) -> None:
        self._running = False
        if self.abort_reason is None:
            self.abort_reason = reason
        await self._on_emergency()

    async def _drive_or_sense(
        self, accel_pos: int, brake_pos: int
    ) -> tuple[AxisMonitor, AxisMonitor]:
        """位置が変わった軸だけ指令し、両軸をまとめ読みする（A7。以前は電流だけ読んでいた）。"""
        dur = self._move_duration()
        a_changed = accel_pos != self._accel_pos_cmd
        b_changed = brake_pos != self._brake_pos_cmd

        async def accel_axis() -> AxisMonitor:
            if a_changed:
                return await self._drive_axis_timed(
                    self._accel_driver, accel_pos, self._accel_pos_cmd, dur
                )
            return await self._accel_driver.read_monitor()

        async def brake_axis() -> AxisMonitor:
            if b_changed:
                return await self._drive_axis_timed(
                    self._brake_driver, brake_pos, self._brake_pos_cmd, dur
                )
            return await self._brake_driver.read_monitor()

        a, b = await asyncio.gather(accel_axis(), brake_axis())
        if a_changed:
            self._accel_pos_cmd = accel_pos
        if b_changed:
            self._brake_pos_cmd = brake_pos
        return a, b

    async def _drive_axis_timed(
        self,
        driver: ActuatorDriverProtocol,
        target_pos: int,
        current_pos: int,
        duration_s: float,
    ) -> AxisMonitor:
        await driver.move_to_position_timed(target_pos, current_pos, duration_s)
        return await driver.read_monitor()


__all__ = [
    "PATTERN_LOOP_INTERVAL_S",
    "STOP_SPEED_KMH",
    "ActuatorDriverProtocol",
    "CANReaderProtocol",
    "OnSample",
    "PatternLoop",
    "CreepLaunchPattern",
    "CruiseStairPattern",
    "LowOpenStairPattern",
    "SpeedTargetPattern",
    "TrimStairPattern",
    "PatternLoopConfig",
]
