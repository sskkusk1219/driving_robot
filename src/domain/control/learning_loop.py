"""学習運転の開ループ実行ループ。

Phase 8 の開度パターン方式を本番実行する。固定開度を順に開ループで指令し、走行全体を
100ms 周期で連続した1本の (実車速, 開度) 軌跡として drive_logs に記録する。収集した連続ログは
後段の Ridge 逆モデル学習（train）が消費する。

DriveLoop（閉ループ FF+PID）と対をなすが、本ループは基準速度追従ではなくパターン列の
状態機械として動く。安全は2段階:
  - スキップ型（非常停止しない）: 過速度（>max_speed）・過G（abs(実測加速度)>max_decel_g、
    加速/減速同一上限）→ 当該パターンの測定を打ち切り次の開度へ。
  - 非常停止型: 過電流・CAN 読取失敗・アクチュエータ失敗・サイクル例外 → on_emergency。
"""

from __future__ import annotations

import asyncio
import logging
from collections import deque
from collections.abc import Awaitable, Callable
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Protocol

from src.domain.control.base_loop import (
    MAX_PENDING_LOG_TASKS,
    WEDGED_CYCLE_TIMEOUT_S,
    CycleLoopBase,
    LogWriterProtocol,
)
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
from src.models.system_state import RealtimeSnapshot

_logger = logging.getLogger(__name__)

LEARNING_LOOP_INTERVAL_S: float = 0.1  # 100ms 周期（drive_logs の記録間隔に一致）
# これ未満を「停車」とみなす。src.domain.control.conversions の共通定数を使う（A2 レビュー指摘）。
STOP_SPEED_KMH: float = VEHICLE_STOP_SPEED_KMH


class _Phase(Enum):
    """パターン実行の内部サブフェーズ。"""

    MEASURE = auto()  # CREEP / CREEP_SETTLE 用: 固定開度を適用して計測
    DRIVE_ACCEL = auto()  # 前半: アクセルをランプで踏み込み cap（0.98×max_speed）到達まで加速
    COAST = auto()  # 中間: accel=brake=0 で惰行（エンジンブレーキ＋走行抵抗の減速率を計測）
    DRIVE_BRAKE = auto()  # 後半: アクセル解放しブレーキをランプで踏み込み減速・停車
    BRAKE_HOLD = auto()  # 定常ブレーキ: 固定ブレーキ開度をランプ後一定保持して定常減速を記録
    CRUISE_TRIM = auto()  # cap 到達後に微小アクセル開度を保持し高速域×微小開度を採取
    DONE = auto()  # 全パターン完了


@dataclass
class LearningLoopConfig:
    """開ループ実行のタイミング・ランプ。テストで短縮できるよう外出しする。"""

    brake_stop_timeout_s: float = field(default=10.0)  # 減速区間が停車しない場合の打ち切り
    # 定常ブレーキ保持（BRAKE_HOLD）が停車しない場合の打ち切り（低ブレーキ開度は減速が緩いため長め）
    brake_hold_timeout_s: float = field(default=20.0)
    # いきなり目標開度を指令せず、この秒数かけて 0→目標へ踏み込む（servo の時間指定移動で実現）。
    # 実機で適合する想定。アクセル/ブレーキで別々に持てる。
    accel_ramp_time_s: float = field(default=1.5)
    brake_ramp_time_s: float = field(default=1.5)
    # ペダルの解放（outgoing ペダルを 0 へ戻す）の時間。順次ハンドオフを滑らかにする。
    pedal_release_time_s: float = field(default=0.4)
    # ガバナ後の指令開度が変化した時に発行する時間指定移動の所要時間（小刻みステップを滑らかに）
    pedal_step_time_s: float = field(default=0.2)
    # 過速度・過G のスキップは単発の CAN ノイズで誤発火しないよう連続 N サイクル成立を要求する
    skip_consecutive_required: int = field(default=2)
    # 包絡線ガバナ（プロファイルの上限G・最高車速を厳守）:
    # - 平滑加速度（この時間窓のスロープ）で G を測り CAN ノイズ誤発火を防ぐ
    # - 加速/減速G が `max_decel_g × g_limit_frac` 超で開度の踏み増しを止め、超過継続なら下げる
    # - 加速は `max_speed × accel_speed_cap_frac` で打ち切り（max_speed 手前で終了）
    g_smoothing_window_s: float = field(default=0.4)
    g_limit_frac: float = field(default=0.98)
    gov_reduce_step_pct: float = field(default=2.0)
    accel_speed_cap_frac: float = field(default=0.98)
    # 加速区間は cap（max_speed×accel_speed_cap_frac）到達を主離脱条件にする。低開度が cap 未満で
    # 頭打ちした場合や、実車の加速がガバナ上限Gより緩い場合の予算上限として timeout を設ける。
    # 高開度（ガバナ加速 0.98×max_decel_g）で cap 到達に要する時間＋十分なマージンを見込む。
    accel_full_range_timeout_s: float = field(default=20.0)
    # 予測的 cap 離脱の先読み時間 [s]。アクセル解放後も車両応答遅れ（むだ時間θ＋時定数τ相当）の
    # 間は惰性で車速が伸び max_speed を超過する（実機で 70% 踏込・13.8km/h/s のとき +16km/h の
    # オーバーシュートを観測）。現在の平滑加速度 × この時間だけ cap の手前で離脱して超過を防ぐ。
    overspeed_lead_s: float = field(default=1.2)
    # オーバースピード回復（過速度デバウンス成立 → DRIVE_BRAKE）で実際に踏むブレーキ開度の下限。
    # パターン自身の brake_opening（COAST_DOWN は 0.0）に関わらずこの値以上を踏むことで、
    # 「回復のため DRIVE_BRAKE に入ったのにブレーキ0%のまま」を防ぐ（ACCEL_SWEEP_RESET_BRAKE_PCT
    # と同じ「中庸＝過G にならず素早く停車」の考え方）。
    overspeed_recovery_brake_pct: float = field(default=30.0)
    # コースト（惰行）: COAST_DOWN が coast_down_stop_speed_kmh まで惰行（速度全域の減速カーブ）。
    # timeout でも前進する（予算上限）。
    coast_down_stop_speed_kmh: float = field(default=5.0)
    coast_timeout_s: float = field(default=6.0)
    # クリープ安定待ち（accel=brake=0）: 車速が安定する＝|実測加速度| が tol 未満を
    # stable_duration 継続したら確定。最低 min_s は待ってから判定し、timeout で打ち切る。
    # クリープが出ず 0km/h のままでも「安定（|加速度|≈0）」として min_s 後に次へ進む。
    creep_settle_min_s: float = field(default=13.0)
    creep_settle_stable_tol_kmhs: float = field(default=0.3)
    creep_settle_stable_duration_s: float = field(default=2.0)
    creep_settle_timeout_s: float = field(default=25.0)


class ActuatorDriverProtocol(Protocol):
    async def move_to_position(self, pos: int) -> None: ...

    async def move_to_position_timed(
        self, target_pos: int, current_pos: int, duration_s: float
    ) -> None: ...

    async def read_current(self) -> float: ...


class CANReaderProtocol(Protocol):
    async def read_speed(self) -> float: ...


class SafetyCheckProtocol(Protocol):
    def check_overcurrent(self, current_ma: float, axis: str) -> bool: ...


class LearningLoop(CycleLoopBase):
    """開度パターンを開ループ実行し連続ログを記録するドメインコンポーネント。

    サイクル実行のライフサイクル・ウォッチドッグ・ログ書込バックログ管理は
    CycleLoopBase（DriveLoop/ScheduleLoop と共通）に委譲する。
    """

    _cycle_label = "学習サイクル"

    def __init__(
        self,
        accel_driver: ActuatorDriverProtocol,
        brake_driver: ActuatorDriverProtocol,
        can_reader: CANReaderProtocol,
        profile: VehicleProfile,
        patterns: list[LearningPattern],
        safety_check: SafetyCheckProtocol,
        on_complete: Callable[[], Awaitable[None]],
        on_emergency: Callable[[], Awaitable[None]],
        log_writer: LogWriterProtocol | None = None,
        session_id: str | None = None,
        interval_s: float = LEARNING_LOOP_INTERVAL_S,
        config: LearningLoopConfig | None = None,
    ) -> None:
        super().__init__(
            interval_s=interval_s,
            on_complete=on_complete,
            on_emergency=on_emergency,
            log_writer=log_writer,
            session_id=session_id,
        )
        self._accel_driver = accel_driver
        self._brake_driver = brake_driver
        self._can_reader = can_reader
        self._profile = profile
        self._patterns = patterns
        self._safety_check = safety_check
        self._config = config if config is not None else LearningLoopConfig()

        self._max_decel_kmhs = max(profile.max_decel_g * G_TO_KMHS, 0.0)
        # 包絡線ガバナが守る上限G（加速・減速同一）と加速の車速キャップ
        self._g_limit_kmhs = self._max_decel_kmhs * self._config.g_limit_frac
        self._accel_speed_cap = profile.max_speed * self._config.accel_speed_cap_frac

        self._pattern_idx = 0
        self._phase = _Phase.MEASURE
        self._phase_started_at: float | None = None
        self._prev_speed: float | None = None
        self._prev_time: float | None = None
        self._skip_count = 0  # 過速度・過G の連続成立サイクル数（ノイズ デバウンス用）
        self._stable_count = 0  # クリープ安定待ちの「安定」連続サイクル数
        # 包絡線ガバナの開度上限（None=未発動）。加速G/減速G が上限に達したら踏み増しを止める
        self._accel_gov_cap: float | None = None
        self._brake_gov_cap: float | None = None
        # 過速度デバウンス成立で DRIVE_BRAKE に入った（能動的な減速回復）ことを示すフラグ。
        # パターンの brake_opening が 0.0（COAST_DOWN 等）でも回復用の実効ブレーキ開度を使う。
        self._overspeed_recovery = False
        # 平滑加速度算出用の (時刻, 車速) 履歴（CAN ノイズで G を誤判定しないため）
        self._speed_hist: deque[tuple[float, float]] = deque()
        # 直近指令位置（時間指定移動の距離算出に使う。指令開度が変化した時のみ移動を発行）
        self._accel_pos_cmd = 0
        self._brake_pos_cmd = 0

    def _reset_for_start(self) -> None:
        self._pattern_idx = 0
        self._prev_speed = None
        self._prev_time = None
        self._skip_count = 0
        self._stable_count = 0
        self._accel_gov_cap = None
        self._brake_gov_cap = None
        self._overspeed_recovery = False
        self._speed_hist.clear()
        # 直近指令位置を初期化（accel=原点、brake=保持ブレーキ位置）
        calib = self._profile.calibration
        if calib is not None:
            self._accel_pos_cmd = calib.accel_zero_pos
            hold_pct = min(
                self._profile.feedforward_params.stop_brake_opening_pct,
                self._profile.max_brake_opening,
            )
            self._brake_pos_cmd = opening_to_position(
                hold_pct, calib.brake_zero_pos, calib.brake_full_pos
            )
        else:
            self._accel_pos_cmd = 0
            self._brake_pos_cmd = 0
        self._phase = self._initial_phase(0)
        self._phase_started_at = None

    @property
    def current_ref_speed(self) -> float | None:
        """学習運転は基準速度を持たないため常に None。"""
        return None

    # ── 1 サイクル ─────────────────────────────────────────────────
    async def _execute_one_cycle(self) -> None:
        if not self._running:
            return

        loop = asyncio.get_running_loop()
        now = loop.time()

        # 全パターン完了 → 正常終了
        if self._pattern_idx >= len(self._patterns):
            self.stop()
            await self._on_complete()
            return

        pattern = self._patterns[self._pattern_idx]
        if self._phase_started_at is None:
            self._phase_started_at = now

        # 実車速を取得（取得失敗は非常停止）
        try:
            speed = await self._can_reader.read_speed()
        except Exception:
            _logger.exception("CAN 車速取得失敗: 非常停止")
            await self._abort_emergency()
            return

        # 平滑加速度（CAN ノイズで G を誤判定しないよう時間窓のスロープを使う）
        self._speed_hist.append((now, speed))
        accel_kmhs = self._smoothed_accel(now)

        # 包絡線ガバナ（上限G 厳守）の開度上限を更新してから指令開度を決定
        self._update_governor(accel_kmhs)
        accel_opening, brake_opening = self._command_openings(pattern, speed, now)
        accel_opening, brake_opening = enforce_pedal_exclusion(accel_opening, brake_opening)
        self._current_accel_opening = accel_opening
        self._current_brake_opening = brake_opening

        calib = self._profile.calibration
        if calib is None:
            _logger.error("キャリブレーションデータがない: 非常停止")
            await self._abort_emergency()
            return

        # 指令開度に対応する位置（ログ＝滑らかな連続軌跡。指令が変化した時のみ時間指定移動を発行）
        accel_pos = opening_to_position(accel_opening, calib.accel_zero_pos, calib.accel_full_pos)
        brake_pos = opening_to_position(brake_opening, calib.brake_zero_pos, calib.brake_full_pos)

        # CAN await 中に stop が入った場合は位置指令を送らない
        if not self._running:
            return

        try:
            # 指令位置が変化した軸だけ時間指定移動を発行し、それ以外は電流のみ読む
            accel_current, brake_current = await self._drive_or_sense(accel_pos, brake_pos)
        except Exception:
            _logger.exception("アクチュエータ通信失敗: 非常停止")
            await self._abort_emergency()
            return

        self._last_snapshot = RealtimeSnapshot(
            actual_speed_kmh=speed,
            accel_pos=accel_pos,
            brake_pos=brake_pos,
            accel_current_ma=accel_current,
            brake_current_ma=brake_current,
            captured_at=loop.time(),
        )

        # 非常停止型安全: 過電流
        if self._safety_check.check_overcurrent(accel_current, "accel"):
            _logger.warning("アクセル過電流: %.1f mA", accel_current)
            await self._abort_emergency()
            return
        if self._safety_check.check_overcurrent(brake_current, "brake"):
            _logger.warning("ブレーキ過電流: %.1f mA", brake_current)
            await self._abort_emergency()
            return

        # 連続ログ記録（基準速度は持たないため ref_speed_kmh=None）
        self._enqueue_log_write(
            DriveLogData(
                ref_speed_kmh=None,
                actual_speed_kmh=speed,
                accel_opening=accel_opening,
                brake_opening=brake_opening,
                accel_pos=accel_pos,
                brake_pos=brake_pos,
                accel_current=accel_current,
                brake_current=brake_current,
            )
        )

        # 状態機械の前進（スキップ型安全も内包）
        self._advance(pattern, speed, accel_kmhs, now)

        self._prev_speed = speed
        self._prev_time = now

    # ── 状態機械 ───────────────────────────────────────────────────
    def _initial_phase(self, idx: int) -> _Phase:
        if idx >= len(self._patterns):
            return _Phase.DONE
        # ACCEL_SWEEP / BRAKE_HOLD / COAST_DOWN / CRUISE_TRIM は加速から始める。
        # CREEP 系は固定開度の MEASURE。
        if self._patterns[idx].kind in (
            PatternKind.ACCEL_SWEEP,
            PatternKind.BRAKE_HOLD,
            PatternKind.COAST_DOWN,
            PatternKind.CRUISE_TRIM,
        ):
            return _Phase.DRIVE_ACCEL
        return _Phase.MEASURE

    def _command_openings(
        self, pattern: LearningPattern, speed: float, now: float
    ) -> tuple[float, float]:
        """ログ記録用の指令開度 (accel%, brake%) を返す（ソフトランプ＝連続軌跡）。

        実駆動はフェーズ入場時の時間指定移動で servo が行うが、ログには ramp_time_s 秒かけて
        0→目標へ線形に踏み込む滑らかな開度軌跡を記録する（学習が消費する連続サンプル）。
        """
        assert self._phase_started_at is not None
        elapsed = max(0.0, now - self._phase_started_at)
        if self._phase is _Phase.DRIVE_ACCEL:
            ramped = pattern.accel_opening * self._ramp_fraction(
                elapsed, self._config.accel_ramp_time_s
            )
            if self._accel_gov_cap is not None:
                ramped = min(ramped, self._accel_gov_cap)  # 包絡線ガバナ（上限G）の踏み増し停止
            return clamp_opening(ramped, self._profile.max_accel_opening), 0.0
        if self._phase is _Phase.COAST:
            # 惰行: 両ペダル 0（エンジンブレーキ＋走行抵抗の自然減速を計測）
            return 0.0, 0.0
        if self._phase is _Phase.CRUISE_TRIM:
            # cap 到達後の微小開度保持: trim_opening を無ランプで一定保持（ブレーキ 0）。
            trim = clamp_opening(pattern.trim_opening, self._profile.max_accel_opening)
            return trim, 0.0
        if self._phase in (_Phase.DRIVE_BRAKE, _Phase.BRAKE_HOLD):
            # ブレーキをランプで踏み込み目標開度へ。ramp 完了後は一定保持（BRAKE_HOLD の定常区間）。
            # オーバースピード回復時はパターンの brake_opening（COAST_DOWN 等は 0.0）に関わらず
            # 回復用の下限開度以上を踏む（そうしないと回復のはずが無ブレーキで再加速してしまう）。
            target_brake = pattern.brake_opening
            if self._phase is _Phase.DRIVE_BRAKE and self._overspeed_recovery:
                target_brake = max(target_brake, self._config.overspeed_recovery_brake_pct)
            ramped = target_brake * self._ramp_fraction(elapsed, self._config.brake_ramp_time_s)
            if self._brake_gov_cap is not None:
                ramped = min(ramped, self._brake_gov_cap)  # 包絡線ガバナ（上限減速G）
            return 0.0, clamp_opening(ramped, self._profile.max_brake_opening)
        # MEASURE（CREEP の段階リリース・CREEP_SETTLE の 0/0）
        return pattern.accel_opening, pattern.brake_opening

    @staticmethod
    def _ramp_fraction(elapsed: float, ramp_time_s: float) -> float:
        """0→目標へ線形に踏み込むための係数 [0,1]。ramp_time_s<=0 は即時(=1.0)。"""
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
            self._advance_coast(pattern, speed, accel_kmhs, elapsed, now)
            return

        if self._phase is _Phase.DRIVE_BRAKE:
            # 減速G の上限は `_update_governor` がブレーキ開度を抑制して守る（ここは前進判定のみ）。
            if speed <= STOP_SPEED_KMH or elapsed >= self._config.brake_stop_timeout_s:
                self._advance_pattern(now)
            return

        if self._phase is _Phase.BRAKE_HOLD:
            # 固定ブレーキを保持して定常減速を記録。停車近傍 or timeout で次パターンへ。
            # 減速G の上限は `_update_governor` が BRAKE_HOLD 経路でブレーキ開度を抑制して守る。
            if speed <= STOP_SPEED_KMH or elapsed >= self._config.brake_hold_timeout_s:
                self._advance_pattern(now)
            return

        if self._phase is _Phase.CRUISE_TRIM:
            self._advance_cruise_trim(pattern, speed, elapsed, now)
            return

        if self._phase is _Phase.MEASURE and pattern.kind is PatternKind.CREEP_SETTLE:
            self._advance_creep_settle(elapsed, accel_kmhs, now)
            return

        if self._phase is _Phase.MEASURE:
            # CREEP は「ブレーキを少しずつ離していく」連続リリース。RETURN を挟まず
            # hold 経過で次の開度（さらに弱いブレーキ）へ進み pedal-off 連続サンプルを確保する。
            if elapsed >= pattern.hold_duration_s:
                self._advance_pattern(now)
            return

    def _advance_drive_accel(
        self, pattern: LearningPattern, speed: float, accel_kmhs: float, elapsed: float, now: float
    ) -> None:
        """加速区間: 上限G をガバナで守りつつ、cap 到達主導で次フェーズへ離脱する。

        - max_speed 超過（デバウンス）→ DRIVE_BRAKE で能動的に減速して復帰（惰行では戻らないため）。
        - 予測的 cap 離脱: `cap − max(0, 平滑加速度) × overspeed_lead_s` 到達 or timeout → 種別
          ごとの次フェーズへ。アクセルを解放しても応答遅れの間は惰性で車速が伸びるため、加速度に
          応じて cap の手前で離脱し max_speed 超過を防ぐ（B-7-5）。加速がほぼ 0（プラトー）なら
          しきい値は cap と一致し従来と同じ。ACCEL_SWEEP → DRIVE_BRAKE（停車復帰）、BRAKE_HOLD →
          BRAKE_HOLD（定常ブレーキ保持）、COAST_DOWN → COAST（惰行）。低開度が頭打ちなら timeout。
        - 加速方向の上限G は `_update_governor` が踏み増しを止めて守る（ここでは離脱要因にしない）。
          プラトー早期離脱は行わない（cap まで加速を伸ばし全車速域を採取するため）。
        """
        cfg = self._config
        if speed > self._profile.max_speed:
            self._skip_count += 1
        else:
            self._skip_count = 0
        if self._skip_count >= cfg.skip_consecutive_required:
            # overspeed → 能動的に減速復帰（回復用ブレーキ下限を適用）
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return

        exit_speed = self._accel_speed_cap - max(0.0, accel_kmhs) * cfg.overspeed_lead_s
        if speed >= exit_speed or elapsed >= cfg.accel_full_range_timeout_s:
            self._enter_phase(self._phase_after_accel(pattern), now)

    @staticmethod
    def _phase_after_accel(pattern: LearningPattern) -> _Phase:
        """加速区間（cap 到達/timeout）離脱後の次フェーズをパターン種別で決める。"""
        if pattern.kind is PatternKind.ACCEL_SWEEP:
            return _Phase.DRIVE_BRAKE  # リセットブレーキで停車へ戻す
        if pattern.kind is PatternKind.BRAKE_HOLD:
            return _Phase.BRAKE_HOLD  # 固定ブレーキを保持して定常減速を記録
        if pattern.kind is PatternKind.CRUISE_TRIM:
            return _Phase.CRUISE_TRIM  # 微小アクセル開度を保持して高速域応答を採取
        return _Phase.COAST  # COAST_DOWN: 惰行

    def _advance_coast(
        self, pattern: LearningPattern, speed: float, accel_kmhs: float, elapsed: float, now: float
    ) -> None:
        """惰行区間: accel=brake=0 で自然減速（エンジンブレーキ＋走行抵抗）を計測。

        COAST_DOWN のみがこのフェーズに入る。低速まで惰行して次パターンへ（速度全域の減速カーブを
        採るため減速平坦化では早期離脱しない）。両ペダル 0 なので緩める対象が無く、過G は安全。
        万一 max_speed 超なら惰行では戻らないので能動的に減速（通常は加速キャップで未然防止）。
        """
        cfg = self._config
        if speed > self._profile.max_speed:
            # overspeed → 能動的に減速復帰（回復用ブレーキ下限を適用）
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return
        if speed <= cfg.coast_down_stop_speed_kmh or elapsed >= cfg.coast_timeout_s:
            self._advance_pattern(now)

    def _advance_cruise_trim(
        self, pattern: LearningPattern, speed: float, elapsed: float, now: float
    ) -> None:
        """微小アクセル開度を保持し、高速域×微小開度の応答を採取する（B-7-2）。

        trim_opening は小さいため通常は惰性で減速しつつ高速域を掃引する。hold_duration_s 経過
        または低速まで落ちたら次パターンへ。万一 trim が加速側に働いて速度が cap 以上へ戻る場合は
        過速度前に離脱する（アクセルは微小なので次パターンへ進めば解放され能動減速は不要）。
        max_speed 超は安全網として能動減速（DRIVE_BRAKE）へ回す。
        """
        cfg = self._config
        if speed > self._profile.max_speed:
            self._enter_phase(_Phase.DRIVE_BRAKE, now, overspeed_recovery=True)
            return
        if speed >= self._accel_speed_cap:
            self._advance_pattern(now)  # cap 以上へ戻った（trim が加速側）→ 過速度前に離脱
            return
        if elapsed >= pattern.hold_duration_s or speed <= cfg.coast_down_stop_speed_kmh:
            self._advance_pattern(now)

    def _advance_creep_settle(self, elapsed: float, accel_kmhs: float, now: float) -> None:
        """accel=brake=0 で車速が安定するまで待つ。安定確定 or タイムアウトで次パターンへ。

        「安定」= |実測加速度| が tol 未満を stable_duration 継続。クリープが立ち上がる猶予を
        与えるため最低 min_s は待つ。クリープが出ず 0km/h のままでも |加速度|≈0 で安定扱いになり、
        min_s 経過後に次の開度振りへ進む。RETURN を挟まず連続軌跡のまま次へ。
        """
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

    def _advance_pattern(self, now: float) -> None:
        self._pattern_idx += 1
        self._enter_phase(self._initial_phase(self._pattern_idx), now)

    def _enter_phase(self, phase: _Phase, now: float, *, overspeed_recovery: bool = False) -> None:
        self._phase = phase
        self._phase_started_at = now
        self._skip_count = 0
        self._stable_count = 0
        self._accel_gov_cap = None
        self._brake_gov_cap = None
        self._overspeed_recovery = overspeed_recovery

    def _update_governor(self, accel_kmhs: float) -> None:
        """包絡線ガバナ: 加速/減速G が上限を超えたら指令開度の踏み増しを止め、なお超過なら下げる。

        平滑加速度 `accel_kmhs`（>0 加速 / <0 減速）で判定。上限は `max_decel_g × g_limit_frac`。
        最初に超過したサイクルで「現在の指令開度」で頭打ちし、超過が続くなら `gov_reduce_step_pct`
        ずつ下げて G を上限内に収める（単調減少のラチェット）。
        """
        limit = self._g_limit_kmhs
        if limit <= 0.0:
            return
        step = self._config.gov_reduce_step_pct
        if self._phase is _Phase.DRIVE_ACCEL and accel_kmhs >= limit:
            if self._accel_gov_cap is None:
                self._accel_gov_cap = self._current_accel_opening
            else:
                self._accel_gov_cap = max(0.0, self._accel_gov_cap - step)
        elif self._phase in (_Phase.DRIVE_BRAKE, _Phase.BRAKE_HOLD) and -accel_kmhs >= limit:
            if self._brake_gov_cap is None:
                self._brake_gov_cap = self._current_brake_opening
            else:
                self._brake_gov_cap = max(0.0, self._brake_gov_cap - step)

    def _smoothed_accel(self, now: float) -> float:
        """平滑加速度 [km/h/s]。時間窓 `g_smoothing_window_s` のスロープ。

        CAN 車速の単発ノイズで G を誤判定しないために窓平均のスロープを使う。履歴が不足する
        （テスト等で 1 サンプルのみ）場合は直近サンプル差分にフォールバックする。
        """
        hist = self._speed_hist
        win = self._config.g_smoothing_window_s
        while len(hist) >= 2 and now - hist[0][0] > win:
            hist.popleft()
        if len(hist) >= 2:
            t0, v0 = hist[0]
            t1, v1 = hist[-1]
            if t1 - t0 > 0.0:
                return (v1 - v0) / (t1 - t0)
        return self._measured_accel(now)

    def _measured_accel(self, now: float) -> float:
        if self._prev_speed is None or self._prev_time is None:
            return 0.0
        dt = now - self._prev_time
        if dt <= 0.0:
            return 0.0
        # _speed_hist の末尾が現在サンプル
        cur = self._speed_hist[-1][1] if self._speed_hist else self._prev_speed
        return (cur - self._prev_speed) / dt

    # ── 補助 ───────────────────────────────────────────────────────
    def _move_duration(self) -> float:
        """指令位置変化時に発行する時間指定移動の所要時間 [s]。

        加速/減速の踏み込み中は小刻みなので短く（`pedal_step_time_s`）、COAST/MEASURE の解放は
        `pedal_release_time_s` で滑らかに戻す。
        """
        if self._phase in (_Phase.DRIVE_ACCEL, _Phase.DRIVE_BRAKE, _Phase.BRAKE_HOLD):
            return self._config.pedal_step_time_s
        return self._config.pedal_release_time_s

    async def _drive_or_sense(self, accel_pos: int, brake_pos: int) -> tuple[float, float]:
        """指令位置が変化した軸だけ時間指定移動を発行し、変化のない軸は電流のみ読む。

        ガバナ後の指令開度に追従して滑らかに動かす（servo が時間指定で補間）。位置が前サイクルと
        同じ軸は無駄な移動指令を出さず電流だけ読む。
        """
        dur = self._move_duration()
        a_changed = accel_pos != self._accel_pos_cmd
        b_changed = brake_pos != self._brake_pos_cmd

        async def accel_axis() -> float:
            if a_changed:
                return await self._drive_axis_timed(
                    self._accel_driver, accel_pos, self._accel_pos_cmd, dur
                )
            return await self._accel_driver.read_current()

        async def brake_axis() -> float:
            if b_changed:
                return await self._drive_axis_timed(
                    self._brake_driver, brake_pos, self._brake_pos_cmd, dur
                )
            return await self._brake_driver.read_current()

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
    ) -> float:
        await driver.move_to_position_timed(target_pos, current_pos, duration_s)
        return await driver.read_current()


__all__ = [
    "LEARNING_LOOP_INTERVAL_S",
    "MAX_PENDING_LOG_TASKS",
    "STOP_SPEED_KMH",
    "WEDGED_CYCLE_TIMEOUT_S",
    "ActuatorDriverProtocol",
    "CANReaderProtocol",
    "LearningLoop",
    "LearningLoopConfig",
    "LogWriterProtocol",
    "SafetyCheckProtocol",
]
