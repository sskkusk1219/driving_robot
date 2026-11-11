"""スタブ車両の状態（スタートスイッチ・シフトレンジ・N ホールド・メンテナンスモード）。

車両シミュレータの状態アイデア（ProblemReport_20261112 の議論で受け取った資料）の状態遷移を、
時刻を引数で受け取る純粋なクラスとして実装する。時計・スレッドを持たないので、
時刻を渡してテストできる。

使い方（スタブ HW）:
    status.update(now, accel_pct, brake_pct, speed_kmh)   # 車速を読むたびに（StubVehicle.advance）
    status.on_press_start(ch, now) / on_press_end(ch, now) # ボタンを押す前後（StubButtonServo）
`active` が False の間は何もしない（手順 2・3 だけの実行は今までどおり）。ボタンを使う手順が
最初に activate() を呼ぶ（hardware.ensure_buttons_connected）。

しきい値は資料の値（スタブ専用なので定数）。ペダルの値は「ペダルに触れた位置（min）を 0 とした
開度」で受け取る（シミュレータが読むゲームペダルの値に相当）。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from tests.research.button_servo import CH_D, CH_N, CH_P, CH_START

STOP_KMH = 0.2  # 停車中 ≤ この値、走行中 > この値 [km/h]
BRAKE_ON_PCT = 0.1  # ブレーキ ON ≥ この値 [%]
ACCEL_TAP_PCT = 1.0  # アクセル 1 回踏み ＝ これ以上 → これ未満 [%]
RLY_OFF_HOLD_S = 2.0  # スタートを押し続けて IGN-ON(RLY-OFF) になる時間 [s]
EMERGENCY_OFF_HOLD_S = 5.0  # IGN-OFF になる時間（緊急 OFF・RLY-OFF からの戻り）[s]
N_HOLD_WINDOW_S = 5.0  # N ホールド: 1 回目の N から 2 回目の N まで [s]
MAINTENANCE_WINDOW_S = 60.0  # メンテナンスモード: RLY-OFF になってから完了まで [s]
MAINTENANCE_TAPS = 3  # 各レンジでのアクセル 1 回踏みの回数（資料の値）
# メンテナンスモードの段（P×3 → N×3 → P×3）
_MAINTENANCE_RANGES = ("P", "N", "P")


class State(Enum):
    IGN_OFF = "IGN-OFF"
    IGN_ON_RLY_ON = "IGN-ON(RLY-ON)"
    IGN_ON_RLY_OFF = "IGN-ON(RLY-OFF)"
    READY = "Ready"

    @property
    def is_ign_on(self) -> bool:
        return self in (State.IGN_ON_RLY_ON, State.IGN_ON_RLY_OFF)


@dataclass
class _Hold:
    """スタートスイッチを押している最中の記録（表 2 の判定用）。"""

    start: float
    # 押した瞬間の遷移の種類。"rly"＝IGN-OFF→IGN-ON(RLY-ON)（2 s で RLY-OFF・5 s で IGN-OFF）、
    # "running"＝走行中で変化なし（5 s で緊急 OFF）、None＝それ以外（変化なし）
    kind: str | None
    rly_done: bool = False
    off_done: bool = False


class VehicleStatus:
    """スタブ車両の状態。"""

    def __init__(self) -> None:
        self.active = False
        self._reset()

    def _reset(self) -> None:
        self.state = State.IGN_OFF
        self.range = "P"
        self.n_hold = False
        self.maintenance = False
        # 直近の入力（ボタンを押した瞬間の判定に使う）
        self.accel_pct = 0.0
        self.brake_pct = 0.0
        self.speed_kmh = 0.0
        self._hold: _Hold | None = None
        self._coast_to_p = False  # IGN-OFF で走っていたとき、停車したら P へ戻す
        self._n_first_t: float | None = None  # N ホールド: P から入れた 1 回目の N の時刻
        self._rly_off_t: float | None = None  # メンテナンスモードの 60 s の起点
        self._m_stage = 0
        self._m_count = 0
        self._accel_high = False

    # ── 公開 ────────────────────────────────────────────────────
    def activate(self) -> None:
        """有効にして IGN-OFF・P・車速 0 から始める。"""
        self._reset()
        self.active = True

    @property
    def stopped(self) -> bool:
        return self.speed_kmh <= STOP_KMH

    @property
    def brake_on(self) -> bool:
        return self.brake_pct >= BRAKE_ON_PCT

    def drive_mode(self) -> str:
        """車速の動かし方。"D"＝速度予測モデル（Ready・D）、"N"＝惰行減速、"P"＝0 km/h。"""
        if self.state is State.READY and self.range == "D":
            return "D"
        return "N" if self.range == "N" else "P"

    def describe(self) -> str:
        """端末表示用（資料の表示例と同じ並び）。"""
        return (
            f"State={self.state.value}  Range={self.range}  "
            f"NHold={'ON' if self.n_hold else 'OFF'}  Maint={'ON' if self.maintenance else 'OFF'}"
        )

    def update(self, now: float, accel_pct: float, brake_pct: float, speed_kmh: float) -> None:
        """ペダルと車速を受け取り、時間経過・ペダル・車速による遷移を進める。"""
        if not self.active:
            return
        self.accel_pct, self.brake_pct, self.speed_kmh = accel_pct, brake_pct, speed_kmh
        self._apply_hold(now)
        # 緊急 OFF の後（走っていた）に停車したら P へ
        if self._coast_to_p and self.state is State.IGN_OFF and self.stopped:
            self.range = "P"
            self._coast_to_p = False
        # メンテナンスモードの 60 s を過ぎたら進行を捨てる
        if (
            self._rly_off_t is not None
            and not self.maintenance
            and now - self._rly_off_t > MAINTENANCE_WINDOW_S
        ):
            self._m_stage = self._m_count = 0
        self._count_accel_tap(now)

    def on_press_start(self, channel: int, now: float) -> None:
        if not self.active:
            return
        if channel == CH_START:
            self._start_pressed(now)
        elif channel in (CH_P, CH_N, CH_D):
            self._shift_pressed({CH_P: "P", CH_N: "N", CH_D: "D"}[channel], now)

    def on_press_end(self, channel: int, now: float) -> None:
        if not self.active:
            return
        if channel == CH_START and self._hold is not None:
            self._apply_hold(now)
            self._hold = None

    # ── スタートスイッチ ────────────────────────────────────────────
    def _start_pressed(self, now: float) -> None:
        """押した瞬間（表 1）。押し続けた分（表 2）は _apply_hold が進める。"""
        s = self.state
        hold_kind: str | None = None
        if s is State.IGN_OFF:
            if not self.brake_on:
                self._enter_ign_on(now)
                hold_kind = "rly"
            elif self.range in ("P", "N"):
                self._to_ready()
        elif s.is_ign_on:
            if self.brake_on and self.range in ("P", "N"):
                self._to_ready()
            elif not self.brake_on:
                if self.stopped:
                    self._to_ign_off()
                else:
                    hold_kind = "running"
        elif s is State.READY:
            if self.stopped:
                self._to_ign_off()
            else:
                hold_kind = "running"
        self._hold = _Hold(start=now, kind=hold_kind)

    def _apply_hold(self, now: float) -> None:
        """押し続けた時間による遷移（表 2）。2 s 未満 / 2 s 以上 5 s 未満 / 5 s 以上。"""
        h = self._hold
        if h is None:
            return
        elapsed = now - h.start
        if h.kind == "rly":
            if not h.rly_done and elapsed >= RLY_OFF_HOLD_S and self.state is State.IGN_ON_RLY_ON:
                h.rly_done = True
                self.state = State.IGN_ON_RLY_OFF
                # 60 s の起点は「押した時刻 ＋ 2 s」（押し続けて RLY-OFF になった時刻）
                self._rly_off_t = h.start + RLY_OFF_HOLD_S
                self._m_stage = self._m_count = 0
            if not h.off_done and elapsed >= EMERGENCY_OFF_HOLD_S and self.state.is_ign_on:
                h.off_done = True
                self._to_ign_off()
        elif h.kind == "running":
            off_now = elapsed >= EMERGENCY_OFF_HOLD_S and self.state is not State.IGN_OFF
            if not h.off_done and off_now:
                h.off_done = True
                self._to_ign_off()

    def _enter_ign_on(self, now: float) -> None:
        self.state = State.IGN_ON_RLY_ON
        self.n_hold = False  # IGN-OFF → IGN-ON を終えた時点で解除（この遷移では N を維持する）
        self._n_first_t = None

    def _to_ready(self) -> None:
        if self.state is State.IGN_OFF:
            self.n_hold = False  # IGN-OFF → Ready を終えた時点で解除（N は維持）
            self._n_first_t = None
        self.state = State.READY

    def _to_ign_off(self) -> None:
        """IGN-OFF へ。このときのレンジ・N ホールド・メンテナンスモードの扱い。"""
        prev = self.range
        self.state = State.IGN_OFF
        self.maintenance = False
        self._rly_off_t = None
        self._m_stage = self._m_count = 0
        self._n_first_t = None
        keep_n = prev == "N" and self.n_hold
        if self.stopped:
            self.range = "N" if keep_n else "P"
            self._coast_to_p = False
        else:
            # 走行中: N へ自動遷移して惰行。停車したら P（N ホールドで直前が N のときは N のまま）。
            # D で走行中の緊急 OFF は N ホールドが有効でも停車したら P
            self.range = "N"
            self._coast_to_p = not keep_n

    # ── シフトレンジ ───────────────────────────────────────────────
    def _shift_pressed(self, target: str, now: float) -> None:
        s = self.state
        if s is State.IGN_OFF:
            return  # IGN-OFF ではシフトキーを無視する
        if target == "P":
            ok = self.stopped
        elif target == "N":
            ok = self.brake_on
        else:  # D
            ok = s is State.READY and self.brake_on
        if target == "N" and self.range == "N" and self._n_first_t is not None:
            # N ホールド: 1 回目の N から 5 s 以内の 2 回目（1 回目だけがシフト条件を要る）
            if s.is_ign_on and now - self._n_first_t <= N_HOLD_WINDOW_S:
                self.n_hold = True
            self._n_first_t = None
            return
        if not ok:
            return
        prev = self.range
        self.range = target
        self._n_first_t = now if (target == "N" and prev == "P" and s.is_ign_on) else None

    # ── アクセル 1 回踏みとメンテナンスモード ─────────────────────────
    def _count_accel_tap(self, now: float) -> None:
        if self.accel_pct >= ACCEL_TAP_PCT:
            self._accel_high = True
            return
        if not self._accel_high:
            return
        self._accel_high = False  # ここで「1% 以上 → 未満」＝ 1 回踏み
        if (
            self.state is not State.IGN_ON_RLY_OFF
            or self.maintenance
            or self._rly_off_t is None
            or now - self._rly_off_t > MAINTENANCE_WINDOW_S
        ):
            return
        want = _MAINTENANCE_RANGES[self._m_stage]
        if self.range == want and self._m_count < MAINTENANCE_TAPS:
            self._m_count += 1
        elif (
            self._m_count == MAINTENANCE_TAPS
            and self._m_stage + 1 < len(_MAINTENANCE_RANGES)
            and self.range == _MAINTENANCE_RANGES[self._m_stage + 1]
        ):
            self._m_stage += 1  # 次の段の 1 回目
            self._m_count = 1
        else:
            # 順序が崩れた（段違いのレンジ・4 回目）: 最初からやり直す。このペダルが P なら 1 回目
            self._m_stage = 0
            self._m_count = 1 if self.range == _MAINTENANCE_RANGES[0] else 0
        if self._m_stage == len(_MAINTENANCE_RANGES) - 1 and self._m_count == MAINTENANCE_TAPS:
            self.maintenance = True
