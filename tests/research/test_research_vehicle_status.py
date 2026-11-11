"""スタブ車両の状態（vehicle_status.py）のテスト（2026-11-12 段3）。

時刻を引数で渡す純粋なクラスなので、時計は進めずに now を直接渡して確かめる。
"""

from __future__ import annotations

import pytest

from tests.research import config as cfgmod
from tests.research import hardware as hwmod
from tests.research.button_servo import CH_D, CH_N, CH_P, CH_START
from tests.research.vehicle_status import State, VehicleStatus


class Car:
    """VehicleStatus を時刻つきで動かす小道具。"""

    def __init__(self) -> None:
        self.vs = VehicleStatus()
        self.vs.activate()
        self.t = 100.0
        self.accel = 0.0
        self.brake = 0.0
        self.speed = 0.0

    def tick(self, dt: float = 0.1) -> None:
        self.t += dt
        self.vs.update(self.t, self.accel, self.brake, self.speed)

    def set(self, *, accel: float | None = None, brake: float | None = None,
            speed: float | None = None) -> None:
        if accel is not None:
            self.accel = accel
        if brake is not None:
            self.brake = brake
        if speed is not None:
            self.speed = speed
        self.tick(0.0)

    def press(self, ch: int, hold_s: float = 0.1) -> None:
        self.tick(0.0)
        self.vs.on_press_start(ch, self.t)
        # 押している間も update が呼ばれる（途中で表 2 の判定が進む）
        n = max(1, round(hold_s / 0.1))
        for _ in range(n):
            self.tick(hold_s / n)
        self.vs.on_press_end(ch, self.t)

    def tap(self, n: int = 1, pct: float = 10.0) -> None:
        for _ in range(n):
            self.set(accel=pct)
            self.tick(0.3)
            self.set(accel=0.0)
            self.tick(0.3)


def _ign_on(car: Car, hold_s: float = 3.0) -> None:
    car.press(CH_START, hold_s)


# ── 無効のとき ───────────────────────────────────────────────────
def test_inactive_status_ignores_everything() -> None:
    vs = VehicleStatus()
    vs.update(1.0, 0.0, 50.0, 0.0)
    vs.on_press_start(CH_START, 1.0)
    vs.on_press_end(CH_START, 4.0)
    assert vs.active is False
    assert vs.state is State.IGN_OFF
    assert vs.range == "P"


def test_activate_starts_from_ign_off_p() -> None:
    car = Car()
    assert (car.vs.state, car.vs.range, car.vs.n_hold, car.vs.maintenance) == (
        State.IGN_OFF, "P", False, False)
    assert car.vs.describe() == "State=IGN-OFF  Range=P  NHold=OFF  Maint=OFF"


# ── 表 1（押した瞬間） ───────────────────────────────────────────
def test_table1_ign_off_brake_off_goes_ign_on_rly_on() -> None:
    car = Car()
    car.press(CH_START, 0.5)
    assert car.vs.state is State.IGN_ON_RLY_ON


def test_table1_ign_off_brake_on_goes_ready() -> None:
    car = Car()
    car.set(brake=5.0)
    car.press(CH_START, 0.5)
    assert car.vs.state is State.READY


def test_table1_ign_on_brake_on_goes_ready_brake_off_stopped_goes_off() -> None:
    car = Car()
    car.press(CH_START, 0.5)  # RLY-ON
    car.set(brake=5.0)
    car.press(CH_START, 0.5)
    assert car.vs.state is State.READY
    car2 = Car()
    car2.press(CH_START, 0.5)
    car2.press(CH_START, 0.5)  # ブレーキ OFF・停車中 → IGN-OFF
    assert car2.vs.state is State.IGN_OFF


def test_table1_ign_on_brake_off_running_no_change() -> None:
    car = Car()
    car.press(CH_START, 0.5)
    car.set(speed=10.0)
    car.press(CH_START, 0.5)
    assert car.vs.state is State.IGN_ON_RLY_ON


def test_table1_ready_stopped_goes_off_running_no_change() -> None:
    car = Car()
    car.set(brake=5.0)
    car.press(CH_START, 0.5)
    assert car.vs.state is State.READY
    car.set(speed=10.0)
    car.press(CH_START, 0.5)
    assert car.vs.state is State.READY
    car.set(speed=0.0)
    car.press(CH_START, 0.5)
    assert car.vs.state is State.IGN_OFF


# ── 表 2（押し続け） ─────────────────────────────────────────────
def test_table2_under_2s_stays_rly_on() -> None:
    car = Car()
    car.press(CH_START, 1.5)
    assert car.vs.state is State.IGN_ON_RLY_ON


def test_table2_2s_to_5s_goes_rly_off_and_records_start_time() -> None:
    car = Car()
    t0 = car.t
    car.press(CH_START, 3.0)
    assert car.vs.state is State.IGN_ON_RLY_OFF
    assert car.vs._rly_off_t == pytest.approx(t0 + 2.0)  # 押した時刻 ＋ 2 s


def test_table2_5s_goes_back_to_ign_off() -> None:
    car = Car()
    car.press(CH_START, 5.2)
    assert car.vs.state is State.IGN_OFF


def test_table2_running_5s_emergency_off_goes_n_and_coasts_then_p() -> None:
    car = Car()
    car.set(brake=5.0)
    car.press(CH_START, 0.5)  # Ready
    car.vs.range = "D"
    car.set(brake=0.0, speed=30.0)
    car.press(CH_START, 3.0)  # 5 s 未満は変化なし
    assert car.vs.state is State.READY
    car.press(CH_START, 5.2)  # 緊急 OFF
    assert car.vs.state is State.IGN_OFF
    assert car.vs.range == "N"  # 走行中は N へ
    car.set(speed=0.0)  # 停車したら P
    assert car.vs.range == "P"


def test_table2_other_transitions_unchanged_by_long_press() -> None:
    car = Car()
    car.set(brake=5.0)
    car.press(CH_START, 6.0)  # → Ready。押し続けても変化なし
    assert car.vs.state is State.READY


# ── レンジの条件 ─────────────────────────────────────────────────
def test_shift_ignored_in_ign_off() -> None:
    car = Car()
    car.set(brake=5.0)
    car.press(CH_N)
    assert car.vs.range == "P"


def test_shift_conditions() -> None:
    car = Car()
    car.press(CH_START, 3.0)  # IGN-ON(RLY-OFF)・ブレーキ OFF
    car.press(CH_N)  # N はブレーキ ON が条件
    assert car.vs.range == "P"
    car.set(brake=5.0)
    car.press(CH_N)
    assert car.vs.range == "N"
    car.press(CH_D)  # D は Ready のみ
    assert car.vs.range == "N"
    car.set(speed=5.0)
    car.press(CH_P)  # P は停車中が条件
    assert car.vs.range == "N"
    car.set(speed=0.0)
    car.press(CH_P)
    assert car.vs.range == "P"


def test_d_needs_ready_and_brake() -> None:
    car = Car()
    car.set(brake=5.0)
    car.press(CH_START, 0.5)  # Ready・P
    car.set(brake=0.0)
    car.press(CH_D)
    assert car.vs.range == "P"
    car.set(brake=5.0)
    car.press(CH_D)
    assert car.vs.range == "D"
    assert car.vs.drive_mode() == "D"


# ── N ホールド ───────────────────────────────────────────────────
def test_n_hold_established_by_two_n_within_5s_from_p() -> None:
    car = Car()
    car.press(CH_START, 3.0)
    car.set(brake=5.0)
    car.press(CH_N)
    assert car.vs.n_hold is False
    car.tick(1.0)
    car.press(CH_N)
    assert car.vs.n_hold is True


def test_n_hold_not_established_after_5s_or_from_non_p() -> None:
    car = Car()
    car.press(CH_START, 3.0)
    car.set(brake=5.0)
    car.press(CH_N)
    car.tick(6.0)
    car.press(CH_N)
    assert car.vs.n_hold is False
    # P を挟んだらやり直し（1 回目から）
    car.set(speed=0.0)
    car.press(CH_P)
    car.press(CH_N)
    car.tick(1.0)
    car.press(CH_N)
    assert car.vs.n_hold is True


def test_n_hold_keeps_n_at_ign_off_and_is_released_at_next_ign_on() -> None:
    car = Car()
    car.press(CH_START, 3.0)
    car.set(brake=5.0)
    car.press(CH_N)
    car.press(CH_N)
    assert car.vs.n_hold
    car.set(brake=5.0)
    car.press(CH_START, 0.5)  # Ready
    assert car.vs.state is State.READY and car.vs.n_hold  # Ready になっても継続
    car.press(CH_START, 0.5)  # IGN-OFF（停車）
    assert car.vs.state is State.IGN_OFF
    assert car.vs.range == "N" and car.vs.n_hold  # N のまま
    car.set(brake=0.0)
    car.press(CH_START, 0.5)  # IGN-OFF → IGN-ON で解除（N は維持）
    assert car.vs.state is State.IGN_ON_RLY_ON
    assert car.vs.range == "N" and car.vs.n_hold is False


def test_n_hold_does_not_apply_to_d_emergency_off() -> None:
    car = Car()
    car.press(CH_START, 3.0)
    car.set(brake=5.0)
    car.press(CH_N)
    car.press(CH_N)
    car.press(CH_START, 0.5)  # Ready・N
    car.press(CH_D)
    car.set(brake=0.0, speed=20.0)
    car.press(CH_START, 5.2)  # D で走行中の緊急 OFF
    assert car.vs.state is State.IGN_OFF and car.vs.range == "N"
    car.set(speed=0.0)
    assert car.vs.range == "P"  # N ホールドが有効でも停車で P


def test_n_hold_keeps_n_after_emergency_off_from_n() -> None:
    car = Car()
    car.press(CH_START, 3.0)
    car.set(brake=5.0)
    car.press(CH_N)
    car.press(CH_N)
    car.press(CH_START, 0.5)  # Ready・N
    car.set(brake=0.0, speed=20.0)
    car.press(CH_START, 5.2)
    car.set(speed=0.0)
    assert car.vs.range == "N"


# ── IGN-OFF のレンジ ─────────────────────────────────────────────
def test_ign_off_stopped_goes_p_without_hold() -> None:
    car = Car()
    car.press(CH_START, 3.0)
    car.set(brake=5.0)
    car.press(CH_N)
    car.press(CH_START, 0.5)  # Ready・N
    car.press(CH_START, 0.5)  # IGN-OFF・停車
    assert car.vs.range == "P"


# ── メンテナンスモード ───────────────────────────────────────────
def _maintenance_sequence(car: Car) -> None:
    car.tap(3)  # P × 3
    car.set(brake=5.0)
    car.press(CH_N)
    car.set(brake=0.0)
    car.tap(3)  # N × 3
    car.press(CH_P)
    car.tap(3)  # P × 3


def test_maintenance_established() -> None:
    car = Car()
    _ign_on(car)
    _maintenance_sequence(car)
    assert car.vs.maintenance is True


def test_maintenance_wrong_order_restarts_within_60s() -> None:
    car = Car()
    _ign_on(car)
    car.tap(2)
    car.set(brake=5.0)
    car.press(CH_N)  # P で 2 回しか踏まずに N へ
    car.set(brake=0.0)
    car.tap(3)
    car.press(CH_P)
    car.tap(3)
    assert car.vs.maintenance is False
    _maintenance_sequence(car)  # 60 s 内ならやり直せる
    assert car.vs.maintenance is True


def test_maintenance_fourth_tap_breaks_order() -> None:
    car = Car()
    _ign_on(car)
    car.tap(4)  # P で 4 回 → 崩れ（最後の 1 回が新しい 1 回目）
    assert car.vs._m_stage == 0 and car.vs._m_count == 1
    car.tap(2)
    car.set(brake=5.0)
    car.press(CH_N)
    car.set(brake=0.0)
    car.tap(3)
    car.press(CH_P)
    car.tap(3)
    assert car.vs.maintenance is True


def test_maintenance_not_established_after_60s() -> None:
    car = Car()
    _ign_on(car)
    car.tick(60.0)
    _maintenance_sequence(car)
    assert car.vs.maintenance is False


def test_maintenance_continues_into_ready_and_clears_at_ign_off() -> None:
    car = Car()
    _ign_on(car)
    _maintenance_sequence(car)
    car.set(brake=5.0)
    car.press(CH_START, 0.5)
    assert car.vs.state is State.READY and car.vs.maintenance is True
    car.press(CH_START, 0.5)
    assert car.vs.state is State.IGN_OFF and car.vs.maintenance is False


def test_taps_outside_rly_off_do_not_count() -> None:
    car = Car()
    car.press(CH_START, 0.5)  # RLY-ON のまま
    car.tap(3)
    assert car.vs._m_count == 0


# ── スタブ車両との接続 ───────────────────────────────────────────
def _stub_hw() -> hwmod.ResearchHardware:
    cfg = cfgmod.load_config(cfgmod.DEFAULT_CONFIG_PATH)
    return hwmod.build_hardware(cfg, hwmod.HW_STUB)


def test_stub_vehicle_unchanged_when_status_inactive() -> None:
    """active でなければ、車速の計算は status なしと 1 行も変わらない。"""
    hw = _stub_hw()
    plain = hwmod.StubVehicle(accel=hw.accel, brake=hw.brake, params=hw.can.vehicle.params)  # type: ignore[union-attr]
    veh = hw.can.vehicle  # type: ignore[union-attr]
    veh.accel.position = 1000
    plain.accel = veh.accel
    for now in (1.0, 1.4, 1.9):
        assert veh.advance(5.0, now) == pytest.approx(plain.advance(5.0, now))


def test_stub_vehicle_modes_p_n_d() -> None:
    hw = _stub_hw()
    veh = hw.can.vehicle  # type: ignore[union-attr]
    vs = hw.vehicle_status
    assert vs is not None
    vs.activate()
    assert veh.advance(10.0, 1.0) == 0.0  # P / IGN-OFF は 0 km/h
    vs.state, vs.range = State.READY, "N"
    veh.last_t = 1.0
    v1 = veh.advance(10.0, 2.0)  # N は惰行減速の半分で減る
    assert 0.0 < v1 < 10.0
    veh.accel.position = 5000  # アクセルは無効
    veh.last_t = 2.0
    assert veh.advance(10.0, 3.0) == pytest.approx(v1)
    veh.brake.position = 5000  # ブレーキを加えると速く減る
    veh.last_t = 3.0
    assert veh.advance(10.0, 4.0) < v1
    vs.range = "D"
    veh.brake.position = 0
    veh.last_t = 4.0
    assert veh.advance(10.0, 5.0) > v1  # D は従来のモデル（アクセル 5000pulse で加速）
