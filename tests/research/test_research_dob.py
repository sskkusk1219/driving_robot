"""C11 の外乱オブザーバ（dob.PedalDisturbanceObserver）と C11 の接続のテスト。2026-10-18 段2q。"""

from __future__ import annotations

import pytest

from tests.research.dob import PEDAL_ACCEL, PEDAL_BRAKE, PedalDisturbanceObserver
from tests.research.ff_candidate import (
    DeviationForwardFeedforward,
    DisturbanceObserverFeedforward,
    make_candidate,
)

DT = 0.1


class _Table:
    """a = −1 + 0.3u（u≥0）/ 0.6u（u<0）。速度に依らない合成の表。"""

    def accel(self, v: float, u: float, *, launch: bool = False, stop: bool = False) -> float:
        return -1.0 + (0.3 * u if u >= 0 else 0.6 * u)


def _obs(tq_a: float | None = 1.0, tq_b: float | None = 1.0,
         la: float | None = 0.0, lb: float | None = 0.0) -> PedalDisturbanceObserver:
    return PedalDisturbanceObserver(
        _Table(), tq_accel_s=tq_a, tq_brake_s=tq_b, lag_accel_s=la, lag_brake_s=lb
    )


def _run(obs: PedalDisturbanceObserver, n: int, *, u: float, extra: float = 0.0,
         t0: float = 0.0, v0: float = 50.0, **kw: object) -> tuple[float, float]:
    """表どおり＋extra [km/h/s] の加速度で n 周期進める。(次の時刻, 次の車速) を返す。"""
    t, v = t0, v0
    args = {"governed": False, "at_limit_now": False, "launch": False, "stop": False}
    args.update(kw)
    for _ in range(n):
        obs.update(t=t, speed_kmh=v, u_now=u, **args)  # type: ignore[arg-type]
        t += DT
        v += (_Table().accel(v, u) + extra) * DT
    return t, v


def test_constant_error_converges_with_time_constant_tq() -> None:
    obs = _obs(tq_a=1.0)
    # 10 周期（1 s = Tq）で 1 − e^−1 ≒ 0.63 に近づく（離散一次遅れ: 1 − 0.9^10 ≒ 0.65）
    _run(obs, 11, u=10.0, extra=-0.4)
    assert obs.d_accel == pytest.approx(-0.4 * (1 - 0.9**10), abs=0.02)
    _run(obs, 150, u=10.0, extra=-0.4, t0=1.1)
    assert obs.d_accel == pytest.approx(-0.4, abs=0.01)
    assert obs.d_brake == 0.0 and obs.n_brake == 0 and obs.n_accel > 100


def test_per_pedal_separation_and_return_value() -> None:
    obs = _obs()
    t, v = _run(obs, 80, u=10.0, extra=-0.3)
    d_accel = obs.d_accel
    assert d_accel == pytest.approx(-0.3, abs=0.01)
    # ブレーキ側: 表より 0.2 浅い（+0.2）。アクセルの推定は動かない
    _run(obs, 80, u=-10.0, extra=0.2, t0=t, v0=v)
    assert obs.d_brake == pytest.approx(0.2, abs=0.01)
    assert obs.d_accel == d_accel
    pedal = obs.update(t=100.0, speed_kmh=50.0, u_now=-5.0, governed=False,
                       at_limit_now=False, launch=False, stop=False)
    assert pedal in (PEDAL_BRAKE, None)


def test_holds_on_coast_governor_launch_stop_and_limit() -> None:
    obs = _obs()
    t, v = _run(obs, 40, u=10.0, extra=-0.3)
    base = obs.d_accel
    n = obs.n_accel
    # 惰行（u=0）では車速が変わっても更新しない
    t, v = _run(obs, 20, u=0.0, extra=-1.0, t0=t, v0=v)
    assert (obs.d_accel, obs.n_accel) == (base, n)
    for kw in ({"governed": True}, {"launch": True}, {"stop": True}, {"at_limit_now": True}):
        t, v = _run(obs, 20, u=10.0, extra=1.0, t0=t, v0=v, **kw)
        assert (obs.d_accel, obs.n_accel) == (base, n), kw


def test_holds_when_tq_is_none_for_that_pedal() -> None:
    obs = _obs(tq_a=None)
    _run(obs, 30, u=10.0, extra=-0.3)
    assert obs.d_accel == 0.0 and obs.n_accel == 0
    _run(obs, 30, u=-10.0, extra=0.3)
    assert obs.n_brake > 0


def test_uses_input_delayed_by_lag() -> None:
    # アクセルの遅れ 0.5 s: u を踏んでから 0.5 s 後の加速度で更新が始まる
    obs = _obs(la=0.5, lb=0.5)
    t = 0.0
    updates = []
    for k in range(20):
        u = 10.0 if k >= 5 else 0.0  # 0.5 s で踏み始める
        r = obs.update(t=t, speed_kmh=50.0 + 0.0 * k, u_now=u, governed=False,
                       at_limit_now=False, launch=False, stop=False)
        updates.append(r)
        t += DT
    first = updates.index(PEDAL_ACCEL)
    assert first == 10  # 踏み始め k=5 の 0.5 s 後（5 周期）
    # 踏み終わって（u=0）も、遅れの分だけ更新が続く
    last_press = 5 + 15
    obs2 = _obs(la=0.5, lb=0.5)
    t = 0.0
    seq = []
    for k in range(30):
        u = 10.0 if 5 <= k < last_press else 0.0
        seq.append(obs2.update(t=t, speed_kmh=50.0, u_now=u, governed=False,
                               at_limit_now=False, launch=False, stop=False))
        t += DT
    assert seq.index(PEDAL_ACCEL) == 10 and max(
        i for i, r in enumerate(seq) if r == PEDAL_ACCEL
    ) == last_press + 4


def test_delayed_governor_and_limit_hold_updates() -> None:
    # ガバナー・最大開度は「遅れた u を出した周期」の状態でも止める
    obs = _obs(la=0.3, lb=0.3)
    t = 0.0
    out = []
    for k in range(15):
        out.append(obs.update(t=t, speed_kmh=50.0, u_now=-5.0, governed=(k == 3),
                              at_limit_now=False, launch=False, stop=False))
        t += DT
    assert out[6] is None  # k=3 のガバナー作動の u が効く周期（3 周期遅れ）
    assert out[5] is PEDAL_BRAKE and out[7] is PEDAL_BRAKE


# ─── C11 の候補クラス ────────────────────────────────────────────────────


def test_make_candidate_c11_and_set_dob() -> None:
    ff = make_candidate("C11")
    assert isinstance(ff, DisturbanceObserverFeedforward)
    assert isinstance(ff, DeviationForwardFeedforward)  # C10 の上乗せをそのまま使う
    assert ff.candidate == "C11"
    assert ff.tq_accel_s is None and ff.tq_brake_s is None
    ff.set_dob(tq_coef=40.0, lag_accel_s=0.25, lag_brake_s=None)
    assert ff.tq_accel_s == pytest.approx(10.0) and ff.tq_brake_s is None
    assert ff.dob_tq_coef == 40.0 and ff.dob_lag_accel_s == 0.25


class _ReleaseTable(_Table):
    """戻しの表つき: 戻し量 r だけ減速が弱まる（a ＝ 本体 ＋ 0.1r）。"""

    def __init__(self) -> None:
        self.calls: list[float | None] = []

    def accel(self, v: float, u: float, *, launch: bool = False, stop: bool = False,
              release: float | None = None) -> float:
        self.calls.append(release)
        return super().accel(v, u) + 0.1 * (release or 0.0)


def test_brake_prediction_uses_release_amount_like_the_plan() -> None:
    """段2q ①' の修正: ブレーキを戻している間は、戻し量（遅れた u − 直近の窓の最小）で
    戻しの表を引く。窓 0（既定）なら release を渡さない（今までと同じ）。"""
    tb = _ReleaseTable()
    obs = PedalDisturbanceObserver(
        tb, tq_accel_s=1.0, tq_brake_s=1.0, lag_accel_s=0.0, lag_brake_s=0.0,
        release_window_s=1.0,
    )
    args = {"governed": False, "at_limit_now": False, "launch": False, "stop": False}
    t, v = 0.0, 50.0
    for _ in range(5):  # −10 で踏んでから
        obs.update(t=t, speed_kmh=v, u_now=-10.0, **args)  # type: ignore[arg-type]
        t += DT
    tb.calls.clear()
    obs.update(t=t, speed_kmh=v, u_now=-6.0, **args)  # type: ignore[arg-type]
    assert tb.calls[-1] == pytest.approx(4.0)  # −6 − (−10)
    plain = _obs()
    _run(plain, 5, u=-10.0)  # release の引数を持たない表でも動く（窓 0）
    assert plain.n_brake > 0


class _AccelReleaseTable(_Table):
    """アクセルの戻しの表つき: アクセルの戻し量 r だけ加速が弱まる（a ＝ 本体 − 0.1r）。"""

    def __init__(self) -> None:
        self.calls: list[tuple[float | None, float | None]] = []

    def accel(self, v: float, u: float, *, launch: bool = False, stop: bool = False,
              release: float | None = None, accel_release: float | None = None) -> float:
        self.calls.append((release, accel_release))
        return super().accel(v, u) - 0.1 * (accel_release or 0.0)


def test_accel_prediction_uses_accel_release_amount() -> None:
    """2026-10-20 段4-3: アクセルを戻している間は、アクセルの戻し量（窓の u の最大 − 遅れた u）
    で表を引く。踏み増し・保持・窓 0（既定）では accel_release を渡さない（今までと同じ）。"""
    tb = _AccelReleaseTable()
    obs = PedalDisturbanceObserver(
        tb, tq_accel_s=1.0, tq_brake_s=1.0, lag_accel_s=0.0, lag_brake_s=0.0,
        release_window_s=1.0,
    )
    args = {"governed": False, "at_limit_now": False, "launch": False, "stop": False}
    t, v = 0.0, 50.0
    for _ in range(5):  # +10 で踏んでから
        obs.update(t=t, speed_kmh=v, u_now=10.0, **args)  # type: ignore[arg-type]
        t += DT
    assert all(c[1] is None for c in tb.calls)  # 保持の間は渡さない
    tb.calls.clear()
    obs.update(t=t, speed_kmh=v, u_now=6.0, **args)  # type: ignore[arg-type]
    assert tb.calls[-1][1] == pytest.approx(4.0)  # 10 − 6
    assert tb.calls[-1][0] is None  # ブレーキの戻し量は渡さない
    assert obs.accel_release_at(t, 6.0) == pytest.approx(4.0)
    assert obs.accel_release_at(t, -3.0) == 0.0  # ブレーキの u には 0
    # 窓 0（既定）は渡さない
    tb2 = _AccelReleaseTable()
    off = PedalDisturbanceObserver(
        tb2, tq_accel_s=1.0, tq_brake_s=1.0, lag_accel_s=0.0, lag_brake_s=0.0
    )
    for u in (10.0, 10.0, 6.0):
        off.update(t=t, speed_kmh=v, u_now=u, **args)  # type: ignore[arg-type]
        t += DT
    assert all(c[1] is None for c in tb2.calls)


class _AccelPushTable(_Table):
    """踏み増しの表つき: 踏み増し量 p だけ加速が強まる（a ＝ 本体 ＋ 0.1p）。"""

    accel_push_grid = (1.0,)

    def __init__(self) -> None:
        self.calls: list[tuple[float | None, float | None]] = []

    def accel(self, v: float, u: float, *, launch: bool = False, stop: bool = False,
              release: float | None = None, accel_release: float | None = None,
              accel_push: float | None = None) -> float:
        self.calls.append((accel_release, accel_push))
        return super().accel(v, u) + 0.1 * (accel_push or 0.0)


def test_accel_prediction_uses_accel_push_amount() -> None:
    """2026-10-23 段5-7b: アクセルを踏み増した直後は、踏み増し量（遅れた u − 窓の u の最小）を
    表に渡す。踏み増しの表が無い表（_AccelReleaseTable）には渡さない。"""
    tb = _AccelPushTable()
    obs = PedalDisturbanceObserver(
        tb, tq_accel_s=1.0, tq_brake_s=1.0, lag_accel_s=0.0, lag_brake_s=0.0,
        release_window_s=1.0,
    )
    args = {"governed": False, "at_limit_now": False, "launch": False, "stop": False}
    t, v = 0.0, 50.0
    for _ in range(5):
        obs.update(t=t, speed_kmh=v, u_now=6.0, **args)  # type: ignore[arg-type]
        t += DT
    tb.calls.clear()
    obs.update(t=t, speed_kmh=v, u_now=10.0, **args)  # type: ignore[arg-type]
    assert tb.calls[-1][1] == pytest.approx(4.0)  # 10 − 6
    assert obs.accel_push_at(t, 10.0) == pytest.approx(4.0)
    assert obs.accel_push_at(t, -3.0) == 0.0
    plain = _AccelReleaseTable()  # 踏み増しの表を持たない表には accel_push を渡さない
    off = PedalDisturbanceObserver(
        plain, tq_accel_s=1.0, tq_brake_s=1.0, lag_accel_s=0.0, lag_brake_s=0.0,
        release_window_s=1.0,
    )
    for u in (6.0, 6.0, 6.0, 10.0):
        off.update(t=t, speed_kmh=v, u_now=u, **args)  # type: ignore[arg-type]
