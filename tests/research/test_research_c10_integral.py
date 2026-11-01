"""C10 の偏差の積分 I（c10_integral.C10Integral と fwd_plan.deviation_offset_detail の
integral_kmhs）のテスト。2026-11-01 段6d（ProblemReport_20261031_効き始めの近くのブレーキ）。"""

from __future__ import annotations

import pytest

from tests.research import config as cfgmod
from tests.research.c10_integral import DT_MAX_S, C10Integral
from tests.research.fwd_plan import deviation_offset_detail
from tests.research.test_research_fwd_plan import AD, BD, FLAG_MEASURED, linear_table

TAU = 5.0
DT = 0.1


def _ci(
    brake: bool = True, accel: bool = True, coef: float = 4.0, low_speed: str = "off"
) -> C10Integral:
    return C10Integral(brake_on=brake, accel_on=accel, ti_coef=coef, low_speed=low_speed)


def _run(ci: C10Integral, is_accel: bool, dev: float, n: int) -> None:
    ci.activate(is_accel)
    for _ in range(n):
        ci.add(is_accel, dev, DT)


# ── 設定 ──


def test_config_defaults_off() -> None:
    fm = cfgmod.ForwardMapSection()
    assert fm.c10_integral_brake is False
    assert fm.c10_integral_accel is False
    assert fm.c10_integral_ti_coef == 4.0
    assert fm.c10_integral_low_speed == "off"  # 2026-11-01 低速へIを広げる 段1


# ── 積分器 ──


@pytest.mark.parametrize("is_accel", [False, True])
def test_accumulates_proportional_to_time(is_accel: bool) -> None:
    ci = _ci()
    _run(ci, is_accel, 0.5, 10)
    assert ci.value(is_accel) == pytest.approx(0.5 * 1.0)
    _run(ci, is_accel, 0.5, 10)
    assert ci.value(is_accel) == pytest.approx(0.5 * 2.0)
    # a_I ＝ I ÷ (τ × Ti)、Ti ＝ 4 × τ
    assert ci.accel_kmhs(is_accel, TAU) == pytest.approx(1.0 / (TAU * 4.0 * TAU))


@pytest.mark.parametrize("is_accel", [False, True])
def test_value_stays_when_deviation_returns_to_zero(is_accel: bool) -> None:
    ci = _ci()
    _run(ci, is_accel, 0.4, 20)
    before = ci.accel_kmhs(is_accel, TAU)
    _run(ci, is_accel, 0.0, 30)
    assert ci.accel_kmhs(is_accel, TAU) == before > 0.0


def test_off_side_has_no_integral() -> None:
    ci = _ci(brake=False, accel=True)
    _run(ci, False, 0.5, 10)
    assert ci.value(False) == 0.0
    assert ci.accel_kmhs(False, TAU) == 0.0
    _run(ci, True, 0.5, 10)
    assert ci.value(True) > 0.0
    assert not _ci(brake=False, accel=False).enabled


def test_tau_missing_or_zero_gives_zero() -> None:
    ci = _ci()
    _run(ci, False, 0.5, 10)
    assert ci.accel_kmhs(False, None) == 0.0
    assert ci.accel_kmhs(False, 0.0) == 0.0


def test_changing_side_resets_the_other() -> None:
    ci = _ci()
    _run(ci, False, 0.5, 10)
    assert ci.i_brake > 0.0
    ci.activate(True)
    assert ci.i_brake == 0.0
    _run(ci, True, -0.5, 10)
    assert ci.i_accel < 0.0
    ci.activate(False)
    assert ci.i_accel == 0.0


def test_reset_clears_everything() -> None:
    ci = _ci()
    _run(ci, False, 0.5, 10)
    ci.note_cut(shallow=True, deep=True)
    ci.reset()
    assert ci.i_brake == ci.i_accel == 0.0
    assert not ci.shallow_cut and not ci.deep_cut


def test_brake_windup_blocks_only_cut_direction() -> None:
    ci = _ci()
    _run(ci, False, 0.5, 10)  # ブレーキは dev ＞ 0 が浅い向き
    v = ci.i_brake
    ci.note_cut(shallow=True, deep=False)
    assert ci.add(False, 0.5, DT) is False and ci.i_brake == v  # 浅い向きは貯めない
    assert ci.add(False, -0.5, DT) is True and ci.i_brake < v  # 逆向きは貯める
    ci.note_cut(shallow=False, deep=True)
    assert ci.add(False, -0.5, DT) is False  # 深い向きは貯めない
    assert ci.add(False, 0.5, DT) is True


def test_accel_windup_blocks_only_cut_direction() -> None:
    ci = _ci()
    _run(ci, True, 0.5, 10)  # アクセルは dev ＜ 0 が浅い向き
    ci.note_cut(shallow=True, deep=False)
    assert ci.add(True, -0.5, DT) is False
    assert ci.add(True, 0.5, DT) is True
    ci.note_cut(shallow=False, deep=True)
    assert ci.add(True, 0.5, DT) is False
    assert ci.add(True, -0.5, DT) is True


def test_dt_is_capped() -> None:
    ci = _ci()
    ci.activate(False)
    ci.add(False, 1.0, 10.0)
    assert ci.i_brake == pytest.approx(DT_MAX_S)
    assert ci.add(False, 1.0, 0.0) is False


# ── 7.5 km/h 未満の行（2026-11-01 ProblemReport_20261101_低速へIを広げる 段1） ──


def _lrow(ci: C10Integral, *, is_accel: bool = False, dev: float = 0.5, **kw: object) -> float:
    return ci.row(
        is_accel=is_accel, eligible=True, dev_kmh=dev, dt_s=DT, tau_s=TAU, **kw  # type: ignore[arg-type]
    )


@pytest.mark.parametrize("is_accel", [False, True])
def test_row_off_low_row_resets_and_matches_old_behavior(is_accel: bool) -> None:
    ci = _ci()
    for _ in range(10):
        assert _lrow(ci, is_accel=is_accel) > 0.0 or is_accel  # 貯まる
    assert ci.value(is_accel) != 0.0
    assert _lrow(ci, is_accel=is_accel, low=True) == 0.0  # 境目の下は今と同じく 0 へ
    assert ci.i_brake == ci.i_accel == 0.0
    # deep_only は off では何も変えない
    _lrow(ci, is_accel=is_accel)
    assert _lrow(ci, is_accel=is_accel, low=True, deep_only=True) == 0.0


def test_row_default_args_equal_old_row() -> None:
    ci = _ci()
    a = _lrow(ci)
    assert a == pytest.approx(ci.accel_kmhs(False, TAU)) and ci.i_brake == pytest.approx(0.5 * DT)
    assert ci.row(is_accel=False, eligible=False, dev_kmh=0.5, dt_s=DT, tau_s=TAU) == 0.0
    assert ci.i_brake == 0.0


@pytest.mark.parametrize("is_accel", [False, True])
def test_row_hold_keeps_value_on_low_rows(is_accel: bool) -> None:
    ci = _ci(low_speed="hold")
    for _ in range(10):
        _lrow(ci, is_accel=is_accel, dev=0.5 if not is_accel else -0.5)
    v, a = ci.value(is_accel), ci.accel_kmhs(is_accel, TAU)
    assert a != 0.0
    for dev in (0.5, -0.5):
        assert _lrow(ci, is_accel=is_accel, dev=dev, low=True) == a  # 値が変わらず a_I を返す
        assert ci.value(is_accel) == v
    # 反対のペダルに替わったら反対側は 0（hold でも activate はする）
    _lrow(ci, is_accel=not is_accel, low=True)
    assert ci.value(is_accel) == 0.0


@pytest.mark.parametrize("is_accel", [False, True])
def test_row_accumulate_accumulates_on_low_rows(is_accel: bool) -> None:
    ci = _ci(low_speed="accumulate")
    dev = -0.5 if is_accel else 0.5
    for n in range(1, 4):
        _lrow(ci, is_accel=is_accel, dev=dev, low=True)
        assert ci.value(is_accel) == pytest.approx(dev * DT * n)


def test_row_deep_only_brake_skips_shallow_direction() -> None:
    ci = _ci(low_speed="accumulate")
    _lrow(ci, dev=0.5, low=True, deep_only=True)  # ブレーキの浅い向き（dev ＞ 0）は貯めない
    assert ci.i_brake == 0.0
    _lrow(ci, dev=-0.5, low=True, deep_only=True)  # 深い向きは貯める
    assert ci.i_brake == pytest.approx(-0.5 * DT)
    before = ci.i_brake
    _lrow(ci, dev=0.5, low=True, deep_only=True)
    assert ci.i_brake == before  # 持っている値は保たれる（浅い向きを止めるだけ）
    _lrow(ci, dev=0.5, low=True)  # deep_only でなければ貯める
    assert ci.i_brake == pytest.approx(before + 0.5 * DT)


@pytest.mark.parametrize("dev", [0.5, -0.5])
def test_row_deep_only_has_no_effect_on_accel(dev: float) -> None:
    a = _ci(low_speed="accumulate")
    b = _ci(low_speed="accumulate")
    _lrow(a, is_accel=True, dev=dev, low=True, deep_only=True)
    _lrow(b, is_accel=True, dev=dev, low=True)
    assert a.i_accel == b.i_accel == pytest.approx(dev * DT)


def test_row_not_eligible_resets_in_every_mode() -> None:
    # 停車速度以下は呼び出し側が eligible ＝ False にして 0 へ戻す
    for mode in ("off", "hold", "accumulate"):
        ci = _ci(low_speed=mode)
        _lrow(ci)
        assert ci.row(
            is_accel=False, eligible=False, dev_kmh=0.5, dt_s=DT, tau_s=TAU, low=True
        ) == 0.0
        assert ci.i_brake == 0.0


# ── 開度への換算（deviation_offset_detail の integral_kmhs） ──


def _detail(effort: float, need: float, dev: float, **kw):
    return deviation_offset_detail(
        linear_table(), effort=effort, flag=FLAG_MEASURED, need_kmhs=need, ref_kmh=50.0,
        row_kmh=50.0, dev_kmh=dev, tau_accel_s=TAU, tau_brake_s=TAU,
        accel_min_pct=AD, brake_min_pct=BD, **kw,
    )


@pytest.mark.parametrize(("effort", "need"), [(-15.0, -2.5), (12.0, 0.5)])
def test_integral_zero_is_identical(effort: float, need: float) -> None:
    for dev in (-1.0, 0.0, 1.0):
        base = _detail(effort, need, dev)
        assert _detail(effort, need, dev, integral_kmhs=0.0) == base


def test_integral_keeps_offset_when_deviation_is_zero_brake() -> None:
    e = -15.0
    p_zero = _detail(e, -2.5, 0.0)[0]
    assert p_zero == e  # P だけなら偏差 0 で上乗せは 0
    # 速すぎ（dev ＜ 0）を貯めた I は負の加速度 → 偏差 0 でも深いまま
    ci = _ci()
    _run(ci, False, -0.5, 30)
    a_i = ci.accel_kmhs(False, TAU)
    assert a_i < 0.0
    assert _detail(e, -2.5, 0.0, integral_kmhs=a_i)[0] < e


def test_integral_keeps_offset_when_deviation_is_zero_accel() -> None:
    e = 12.0
    ci = _ci()
    _run(ci, True, 0.5, 30)  # 遅れ（dev ＞ 0）→ 偏差 0 でも踏み増したまま
    a_i = ci.accel_kmhs(True, TAU)
    assert a_i > 0.0
    assert _detail(e, 0.5, 0.0, integral_kmhs=a_i)[0] > e


def test_integral_is_clamped_at_min_like_p() -> None:
    # 浅くする向きの I が大きくても min で止まる（反対のペダルには移らない）
    new, clamped = _detail(-(BD + 0.5), -2.5, 0.0, integral_kmhs=50.0)
    assert new == -BD and clamped


# ── 1 周期の流れ（row）: 貯める行とリセット ──


def _row(ci: C10Integral, is_accel: bool, dev: float = 0.5, **kw) -> float:
    args = {"eligible": True, "tau_s": TAU, **kw}
    return ci.row(is_accel=is_accel, dev_kmh=dev, dt_s=DT, **args)


@pytest.mark.parametrize("is_accel", [False, True])
def test_row_accumulates_then_resets_when_not_eligible(is_accel: bool) -> None:
    # eligible ＝ 7.5 km/h 以上・停車保持でない。7.5 未満・停車は eligible=False
    ci = _ci()
    a = [_row(ci, is_accel) for _ in range(20)]
    assert a[-1] > a[0] > 0.0
    assert _row(ci, is_accel, eligible=False) == 0.0
    assert ci.i_brake == ci.i_accel == 0.0


def test_row_resets_when_side_switches_and_when_other_pedal() -> None:
    ci = _ci()
    for _ in range(20):
        _row(ci, False)
    assert ci.i_brake > 0.0
    _row(ci, True)  # FF がアクセルの行へ
    assert ci.i_brake == 0.0 and ci.i_accel > 0.0
    ci2 = _ci(accel=False)
    for _ in range(20):
        _row(ci2, False)
    assert _row(ci2, True) == 0.0  # off の側の行は貯めず、ブレーキの分も 0 に戻す
    assert ci2.i_brake == 0.0


def test_row_without_tau_resets() -> None:
    ci = _ci()
    for _ in range(5):
        _row(ci, False)
    assert _row(ci, False, tau_s=None) == 0.0
    assert ci.i_brake == 0.0
