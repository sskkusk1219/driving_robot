"""C10 の偏差の積分 I（2026-11-01 ProblemReport_20261031_効き始めの近くのブレーキ 段6d）。

P（偏差 ÷ τ）は偏差が 0 に向かうと 0 になり、指令が FF＋DOB へ戻る。この戻しが急な戻しの線を
踏んで減速が弱まり、また速すぎになる。そこで偏差を時間で足し（I_dev ＋＝ dev × dt）、
a_I ＝ I_dev ÷（τ × Ti）、Ti ＝ ti_coef × τ を P の加速度に足して、P と同じ入口
（`fwd_plan.deviation_offset_detail` の integral_kmhs）で開度に直す。

ブレーキ・アクセルは別々の量を持ち、片方に貯めるともう片方は 0 に戻す。
dev ＝ 基準 − 実車速（遅れていれば正）。浅くする向き（shallow）は、ブレーキは dev ＞ 0、
アクセルは dev ＜ 0。切られた向き（min で浅く／上限で深く）にさらに貯まる場合だけ止める
（ワインドアップ対策）。
"""

from __future__ import annotations

DT_MAX_S = 0.5  # 周期が途切れたときに 1 回で足し込む上限 [s]


class C10Integral:
    def __init__(
        self, *, brake_on: bool, accel_on: bool, ti_coef: float, low_speed: str = "off"
    ) -> None:
        self.low_speed = low_speed  # "off" | "hold" | "accumulate"（段1。境目の下の行の扱い）
        self.brake_on = brake_on
        self.accel_on = accel_on
        self.ti_coef = ti_coef
        self.i_brake = 0.0  # [km/h·s]
        self.i_accel = 0.0
        self.shallow_cut = False  # 直前の周期で浅い向きに切られたか
        self.deep_cut = False  # 直前の周期で深い向きに切られたか

    @property
    def enabled(self) -> bool:
        return self.brake_on or self.accel_on

    def reset(self) -> None:
        self.i_brake = 0.0
        self.i_accel = 0.0
        self.shallow_cut = False
        self.deep_cut = False

    def side_on(self, is_accel: bool) -> bool:
        return self.accel_on if is_accel else self.brake_on

    def value(self, is_accel: bool) -> float:
        return self.i_accel if is_accel else self.i_brake

    def accel_kmhs(self, is_accel: bool, tau_s: float | None) -> float:
        """a_I ＝ I_dev ÷（τ × Ti）、Ti ＝ ti_coef × τ [km/h/s]。off・τ なし・τ ≦ 0 は 0"""
        if not self.side_on(is_accel) or tau_s is None or tau_s <= 0.0 or self.ti_coef <= 0.0:
            return 0.0
        return self.value(is_accel) / (tau_s * self.ti_coef * tau_s)

    def activate(self, is_accel: bool) -> None:
        """この側で貯める周期に入る。反対側は 0 に戻す。"""
        if is_accel:
            self.i_brake = 0.0
        else:
            self.i_accel = 0.0

    def add(self, is_accel: bool, dev_kmh: float, dt_s: float, skip_shallow: bool = False) -> bool:
        """I_dev ＋＝ dev × dt。切られた向きにさらに貯まる場合は貯めない。貯めたら True。
        skip_shallow（段1: 停C の行のブレーキ）が true なら浅い向きは常に貯めない。"""
        if not self.side_on(is_accel) or dt_s <= 0.0:
            return False
        shallow_dir = dev_kmh < 0.0 if is_accel else dev_kmh > 0.0
        deep_dir = dev_kmh > 0.0 if is_accel else dev_kmh < 0.0
        if skip_shallow and shallow_dir:
            return False
        if (self.shallow_cut and shallow_dir) or (self.deep_cut and deep_dir):
            return False
        step = dev_kmh * min(dt_s, DT_MAX_S)
        if is_accel:
            self.i_accel += step
        else:
            self.i_brake += step
        return True

    def note_cut(self, *, shallow: bool, deep: bool) -> None:
        """この周期の指令が切られたか（min・上限・片足ルール・ガバナー）。次の周期の貯め方に効く。"""
        self.shallow_cut = shallow
        self.deep_cut = deep

    def row(
        self,
        *,
        is_accel: bool,
        eligible: bool,
        dev_kmh: float,
        dt_s: float,
        tau_s: float | None,
        low: bool = False,
        deep_only: bool = False,
    ) -> float:
        """1 周期の流れ: 貯める行（eligible ＝ 場面の条件: FF がこのペダル・停車保持/停車速度以下
        でない）なら反対側を 0 に戻して貯め、a_I を返す。貯めない行は両方 0 に戻して 0 を返す。
        段1: low ＝ 基準・実車速が境目（7.5 km/h）より下の行。low_speed が off なら今と同じく
        0 に戻す・hold なら貯めずに今の値の a_I を返す・accumulate なら貯める。
        deep_only ＝ 停C の行。accumulate の low の行のブレーキは深い向き（dev ＜ 0）だけ貯める
        （アクセルは影響なし）。"""
        if not (eligible and self.side_on(is_accel) and tau_s is not None and tau_s > 0.0):
            self.reset()
            return 0.0
        if low and self.low_speed not in ("hold", "accumulate"):
            self.reset()
            return 0.0
        self.activate(is_accel)
        if not (low and self.low_speed == "hold"):
            skip_shallow = low and deep_only and not is_accel and self.low_speed == "accumulate"
            self.add(is_accel, dev_kmh, dt_s, skip_shallow=skip_shallow)
        return self.accel_kmhs(is_accel, tau_s)
