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
    def __init__(self, *, brake_on: bool, accel_on: bool, ti_coef: float) -> None:
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

    def add(self, is_accel: bool, dev_kmh: float, dt_s: float) -> bool:
        """I_dev ＋＝ dev × dt。切られた向きにさらに貯まる場合は貯めない。貯めたら True。"""
        if not self.side_on(is_accel) or dt_s <= 0.0:
            return False
        shallow_dir = dev_kmh < 0.0 if is_accel else dev_kmh > 0.0
        deep_dir = dev_kmh > 0.0 if is_accel else dev_kmh < 0.0
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
        self, *, is_accel: bool, eligible: bool, dev_kmh: float, dt_s: float, tau_s: float | None
    ) -> float:
        """1 周期の流れ: 貯める行（eligible ＝ 7.5 km/h 以上・FF がこのペダル・停車保持でない）なら
        反対側を 0 に戻して貯め、a_I を返す。貯めない行は両方 0 に戻して 0 を返す。"""
        if not (eligible and self.side_on(is_accel) and tau_s is not None and tau_s > 0.0):
            self.reset()
            return 0.0
        self.activate(is_accel)
        self.add(is_accel, dev_kmh, dt_s)
        return self.accel_kmhs(is_accel, tau_s)
