"""格子ステップ走行の「何を測るか・開度をいくつにするか」を決める純ロジック（20260925 段3）。

ハード非依存（numpy のみ）。`pattern_loop.PatternLoop` が走行中にこのプランナーへ問い合わせる。
`wltp_grid` は import しない（wltp_grid → mode_drive → pattern_drive → pattern_loop → 本モジュール
の循環になるため。WLTP の集計は呼び出し側が配列で渡す）。

走り方（1 ステーション = ある車速。詳細は計画ファイル）:
    PI で車速を保つ → 落ち着いたら定常開度 u0 を決める → 開度を固定して数秒走り、加速度を測る
    ステップの順: 定速（u0 のまま）→ 惰行（両ペダル 0%）→ 減速の列（緩い順）→ 加速の列（小さい順）
    2026-09-27 段7a（ProblemReport_20260925 段7）: 惰行はコーストダウンの実測（`coast_fn`）が
    使えるステーションでは測らない（惰行ステップを省く）。使えない車速帯だけ従来どおり惰行ステップ
    を入れる（フォールバック）

開度の決め方（車両ごとに自動。感度 g = 開度 1% あたりの加速度 [km/h/s per %]）:
    アクセルの加速: u = u0 + (狙い a − a_hold) / g_accel（u0 は PI が落ち着いた開度。
        a_hold は u0 のまま
        走った定速ステップの実測加速度。PI は近づく途中で「落ち着いた」と判定するので u0 は偏り、
        実機（20260926）では a_hold が +0.19〜+0.49 km/h/s だった。実測の残差 σ は 0.03 km/h で
        揺れてはいないので、基準は加速度 0 ではなく (u0, a_hold) の実測点にする）
    緩い減速（惰行より緩い）: アクセルを u0 から不感帯へ向けて絞る。惰行(不感帯, a_coast)と
        定速(u0, a_hold)の 2 点の間を直線で結ぶ（g を仮定しない）
    強い減速（惰行より強い）: u = ブレーキ不感帯 + (惰行の a − 狙い a) / g_brake。ただし不感帯からの
        踏み込み量は、そのステーションで試した最大の 2 倍まで（未試行なら停車保持開度まで）。
        15 km/h ではブレーキ 7〜19% が惰行と同じ減速で g が極小になり、開度が上限 80% まで伸びた
        （20260926 の実機）ための安全上限。2 倍は仮の値
    g は 1 回測るごとに割線で更新して次のステップ・次のステーションへ引き継ぐ。初期値は大きめ
    （踏み増し Δ = 狙い a ÷ g が小さくなる ＝ 1 回目は弱く外れる側。強すぎるより安全）
    2026-09-28 段7c（ProblemReport_20260925 段7）: そのステーションでまだ測っていないペダルは、
    HOLD・COAST の実測直後に一度だけ、初期値の代わりに `gain_fn`（上限 G の予測マップの、その
    車速帯で一番効いた実測点）から感度を引く（`_seed_gain_from_glimit`）。実機（20260928）で
    初期値 2.0 が実測の 1/8 しかなく、GRID_RETURN の開度が狙いの半分未満にしかならなかった対策。
    以降は従来どおり `_update_gain` の実測で上書きする

打ち切り・やり直し:
    - 実測 a が「その車速帯の WLTP の最大（減速は最小）× overshoot_frac」を超えたら、その向きの
      残りをとばす。G ガバナが作動したときも同じ
    - 実測 a が同じ向きの別の狙いのセルに入ったら、そのセルは測れたことにしてとばす
    - 狙いのセルに届かなければ、更新した g で max_tries 回まで試す
"""

from __future__ import annotations

import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass, replace
from enum import StrEnum

import numpy as np

from tests.research.g_limit import PEDAL_ACCEL, PEDAL_BRAKE

_MIN_STEP_PCT = 0.05  # 開度の差がこれ以下なら感度を更新しない（0 割り・ノイズ防止）
_MIN_RESPONSE_KMHS = 0.02  # 加速度の変化がこれ以下なら感度を更新しない
_BRAKE_DEPTH_GROWTH = 2.0  # ブレーキ踏み込み量の伸びの上限倍率（仮の値。データの根拠なし）


@dataclass(frozen=True)
class GridSettings:
    """格子ステップ走行の調整値（config の `learning.grid_*` から作る）。"""

    station_min_kmh: float = 10.0  # これ未満の車速帯はステーションを置かない（発進・停車が担当）
    settle_tol_kmh: float = 1.0  # 「落ち着いた」とみなす車速の許容幅
    settle_s: float = 3.0  # 許容幅の中にこの秒数いたら落ち着いたとする
    settle_timeout_s: float = 40.0  # 落ち着かないときの打ち切り（そのステーションの残りをとばす）
    step_window_s: float = 3.0  # 開度固定の 1 ステップの長さ
    step_lag_s: float = 0.5  # ステップの頭のこの秒数は傾きの計算から除く（ペダル・応答の遅れ）
    step_band_max_kmh: float = 10.0  # ステーションからこれ以上離れたらステップを終える
    step_min_fit_s: float = 1.0  # 中心から踏んで傾きを測れる時間がこれ未満なら助走をつける
    overshoot_frac: float = 1.2  # WLTP の最大（最小）加速度のこの倍を超えたら打ち切り
    max_tries: int = 2  # 1 つの狙いに使う最大回数（やり直し 1 回）
    gain_init: float = 2.0  # アクセルの感度の初期値 [km/h/s per %]
    brake_gain_init: float = 2.0  # ブレーキの感度の初期値
    gain_min: float = 0.05
    gain_max: float = 20.0
    hold_kp_norm: float = 0.36  # PI 保持: kp = この値 / g_accel
    hold_ki_norm: float = 0.06
    hold_max_rate_pct_s: float = 1.0  # PI の開度変化の上限 [%/s]（g では割らない。下記参照）
    launch_end_kmh: float = 20.0  # 発進・停車セルが担当する車速の上端
    wltp_min_s: float = 5.0  # WLTP がこれ以上要るセルだけ狙う
    # 2026-09-27 段7b（ProblemReport_20260925 段7）: ステップの後・助走の目標を決めたときの
    # 「ステーションの目標へ戻る」区間。目標から離れていれば固定開度（GRID_RETURN）で速く戻り、
    # 近づいたら PI（CRUISE_HOLD）へ切り替える
    grid_return_accel_kmhs: float = 2.0  # GRID_RETURN の狙い加速度（目標より遅いとき）
    grid_return_switch_kmh: float = 1.5  # これ以下まで近づいたら CRUISE_HOLD の PI に切り替える


@dataclass(frozen=True)
class Target:
    """狙いの加速度。`a_kmhs` はセルの中の WLTP の平均、[lo, hi) はセルの範囲。"""

    a_kmhs: float
    lo_kmhs: float
    hi_kmhs: float

    def contains(self, a_kmhs: float) -> bool:
        return self.lo_kmhs <= a_kmhs < self.hi_kmhs


@dataclass(frozen=True)
class StationPlan:
    speed_kmh: float
    speed_lo_kmh: float
    speed_hi_kmh: float
    decel: tuple[Target, ...]  # 緩い順（0 に近い順）
    accel: tuple[Target, ...]  # 小さい順
    a_max_kmhs: float  # この車速帯の WLTP の最大加速度（≥ 0）
    a_min_kmhs: float  # 最小加速度（≤ 0）


@dataclass(frozen=True)
class LaunchPlan:
    """0 〜 launch_end_kmh を 1 本で走る発進（加速）・停車（減速）の狙い。"""

    end_kmh: float
    accel: tuple[Target, ...]
    decel: tuple[Target, ...]
    a_max_kmhs: float
    a_min_kmhs: float


class StepKind(StrEnum):
    HOLD = "hold"  # 定速: 開度 u0 のまま
    COAST = "coast"  # 惰行: 両ペダル 0%
    DECEL_ACCEL = "decel_accel"  # 緩い減速: アクセルを絞る
    DECEL_BRAKE = "decel_brake"  # 強い減速: ブレーキ
    ACCEL = "accel"  # 加速
    LAUNCH = "launch"  # 停車からの発進（アクセル固定）
    STOP = "stop"  # 停車までの減速（ブレーキ固定）


_ACCEL_KINDS = (StepKind.ACCEL, StepKind.LAUNCH)
_DECEL_KINDS = (StepKind.DECEL_ACCEL, StepKind.DECEL_BRAKE, StepKind.STOP)


@dataclass(frozen=True)
class Step:
    kind: StepKind
    accel_pct: float
    brake_pct: float
    target: Target | None
    tries: int = 1
    approach_kmh: float | None = None  # 助走の車速（強いステップだけ。None なら中心から踏む）
    base_pct: float | None = None  # ACCEL の基準開度（助走のときは助走の車速で落ち着いた開度 u0'）
    base_a_kmhs: float = 0.0  # ACCEL の基準開度での加速度（助走なしは定速ステップの実測、助走は 0）
    capped: bool = False  # 開度を上限 G の予測（cap_fn）で頭打ちにした


@dataclass(frozen=True)
class StepResult:
    speed_kmh: float
    kind: StepKind
    target_a_kmhs: float  # 狙い（定速・惰行は nan）
    accel_pct: float
    brake_pct: float
    a_meas_kmhs: float
    gain_before: float  # そのペダルの感度（更新前。定速・惰行・アクセルを絞る減速は nan）
    gain_after: float
    verdict: str  # "OK" / "定速" / "惰行" / "やり直し" / "未達" / "打切り" / "上限G"
    tries: int
    also_hit: int = 0  # このステップで一緒に測れたことになった他の狙いの数
    approach_kmh: float = float("nan")  # 助走の車速（助走なしは nan）
    fit_points: int = 0  # 傾きの当てはめに使った点数（0 = 記録なし）
    fit_resid_kmh: float = float("nan")  # 当てはめの残差の標準偏差 [km/h]


@dataclass
class _Item:
    kind: StepKind
    target: Target | None = None
    tries: int = 1
    accel_override: float | None = None  # やり直しのアクセル開度（アクセルを絞る減速用）


# ─────────────────────────────────────────────────────────────────────
# 純関数
# ─────────────────────────────────────────────────────────────────────


def fit_slope(times_s: Sequence[float], speeds_kmh: Sequence[float]) -> float:
    """車速 [km/h] の時間 [s] に対する最小二乗の傾き [km/h/s]。2 点未満・時間幅 0 なら 0。"""
    if len(times_s) < 2 or times_s[-1] - times_s[0] <= 0.0:
        return 0.0
    slope, _ = np.polyfit(np.asarray(times_s, dtype=float), np.asarray(speeds_kmh, dtype=float), 1)
    return float(slope)


def _clamp(value: float, lo: float, hi: float) -> float:
    return min(max(value, lo), hi)


def pi_hold_step(
    base: float, integral: float, v_star: float, v_meas: float, *,
    kp: float, ki: float, min_pct: float, max_pct: float, max_rate_pct_s: float, dt: float,
) -> tuple[float, float]:
    """定速保持の PI 1 周期ぶん。戻り値は `(新しい開度, 新しい積分)`。

    `excite._pi_hold_step` と同じ（P + 条件付き積分 + レート制限。出力クランプだけでなく
    レート制限も飽和として扱い、実際に反映できた分まで積分を巻き戻す = back-calculation）。
    停車から最初のステーションまで一気に上げるとき、レート制限で頭打ちのまま積分だけが伸びて
    目標を大きく行き過ぎるのを防ぐ。excite からは import しない（循環するため移植）。
    """
    error = v_star - v_meas
    step_limit = max_rate_pct_s * dt
    candidate_integral = integral + ki * error * dt
    unsaturated = kp * error + candidate_integral
    desired = _clamp(unsaturated, min_pct, max_pct)
    new_base = base + _clamp(desired - base, -step_limit, step_limit)
    if abs(new_base - unsaturated) < 1e-9:
        return new_base, candidate_integral
    return new_base, new_base - kp * error


def _split_targets(
    seconds_row: np.ndarray, mean_row: np.ndarray, accel_edges: Sequence[float], min_s: float
) -> tuple[tuple[Target, ...], tuple[Target, ...]]:
    """1 行ぶんの (減速の狙い, 加速の狙い)。0 を含む列（定速）は狙わない（定速ステップが担当）。"""
    decel: list[Target] = []
    accel: list[Target] = []
    for j in range(len(seconds_row)):
        if seconds_row[j] < min_s or math.isnan(float(mean_row[j])):
            continue
        lo, hi = float(accel_edges[j]), float(accel_edges[j + 1])
        target = Target(float(mean_row[j]), lo, hi)
        if lo < 0.0 < hi:
            continue
        (decel if hi <= 0.0 else accel).append(target)
    decel.sort(key=lambda t: -t.a_kmhs)
    accel.sort(key=lambda t: t.a_kmhs)
    return tuple(decel), tuple(accel)


def plan_stations(
    seconds: np.ndarray,
    mean_accel: np.ndarray,
    max_accel_by_speed: np.ndarray,
    min_accel_by_speed: np.ndarray,
    speed_edges: Sequence[float],
    accel_edges: Sequence[float],
    settings: GridSettings,
) -> list[StationPlan]:
    """車速帯ごとのステーション（帯の中心）。`station_min_kmh` 未満と WLTP がほぼ無い帯は除く。"""
    plans: list[StationPlan] = []
    for i in range(len(speed_edges) - 1):
        lo, hi = float(speed_edges[i]), float(speed_edges[i + 1])
        if lo < settings.station_min_kmh or seconds[i].sum() < settings.wltp_min_s:
            continue
        decel, accel = _split_targets(seconds[i], mean_accel[i], accel_edges, settings.wltp_min_s)
        a_max = float(max_accel_by_speed[i]) if not math.isnan(max_accel_by_speed[i]) else 0.0
        a_min = float(min_accel_by_speed[i]) if not math.isnan(min_accel_by_speed[i]) else 0.0
        plans.append(
            StationPlan(0.5 * (lo + hi), lo, hi, decel, accel, max(a_max, 0.0), min(a_min, 0.0))
        )
    return plans


def plan_launch(
    seconds: np.ndarray,
    mean_accel: np.ndarray,
    max_accel_by_speed: np.ndarray,
    min_accel_by_speed: np.ndarray,
    speed_edges: Sequence[float],
    accel_edges: Sequence[float],
    settings: GridSettings,
) -> LaunchPlan:
    """0 〜 launch_end_kmh の車速帯をまとめた 1 行として、発進・停車の狙いを作る。"""
    rows = [
        i for i in range(len(speed_edges) - 1)
        if speed_edges[i] >= 0.0 and speed_edges[i + 1] <= settings.launch_end_kmh
    ]
    n_accel = seconds.shape[1]
    sec = np.zeros(n_accel)
    weighted = np.zeros(n_accel)
    for i in rows:
        for j in range(n_accel):
            if seconds[i, j] > 0.0 and not math.isnan(float(mean_accel[i, j])):
                sec[j] += seconds[i, j]
                weighted[j] += seconds[i, j] * mean_accel[i, j]
    with np.errstate(invalid="ignore", divide="ignore"):
        mean = np.where(sec > 0.0, weighted / np.where(sec > 0.0, sec, 1.0), np.nan)
    decel, accel = _split_targets(sec, mean, accel_edges, settings.wltp_min_s)
    maxima = [float(max_accel_by_speed[i]) for i in rows if not math.isnan(max_accel_by_speed[i])]
    minima = [float(min_accel_by_speed[i]) for i in rows if not math.isnan(min_accel_by_speed[i])]
    return LaunchPlan(
        settings.launch_end_kmh, accel, decel,
        max(max(maxima, default=0.0), 0.0), min(min(minima, default=0.0), 0.0),
    )


# ─────────────────────────────────────────────────────────────────────
# プランナー
# ─────────────────────────────────────────────────────────────────────


class GridPlanner:
    """走行中に次のステップの開度を決め、測定結果で感度を更新する（1 回の走行で 1 つ）。"""

    def __init__(
        self,
        settings: GridSettings,
        *,
        accel_deadband_pct: float,
        brake_deadband_pct: float,
        max_accel_pct: float,
        max_brake_pct: float,
        max_speed_kmh: float = math.inf,
        stop_brake_pct: float | None = None,
        cap_fn: Callable[[str, float], float | None] | None = None,
        coast_fn: Callable[[float], float | None] | None = None,
        gain_fn: Callable[[str, float], tuple[float, float] | None] | None = None,
    ) -> None:
        self.settings = settings
        # 上限 G に届く開度の予測（段6a）。`cap_fn(ペダル, 車速) -> 開度 [%]`
        # （測れていなければ None）。無ければ従来どおり（開度は上限をかけない）
        self._cap_fn = cap_fn
        # コーストダウンの実測から惰行の加速度を引く（段7a）。`coast_fn(車速) -> 惰行の加速度
        # [km/h/s、負]`（測れていなければ None）。無ければ従来どおり惰行ステップで測る
        self._coast_fn = coast_fn
        # そのステーションでまだ測っていないペダルの最初の感度を、実測の一番効いた点から引く
        # （段7c）。`gain_fn(ペダル, 車速) -> (開度, 加速度)`（無ければ None）。無ければ従来どおり
        # 仮定の初期値（gain_init/brake_gain_init）のまま
        self._gain_fn = gain_fn
        self.g_accel = settings.gain_init
        self.g_brake = settings.brake_gain_init
        self.results: list[StepResult] = []
        self._accel_db = accel_deadband_pct
        self._brake_db = brake_deadband_pct
        self._max_accel = max_accel_pct
        self._max_brake = max_brake_pct
        self._max_speed = max_speed_kmh  # 減速の助走の車速の上限（走行できる最高車速の少し手前）
        self._plan: StationPlan | LaunchPlan | None = None
        self._queue: list[_Item] = []
        # ブレーキの踏み込み量の初期上限（未試行のとき）。停車復帰で毎回踏んでいる実績のある開度
        self._stop_brake = max_brake_pct if stop_brake_pct is None else stop_brake_pct
        self._brake_tried_max = 0.0  # このステーションで試した、不感帯からの最大の踏み込み量
        self._u0 = accel_deadband_pct
        self._a_hold = 0.0  # 定速ステップ（u0 のまま）の実測加速度
        self._a_coast: float | None = None
        self._coast_by_speed: dict[float, float] = {}
        self._g_accel_by_speed: dict[float, float] = {}  # ステーションごとの最後の感度
        self._g_brake_by_speed: dict[float, float] = {}
        self._fit: tuple[int, float] = (0, float("nan"))

    # ── 進行 ─────────────────────────────────────────────────────
    def begin_station(self, plan: StationPlan, u0_pct: float) -> None:
        """ステーションの測定を始める。`u0_pct` は PI が落ち着いたときの定常開度。

        惰行は `coast_fn`（段7a）が測れればそこから引き、惰行ステップは入れない。測れなければ
        従来どおり惰行ステップをキューに入れる（フォールバック。単体テストとの互換）。
        """
        self._plan = plan
        self._u0 = _clamp(u0_pct, self._accel_db, self._max_accel)
        self._a_hold = 0.0
        self._brake_tried_max = 0.0
        coast = self._coast_fn(plan.speed_kmh) if self._coast_fn is not None else None
        if coast is not None:
            self._a_coast = coast
            self._coast_by_speed[plan.speed_kmh] = coast
            self._queue = [_Item(StepKind.HOLD)]
            self._seed_gain_from_glimit(PEDAL_BRAKE, self._brake_db, coast)
        else:
            self._a_coast = None
            self._queue = [_Item(StepKind.HOLD), _Item(StepKind.COAST)]
        self._queue += [_Item(StepKind.DECEL_ACCEL, t) for t in plan.decel]
        self._queue += [_Item(StepKind.ACCEL, t) for t in plan.accel]

    def begin_launch(self, plan: LaunchPlan) -> None:
        """発進・停車セルの測定を始める。発進と停車を交互に並べる。"""
        self._plan = plan
        self._u0 = self._accel_db
        self._a_hold = 0.0
        self._a_coast = None
        self._brake_tried_max = 0.0
        # 感度は、直前（高い車速）のステーションの値ではなく、発進・停車の車速に最も近い
        # ステーションの値から始める（135 km/h の g_brake 1.4 が低速では 30 倍大きく、
        # 停車ステップの開度が不感帯ちょうどになった。20260926 の実機）
        mid = 0.5 * plan.end_kmh
        if self._g_accel_by_speed:
            near = min(self._g_accel_by_speed, key=lambda v: abs(v - mid))
            self.g_accel = self._g_accel_by_speed[near]
        if self._g_brake_by_speed:
            near = min(self._g_brake_by_speed, key=lambda v: abs(v - mid))
            self.g_brake = self._g_brake_by_speed[near]
        self._queue = []
        for k in range(max(len(plan.accel), len(plan.decel))):
            if k < len(plan.accel):
                self._queue.append(_Item(StepKind.LAUNCH, plan.accel[k]))
            if k < len(plan.decel):
                self._queue.append(_Item(StepKind.STOP, plan.decel[k]))

    @property
    def remaining(self) -> int:
        return len(self._queue)

    @property
    def u0(self) -> float:
        """今のステーションの定常開度 [%]（begin_station で固定）。"""
        return self._u0

    @property
    def a_hold(self) -> float:
        """定速ステップ（u0 のまま）の実測加速度 [km/h/s]（段7b: GRID_RETURN の開度の基準）。"""
        return self._a_hold

    def peek_kind(self) -> StepKind | None:
        """次のステップの種類（取り出さない）。無ければ None。"""
        return self._queue[0].kind if self._queue else None

    def drop_next(self) -> None:
        """次のステップを測らずに捨てる（走行の状態が合わないとき）。"""
        if self._queue:
            self._queue.pop(0)

    def abort_remaining(self) -> None:
        """今のステーション（発進・停車セル）の残りをすべて捨てる（落ち着かない・行き過ぎ等）。"""
        self._queue = []

    def is_strong(self, target: Target) -> bool:
        """強い狙いか（ステーションの中心から踏むと、帯を出るまでに傾きを測る時間が足りない）。"""
        s = self.settings
        a = abs(target.a_kmhs)
        return a > 0.0 and s.step_band_max_kmh / a - s.step_lag_s < s.step_min_fit_s

    def peek_approach_kmh(self) -> float | None:
        """次のステップに助走が要るなら、その車速（帯の手前の端）。要らなければ None。

        加速はステーションの下側（下限は station_min_kmh）、減速は上側（上限は最高車速の手前）へ
        下がって（上がって）落ち着いてから踏む。中心とほぼ同じ車速になるなら助走は付けない。
        """
        plan = self._plan
        if not self._queue or not isinstance(plan, StationPlan):
            return None
        target = self._queue[0].target
        if target is None or not self.is_strong(target):
            return None
        s = self.settings
        if target.a_kmhs > 0.0:
            v = max(plan.speed_kmh - s.step_band_max_kmh, s.station_min_kmh)
        else:
            v = min(plan.speed_kmh + s.step_band_max_kmh, self._max_speed)
        return None if abs(v - plan.speed_kmh) < s.settle_tol_kmh else v

    def next_step(self, u0_pct: float | None = None) -> Step | None:
        """次のステップ（開度つき）を取り出す。無ければ None。

        `u0_pct` は助走のステップの基準開度（助走の車速で落ち着いた開度）。助走が要らない
        ステップでは使わない（ステーションの u0 を使う）。
        """
        if not self._queue:
            return None
        approach = self.peek_approach_kmh()
        item = self._queue.pop(0)
        base, base_a = self._u0, self._a_hold
        if approach is not None and u0_pct is not None:
            base, base_a = _clamp(u0_pct, self._accel_db, self._max_accel), 0.0
        step = self._realize(item, base, base_a)
        if approach is None:
            return step
        return replace(step, approach_kmh=approach)

    def _realize(self, item: _Item, base_pct: float, base_a: float = 0.0) -> Step:
        kind, target = item.kind, item.target
        if kind is StepKind.HOLD:
            return Step(kind, self._u0, 0.0, None, item.tries)
        if kind is StepKind.COAST:
            return Step(kind, 0.0, 0.0, None, item.tries)
        assert target is not None
        if kind is StepKind.DECEL_ACCEL:
            u = item.accel_override
            if u is None:
                u = self._decel_accel_opening(target.a_kmhs)
            if u is None:  # 惰行より強い（または補間できない）: ブレーキで測る
                kind = StepKind.DECEL_BRAKE
            else:
                return Step(kind, _clamp(u, self._accel_db, self._u0), 0.0, target, item.tries)
        if kind in (StepKind.DECEL_BRAKE, StepKind.STOP):
            base = self._coast_base()
            depth = max(0.0, base - target.a_kmhs) / self.g_brake
            depth = min(depth, self._brake_depth_limit())
            u = _clamp(self._brake_db + depth, self._brake_db, self._max_brake)
            u, capped = self._apply_cap(PEDAL_BRAKE, u)
            return Step(kind, 0.0, u, target, item.tries, capped=capped)
        # ACCEL / LAUNCH
        base = None
        if kind is StepKind.ACCEL:
            base = base_pct
            u = base + max(0.0, target.a_kmhs - base_a) / self.g_accel
        else:  # LAUNCH: 不感帯では走行抵抗（最寄りの惰行の減速）で減速するので、その分を見込む
            u = self._accel_db + max(0.0, target.a_kmhs - self._coast_base()) / self.g_accel
        u, capped = self._apply_cap(PEDAL_ACCEL, _clamp(u, self._accel_db, self._max_accel))
        return Step(
            kind, u, 0.0, target, item.tries,
            base_pct=base, base_a_kmhs=base_a if base is not None else 0.0, capped=capped,
        )

    def _decel_accel_opening(self, a_target: float) -> float | None:
        """惰行（不感帯）と定速（u0・実測 a_hold）の 2 点の直線で、狙いの減速になるアクセル開度。"""
        if self._a_coast is None:
            return None
        span = self._a_hold - self._a_coast
        if span < 0.05 or a_target <= self._a_coast or self._u0 - self._accel_db < _MIN_STEP_PCT:
            return None
        frac = (a_target - self._a_coast) / span
        return self._accel_db + (self._u0 - self._accel_db) * _clamp(frac, 0.0, 1.0)

    def _cap_speed(self, pedal: str) -> float:
        """開度の上限を引く車速。ステップ中に通りうる帯の中で、そのペダルが一番効く端。

        ブレーキは速いほど、アクセルは遅いほど効く。1 つの開度で帯の端まで通るので、
        中心の効きで決めると効く端で上限 G を超える（20260926: 25 km/h 中心の感度で決めた
        ブレーキ 34% を 34.6 km/h から踏んで 0.47G）。
        """
        plan = self._plan
        s = self.settings
        if isinstance(plan, StationPlan):
            if pedal == PEDAL_BRAKE:
                return min(plan.speed_kmh + s.step_band_max_kmh, self._max_speed)
            return max(plan.speed_kmh - s.step_band_max_kmh, s.station_min_kmh)
        if isinstance(plan, LaunchPlan):
            return plan.end_kmh if pedal == PEDAL_BRAKE else 0.0
        return 0.0

    def _apply_cap(self, pedal: str, opening: float) -> tuple[float, bool]:
        """開度を上限 G の予測で頭打ちにする。戻り値は (開度, 頭打ちにしたか)。"""
        if self._cap_fn is None:
            return opening, False
        cap = self._cap_fn(pedal, self._cap_speed(pedal))
        if cap is None or opening <= cap:
            return opening, False
        return cap, True

    def _brake_depth_limit(self) -> float:
        """ブレーキ踏み込み量の上限（試した最大の 2 倍。未試行は停車保持開度まで）。"""
        if self._brake_tried_max > 0.0:
            return _BRAKE_DEPTH_GROWTH * self._brake_tried_max
        return max(self._stop_brake - self._brake_db, _MIN_STEP_PCT)

    def _coast_base(self) -> float:
        """ブレーキ・発進の開度の基準にする惰行の加速度（ステーションでは測った値、発進・停車は最寄り）。

        発進・停車セルは coast_fn(0.5 x plan.end_kmh)（段7a）を優先し、無ければ従来どおり
        測ったステーションの中で一番近い車速の惰行を使う。
        """
        if self._a_coast is not None:
            return self._a_coast
        plan = self._plan
        if isinstance(plan, LaunchPlan):
            mid = 0.5 * plan.end_kmh
            coast = self._coast_fn(mid) if self._coast_fn is not None else None
            if coast is not None:
                return coast
            if self._coast_by_speed:
                v = min(self._coast_by_speed, key=lambda s: abs(s - mid))
                return self._coast_by_speed[v]
        return 0.0

    # ── 記録 ─────────────────────────────────────────────────────
    def record(
        self, step: Step, a_meas_kmhs: float, *, governed: bool = False,
        fit_points: int = 0, fit_resid_kmh: float = float("nan"),
    ) -> StepResult:
        """測った加速度を記録し、感度の更新・打ち切り・やり直しを決める。

        `fit_points`/`fit_resid_kmh` は傾きの当てはめの点数と残差（`grid_settle` の集計用。
        判定には使わない）。
        """
        self._fit = (fit_points, fit_resid_kmh)
        plan = self._plan
        assert plan is not None
        speed = plan.speed_kmh if isinstance(plan, StationPlan) else 0.5 * plan.end_kmh
        nan = float("nan")
        if step.kind is StepKind.HOLD:
            self._a_hold = a_meas_kmhs
            if isinstance(plan, StationPlan):  # 発進・停車セルは今まで通り（直前の感度を引き継ぐ）
                self._seed_gain_from_glimit(PEDAL_ACCEL, self._u0, a_meas_kmhs)
            return self._append(step, speed, a_meas_kmhs, nan, nan, "定速", 0)
        if step.kind is StepKind.COAST:
            self._a_coast = a_meas_kmhs
            self._coast_by_speed[speed] = a_meas_kmhs
            if isinstance(plan, StationPlan):
                self._seed_gain_from_glimit(PEDAL_BRAKE, self._brake_db, a_meas_kmhs)
            return self._append(step, speed, a_meas_kmhs, nan, nan, "惰行", 0)

        assert step.target is not None
        if step.kind in (StepKind.DECEL_BRAKE, StepKind.STOP):
            self._brake_tried_max = max(self._brake_tried_max, step.brake_pct - self._brake_db)
        g_before, g_after = self._update_gain(step, a_meas_kmhs)
        if isinstance(plan, StationPlan):
            self._g_accel_by_speed[speed] = self.g_accel
            self._g_brake_by_speed[speed] = self.g_brake
        is_accel = step.kind in _ACCEL_KINDS
        limit = (
            plan.a_max_kmhs * self.settings.overshoot_frac
            if is_accel
            else plan.a_min_kmhs * self.settings.overshoot_frac
        )
        exceeded = a_meas_kmhs > limit if is_accel else a_meas_kmhs < limit
        if governed or exceeded:
            self._drop_direction(is_accel)
            return self._append(step, speed, a_meas_kmhs, g_before, g_after, "打切り", 0)

        if step.target.contains(a_meas_kmhs):
            also = self._drop_hit_cells(is_accel, a_meas_kmhs)
            return self._append(step, speed, a_meas_kmhs, g_before, g_after, "OK", also)
        also = self._drop_hit_cells(is_accel, a_meas_kmhs)
        if step.capped:
            # 上限 G の予測で頭打ちにしたので、これ以上深くは踏めない。残りの強い狙いも同じ
            # 理由で届かないので捨てる（通し掃引が担当）
            self._drop_direction(is_accel)
            return self._append(step, speed, a_meas_kmhs, g_before, g_after, "上限G", also)
        if step.tries < self.settings.max_tries:
            self._queue.insert(0, self._retry_item(step, a_meas_kmhs))
            return self._append(step, speed, a_meas_kmhs, g_before, g_after, "やり直し", also)
        return self._append(step, speed, a_meas_kmhs, g_before, g_after, "未達", also)

    def _seed_gain_from_glimit(self, pedal: str, base_pct: float, base_a_signed: float) -> None:
        """まだ測っていないペダルの最初の感度を、`gain_fn` の実測点から引く（段7c）。

        HOLD（アクセル。`base_pct`=u0、`base_a_signed`=実測 a_hold）・COAST（ブレーキ。
        `base_pct`=不感帯、`base_a_signed`=実測 a_coast）を測った直後に一度だけ呼ばれる。以降の
        DECEL_ACCEL・DECEL_BRAKE・ACCEL・STOP は今までどおり `_update_gain` の実測で上書きされる
        （ここでの値は最初の 1 回の当て推量を良くするだけ）。`gain_fn` が無い・点が無い・差が
        小さすぎるときは何もしない（初期値 gain_init/brake_gain_init のまま。従来どおり）。

        `base_a_signed` は車速の実測の傾きそのまま（ブレーキも惰行と同じ符号。負）。`gain_fn`
        （`GLimitMap.strongest_point`）はペダルの向きを正とする値（`g_limit.add` の格納どおり。
        ブレーキは減速が正）を返すため、ブレーキは符号を反転して合わせる。

        20260928 実機: アクセルの感度が初期値 2.0（実測の 8 倍）のまま GRID_RETURN の開度が
        浅すぎ、190 s 止まった一因になった（`pattern_loop._step_grid_return` の実測補正と合わせて
        修正）。
        """
        if self._gain_fn is None or not isinstance(self._plan, StationPlan):
            return
        point = self._gain_fn(pedal, self._plan.speed_kmh)
        if point is None:
            return
        u, a = point
        base_a = base_a_signed if pedal == PEDAL_ACCEL else -base_a_signed
        du, da = u - base_pct, a - base_a
        if du <= _MIN_STEP_PCT or da <= _MIN_RESPONSE_KMHS:
            return
        gain = _clamp(da / du, self.settings.gain_min, self.settings.gain_max)
        if pedal == PEDAL_ACCEL:
            self.g_accel = gain
        else:
            self.g_brake = gain

    def _update_gain(self, step: Step, a_meas: float) -> tuple[float, float]:
        """割線で感度を更新する。アクセルを絞る減速は g を持たないので nan を返す。"""
        s = self.settings
        if step.kind in _ACCEL_KINDS:
            before = self.g_accel
            if step.kind is StepKind.ACCEL:
                base = step.base_pct if step.base_pct is not None else self._u0
                du, da = step.accel_pct - base, a_meas - step.base_a_kmhs
            else:
                du, da = step.accel_pct - self._accel_db, a_meas - self._coast_base()
            if du > _MIN_STEP_PCT and da > _MIN_RESPONSE_KMHS:
                self.g_accel = _clamp(da / du, s.gain_min, s.gain_max)
            elif du > _MIN_STEP_PCT:  # 踏んだのに無反応: 感度を過大に見ている。半分にして深く
                self.g_accel = _clamp(self.g_accel / 2.0, s.gain_min, s.gain_max)
            return before, self.g_accel
        if step.kind in (StepKind.DECEL_BRAKE, StepKind.STOP):
            before = self.g_brake
            du, da = step.brake_pct - self._brake_db, self._coast_base() - a_meas
            if du > _MIN_STEP_PCT and da > _MIN_RESPONSE_KMHS:
                self.g_brake = _clamp(da / du, s.gain_min, s.gain_max)
            elif du > _MIN_STEP_PCT:
                self.g_brake = _clamp(self.g_brake / 2.0, s.gain_min, s.gain_max)
            return before, self.g_brake
        return float("nan"), float("nan")

    def _retry_item(self, step: Step, a_meas: float) -> _Item:
        assert step.target is not None
        item = _Item(step.kind, step.target, step.tries + 1)
        a_coast = self._a_coast
        if step.kind is StepKind.DECEL_ACCEL and a_coast is not None:
            local_gain = (self._a_hold - a_coast) / max(self._u0 - self._accel_db, _MIN_STEP_PCT)
            if local_gain > 0.0:
                item.accel_override = step.accel_pct + (step.target.a_kmhs - a_meas) / local_gain
        return item

    def _drop_direction(self, is_accel: bool) -> None:
        kinds = _ACCEL_KINDS if is_accel else _DECEL_KINDS
        self._queue = [it for it in self._queue if it.kind not in kinds]

    def _drop_hit_cells(self, is_accel: bool, a_meas: float) -> int:
        """同じ向きの残りの狙いのうち、今の実測が入るセルを測れたことにして取り除く。"""
        kinds = _ACCEL_KINDS if is_accel else _DECEL_KINDS
        keep: list[_Item] = []
        dropped = 0
        for it in self._queue:
            if it.kind in kinds and it.target is not None and it.target.contains(a_meas):
                dropped += 1
            else:
                keep.append(it)
        self._queue = keep
        return dropped

    def _append(
        self, step: Step, speed: float, a_meas: float, g_before: float, g_after: float,
        verdict: str, also: int,
    ) -> StepResult:
        result = StepResult(
            speed_kmh=speed, kind=step.kind,
            target_a_kmhs=step.target.a_kmhs if step.target is not None else float("nan"),
            accel_pct=step.accel_pct, brake_pct=step.brake_pct, a_meas_kmhs=a_meas,
            gain_before=g_before, gain_after=g_after, verdict=verdict, tries=step.tries,
            also_hit=also,
            approach_kmh=step.approach_kmh if step.approach_kmh is not None else float("nan"),
            fit_points=self._fit[0], fit_resid_kmh=self._fit[1],
        )
        self.results.append(result)
        return result


def status_line(result: StepResult) -> str:
    """ターミナルに出す 1 行: 車速 / 狙い a / ペダル・開度 / 実測 a / g / 判定。"""
    pedal = (
        f"アクセル {result.accel_pct:5.2f}%" if result.brake_pct <= 0.0 and result.accel_pct > 0.0
        else f"ブレーキ {result.brake_pct:5.2f}%" if result.brake_pct > 0.0
        else "両ペダル 0%   "
    )
    target = "  —  " if math.isnan(result.target_a_kmhs) else f"{result.target_a_kmhs:+5.2f}"
    gain = (
        "" if math.isnan(result.gain_before)
        else f"  g {result.gain_before:.2f}→{result.gain_after:.2f}"
    )
    also = f"（他 {result.also_hit} セル）" if result.also_hit else ""
    approach = "" if math.isnan(result.approach_kmh) else f" 助走 {result.approach_kmh:.0f}km/h"
    return (
        f"格子 {result.speed_kmh:5.1f}km/h {result.kind.value:<11} 狙い {target} "
        f"{pedal} 実測 {result.a_meas_kmhs:+5.2f}{gain}  {result.verdict}"
        f"{'（' + str(result.tries) + '回目）' if result.tries > 1 else ''}{also}{approach}"
    )


__all__ = [
    "GridPlanner",
    "GridSettings",
    "LaunchPlan",
    "StationPlan",
    "Step",
    "StepKind",
    "StepResult",
    "Target",
    "fit_slope",
    "pi_hold_step",
    "plan_launch",
    "plan_stations",
    "status_line",
]
