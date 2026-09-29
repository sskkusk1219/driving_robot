"""プライマリー KPI の計算（手順 3 以降のモード走行・手順 4/6/8 の適合で使う）。

定義は docs/product-requirements.md のプライマリー KPI と本番 src/domain/control/kpi_monitor.py に
合わせ、しきい値は config_testVehicle.yaml の kpi セクションを使う。

    最大逸脱   … |実車速 − 基準車速| の最大（例外なし）
    p95        … |実車速 − 基準車速| の 95 パーセンタイル
    符号反転   … 偏差が ±reversal_band_kmh を両側で超えて入れ替わった回数（帯の中は直前の
                 符号を保持）。任意の reversal_window_s 窓での最大回数

本番 KPIMonitor との違い:
    - 走行中に逐次集計するのではなく、記録した 0.1s 刻みの行（CSV と同じ行）からまとめて計算する。
      CSV から計算し直しても同じ値になるようにするため。
    - p95 は numpy の線形補間パーセンタイル（本番は 0.01 km/h ビンの上端で、最大 +0.01 保守側）。
"""

from __future__ import annotations

from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from tests.research.config import KpiSection


@dataclass(frozen=True)
class DeviationEpisode:
    """|偏差| が最大逸脱のしきい値を連続で超えた 1 区間。"""

    start_s: float
    end_s: float  # 最後に超えていた行の時刻
    peak_kmh: float  # 符号付き（+: 実車速が速い / −: 遅い）
    peak_t_s: float

    @property
    def duration_s(self) -> float:
        return self.end_s - self.start_s


@dataclass(frozen=True)
class KpiResult:
    n_samples: int
    max_abs_kmh: float
    max_abs_t_s: float
    p95_kmh: float
    reversal_max_per_window: int
    reversal_max_t_s: float | None  # 最大回数に達した時刻（反転が 0 回なら None）
    time_over_limit_s: float  # |偏差| > 最大逸脱しきい値 だった時間
    episodes: tuple[DeviationEpisode, ...]
    limits: KpiSection

    @property
    def max_ok(self) -> bool:
        return self.max_abs_kmh <= self.limits.max_abs_deviation_kmh

    @property
    def p95_ok(self) -> bool:
        return self.p95_kmh <= self.limits.p95_deviation_kmh

    @property
    def reversal_ok(self) -> bool:
        return self.reversal_max_per_window <= self.limits.reversal_limit_per_window

    @property
    def passed(self) -> bool:
        return self.n_samples > 0 and self.max_ok and self.p95_ok and self.reversal_ok

    @property
    def passed_count(self) -> int:
        return sum((self.max_ok, self.p95_ok, self.reversal_ok))


def sample_interval_s(t_s: Sequence[float]) -> float:
    """行の時間間隔の代表値（中央値）。1 行以下なら 0.1s とみなす。"""
    if len(t_s) < 2:
        return 0.1
    return float(np.median(np.diff(np.asarray(t_s, dtype=float))))


def reversal_max(
    t_s: Sequence[float], deviation: Sequence[float], *, band_kmh: float, window_s: float
) -> tuple[int, float | None]:
    """任意の window_s 窓での符号反転の最大回数と、その回数に達した時刻を返す。

    数え方は本番 KPIMonitor と同じ。
    """
    last_sign = 0
    times: deque[float] = deque()
    best = 0
    best_t: float | None = None
    for t, dev in zip(t_s, deviation, strict=True):
        sign = 1 if dev > band_kmh else -1 if dev < -band_kmh else 0
        if sign == 0:
            continue
        if last_sign != 0 and sign != last_sign:
            times.append(t)
            while times and t - times[0] > window_s:
                times.popleft()
            if len(times) > best:
                best, best_t = len(times), t
        last_sign = sign
    return best, best_t


def find_episodes(
    t_s: Sequence[float], deviation: Sequence[float], *, limit_kmh: float
) -> list[DeviationEpisode]:
    """|偏差| > limit_kmh が連続した区間を時刻順に返す。"""
    episodes: list[DeviationEpisode] = []
    start: int | None = None
    for i, dev in enumerate([*deviation, 0.0]):  # 番兵で最後の区間を閉じる
        if abs(dev) > limit_kmh:
            if start is None:
                start = i
            continue
        if start is None:
            continue
        seg = range(start, i)
        peak = max(seg, key=lambda j: abs(deviation[j]))
        episodes.append(
            DeviationEpisode(
                start_s=t_s[start],
                end_s=t_s[i - 1],
                peak_kmh=deviation[peak],
                peak_t_s=t_s[peak],
            )
        )
        start = None
    return episodes


# ─────────────────────────────────────────────────────────────────────
# 速度帯のラベル（mode_report.py 4.3 節・compare_runs.py の帯別表と同じ区切り）
#
# mode_report.py が kpi.py を import している（逆向きの import は循環になるため不可）ので、
# 区切りの定義はこちら側に置き、mode_report.py 側から import する。
# ─────────────────────────────────────────────────────────────────────

SPEED_BANDS_KMH: tuple[float, ...] = (0.0, 20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 1000.0)


def speed_band_label(ref_kmh: float) -> str:
    """基準車速が属する帯のラベル（例: "20〜40"）。"""
    for lo, hi in zip(SPEED_BANDS_KMH, SPEED_BANDS_KMH[1:], strict=False):
        if ref_kmh < hi:
            return f"{lo:.0f}〜{hi:.0f}" if hi < 1000 else f"{lo:.0f}〜"
    return ""


def speed_band_order() -> list[str]:
    return [speed_band_label(lo) for lo in SPEED_BANDS_KMH[:-1]]


def compute_kpi(t_s: Sequence[float], deviation: Sequence[float], limits: KpiSection) -> KpiResult:
    """時刻 [s] と偏差（実車速 − 基準車速）[km/h] の列からプライマリー KPI を計算する。"""
    if len(t_s) != len(deviation):
        raise ValueError("t_s と deviation の長さが一致しません")
    if not deviation:
        return KpiResult(0, 0.0, 0.0, 0.0, 0, None, 0.0, (), limits)
    abs_dev = np.abs(np.asarray(deviation, dtype=float))
    i_max = int(np.argmax(abs_dev))
    reversals, reversal_t = reversal_max(
        t_s, deviation, band_kmh=limits.reversal_band_kmh, window_s=limits.reversal_window_s
    )
    over = int(np.count_nonzero(abs_dev > limits.max_abs_deviation_kmh))
    return KpiResult(
        n_samples=len(deviation),
        max_abs_kmh=float(abs_dev[i_max]),
        max_abs_t_s=float(t_s[i_max]),
        p95_kmh=float(np.percentile(abs_dev, 95)),
        reversal_max_per_window=reversals,
        reversal_max_t_s=reversal_t,
        time_over_limit_s=over * sample_interval_s(t_s),
        episodes=tuple(
            find_episodes(t_s, deviation, limit_kmh=limits.max_abs_deviation_kmh)
        ),
        limits=limits,
    )


# ─────────────────────────────────────────────────────────────────────
# ばたつき指標（手順3・FF のみ／C5 の実機走行で見えた 1.4Hz 付近の小刻みな往復を測る）
#
# 背景: 2026-09-21 の手順3（FF のみ・C5）の実機走行で、アクセル指令が 1.4Hz 付近で小刻みに
# 往復していた。走行のたびに同じ定義で比べられるように、この帯（CHATTER_BAND_HZ）の帯 RMS・
# 卓越周波数・符号反転・保持時間をプライマリー KPI とは別にまとめて測る。読み取り専用の解析で、
# 制御ロジック（ff_candidate.py / mode_drive.py）には一切手を入れない。
# ─────────────────────────────────────────────────────────────────────

#: 数値列（リストでも numpy 配列でもよい）。ばたつき指標はフィルタ後の配列を
#: そのまま次の関数へ渡すため、公開関数はどちらも受ける。
Floats = Sequence[float] | np.ndarray

CHATTER_BAND_HZ: tuple[float, float] = (0.9, 1.7)  # ばたつき帯
CHATTER_WINDOW_S: float = 15.0  # 帯 RMS を測る窓
CHATTER_MIN_SPEED_KMH: float = 5.0  # この基準車速を下回る行を含む窓は使わない（停車・発進は別現象）
CHATTER_HOLD_TOL_PCT: float = 0.25  # 「開度を保てた」とみなす幅
SMOOTH_CUTOFF_HZ: float = 0.9  # ばたつきだけを消すローパス（これより高い周波数を消す）


def _butter_sos_or_none(
    order: int, wn: float | Sequence[float], btype: str
) -> np.ndarray | None:
    """正規化周波数（0〜1、1 がナイキスト周波数）が範囲外＝サンプリングが粗すぎるときは
    None を返す（呼び出し側でゼロ配列にフォールバックするため）。"""
    from scipy import signal  # noqa: PLC0415

    wn_arr = np.atleast_1d(np.asarray(wn, dtype=float))
    if np.any(wn_arr <= 0.0) or np.any(wn_arr >= 1.0):
        return None
    return np.asarray(signal.butter(order, wn, btype=btype, output="sos"), dtype=float)


def bandpass(
    values: Floats, *, dt: float, band: tuple[float, float] = CHATTER_BAND_HZ
) -> np.ndarray:
    """2次 Butterworth バンドパスの零位相フィルタ（butter(output="sos") + sosfiltfilt）。

    行数が少なく零位相フィルタのパディング長を満たせない・dt が粗すぎて帯域を正規化できない
    ときは、ばたつき無しとみなしゼロ配列を返す（例外を投げない。呼び出し元の chatter_metrics
    が短い走行やテスト用の短い合成データでも落ちないようにするため）。
    """
    from scipy import signal  # noqa: PLC0415

    x = np.asarray(values, dtype=float)
    if x.size == 0 or dt <= 0.0:
        return np.zeros_like(x)
    nyquist_hz = 0.5 / dt
    sos = _butter_sos_or_none(2, [band[0] / nyquist_hz, band[1] / nyquist_hz], "bandpass")
    if sos is None:
        return np.zeros_like(x)
    try:
        return signal.sosfiltfilt(sos, x)
    except ValueError:
        return np.zeros_like(x)


def lowpass(
    values: Floats, *, dt: float, cutoff_hz: float = SMOOTH_CUTOFF_HZ
) -> np.ndarray:
    """2次 Butterworth ローパスの零位相フィルタ（butter(output="sos") + sosfiltfilt）。

    短すぎる行数のときのフォールバックは bandpass() と同じ（ゼロ配列）。
    """
    from scipy import signal  # noqa: PLC0415

    x = np.asarray(values, dtype=float)
    if x.size == 0 or dt <= 0.0:
        return np.zeros_like(x)
    nyquist_hz = 0.5 / dt
    sos = _butter_sos_or_none(2, cutoff_hz / nyquist_hz, "lowpass")
    if sos is None:
        return np.zeros_like(x)
    try:
        return signal.sosfiltfilt(sos, x)
    except ValueError:
        return np.zeros_like(x)


def band_rms_windows(
    t_s: Sequence[float],
    values: Floats,
    ref_kmh: Floats,
    *,
    band: tuple[float, float] = CHATTER_BAND_HZ,
    window_s: float = CHATTER_WINDOW_S,
    min_speed_kmh: float = CHATTER_MIN_SPEED_KMH,
) -> list[tuple[float, float]]:
    """非重複の window_s 窓ごとの (窓の平均基準車速 [km/h], バンドパス後の標準偏差) を返す。

    窓内の基準車速が 1 行でも min_speed_kmh を下回る窓は捨てる（停車・発進は別現象のため）。
    バンドパスは全区間に 1 回かけてから窓に切る（窓ごとにフィルタを掛け直すと、窓の境界ごとに
    フィルタの立ち上がり・立ち下がりが乗って過大評価になる）。
    """
    dt = sample_interval_s(t_s)
    n_per_window = round(window_s / dt) if dt > 0.0 else 0
    if n_per_window <= 0:
        return []
    filtered = bandpass(values, dt=dt, band=band)
    ref = np.asarray(ref_kmh, dtype=float)
    out: list[tuple[float, float]] = []
    n = len(filtered)
    for start in range(0, n - n_per_window + 1, n_per_window):
        end = start + n_per_window
        ref_win = ref[start:end]
        if np.any(ref_win < min_speed_kmh):
            continue
        out.append((float(np.mean(ref_win)), float(np.std(filtered[start:end]))))
    return out


def dominant_frequency(
    values: Floats, *, dt: float, lo: float = 0.3, hi: float = 3.0
) -> float:
    """scipy.signal.welch のパワーが lo〜hi Hz で最大になる周波数 [Hz]。

    nperseg は 2048（ただし len(values) の方が小さいときはそちらに丸める）。データが短すぎる・
    lo〜hi Hz にビンが 1 つも無いときは 0.0。
    """
    from scipy import signal  # noqa: PLC0415

    x = np.asarray(values, dtype=float)
    if x.size < 2 or dt <= 0.0:
        return 0.0
    nperseg = min(2048, x.size)
    freqs, power = signal.welch(x, fs=1.0 / dt, nperseg=nperseg)
    mask = (freqs >= lo) & (freqs <= hi)
    if not np.any(mask):
        return 0.0
    return float(freqs[mask][int(np.argmax(power[mask]))])


def direction_reversals(values: Floats) -> int:
    """差分の符号が入れ替わった回数（差分 0 の行は符号を持たないものとして飛ばす）。"""
    last_sign = 0
    count = 0
    prev: float | None = None
    for v in values:
        if prev is None:
            prev = v
            continue
        diff = v - prev
        prev = v
        if diff == 0.0:
            continue
        sign = 1 if diff > 0.0 else -1
        if last_sign != 0 and sign != last_sign:
            count += 1
        last_sign = sign
    return count


def pedal_reversals(values: Floats, *, hyst: float) -> int:
    """山（谷）から hyst 以上戻ったときだけ向きが変わったと認めて数える往復回数。

    direction_reversals は 1 パルスの上下も数えるが、こちらは hyst 未満の細かい上下を無視する。
    一方向に踏み増す・戻すだけなら 0 回。最初の向きが決まった時点では数えない。
    """
    direction = 0  # 0=未確定、1=上げ、-1=下げ
    extreme: float | None = None
    count = 0
    for v in values:
        if extreme is None:
            extreme = v
        elif direction == 0:
            if abs(v - extreme) >= hyst:
                direction = 1 if v > extreme else -1
                extreme = v
        elif direction == 1:
            if v > extreme:
                extreme = v
            elif extreme - v >= hyst:
                direction, extreme, count = -1, v, count + 1
        else:
            if v < extreme:
                extreme = v
            elif v - extreme >= hyst:
                direction, extreme, count = 1, v, count + 1
    return count


def _accel_runs(
    t_s: Sequence[float], accel_pct: Floats, ref_kmh: Floats, phase: Sequence[str],
    *, window_s: float = 0.0,
) -> list[tuple[int, list[float]]]:
    """評価対象（phase=="ACCEL"・基準車速が CHATTER_MIN_SPEED_KMH 以上）の連続区間を
    (窓番号, アクセル指令の列) で返す。window_s>0 のときは窓の境目でも区間を切る。"""
    runs: list[tuple[int, list[float]]] = []
    current: list[float] = []
    current_win = -1
    t0 = float(t_s[0]) if len(t_s) else 0.0
    for i in range(len(t_s)):
        win = int((t_s[i] - t0) // window_s) if window_s > 0.0 else 0
        if phase[i] == "ACCEL" and ref_kmh[i] >= CHATTER_MIN_SPEED_KMH:
            if current and win != current_win:
                runs.append((current_win, current))
                current = []
            current_win = win
            current.append(float(accel_pct[i]))
        elif current:
            runs.append((current_win, current))
            current = []
    if current:
        runs.append((current_win, current))
    return runs


def pedal_reversal_rates(
    t_s: Sequence[float], accel_pct: Floats, ref_kmh: Floats, phase: Sequence[str],
    *, hyst_pct: float, window_s: float, min_window_active_s: float,
) -> tuple[float, float]:
    """アクセル指令の往復回数を (全体 [回/s], window_s ごとの最大 [回/s]) で返す。

    分母は評価対象の行数×行間隔（アクセルを操作している時間）。ブレーキ区間・停車付近をまたいで
    数えない。窓は経過時間で重ならずに区切り、対象時間が min_window_active_s 未満の窓は除く。
    対象が無ければ (0.0, 0.0)。
    """
    if len(t_s) < 2:
        return 0.0, 0.0
    dt = sample_interval_s(t_s)
    whole = _accel_runs(t_s, accel_pct, ref_kmh, phase)
    total_s = sum(len(v) for _, v in whole) * dt
    overall = (
        sum(pedal_reversals(v, hyst=hyst_pct) for _, v in whole) / total_s if total_s > 0.0 else 0.0
    )
    count_by_win: dict[int, int] = {}
    rows_by_win: dict[int, int] = {}
    for win, v in _accel_runs(t_s, accel_pct, ref_kmh, phase, window_s=window_s):
        count_by_win[win] = count_by_win.get(win, 0) + pedal_reversals(v, hyst=hyst_pct)
        rows_by_win[win] = rows_by_win.get(win, 0) + len(v)
    rates = [
        count_by_win[w] / (rows_by_win[w] * dt)
        for w in rows_by_win
        if rows_by_win[w] * dt >= min_window_active_s
    ]
    return overall, max(rates, default=0.0)


def hold_durations_s(values: Floats, *, dt: float, tol: float) -> list[float]:
    """値が「直前に保持した値」から ±tol を超えるまで留まった連続時間 [s] の一覧。

    超えたらその時点の値を新しい保持値にする。連続 n 行が同じ保持値の範囲内なら、
    その区間の経過時間は最初の行から最後の行までの (n - 1) * dt（1 行だけなら 0s、
    次の行で即座に tol を超えて抜けた区間）。
    """
    vals = list(values)
    if not vals:
        return []
    durations: list[float] = []
    hold_value = vals[0]
    count = 1
    for v in vals[1:]:
        if abs(v - hold_value) <= tol:
            count += 1
        else:
            durations.append((count - 1) * dt)
            hold_value = v
            count = 1
    durations.append((count - 1) * dt)
    return durations


@dataclass(frozen=True)
class ChatterMetrics:
    """1.4Hz 付近のばたつきをまとめて測る指標（読み取り専用解析。プライマリー KPI とは別枠）。"""

    dominant_hz: float  # 偏差（実車速 − 基準車速）の卓越周波数。理由は chatter_metrics の注
    speed_band_rms_kmh: float  # 実車速の帯 RMS（窓の中央値）
    deviation_band_rms_kmh: float
    ref_band_rms_kmh: float  # 基準車速の帯 RMS（比較用。小さいほど「目標のせいではない」）
    accel_band_rms_pct: float  # アクセル指令の帯 RMS
    by_speed_band: dict[str, float]  # 基準車速帯別の実車速 帯 RMS 中央値（キーは "20〜40" など）
    accel_reversals_per_s: float  # phase=="ACCEL" の行のみ
    accel_travel_pct: float  # phase=="ACCEL" の行の Σ|Δ指令|
    accel_active_s: float
    brake_reversals_per_s: float  # phase=="BRAKE" の行のみ
    brake_travel_pct: float
    brake_active_s: float
    hold_median_s: float  # アクセル指令を ±CHATTER_HOLD_TOL_PCT で保てた時間
    hold_p90_s: float
    pedal_reversal_per_s: float  # KPI: アクセル指令の往復回数（全体）
    pedal_reversal_window_max_per_s: float  # KPI: 同・window_s ごとの最大
    reversal_raw: int  # 既存 KPI の符号反転（生の偏差）
    reversal_smoothed: int  # 偏差を lowpass した後の符号反転
    p95_raw_kmh: float
    p95_smoothed_kmh: float


def _median_band_rms(windows: Sequence[tuple[float, float]]) -> float:
    return float(np.median([rms for _, rms in windows])) if windows else 0.0


def _travel_pct(values: Floats) -> float:
    arr = np.asarray(values, dtype=float)
    return float(np.sum(np.abs(np.diff(arr)))) if arr.size >= 2 else 0.0


def chatter_metrics(
    t_s: Sequence[float],
    ref_kmh: Sequence[float],
    actual_kmh: Sequence[float],
    accel_pct: Sequence[float],
    brake_pct: Sequence[float],
    phase: Sequence[str],
    limits: KpiSection,
) -> ChatterMetrics:
    """ばたつき指標をまとめて計算する。

    実開度 mm は ModeRow に無いため、すべて指令ベース（%）で測る。行が少なすぎて窓が
    1 つも取れない等のときは、該当項目を例外を投げずに 0.0（もしくは空 dict / 0 回）に落とす。

    limits は既存の KpiSection（reversal_band_kmh・reversal_window_s と pedal_reversal_* を使う。
    最大逸脱・p95 のしきい値は使わない）。reversal_raw / reversal_smoothed は既存の
    reversal_max() をそのまま再利用し、しきい値の定義を二重実装しない。
    """
    n = len(t_s)
    if n < 2:
        return ChatterMetrics(
            dominant_hz=0.0, speed_band_rms_kmh=0.0, deviation_band_rms_kmh=0.0,
            ref_band_rms_kmh=0.0, accel_band_rms_pct=0.0, by_speed_band={},
            accel_reversals_per_s=0.0, accel_travel_pct=0.0, accel_active_s=0.0,
            brake_reversals_per_s=0.0, brake_travel_pct=0.0, brake_active_s=0.0,
            hold_median_s=0.0, hold_p90_s=0.0,
            pedal_reversal_per_s=0.0, pedal_reversal_window_max_per_s=0.0,
            reversal_raw=0, reversal_smoothed=0,
            p95_raw_kmh=0.0, p95_smoothed_kmh=0.0,
        )
    dt = sample_interval_s(t_s)
    ref = np.asarray(ref_kmh, dtype=float)
    actual = np.asarray(actual_kmh, dtype=float)
    deviation = actual - ref

    speed_windows = band_rms_windows(t_s, actual, ref_kmh)
    dev_windows = band_rms_windows(t_s, deviation, ref_kmh)
    ref_windows = band_rms_windows(t_s, ref_kmh, ref_kmh)
    accel_windows = band_rms_windows(t_s, accel_pct, ref_kmh)

    by_speed_band: dict[str, float] = {}
    for name in speed_band_order():
        rms_vals = [rms for mean_ref, rms in speed_windows if speed_band_label(mean_ref) == name]
        if rms_vals:
            by_speed_band[name] = float(np.median(rms_vals))

    accel_vals = [accel_pct[i] for i in range(n) if phase[i] == "ACCEL"]
    brake_vals = [brake_pct[i] for i in range(n) if phase[i] == "BRAKE"]
    accel_active_s = len(accel_vals) * dt
    brake_active_s = len(brake_vals) * dt

    holds = hold_durations_s(accel_vals, dt=dt, tol=CHATTER_HOLD_TOL_PCT)

    reversal_raw, _ = reversal_max(
        t_s, deviation.tolist(),
        band_kmh=limits.reversal_band_kmh, window_s=limits.reversal_window_s,
    )
    smoothed_dev = lowpass(deviation, dt=dt)
    reversal_smoothed, _ = reversal_max(
        t_s, smoothed_dev.tolist(),
        band_kmh=limits.reversal_band_kmh, window_s=limits.reversal_window_s,
    )

    pedal_overall, pedal_window_max = pedal_reversal_rates(
        t_s, accel_pct, ref, phase, hyst_pct=limits.pedal_reversal_hyst_pct,
        window_s=limits.pedal_reversal_window_s,
        min_window_active_s=limits.pedal_reversal_min_window_s,
    )

    return ChatterMetrics(
        # 実車速そのものに welch を掛けると WLTP の加減速（低周波・大振幅）に埋もれてしまうため、
        # 基準車速を差し引いた偏差（＝実車速の細かい揺れそのもの）で卓越周波数を出す。
        dominant_hz=dominant_frequency(deviation, dt=dt),
        speed_band_rms_kmh=_median_band_rms(speed_windows),
        deviation_band_rms_kmh=_median_band_rms(dev_windows),
        ref_band_rms_kmh=_median_band_rms(ref_windows),
        accel_band_rms_pct=_median_band_rms(accel_windows),
        by_speed_band=by_speed_band,
        accel_reversals_per_s=(
            direction_reversals(accel_vals) / accel_active_s if accel_active_s > 0.0 else 0.0
        ),
        accel_travel_pct=_travel_pct(accel_vals),
        accel_active_s=accel_active_s,
        brake_reversals_per_s=(
            direction_reversals(brake_vals) / brake_active_s if brake_active_s > 0.0 else 0.0
        ),
        brake_travel_pct=_travel_pct(brake_vals),
        brake_active_s=brake_active_s,
        hold_median_s=float(np.median(holds)) if holds else 0.0,
        hold_p90_s=float(np.percentile(holds, 90)) if holds else 0.0,
        pedal_reversal_per_s=pedal_overall,
        pedal_reversal_window_max_per_s=pedal_window_max,
        reversal_raw=reversal_raw,
        reversal_smoothed=reversal_smoothed,
        p95_raw_kmh=float(np.percentile(np.abs(deviation), 95)) if deviation.size else 0.0,
        p95_smoothed_kmh=(
            float(np.percentile(np.abs(smoothed_dev), 95)) if smoothed_dev.size else 0.0
        ),
    )
