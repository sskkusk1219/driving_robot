"""研究環境の物理定数推定（本番 `estimate_dynamics_params` の移植）。

連続走行ログから、停車保持ブレーキ・クリープ車速/加速率・惰行減速カーブ・ペダルゲイン曲線・
不感帯を推定する。`src/domain/model_training.py` のうち `estimate_dynamics_params` と
その下請けだけを `tests/research` に持ち込んだもの（`tests/` だけで完結させるため。
ProblemReport_20260924）。
**ロジックは本番と同じ**。推定器・特徴量（`train_inverse_model` 側）は `ff_model.py` にある。
"""

from __future__ import annotations

from dataclasses import replace

import numpy as np

from tests.research.ff_model import DEFAULT_DT_S, STOP_SPEED_KMH, group_by_session
from tests.research.research_types import DriveLog, FeedforwardParams, coast_decel_at

# 物理定数推定用のしきい値（本番と同値。STOP_SPEED_KMH は ff_model のものを使う）
CREEP_STEADY_TOL_KMHS: float = 0.3  # クリープ定常判定の |加速度| しきい値
MIN_OBS_SAMPLES: int = 5  # 各定数を上書きするのに必要な最小観測サンプル数

# 不感帯（accel/brake）推定用: 開度→応答(加速度)曲線をビン分割し、開度ゼロ近傍の
# ベースライン応答をマージン以上上回る最初のビンの下端を不感帯境界とみなす
# （学習運転の ACCEL_DEADBAND_PROBE / BRAKE_HOLD 低開度段が主なサンプル源）。
DEADBAND_BIN_WIDTH_PCT: float = 0.5  # 開度ビン幅
DEADBAND_SCAN_MAX_PCT: float = 10.0  # この開度まで探索（探索上限＝クランプ上限を兼ねる）
DEADBAND_ONSET_MARGIN_KMHS: float = 0.3  # ベースラインを上回る「応答あり」判定マージン
DEADBAND_MIN_BIN_SAMPLES: int = 5  # ビンを採用するのに必要な最小サンプル数


def _median_or_none(values: list[float]) -> float | None:
    """十分なサンプルがあれば中央値、なければ None を返す。"""
    return float(np.median(values)) if len(values) >= MIN_OBS_SAMPLES else None


def _estimate_onset_deadband_pct(openings: np.ndarray, response: np.ndarray) -> float | None:
    """開度→応答曲線から不感帯（無反応域）の境界開度を推定する。

    開度を `DEADBAND_BIN_WIDTH_PCT` 幅でビン分割し、開度ゼロ近傍ビン（bin 0）の中央値応答を
    「無反応時ベースライン」とする。それを `DEADBAND_ONSET_MARGIN_KMHS` 以上上回る最初のビンの
    下端開度を不感帯境界として返す。各ビンは `DEADBAND_MIN_BIN_SAMPLES` 以上のサンプルを要求し
    （統計的信頼性）、満たさないビンはスキップする。ベースラインが求まらない、または境界が
    見つからない場合は None を返す（呼び出し元は既存値を保持する）。

    Args:
        openings: 開度サンプル列 [%]（0 以上、`DEADBAND_SCAN_MAX_PCT` 程度までを想定）
        response: 対応する応答（アクセルは dv、ブレーキは -dv）[km/h/s]。大きいほど反応あり。
    """
    if len(openings) == 0:
        return None

    n_bins = int(DEADBAND_SCAN_MAX_PCT / DEADBAND_BIN_WIDTH_PCT) + 1
    bin_medians: list[float | None] = []
    for i in range(n_bins):
        lo = i * DEADBAND_BIN_WIDTH_PCT
        hi = (i + 1) * DEADBAND_BIN_WIDTH_PCT
        mask = (openings >= lo) & (openings < hi)
        if int(np.count_nonzero(mask)) < DEADBAND_MIN_BIN_SAMPLES:
            bin_medians.append(None)
            continue
        bin_medians.append(float(np.median(response[mask])))

    baseline = bin_medians[0]
    if baseline is None:
        return None

    for i in range(1, n_bins):
        m = bin_medians[i]
        if m is None:
            continue
        if m >= baseline + DEADBAND_ONSET_MARGIN_KMHS:
            return i * DEADBAND_BIN_WIDTH_PCT
    return None


# 惰行減速カーブの速度ビン幅 [km/h] と各ビンの最少サンプル数。コーストダウン完走
# （coast_timeout_s=90）で 5〜cap の全域が埋まる想定。ビンが 2 個未満なら未同定
#（呼び出し元が既存カーブ or engine_brake_decel_kmhs 定数へフォールバック）。
COAST_CURVE_BIN_KMH: float = 10.0
COAST_CURVE_MIN_BIN_SAMPLES: int = 8


def _estimate_coast_decel_curve(
    speeds: np.ndarray, decels: np.ndarray
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """惰行サンプル（速度・減速量[正値]）から速度依存の惰行減速カーブを推定する。

    速度を COAST_CURVE_BIN_KMH 幅でビン分割し、各ビンの中央値をビン中心速度に対応づける。
    サンプル不足のビンはスキップ。有効ビンが 2 個未満なら空タプル（未同定＝呼び出し元が
    フォールバック）。単一定数 engine_brake_decel_kmhs では表現できない速度依存性
    （低速ほどエンジンブレーキが効く等）を捉え、フェーズ分類（pedal_plan.coast_accel）と
    FF レジーム合成（feedforward）の基準線 coast_decel_at に供給する。
    """
    if len(speeds) == 0:
        return (), ()
    v_max = float(speeds.max())
    n_bins = int(v_max / COAST_CURVE_BIN_KMH) + 1
    out_speeds: list[float] = []
    out_decels: list[float] = []
    for i in range(n_bins):
        lo = i * COAST_CURVE_BIN_KMH
        hi = lo + COAST_CURVE_BIN_KMH
        mask = (speeds >= lo) & (speeds < hi)
        if int(np.count_nonzero(mask)) < COAST_CURVE_MIN_BIN_SAMPLES:
            continue
        out_speeds.append(lo + COAST_CURVE_BIN_KMH / 2.0)
        out_decels.append(float(np.median(decels[mask])))
    if len(out_speeds) < 2:
        return (), ()
    return tuple(out_speeds), tuple(out_decels)


# ペダルゲイン曲線の同定パラメータ。開度→加速度の比は低開度ほど S/N が悪い（CAN 車速の
# 10Hz 隣接差 std≈0.25km/h ＝ 加速度換算 2.5km/h/s のノイズが乗る）ため、不感帯超で
# PEDAL_GAIN_MIN_OPENING_PCT 以上の「よく効いている」サンプルだけからゲインを推定し、
# 低開度域へは原点通過の線形として内挿する。プラントの一次遅れ（τ≈2s）で踏み込み直後は
# 加速度が立ち上がりきらずゲインを過小評価するので、開度が直近 PEDAL_GAIN_STEADY_S で
# ほぼ動いていないサンプルに限る。
PEDAL_GAIN_MIN_OPENING_PCT: float = 5.0  # 不感帯からこれ以上踏み込んだサンプルのみ使う
PEDAL_GAIN_STEADY_S: float = 1.0  # 開度が定常とみなす遡り時間 [s]
PEDAL_GAIN_STEADY_TOL_PCT: float = 1.0  # 上記区間での開度変化の許容幅 [%]


def _estimate_pedal_gain_curve(
    speeds: np.ndarray, gains: np.ndarray
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    """ペダルゲインのサンプル（速度・ゲイン[正値]）から速度依存のゲイン曲線を推定する。

    速度を COAST_CURVE_BIN_KMH 幅でビン分割し、各ビンの中央値をビン中心速度に対応づける
    （_estimate_coast_decel_curve と同じ形式・粒度）。非正のゲイン（ノイズで符号が反転した
    サンプル）は捨てる。有効ビンが 2 個未満なら空タプル（未同定＝呼び出し元がフォールバック）。
    """
    if len(speeds) == 0:
        return (), ()
    valid = gains > 0.0
    speeds = speeds[valid]
    gains = gains[valid]
    if len(speeds) == 0:
        return (), ()
    v_max = float(speeds.max())
    n_bins = int(v_max / COAST_CURVE_BIN_KMH) + 1
    out_speeds: list[float] = []
    out_gains: list[float] = []
    for i in range(n_bins):
        lo = i * COAST_CURVE_BIN_KMH
        hi = lo + COAST_CURVE_BIN_KMH
        mask = (speeds >= lo) & (speeds < hi)
        if int(np.count_nonzero(mask)) < COAST_CURVE_MIN_BIN_SAMPLES:
            continue
        out_speeds.append(lo + COAST_CURVE_BIN_KMH / 2.0)
        out_gains.append(float(np.median(gains[mask])))
    if len(out_speeds) < 2:
        return (), ()
    return tuple(out_speeds), tuple(out_gains)


def _merge_pedal_gain_curves(
    speeds_a: tuple[float, ...],
    gains_a: tuple[float, ...],
    speeds_b: tuple[float, ...],
    gains_b: tuple[float, ...],
) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    """accel/brake のゲイン曲線を単一の速度グリッドへ揃える。

    FeedforwardParams は速度グリッドを 1 本しか持たない（惰行カーブと同じ構造）。両側とも
    同じビン幅で推定するが、埋まるビンは踏み方によってずれるため、和集合のグリッドへ線形
    補間（範囲外は端点クランプ）して載せ替える。片側が未同定ならそちらは空タプルのまま返し、
    利用側（pedal_gain_at）がその向きだけ「未同定」と判定できるようにする。
    """
    if not speeds_a and not speeds_b:
        return (), (), ()
    grid = tuple(sorted(set(speeds_a) | set(speeds_b)))
    merged_a = (
        tuple(float(np.interp(v, speeds_a, gains_a)) for v in grid) if speeds_a else ()
    )
    merged_b = (
        tuple(float(np.interp(v, speeds_b, gains_b)) for v in grid) if speeds_b else ()
    )
    return grid, merged_a, merged_b


def _steady_opening_mask(openings: np.ndarray, dt: float) -> np.ndarray:
    """開度が直近 PEDAL_GAIN_STEADY_S でほぼ動いていないサンプルの真偽列を返す。

    踏み込み・戻し直後はプラントの一次遅れで加速度が立ち上がりきっておらず、そのまま
    ゲインを取ると過小評価になる（結果として解析プランが踏み過ぎになる）。
    """
    n = len(openings)
    back = max(1, int(round(PEDAL_GAIN_STEADY_S / dt)))
    mask = np.zeros(n, dtype=bool)
    if n > back:
        mask[back:] = np.abs(openings[back:] - openings[:-back]) < PEDAL_GAIN_STEADY_TOL_PCT
    return mask


def estimate_dynamics_params(logs: list[DriveLog], current: FeedforwardParams) -> FeedforwardParams:
    """連続走行ログから物理定数を推定し、十分なサンプルがある項目のみ上書きする。

    観測が不足する項目は current の値を保持する。不感帯（accel/brake）は、学習運転の
    ACCEL_DEADBAND_PROBE（低アクセル開度の意図的保持）・BRAKE_HOLD の低開度段から得られる
    開度→応答曲線のオンセット検出で推定する（`_estimate_onset_deadband_pct`）。これらの
    専用プローブが無い旧ログのみの場合は十分なビンが埋まらず None となり、既存値を保持する。
    """
    accel_db = current.accel_deadband_pct
    brake_db = current.brake_deadband_pct

    stop_brakes: list[float] = []
    creep_speeds: list[float] = []
    eng_decels: list[float] = []
    eng_speeds: list[float] = []
    creep_rates: list[float] = []
    accel_scan_openings: list[float] = []
    accel_scan_dv: list[float] = []
    brake_scan_openings: list[float] = []
    brake_scan_decel: list[float] = []
    gain_accel_speeds: list[float] = []
    gain_accel_values: list[float] = []
    gain_brake_speeds: list[float] = []
    gain_brake_values: list[float] = []

    for session_logs in group_by_session(logs):
        if len(session_logs) < 2:
            continue
        speed = np.clip(
            np.array([lg.actual_speed_kmh for lg in session_logs], dtype=float), 0.0, None
        )
        accel = np.array([lg.accel_opening for lg in session_logs], dtype=float)
        brake = np.array([lg.brake_opening for lg in session_logs], dtype=float)

        epochs = np.array([lg.timestamp.timestamp() for lg in session_logs])
        d = np.diff(epochs)
        d = d[d > 0.0]
        dt = float(np.median(d)) if len(d) > 0 else DEFAULT_DT_S
        if dt <= 0.0:
            dt = DEFAULT_DT_S

        dv = np.diff(speed) / dt  # i→i+1 の加速度 [km/h/s]（長さ n-1）
        sp = speed[:-1]  # dv に揃えた始点速度
        # deadband_pct 以下は「ペダルオフ」とみなす。strict < だと deadband=0.0
        # （合法値）のとき常に False になり物理定数推定が無音で全滅する（D5 レビュー指摘）。
        pedal_off = (accel[:-1] <= accel_db) & (brake[:-1] <= brake_db)

        # 停車保持ブレーキ: 停車中にかけているブレーキ開度
        m_stop = (speed < STOP_SPEED_KMH) & (brake >= brake_db)
        stop_brakes.extend(brake[m_stop].tolist())

        # クリープ車速: ペダルオフで定常（|dv| 小）かつ動いている
        m_creep = pedal_off & (sp > STOP_SPEED_KMH) & (np.abs(dv) < CREEP_STEADY_TOL_KMHS)
        creep_speeds.extend(sp[m_creep].tolist())

        # エンジンブレーキ減速量: ペダルオフ・クリープ超・減速中（惰行減速カーブ用に速度も保持）
        m_eng = pedal_off & (sp > current.creep_speed_kmh) & (dv < 0.0)
        eng_decels.extend((-dv[m_eng]).tolist())
        eng_speeds.extend(sp[m_eng].tolist())

        # クリープ加速率: ペダルオフ・低速・加速中
        m_rate = pedal_off & (sp >= STOP_SPEED_KMH) & (sp < current.creep_speed_kmh) & (dv > 0.0)
        creep_rates.extend(dv[m_rate].tolist())

        # 不感帯推定用スキャンサンプル: 他ペダルオフ・探索上限以下の開度域で開度→応答を収集
        m_accel_scan = (brake[:-1] <= brake_db) & (accel[:-1] <= DEADBAND_SCAN_MAX_PCT)
        accel_scan_openings.extend(accel[:-1][m_accel_scan].tolist())
        accel_scan_dv.extend(dv[m_accel_scan].tolist())

        m_brake_scan = (accel[:-1] <= accel_db) & (brake[:-1] <= DEADBAND_SCAN_MAX_PCT)
        brake_scan_openings.extend(brake[:-1][m_brake_scan].tolist())
        brake_scan_decel.extend((-dv[m_brake_scan]).tolist())

        # ペダルゲイン: 惰行基準からの加速度差 ÷ 不感帯超の開度。片ペダルのみ・十分踏んで
        # いる・開度が定常のサンプルに限る。クリープ域は惰行基準が別式（coast_accel）なので
        # 除外する。惰行基準は既存カーブ（前回同定値）で引く——今回の同定結果を使うと
        # ビンの埋まり方でゲインが揺れるため、参照は安定した current 側に固定する。
        a_coast = np.array(
            [-coast_decel_at(current, float(x)) for x in sp], dtype=float
        )
        steady_accel = _steady_opening_mask(accel, dt)[:-1]
        steady_brake = _steady_opening_mask(brake, dt)[:-1]
        m_gain_a = (
            (brake[:-1] <= brake_db)
            & (accel[:-1] - accel_db >= PEDAL_GAIN_MIN_OPENING_PCT)
            & (sp > current.creep_speed_kmh)
            & steady_accel
        )
        if np.any(m_gain_a):
            gain_accel_speeds.extend(sp[m_gain_a].tolist())
            gain_accel_values.extend(
                ((dv[m_gain_a] - a_coast[m_gain_a]) / (accel[:-1][m_gain_a] - accel_db)).tolist()
            )
        m_gain_b = (
            (accel[:-1] <= accel_db)
            & (brake[:-1] - brake_db >= PEDAL_GAIN_MIN_OPENING_PCT)
            & (sp > current.creep_speed_kmh)
            & steady_brake
        )
        if np.any(m_gain_b):
            gain_brake_speeds.extend(sp[m_gain_b].tolist())
            gain_brake_values.extend(
                ((a_coast[m_gain_b] - dv[m_gain_b]) / (brake[:-1][m_gain_b] - brake_db)).tolist()
            )

    new_stop = _median_or_none(stop_brakes)
    new_creep_speed = _median_or_none(creep_speeds)
    new_eng = _median_or_none(eng_decels)
    new_rate = _median_or_none(creep_rates)
    curve_speeds, curve_decels = _estimate_coast_decel_curve(
        np.array(eng_speeds), np.array(eng_decels)
    )
    gain_speeds_a, gain_values_a = _estimate_pedal_gain_curve(
        np.array(gain_accel_speeds), np.array(gain_accel_values)
    )
    gain_speeds_b, gain_values_b = _estimate_pedal_gain_curve(
        np.array(gain_brake_speeds), np.array(gain_brake_values)
    )
    # ペダルゲインは accel/brake で共通の速度グリッドに載せる（FeedforwardParams が
    # 単一グリッドを持つ）。片側しか埋まらない場合は埋まった側のグリッドへ他方を補間する。
    gain_speeds, gain_accel, gain_brake = _merge_pedal_gain_curves(
        gain_speeds_a, gain_values_a, gain_speeds_b, gain_values_b
    )
    new_accel_db = _estimate_onset_deadband_pct(
        np.array(accel_scan_openings), np.array(accel_scan_dv)
    )
    new_brake_db = _estimate_onset_deadband_pct(
        np.array(brake_scan_openings), np.array(brake_scan_decel)
    )

    return replace(
        current,
        stop_brake_opening_pct=(
            new_stop if new_stop is not None else current.stop_brake_opening_pct
        ),
        creep_speed_kmh=(
            new_creep_speed if new_creep_speed is not None else current.creep_speed_kmh
        ),
        engine_brake_decel_kmhs=(
            new_eng if new_eng is not None else current.engine_brake_decel_kmhs
        ),
        coast_decel_speeds_kmh=(
            curve_speeds if curve_speeds else current.coast_decel_speeds_kmh
        ),
        coast_decel_kmhs=(curve_decels if curve_decels else current.coast_decel_kmhs),
        pedal_gain_speeds_kmh=(gain_speeds if gain_speeds else current.pedal_gain_speeds_kmh),
        accel_gain_kmhs_per_pct=(
            gain_accel if gain_speeds else current.accel_gain_kmhs_per_pct
        ),
        brake_gain_kmhs_per_pct=(
            gain_brake if gain_speeds else current.brake_gain_kmhs_per_pct
        ),
        creep_rate_kmhs=(new_rate if new_rate is not None else current.creep_rate_kmhs),
        accel_deadband_pct=(
            new_accel_db if new_accel_db is not None else current.accel_deadband_pct
        ),
        brake_deadband_pct=(
            new_brake_db if new_brake_db is not None else current.brake_deadband_pct
        ),
    )
