"""走行セッションの車速追従 KPI を drive_logs から解析するスクリプト。

Stage A/B/C の実機検証ゲート判定に使う。プライマリー KPI（p95≤0.4 / max≤1.0 /
符号反転≤1回/5s窓）に加え、Stage A の系統ラグ（実車速の基準に対する進み/遅れ）を
相互相関のラグ走査で確認する。

使い方:
    python -m scripts.analyze_session <session_id> [--latest] [--dsn ...]
    python -m scripts.analyze_session --latest auto      # 直近の auto セッション

DATABASE_URL 環境変数（既定 postgresql://localhost/driving_robot）を参照する。
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
from typing import Any

import asyncpg
import numpy as np

_HARD_LIMIT_KMH = 1.0
# kpi_monitor.KPI_P95_LIMIT_KMH と同値に保つこと（2026-09-09: 0.2→0.4。実定数と表示が
# 食い違っており、合格しているのに NG 表示になっていた）。
_P95_LIMIT_KMH = 0.4
_REVERSAL_WINDOW_S = 5.0
# kpi_monitor.SIGN_REVERSAL_AMPLITUDE_KMH と同値に保つこと（2026-09-09: 0.05→0.3。
# CAN 車速の 10Hz 隣接差 std が 0.22-0.25km/h あり、0.05 は測定ノイズを数えていた）。
_SIGN_REVERSAL_AMPLITUDE_KMH = 0.3
# ペダル「踏んでいる」しきい値 [%]（kpi_monitor._PEDAL_ON_THRESHOLD_PCT と揃える）。
_PEDAL_ON_THRESHOLD_PCT = 0.5
# B-7' ゲート目安: 110-135km/h 帯の accel ON 立ち上がり回数/min（現状 36、目標 ≤10）。
_PEDAL_ON_GATE_PER_MIN = 10.0


async def _fetch_logs(dsn: str, session_id: str) -> list[asyncpg.Record]:
    conn = await asyncpg.connect(dsn)
    try:
        return await conn.fetch(
            """
            SELECT timestamp, ref_speed_kmh, actual_speed_kmh, accel_opening, brake_opening,
                   plan_effort_pct, trim_effort_pct, applied_effort_pct, phase
            FROM drive_logs WHERE session_id = $1 ORDER BY timestamp
            """,
            session_id,
        )
    finally:
        await conn.close()


async def _fetch_profile_params(dsn: str, session_id: str) -> dict | None:
    """セッションのプロファイルの車両定数（クリープ・エンジンブレーキ）を取得する。

    不要切替の「軌跡要求切替」を pedal_plan の分類器で計算するのに使う。取れなければ None。
    """
    conn = await asyncpg.connect(dsn)
    try:
        row = await conn.fetchrow(
            "SELECT p.feedforward_params FROM drive_sessions s "
            "JOIN vehicle_profiles p ON p.id = s.profile_id WHERE s.id = $1",
            session_id,
        )
    finally:
        await conn.close()
    if row is None or row["feedforward_params"] is None:
        return None
    raw = row["feedforward_params"]
    return json.loads(raw) if isinstance(raw, str) else dict(raw)


async def _latest_session(dsn: str, run_type: str | None) -> str | None:
    conn = await asyncpg.connect(dsn)
    try:
        if run_type:
            row = await conn.fetchrow(
                "SELECT id FROM drive_sessions WHERE run_type = $1 "
                "ORDER BY started_at DESC LIMIT 1",
                run_type,
            )
        else:
            row = await conn.fetchrow(
                "SELECT id FROM drive_sessions ORDER BY started_at DESC LIMIT 1"
            )
        return str(row["id"]) if row else None
    finally:
        await conn.close()


def _switch_metrics(
    records: list[asyncpg.Record], t: np.ndarray, ref: np.ndarray, dt: float, params: dict | None
) -> dict[str, float]:
    """アクセル⇔ブレーキ交互踏み（実測）と軌跡要求切替、その差＝不要切替を計算する。

    実測: 調停後のアクセル/ブレーキ開度から、異種ペダルが 2s 以内に踏まれた回数。
    要求: 基準軌跡を pedal_plan の分類器でフェーズ分けした DRIVE⇄BRAKE 切替（マージ後）。
    不要切替 = 実測 − 要求（回/min）。人間的な操作なら ≈0。params が無ければ要求は NaN。
    """
    acc = np.array([r["accel_opening"] for r in records], dtype=float)
    brk = np.array([r["brake_opening"] for r in records], dtype=float)
    dur_min = (t[-1] / 60.0) if t[-1] > 0 else 0.0

    def count_switches(pedal_state: np.ndarray) -> int:
        sw = 0
        last = 0
        last_t: float | None = None
        for i, s in enumerate(pedal_state):
            if s != 0 and s != last:
                if last != 0 and last_t is not None and (t[i] - last_t) < 2.0:
                    sw += 1
                last, last_t = int(s), float(t[i])
            elif s != 0:
                last_t = float(t[i])
        return sw

    measured_state = np.where(
        acc > _PEDAL_ON_THRESHOLD_PCT, 1, np.where(brk > _PEDAL_ON_THRESHOLD_PCT, -1, 0)
    )
    measured = count_switches(measured_state)
    measured_per_min = measured / dur_min if dur_min > 0 else 0.0

    required_per_min = float("nan")
    if params is not None:
        from src.domain.control.pedal_plan import (
            classify_phases,
            merge_micro_phases,
            required_accel,
        )
        from src.models.profile import FeedforwardParams

        ffp = FeedforwardParams(
            creep_speed_kmh=params.get("creep_speed_kmh", 7.0),
            creep_rate_kmhs=params.get("creep_rate_kmhs", 0.5),
            engine_brake_decel_kmhs=params.get("engine_brake_decel_kmhs", 1.0),
        )
        a_req = required_accel(ref, dt)
        phases = merge_micro_phases(classify_phases(ref, a_req, ffp), dt)
        from src.domain.control.pedal_plan import PlanPhase

        req_state = np.array(
            [1 if p == PlanPhase.DRIVE else (-1 if p == PlanPhase.BRAKE else 0) for p in phases]
        )
        required = count_switches(req_state)
        required_per_min = required / dur_min if dur_min > 0 else 0.0

    excess = (measured_per_min - required_per_min) if params is not None else float("nan")
    return {
        "seesaw_measured_per_min": measured_per_min,
        "seesaw_required_per_min": required_per_min,
        "seesaw_excess_per_min": excess,
    }


# 速度帯の区切り [km/h]（20km/h 刻み）。最大逸脱がどの速度域で出ているかを見る
# （実機 5ac4f31d では 40-60km/h 帯が p95 3.77 と突出し、軌跡頂点のエピソードがここにある）。
_SPEED_BANDS_KMH = (0.0, 20.0, 40.0, 60.0, 80.0, 100.0, 120.0, 1e9)
# エピソード表に出す上位件数（違反時間の降順）。
_TOP_EPISODES = 6


def _sample_dts(t: np.ndarray) -> np.ndarray:
    """隣接サンプル間隔 [s]（kpi_monitor と同じ 0〜0.5s クランプ）。長さは len(t)-1。"""
    return np.clip(np.diff(t), 1e-6, 0.5)


def _violation_episodes(
    t: np.ndarray, dev: np.ndarray, phases: np.ndarray, dts: np.ndarray
) -> list[dict[str, float | str]]:
    """|dev| > 1.0 km/h の連続区間をエピソードとして切り出す。

    最大逸脱 KPI（PRD: 全走行区間で 1.0km/h を超えないこと・例外なし）は max だけ見ても
    改善が測れない。実機 13 反復では max が 3.82〜5.19 で横ばいのまま p95 だけが半減しており、
    「何回・何秒・どこで・どれだけ超えたか」に分解しないと打ち手の効果が判定できなかった
    （docs/Problem/引き継ぎ20260909.md 3-2, 3-3）。

    Returns:
        エピソードの辞書リスト（開始/終了時刻、継続時間、符号つきピーク偏差、
        超過積分 ∫(|dev|−1.0)dt、ピーク時のフェーズ、ピーク時の基準速度）。
    """
    over = np.abs(dev) > _HARD_LIMIT_KMH
    episodes: list[dict[str, float | str]] = []
    i = 0
    n = len(over)
    while i < n:
        if not over[i]:
            i += 1
            continue
        j = i
        while j + 1 < n and over[j + 1]:
            j += 1
        seg = slice(i, j + 1)
        # 区間内の各サンプルが代表する時間幅（末尾サンプルは直前の間隔を流用）。
        seg_dts = dts[i : j + 1] if j + 1 <= len(dts) else dts[i:]
        if seg_dts.size == 0:
            seg_dts = np.array([float(np.median(dts))])
        peak_idx = i + int(np.argmax(np.abs(dev[seg])))
        integral = float(
            np.sum((np.abs(dev[seg])[: seg_dts.size] - _HARD_LIMIT_KMH) * seg_dts)
        )
        episodes.append(
            {
                "t_start": float(t[i]),
                "t_end": float(t[j]),
                "duration_s": float(np.sum(seg_dts)),
                "peak_dev": float(dev[peak_idx]),
                "over_integral": integral,
                "phase": str(phases[peak_idx]) if phases[peak_idx] else "-",
                "t_peak": float(t[peak_idx]),
            }
        )
        i = j + 1
    return episodes


def _breakdown_metrics(
    ref: np.ndarray, dev: np.ndarray, phases: np.ndarray
) -> dict[str, dict[str, tuple[float, float, float]]]:
    """フェーズ別・速度帯別の偏差 p95 / max / 滞在サンプル数を返す。

    実機 5ac4f31d では brake フェーズの p95 2.30 に対し drive 1.24 と制動側が倍悪い。
    フェーズ別に出るのは従来トリム RMS だけで、偏差そのものの分解が無かった。
    速度帯は基準速度（ref）で切る——実車速で切ると偏差そのものが帯の割り当てを動かすため。
    """
    abs_dev = np.abs(dev)

    def stats(mask: np.ndarray) -> tuple[float, float, float]:
        if not mask.any():
            return (float("nan"), float("nan"), 0.0)
        d = abs_dev[mask]
        return (float(np.percentile(d, 95)), float(d.max()), float(mask.sum()))

    by_phase = {
        name: stats(phases == name) for name in ("drive", "coast", "brake", "stop")
    }
    by_band: dict[str, tuple[float, float, float]] = {}
    for lo, hi in zip(_SPEED_BANDS_KMH[:-1], _SPEED_BANDS_KMH[1:], strict=False):
        label = f"{lo:.0f}-{hi:.0f}" if hi < 1e8 else f"{lo:.0f}+"
        by_band[label] = stats((ref >= lo) & (ref < hi))
    return {"phase": by_phase, "band": by_band}


def _analyze(
    records: list[asyncpg.Record], params: dict | None = None
) -> dict[str, Any]:
    ts = np.array([r["timestamp"].timestamp() for r in records], dtype=float)
    t = ts - ts[0]
    ref = np.array([r["ref_speed_kmh"] for r in records], dtype=float)
    act = np.array([r["actual_speed_kmh"] for r in records], dtype=float)
    dev = act - ref
    abs_dev = np.abs(dev)
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.1

    # 符号反転（ノイズフロア超）の 5s 窓最大回数
    signs = np.where(
        dev > _SIGN_REVERSAL_AMPLITUDE_KMH,
        1,
        np.where(dev < -_SIGN_REVERSAL_AMPLITUDE_KMH, -1, 0),
    )
    rev_times = []
    last = 0
    for i, s in enumerate(signs):
        if s != 0:
            if last != 0 and s != last:
                rev_times.append(t[i])
            last = s
    rev = np.array(rev_times)
    max_rev_5s = 0
    if len(rev):
        max_rev_5s = int(max(((rev >= x) & (rev < x + _REVERSAL_WINDOW_S)).sum()
                             for x in np.arange(0.0, t[-1], 1.0)))

    # 系統ラグ: 実車速を ±N サンプルずらしたときの相関が最大になるラグ（負=実車速が先行）
    best_lag = 0
    best_corr = -2.0
    for lag in range(-20, 21):
        if lag >= 0:
            a, r = (act[lag:], ref[: len(ref) - lag]) if lag else (act, ref)
        else:
            a, r = act[:lag], ref[-lag:]
        if len(a) < 10:
            continue
        c = float(np.corrcoef(a, r)[0, 1])
        if c > best_corr:
            best_corr, best_lag = c, lag

    pedal = _pedal_activity(records, t, act, dt)
    switches = _switch_metrics(records, t, ref, dt, params)
    effort = _effort_metrics(records, t)

    # 最大逸脱 KPI の内訳（エピソード分解・フェーズ別・速度帯別）
    phases = np.array([(r["phase"] or "") for r in records])
    dts = _sample_dts(t)
    episodes = _violation_episodes(t, dev, phases, dts)
    breakdown = _breakdown_metrics(ref, dev, phases)
    over_integral = float(np.sum(np.maximum(0.0, abs_dev[1:] - _HARD_LIMIT_KMH) * dts))

    return {
        "episodes": episodes,
        "breakdown": breakdown,
        "over_limit_integral": over_integral,
        "viol_episode_count": float(len(episodes)),
        "n": float(len(t)),
        "dt_s": dt,
        "duration_s": float(t[-1]),
        "ref_max_kmh": float(ref.max()),
        "dev_p50": float(np.percentile(abs_dev, 50)),
        "dev_p95": float(np.percentile(abs_dev, 95)),
        "dev_max": float(abs_dev.max()),
        "viol_frac": float((abs_dev > _HARD_LIMIT_KMH).mean()),
        "viol_time_s": float((abs_dev > _HARD_LIMIT_KMH).sum() * dt),
        "reversal_max_per_5s": float(max_rev_5s),
        "best_lag_s": best_lag * dt,
        "best_lag_corr": best_corr,
        **pedal,
        **switches,
        **effort,
    }


def _effort_metrics(
    records: list[asyncpg.Record], t: np.ndarray
) -> dict[str, float]:
    """プラン学習の effort 内訳（トリム寄与率・滑らかさ・フェーズ逸脱）と reward を計算する。

    新列（plan/trim/applied/phase）が NULL の旧セッションでは `has_effort=0` を返して表示を
    スキップさせる。reward は kpi_monitor と同じ集計を再現して reward_score に渡す。
    """
    applied_raw = [r["applied_effort_pct"] for r in records]
    if all(v is None for v in applied_raw):
        return {"has_effort": 0.0}

    def col(key: str) -> np.ndarray:
        return np.array([(r[key] if r[key] is not None else 0.0) for r in records], dtype=float)

    plan = col("plan_effort_pct")
    trim = col("trim_effort_pct")
    applied = col("applied_effort_pct")
    phases = [r["phase"] for r in records]
    duration = float(t[-1]) if t[-1] > 0 else 1.0

    plan_rms = float(np.sqrt(np.mean(plan**2)))
    trim_rms = float(np.sqrt(np.mean(trim**2)))
    trim_share = trim_rms / max(plan_rms, 0.1)
    # applied 変化率 RMS（滑らかさ）。dt が 0 のギャップは無視。
    d_applied = np.diff(applied)
    dts = np.clip(np.diff(t), 1e-6, 0.5)
    rate_rms = float(np.sqrt(np.sum((d_applied**2) / dts) / duration))
    # フェーズ別トリム RMS
    ph = np.array([p if p is not None else "" for p in phases])
    def phase_trim_rms(name: str) -> float:
        mask = ph == name
        return float(np.sqrt(np.mean(trim[mask] ** 2))) if mask.any() else 0.0
    # フェーズ逸脱量（COAST で踏む・DRIVE のブレーキ・BRAKE のアクセル）の時間平均
    viol = 0.0
    for i in range(1, len(applied)):
        a = applied[i]
        p = phases[i]
        v = 0.0
        if p == "coast":
            v = abs(a)
        elif p == "drive":
            v = max(0.0, -a)
        elif p == "brake":
            v = max(0.0, a)
        viol += v * float(dts[i - 1])
    phase_violation = viol / duration

    # reward の再計算（reward_score は kpi_monitor 集計キーを参照する）。over_limit 積分を
    # dev 系列から実計算して kpi_monitor と整合させる（reward の支配項＝10×over_limit なので、
    # 0 固定だと保存 reward と約 10 倍乖離する）。踏み替え/min は _pedal_activity の別集計に委ね
    # 近似 0（0.2 重みで影響小）。
    from src.domain.control.kpi_monitor import KPI_HARD_LIMIT_KMH
    from src.domain.control.reward import reward_score

    dev = np.array(
        [abs(r["actual_speed_kmh"] - r["ref_speed_kmh"]) for r in records], dtype=float
    )
    # over = max(0, |dev|−hard_limit) の時間積分（kpi_monitor と同じ。dts は clip 済み）。
    over = np.maximum(0.0, dev[1:] - KPI_HARD_LIMIT_KMH)
    over_limit_integral = float(np.sum(over * dts))
    reward = reward_score(
        {
            "p95_kmh": float(np.percentile(dev, 95)),
            "max_abs_deviation_kmh": float(dev.max()),
            "over_limit_integral_kmhs": over_limit_integral,
            "effort_rate_rms_pct_s": rate_rms,
            "pedal_switch_per_min": 0.0,
            "phase_violation_pct": phase_violation,
        }
    )
    return {
        "has_effort": 1.0,
        "plan_rms": plan_rms,
        "trim_rms": trim_rms,
        "trim_share": trim_share,
        "effort_rate_rms": rate_rms,
        "phase_violation": phase_violation,
        "trim_rms_drive": phase_trim_rms("drive"),
        "trim_rms_coast": phase_trim_rms("coast"),
        "trim_rms_brake": phase_trim_rms("brake"),
        "reward": reward,
    }


def _pedal_activity(
    records: list[asyncpg.Record], t: np.ndarray, act: np.ndarray, dt: float
) -> dict[str, float]:
    """ペダルハンチング指標（B-7-4/B-7'）: アクセル ON-OFF 立ち上がり回数・速度帯別・OFF滞在。

    accel ON = 開度 > _PEDAL_ON_THRESHOLD_PCT。立ち上がり（0→非0）を全体と 110-135km/h 帯で
    数え、走行時間で正規化する（回/min）。OFF 滞在（ON→OFF→ON の谷）の中央値も出す。
    """
    acc = np.array([r["accel_opening"] for r in records], dtype=float)
    on = acc > _PEDAL_ON_THRESHOLD_PCT
    rising = np.flatnonzero((~on[:-1]) & on[1:]) + 1  # 立ち上がりサンプル位置
    dur_min = (t[-1] / 60.0) if t[-1] > 0 else 0.0
    total_on = int(rising.size)
    on_per_min = total_on / dur_min if dur_min > 0 else 0.0
    # 高速帯 110-135km/h に限定した立ち上がり密度（滞在時間で正規化）
    hs = (act >= 110.0) & (act < 135.0)
    hs_rise = int(((~on[:-1]) & on[1:] & hs[1:]).sum())
    hs_dwell_min = (float(hs.sum()) * dt) / 60.0
    hs_on_per_min = hs_rise / hs_dwell_min if hs_dwell_min > 0 else 0.0
    # OFF 滞在（立ち下がり→次の立ち上がりまでの時間）の中央値
    falling = np.flatnonzero(on[:-1] & (~on[1:])) + 1
    off_dwells = []
    ri = 0
    for f in falling:
        while ri < rising.size and rising[ri] <= f:
            ri += 1
        if ri < rising.size:
            off_dwells.append(float(t[rising[ri]] - t[f]))
    off_median = float(np.median(off_dwells)) if off_dwells else 0.0
    return {
        "accel_on_count": float(total_on),
        "accel_on_per_min": on_per_min,
        "accel_on_per_min_110_135": hs_on_per_min,
        "accel_off_dwell_median_s": off_median,
    }


def _print_violation_breakdown(m: dict[str, Any]) -> None:
    """最大逸脱 KPI の内訳（エピソード・フェーズ別・速度帯別）を表示する。

    「例外なし」が要求なので達成すべき違反回数は 0。max 1 点だけでは打ち手の効果が
    測れないため、回数・時間・占有率・超過積分・発生箇所まで出す。
    """
    episodes: list[dict[str, Any]] = m.get("episodes", [])
    print("  --- 最大逸脱の内訳（|dev| > 1.0 km/h） ---")
    print(
        f"     違反エピソード = {len(episodes)} 回 / "
        f"合計 {m['viol_time_s']:.1f}s（走行時間の {100 * m['viol_frac']:.1f}%）"
    )
    print(f"     超過積分 ∫(|dev|−1.0)dt = {m['over_limit_integral']:.2f} km/h·s")
    if episodes:
        top = sorted(episodes, key=lambda e: -float(e["duration_s"]))[:_TOP_EPISODES]
        # 占有率は「違反時間に占める割合」（機構ごとの寄与を比べる指標）。走行時間比では
        # どれも数%になって差が読めない（引き継ぎ 3-3 の機構別内訳と同じ取り方）。
        viol_total = sum(float(e["duration_s"]) for e in episodes) or 1.0
        print(f"     上位 {len(top)} エピソード（違反時間の降順）:")
        print(
            f"       {'区間[s]':>15s} {'継続':>6s} {'占有':>6s} "
            f"{'ピーク偏差':>10s} {'超過積分':>9s}  フェーズ"
        )
        for e in top:
            span = f"{float(e['t_start']):.1f}-{float(e['t_end']):.1f}"
            share = 100.0 * float(e["duration_s"]) / viol_total
            print(
                f"       {span:>15s} {float(e['duration_s']):5.1f}s {share:5.1f}% "
                f"{float(e['peak_dev']):+10.2f} {float(e['over_integral']):9.2f}"
                f"  {e['phase']}"
            )

    bd = m.get("breakdown")
    if not bd:
        return

    def row(label: str, st: tuple[float, float, float]) -> str:
        p95, mx, n = st
        if n <= 0:
            return f"       {label:>10s}      -        -        0"
        return f"       {label:>10s} {p95:8.2f} {mx:8.2f} {n:8.0f}"

    print("     フェーズ別 |dev|:")
    print(f"       {'':>10s} {'p95':>8s} {'max':>8s} {'n':>8s}")
    for name, st in bd["phase"].items():
        print(row(name, st))
    print("     速度帯別 |dev|（基準速度で区分）:")
    print(f"       {'km/h':>10s} {'p95':>8s} {'max':>8s} {'n':>8s}")
    for label, st in bd["band"].items():
        if st[2] > 0:
            print(row(label, st))


def _print_report(session_id: str, m: dict[str, Any]) -> None:
    def ok(passed: bool) -> str:
        return "OK " if passed else "NG "

    print(f"session {session_id}")
    print(f"  samples={m['n']:.0f} dt={m['dt_s']:.3f}s dur={m['duration_s']:.1f}s "
          f"ref_max={m['ref_max_kmh']:.1f}km/h")
    print("  --- プライマリー KPI ---")
    print(
        f"  {ok(m['dev_p95'] <= _P95_LIMIT_KMH)}|dev| p95 = {m['dev_p95']:.3f} km/h "
        f"(≤{_P95_LIMIT_KMH})"
    )
    print(f"  {ok(m['dev_max'] <= _HARD_LIMIT_KMH)}|dev| max = {m['dev_max']:.3f} km/h (≤1.0)")
    print(f"  {ok(m['reversal_max_per_5s'] <= 1.0)}符号反転 max/5s = "
          f"{m['reversal_max_per_5s']:.0f} (≤1)")
    print(f"     |dev| p50 = {m['dev_p50']:.3f} km/h")
    print(f"     違反時間 = {m['viol_time_s']:.1f}s ({100 * m['viol_frac']:.1f}%)")
    _print_violation_breakdown(m)
    print("  --- Stage A 系統ラグ ---")
    lag_ok = abs(m["best_lag_s"]) <= 0.1
    print(f"  {ok(lag_ok)}最良ラグ = {m['best_lag_s']:+.2f}s "
          f"(corr={m['best_lag_corr']:.4f})  負=実車速が先行")
    print("  --- B-7 ペダルハンチング ---")
    hs_ok = m["accel_on_per_min_110_135"] <= _PEDAL_ON_GATE_PER_MIN
    print(f"  {ok(hs_ok)}accel ON 110-135km/h = {m['accel_on_per_min_110_135']:.1f} 回/min (≤10)")
    print(f"     accel ON 全体 = {m['accel_on_count']:.0f} 回 "
          f"({m['accel_on_per_min']:.1f} 回/min)")
    print(f"     OFF 滞在中央値 = {m['accel_off_dwell_median_s']:.2f}s")
    print("  --- 不要切替（アクセル⇔ブレーキ交互踏み） ---")
    excess = m.get("seesaw_excess_per_min", float("nan"))
    req = m.get("seesaw_required_per_min", float("nan"))
    meas = m.get("seesaw_measured_per_min", float("nan"))
    if excess == excess:  # NaN でない（プロファイル定数が取得できた）
        print(f"  {ok(excess <= 2.0)}不要切替 = {excess:.1f} 回/min "
              f"（実測 {meas:.1f} − 軌跡要求 {req:.1f}、目安 ≤2）")
    else:
        print(f"     交互踏み（実測）= {meas:.1f} 回/min"
              "（軌跡要求は算出不可＝プロファイル定数なし）")

    # プラン学習の effort 内訳（トリム寄与率＝PID フィードバック最小化の指標・reward）
    if m.get("has_effort", 0.0) >= 1.0:
        print("  --- プラン学習 effort 内訳 ---")
        print(f"     reward = {m['reward']:+.4f}（大きいほど良い）")
        print(f"     トリム寄与率 = {m['trim_share']:.3f} "
              f"（trim_rms {m['trim_rms']:.2f} / plan_rms {m['plan_rms']:.2f}%）")
        print(f"     effort 変化率 RMS = {m['effort_rate_rms']:.2f} %/s（小さいほど滑らか）")
        print(f"     フェーズ逸脱 = {m['phase_violation']:.3f} %（安全網介入量、目安 ≈0）")
        print(f"     フェーズ別トリム RMS: DRIVE {m['trim_rms_drive']:.2f} "
              f"COAST {m['trim_rms_coast']:.2f} BRAKE {m['trim_rms_brake']:.2f} %")


async def _main() -> None:
    ap = argparse.ArgumentParser(description="走行セッションの車速追従 KPI を解析する")
    ap.add_argument("session_id", nargs="?", help="解析対象の session_id (UUID)")
    ap.add_argument("--latest", metavar="RUN_TYPE", nargs="?", const="",
                    help="直近セッションを解析（RUN_TYPE 省略で全種別から最新）")
    ap.add_argument("--dsn", default=os.environ.get("DATABASE_URL",
                                                    "postgresql://localhost/driving_robot"))
    args = ap.parse_args()

    session_id = args.session_id
    if args.latest is not None:
        session_id = await _latest_session(args.dsn, args.latest or None)
        if session_id is None:
            print("該当セッションがありません")
            return
    if not session_id:
        ap.error("session_id か --latest を指定してください")

    records = await _fetch_logs(args.dsn, session_id)
    if not records:
        print(f"session {session_id} にログがありません")
        return
    params = await _fetch_profile_params(args.dsn, session_id)
    _print_report(session_id, _analyze(records, params))


if __name__ == "__main__":
    asyncio.run(_main())
