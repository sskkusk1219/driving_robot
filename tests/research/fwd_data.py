"""順モデル（車速 × 効く開度 → 加速度）の学習用の行を走行 CSV から作る（段2f）。

効く開度 u [%]（アクセル＋・ブレーキ−の 1 本）:
    アクセル実開度 > アクセル不感帯 なら +(実開度 − 不感帯)
    ブレーキ実開度 > ブレーキ不感帯 なら −(実開度 − 不感帯)
    どちらも不感帯以下なら 0（惰行）。両方が不感帯を超える行は捨てる。

表の定義（Apollo の形）: 「時刻 t の車速 × 時刻 t の開度 u → 時刻 t + L の加速度」。
u の元になる開度は opening_source で選ぶ（既定 "cmd" = 最終指令の開度 `*_cmd_pct`。待機位置を含む。
"actual" = 実開度 `*_actual_pct`）。

加速度 [km/h/s]: 実車速の、時刻 t + 遅れ L を中心にした幅 accel_window_s の最小二乗の傾き。
遅れ L は u > 0 の行が accel_lag_s、u < 0 の行が brake_lag_s、u = 0（惰行）の行が coast_lag_s
（None なら accel_lag_s）。
傾きの窓はセグメント（section が同じで行間が SEGMENT_GAP_S 未満の連続区間）の中だけで取る。

マスの集計（cell_counts）と「モード（検証ログ）が通るのに学習が薄いマス」（thin_cells）も持つ。
"""

from __future__ import annotations

import csv
import math
from collections.abc import Collection, Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

from tests.research.drive_log import (
    SECTION_MODE_DRIVE,
    SECTION_PATTERN_DRIVE,
    SECTION_PEDAL_SEARCH,
    actual_opening,
    cmd_opening,
)
from tests.research.ff_bias import window_slope

DEFAULT_SECTIONS: tuple[str, ...] = (SECTION_PATTERN_DRIVE, SECTION_MODE_DRIVE)
# 2026-10-18 段2o-10: 低速の表（発進側・停止側）の行を phase で取るための名前。CSV の `phase` 列と
# 同じ文字列（pedal_search.PHASE_* と同じ。tests/research 内で pedal_search に依らないよう持ち込み）
PHASE_CREEP_WAIT = "CREEP_WAIT"  # PEDAL_SEARCH: 両ペダル min・停車からクリープで 5 km/h まで
PHASE_BRAKE_SEARCH = "BRAKE_SEARCH"  # PEDAL_SEARCH: クリープからブレーキを刻んで踏み増して止める
PHASE_BRAKE_STEP = "BRAKE_STEP"  # PRE_DRIVE_CHECK: ブレーキを刻んで停止確認
# PATTERN_DRIVE: G ランプの発進でブレーキをゆっくり離す（動き出す前の）区間
PHASE_RAMP_LAUNCH_RELEASE = "RAMP_LAUNCH_RELEASE"
SEGMENT_GAP_S = 0.5  # 仮値: 行間がこれ以上空いたらセグメントを切る
STOP_SPEED_KMH = 0.5  # 実車速がこれ未満の行は停車として捨てる
DEFAULT_ACCEL_WINDOW_S = 1.0  # 加速度の窓幅（前後 0.5 秒）
# 指令 → 加速度の遅れ L [s]。データ由来（指令ベースの振り。検証 052422、2026-10-03）:
# そのペダルの行の「外れの 3 秒平均 p95」が最小になる値（0.05 刻み。fwd_tool score の (iii)）。
# 選んだ値: アクセル 0.05・ブレーキ 0.05（どちらも 3 秒平均 p95 が最小。惰行行は 0 が最小だが
# アクセルの L を使う）。ブレーキは 0.05〜0.15 でほぼ平ら（p95 0.479〜0.483）。
DEFAULT_ACCEL_LAG_S = 0.05
DEFAULT_BRAKE_LAG_S = 0.05
# 指令開度と実開度のずれ [%] がこれを超えた行は学習から外す。表の開度のマス幅（u_step_pct＝2%）に
# 合わせた。これ以上ずれると、指令で数えた行が隣のマスに入ってしまう。2026-10-10 段2l-10
DEFAULT_ACTUATOR_LAG_TOL_PCT = 2.0
OPENING_CMD = "cmd"
OPENING_ACTUAL = "actual"

DROP_STOPPED = "停車（実車速 < 0.5 km/h）"
DROP_NO_OPENING = "実開度が空"
DROP_NO_SPEED = "実車速が空"
DROP_GOVERNOR = "減速Gガバナー作動"
DROP_ACTUATOR_LAG = "指令と実開度がずれた"
DROP_BOTH_PEDALS = "両ペダルが不感帯超え"
DROP_NO_WINDOW = "加速度の窓が取れない（セグメント端）"
DROP_REASONS: tuple[str, ...] = (
    DROP_NO_OPENING,
    DROP_NO_SPEED,
    DROP_STOPPED,
    DROP_GOVERNOR,
    DROP_ACTUATOR_LAG,
    DROP_BOTH_PEDALS,
    DROP_NO_WINDOW,
)

Cell = tuple[int, int]  # (車速のマス番号, u のマス番号)


@dataclass(frozen=True)
class FwdRows:
    """学習用の行。配列はすべて同じ長さ。"""

    speed_kmh: np.ndarray
    u_pct: np.ndarray
    accel_kmhs: np.ndarray
    t_s: np.ndarray
    dt_s: float  # ログの刻みの中央値 [s]（行数 → 秒数の換算）
    source: str
    dropped: dict[str, int] = field(default_factory=dict)
    sections: tuple[str, ...] = ()  # 使えた行の section
    dt_row_s: np.ndarray = field(default_factory=lambda: np.zeros(0))  # 行ごとの秒数
    # 2026-10-14 段2o-5: 各行の「戻し量」[%]。直近 RELEASE_WINDOW_S 秒（その行を含む）の u の
    # 最小からの戻り量で、u < 0 の行だけ正（u − 最小）、それ以外は 0。行を捨てる前の
    # 全時系列で計算する（release_amount）。省略（長さ 0）なら全行 0（古い呼び出し・テスト用）
    release_pct: np.ndarray = field(default_factory=lambda: np.zeros(0))
    # 2026-10-14 段2o-5: 「発進の区間」（launch_mask）の行が True。C8 のブレーキ側の学習から外すため
    # （発進は戻しではなく attach_launch の別の表が受け持つ）。read_fwd_rows が mark_launch=True で
    # 作る。省略（長さ 0）なら全行 False
    launch_row: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=bool))

    def __post_init__(self) -> None:
        n = len(self.speed_kmh)
        if len(self.release_pct) == 0 and n > 0:
            object.__setattr__(self, "release_pct", np.zeros(n))
        if len(self.launch_row) == 0 and n > 0:
            object.__setattr__(self, "launch_row", np.zeros(n, dtype=bool))

    def __len__(self) -> int:
        return len(self.speed_kmh)

    @property
    def seconds(self) -> float:
        """使えた行の秒数。"""
        return float(np.sum(self.dt_row_s))

    def dropped_seconds(self) -> dict[str, float]:
        return {k: n * self.dt_s for k, n in self.dropped.items()}


# 仮値（戻し量 release_amount）: 「山」を見る窓の長さ [s]。根拠: モード（WLTP）のブレーキの戻しは
# 1〜3 秒で終わる（段2o-4 の数え）。実機の後に決め直す。fwd_plan の再解も同じ値を使う
RELEASE_WINDOW_S = 3.0


def release_amount(
    t_s: np.ndarray, u_pct: np.ndarray, window_s: float = RELEASE_WINDOW_S
) -> np.ndarray:
    """戻し量 [%]（2026-10-14 段2o-5）。1 本の連続した時系列（時刻昇順）について、各点の
    「直近 window_s 秒（その点を含む）の u の最小からの戻り量」を、u < 0 の点だけ u − 最小として返す
    （u >= 0 の点は 0）。u が NaN の点は最小の計算から外す（その点自身の戻し量は 0）。
    例: u = −6 から −4 へ戻した点は 2、−6 のままなら 0。
    """
    t = np.asarray(t_s, dtype=float)
    u = np.asarray(u_pct, dtype=float)
    out = np.zeros(len(t))
    start = 0
    for i in range(len(t)):
        while t[start] < t[i] - window_s - 1e-9:
            start += 1
        if not (u[i] < 0.0):  # NaN も含めて 0
            continue
        win = u[start : i + 1]
        out[i] = u[i] - float(np.nanmin(win))
    return out


def effective_opening(
    accel_pct: float,
    brake_pct: float,
    accel_deadband_pct: float,
    brake_deadband_pct: float,
) -> float | None:
    """効く開度 u [%]。両ペダルが不感帯を超えるときは None（捨てる行）。"""
    accel_on = accel_pct > accel_deadband_pct
    brake_on = brake_pct > brake_deadband_pct
    if accel_on and brake_on:
        return None
    if accel_on:
        return accel_pct - accel_deadband_pct
    if brake_on:
        return -(brake_pct - brake_deadband_pct)
    return 0.0


def _num(text: str | None) -> float | None:
    if text is None or text.strip() == "":
        return None
    try:
        v = float(text)
    except ValueError:
        return None
    return v if math.isfinite(v) else None


def _truthy(text: str | None) -> bool:
    return (text or "").strip().lower() not in ("", "0", "false", "0.0")


def _cmd_or_none(row: dict[str, str], axis: str) -> float | None:
    """最終指令の開度 [%]。列が無い・空・読めない行は None。"""
    try:
        return cmd_opening(row, axis)
    except (KeyError, ValueError):
        return None


def read_fwd_rows(
    path: Path,
    *,
    accel_deadband_pct: float,
    brake_deadband_pct: float,
    sections: Collection[str] = DEFAULT_SECTIONS,
    accel_window_s: float = DEFAULT_ACCEL_WINDOW_S,
    accel_lag_s: float = DEFAULT_ACCEL_LAG_S,
    brake_lag_s: float = DEFAULT_BRAKE_LAG_S,
    coast_lag_s: float | None = None,
    opening_source: str = OPENING_CMD,
    actuator_lag_tol_pct: float | None = DEFAULT_ACTUATOR_LAG_TOL_PCT,
    keep_stopped: bool = False,
    mark_launch: bool = True,
    phases: Collection[str] | None = None,
) -> FwdRows:
    """走行 CSV 1 本から学習用の行を作る。捨てた行は理由ごとに dropped に数える。

    actuator_lag_tol_pct: アクセルかブレーキのどちらかで |指令開度 − 実開度| がこれを超えた行を
    捨てる（DROP_ACTUATOR_LAG）。None なら外さない。実開度の列が無い・空の行は捨てない。
    指令（`*_cmd_pct`）も実開度（`*_actual_pct`）も同じ物差し（待機位置 min を含む %）。
    keep_stopped: True なら停車（実車速 < STOP_SPEED_KMH）の行も捨てない（DROP_STOPPED に数えない。
    発進の区間 `launch_mask` を作るため。2026-10-14 段2o-4）。既定 False は今までどおり。
    mark_launch: True（既定。keep_stopped=False のときだけ働く）なら、`launch_row` に発進の区間の
    行を印す（2026-10-14 段2o-5）。pattern_drive が発進側の表（attach_launch）に使う行と同じ定義
    （実開度・ずれ許容なし・停車の行を残す・窓 LAUNCH_ACCEL_WINDOW_S で読み直して launch_mask）で
    判定し、時刻が一致する行（elapsed_s の差が刻みの半分未満）に写す。
    phases: 渡すと、CSV の `phase` 列がこの中にある行だけを読む（sections と両方で絞る。2026-10-18
    段2o-10。手順2 の CREEP_WAIT・BRAKE_SEARCH など phase 単位で行を取るため）。None（既定）は
    phase で絞らない（今までと同じ）。
    """
    if opening_source not in (OPENING_CMD, OPENING_ACTUAL):
        raise ValueError(f"opening_source は cmd か actual: {opening_source!r}")
    pick = _cmd_or_none if opening_source == OPENING_CMD else actual_opening
    lag_coast = accel_lag_s if coast_lag_s is None else coast_lag_s
    dropped = dict.fromkeys(DROP_REASONS, 0)
    # セグメントごとの行: (t, 実車速, accel実開度, brake実開度, ガバナー, section, ずれ超え)
    segments: list[list[tuple[float, float, float | None, float | None, bool, str, bool]]] = []
    prev_t = math.nan
    prev_section = ""
    with path.open(newline="", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            section = r.get("section") or ""
            t = _num(r.get("elapsed_s"))
            if section not in sections or t is None:
                continue
            if phases is not None and (r.get("phase") or "") not in phases:
                continue
            speed = _num(r.get("actual_speed_kmh"))
            new_seg = (
                not segments
                or section != prev_section
                or not (0.0 < t - prev_t < SEGMENT_GAP_S)
            )
            if new_seg:
                segments.append([])
            prev_t, prev_section = t, section
            lagged = False
            if actuator_lag_tol_pct is not None:
                for axis in ("accel", "brake"):
                    cmd, act = _cmd_or_none(r, axis), actual_opening(r, axis)
                    if cmd is None or act is None:
                        continue
                    if abs(cmd - act) > actuator_lag_tol_pct:
                        lagged = True
            segments[-1].append(
                (
                    t,
                    math.nan if speed is None else speed,
                    pick(r, "accel"),
                    pick(r, "brake"),
                    _truthy(r.get("governor_active")),
                    section,
                    lagged,
                )
            )

    speed_o: list[float] = []
    u_o: list[float] = []
    acc_o: list[float] = []
    t_o: list[float] = []
    rel_o: list[float] = []
    secs: set[str] = set()
    all_dt: list[float] = []
    for seg in segments:
        ts = np.array([x[0] for x in seg])
        all_dt.extend(np.diff(ts).tolist())
        # 傾きは実車速が読めた点だけで取る
        ok = np.array([np.isfinite(x[1]) for x in seg])
        sp = np.array([x[1] for x in seg])
        us: list[float | None] = []
        # 2026-10-14 段2o-5: 戻し量は行を捨てる前の全時系列の u で計算する（開度が読めた行は
        # 停車・ガバナー・ずれ超えで捨てる行も含める。両ペダルが不感帯超えの行は NaN）
        u_full = np.full(len(seg), np.nan)
        for i, x in enumerate(seg):
            if x[2] is not None and x[3] is not None:
                uf = effective_opening(x[2], x[3], accel_deadband_pct, brake_deadband_pct)
                if uf is not None:
                    u_full[i] = uf
        rel_full = release_amount(ts, u_full)
        for i, (_t, v, ap, bp, gov, _s, lagged) in enumerate(seg):
            if ap is None or bp is None:
                dropped[DROP_NO_OPENING] += 1
                us.append(None)
            elif not ok[i]:
                dropped[DROP_NO_SPEED] += 1
                us.append(None)
            elif v < STOP_SPEED_KMH and not keep_stopped:
                dropped[DROP_STOPPED] += 1
                us.append(None)
            elif gov:
                dropped[DROP_GOVERNOR] += 1
                us.append(None)
            elif lagged:
                dropped[DROP_ACTUATOR_LAG] += 1
                us.append(None)
            else:
                u = effective_opening(ap, bp, accel_deadband_pct, brake_deadband_pct)
                if u is None:
                    dropped[DROP_BOTH_PEDALS] += 1
                us.append(u)
        keep = [i for i, u in enumerate(us) if u is not None]
        if not keep or int(ok.sum()) == 0:
            continue
        u_arr = np.array([us[i] for i in keep], dtype=float)
        t_keep = ts[keep]
        lag = np.where(u_arr > 0.0, accel_lag_s, np.where(u_arr < 0.0, brake_lag_s, lag_coast))
        slope = window_slope(ts[ok], sp[ok], t_keep + lag, accel_window_s)
        for j, i in enumerate(keep):
            if not np.isfinite(slope[j]):
                dropped[DROP_NO_WINDOW] += 1
                continue
            speed_o.append(float(sp[i]))
            u_o.append(float(u_arr[j]))
            acc_o.append(float(slope[j]))
            t_o.append(float(t_keep[j]))
            rel_o.append(float(rel_full[i]))
            secs.add(seg[i][5])
    dt = float(np.median(all_dt)) if all_dt else 0.0
    n = len(speed_o)
    out = FwdRows(
        speed_kmh=np.array(speed_o, dtype=float),
        u_pct=np.array(u_o, dtype=float),
        accel_kmhs=np.array(acc_o, dtype=float),
        t_s=np.array(t_o, dtype=float),
        dt_s=dt,
        source=path.name,
        dropped=dropped,
        sections=tuple(sorted(secs)),
        dt_row_s=np.full(n, dt),
        release_pct=np.array(rel_o, dtype=float),
    )
    if mark_launch and not keep_stopped and n > 0:
        la = read_fwd_rows(
            path,
            accel_deadband_pct=accel_deadband_pct,
            brake_deadband_pct=brake_deadband_pct,
            sections=sections,
            accel_window_s=LAUNCH_ACCEL_WINDOW_S,
            accel_lag_s=accel_lag_s,
            brake_lag_s=brake_lag_s,
            coast_lag_s=coast_lag_s,
            opening_source=OPENING_ACTUAL,
            actuator_lag_tol_pct=None,
            keep_stopped=True,
            mark_launch=False,
            phases=phases,
        )
        lm = launch_mask(la) if len(la) else np.zeros(0, dtype=bool)
        if lm.any():
            flag = np.zeros(n, dtype=bool)
            tl = la.t_s
            for i, ti in enumerate(out.t_s.tolist()):
                k = int(np.searchsorted(tl, ti))
                near = [j for j in (k - 1, k) if 0 <= j < len(tl)]
                j = min(near, key=lambda q: abs(tl[q] - ti))
                flag[i] = bool(lm[j]) and abs(tl[j] - ti) < max(dt, 1e-9) / 2.0
            out = replace(out, launch_row=flag)
    return out


# 発進の区間を読み直すときの加速度の窓 [s]。pattern_drive.LAUNCH_ACCEL_WINDOW_S と同じ値
# （tests/research は pattern_drive を import しないので持ち込み。変えるときは両方）
LAUNCH_ACCEL_WINDOW_S = 0.5
# 仮値: 発進の区間で「車速が下がっていない」とみなす加速度の下限（−この値）[km/h/s]。
# 停車中の傾きの雑音ぶん。段2o-4 の確認で発進の区間の行を見て決める
LAUNCH_ACCEL_TOL_KMHS = 0.05
# 仮値（stop_approach_rows）: 「停車の手前」とみなす、停車の行までの最大の秒数 [s]。モードの低速の
# 減速は数秒で止まる。段2o-10 の確認で行数を見て決める
STOP_LOOKAHEAD_S = 5.0


def take_rows(rows: FwdRows, mask: np.ndarray) -> FwdRows:
    """mask（bool、長さ len(rows)）が True の行だけの FwdRows。dropped・dt_s・source は引き継ぐ。"""
    m = np.asarray(mask, dtype=bool)
    return FwdRows(
        speed_kmh=rows.speed_kmh[m],
        u_pct=rows.u_pct[m],
        accel_kmhs=rows.accel_kmhs[m],
        t_s=rows.t_s[m],
        dt_s=rows.dt_s,
        source=rows.source,
        dropped=dict(rows.dropped),
        sections=rows.sections,
        dt_row_s=rows.dt_row_s[m],
        release_pct=rows.release_pct[m],
        launch_row=rows.launch_row[m],
    )


def launch_mask(rows: FwdRows, *, gap_s: float = SEGMENT_GAP_S) -> np.ndarray:
    """「発進の区間」の行が True の配列（2026-10-14 段2o-4）。
    rows は keep_stopped=True で読んだ時刻順の行。

    区間の始まりは停車の行（実車速 < STOP_SPEED_KMH。ただし u <= 0 のとき。停車中に
    アクセルを踏んでいる行は False で、区間も始めない）。続く行は、前の行から途切れていない
    （時刻の間隔 < gap_s）・アクセルを踏んでいない（u <= 0）・車速が下がっていない
    （加速度 >= −LAUNCH_ACCEL_TOL_KMHS）のすべてを満たす間だけ True。
    外れた行で区間は終わり、次に停車の行が来るまで False。
    """
    n = len(rows)
    out = np.zeros(n, dtype=bool)
    inside = False
    prev_t = math.nan
    for i in range(n):
        t = float(rows.t_s[i])
        u = float(rows.u_pct[i])
        if rows.speed_kmh[i] < STOP_SPEED_KMH:
            inside = u <= 0.0
        elif inside:
            inside = (
                0.0 < t - prev_t < gap_s
                and u <= 0.0
                and float(rows.accel_kmhs[i]) >= -LAUNCH_ACCEL_TOL_KMHS
            )
        out[i] = inside
        prev_t = t
    return out


def stop_approach_rows(
    rows: FwdRows, *, lookahead_s: float = STOP_LOOKAHEAD_S, gap_s: float = SEGMENT_GAP_S
) -> np.ndarray:
    """「停車の手前」の行が True の配列（2026-10-18 段2o-10）。rows は keep_stopped=True で読んだ
    時刻順の行。動いている行（実車速 >= STOP_SPEED_KMH）のうち、途切れずに lookahead_s 秒以内に
    停車の行（実車速 < STOP_SPEED_KMH）が来て、かつ u <= 0 の行。減速してブレーキで止まる区間。"""
    n = len(rows)
    out = np.zeros(n, dtype=bool)
    stopped_t = math.nan  # 後ろから見て、途切れずにつながる直近の停車の時刻
    for i in range(n - 1, -1, -1):
        t = float(rows.t_s[i])
        if i + 1 < n and not (0.0 < float(rows.t_s[i + 1]) - t < gap_s):
            stopped_t = math.nan
        if rows.speed_kmh[i] < STOP_SPEED_KMH:
            stopped_t = t
        elif math.isfinite(stopped_t) and stopped_t - t <= lookahead_s:
            out[i] = float(rows.u_pct[i]) <= 0.0
    return out


@dataclass(frozen=True)
class Breakaway:
    """動き出す開度（ブレーキ。min からの量 u [%]、負）。source は "release" か "brake_search"。"""

    u_pct: float
    source: str
    per_segment_u_pct: tuple[float, ...]


def _read_plain(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def _value_at(ts: list[float], vals: list[float], t: float) -> float | None:
    """時刻 t 以前でいちばん近い行の値（無ければ None）。"""
    k = int(np.searchsorted(np.asarray(ts), t, side="right")) - 1
    return vals[k] if 0 <= k < len(vals) else None


def launch_breakaway(
    path: Path,
    *,
    brake_min_pct: float,
    brake_lag_s: float,
    min_speed_kmh: float = STOP_SPEED_KMH,
) -> Breakaway | None:
    """「動き出す開度」を CSV から求める（2026-10-18 段2o-10）。

    - RAMP_LAUNCH_RELEASE 区間（PATTERN_DRIVE。途切れない連続した行）ごとに、区間の始まり以降で
      実車速が min_speed_kmh 以上になった最初の行の時刻 t_move を探し、(t_move − brake_lag_s)
      時点のブレーキ実開度から u ＝ −(実開度 − min) を求める。全区間の中央値が u_pct、
      source ＝ "release"。
    - その phase が無い古いログは、BRAKE_SEARCH（PEDAL_SEARCH）の区間ごとに、実車速が
      STOP_SPEED_KMH 未満になった最初の行（止まった点。踏み増す向き）の (t − brake_lag_s) 時点の
      ブレーキ実開度から同様に求める。source ＝ "brake_search"。
    どちらも求まらなければ None。実開度の列が無い・空の行は使わない。
    """
    rows = _read_plain(path)
    t_all: list[float] = []
    v_all: list[float] = []
    b_all: list[float] = []
    sec_all: list[str] = []
    ph_all: list[str] = []
    for r in rows:
        t = _num(r.get("elapsed_s"))
        v = _num(r.get("actual_speed_kmh"))
        b = actual_opening(r, "brake")
        if t is None or v is None or b is None:
            continue
        t_all.append(t)
        v_all.append(v)
        b_all.append(b)
        sec_all.append(r.get("section") or "")
        ph_all.append(r.get("phase") or "")

    def u_at(t: float) -> float | None:
        b = _value_at(t_all, b_all, t)
        return None if b is None else -max(b - brake_min_pct, 0.0)

    def segments(section: str, phase: str) -> list[tuple[int, int]]:
        out: list[tuple[int, int]] = []
        start = -1
        for i in range(len(t_all)):
            inside = sec_all[i] == section and ph_all[i] == phase
            cut = start >= 0 and not (0.0 < t_all[i] - t_all[i - 1] < SEGMENT_GAP_S * 4)
            if start >= 0 and (not inside or cut):
                out.append((start, i - 1))
                start = -1
            if inside and start < 0:
                start = i
        if start >= 0:
            out.append((start, len(t_all) - 1))
        return out

    found: list[float] = []
    rel = segments(SECTION_PATTERN_DRIVE, PHASE_RAMP_LAUNCH_RELEASE)
    if rel:
        for k, (s0, _s1) in enumerate(rel):
            end = rel[k + 1][0] if k + 1 < len(rel) else len(t_all)
            for i in range(s0, end):
                if sec_all[i] == SECTION_PATTERN_DRIVE and v_all[i] >= min_speed_kmh:
                    u = u_at(t_all[i] - brake_lag_s)
                    if u is not None:
                        found.append(u)
                    break
        if found:
            return Breakaway(float(np.median(found)), "release", tuple(found))
    for s0, s1 in segments(SECTION_PEDAL_SEARCH, PHASE_BRAKE_SEARCH):
        for i in range(s0, s1 + 1):
            if v_all[i] < STOP_SPEED_KMH:
                u = u_at(t_all[i] - brake_lag_s)
                if u is not None:
                    found.append(u)
                break
    if found:
        return Breakaway(float(np.median(found)), "brake_search", tuple(found))
    return None


def concat_rows(parts: Sequence[FwdRows], source: str = "") -> FwdRows:
    """複数ログの行を連結する。dropped は足し、dt_s は行ごとの秒数の中央値。"""
    dropped: dict[str, int] = dict.fromkeys(DROP_REASONS, 0)
    for p in parts:
        for k, n in p.dropped.items():
            dropped[k] = dropped.get(k, 0) + n
    dt_row = np.concatenate([p.dt_row_s for p in parts]) if parts else np.zeros(0)

    def cat(name: str) -> np.ndarray:
        return np.concatenate([getattr(p, name) for p in parts]) if parts else np.zeros(0)

    return FwdRows(
        speed_kmh=cat("speed_kmh"),
        u_pct=cat("u_pct"),
        accel_kmhs=cat("accel_kmhs"),
        t_s=cat("t_s"),
        dt_s=float(np.median(dt_row)) if len(dt_row) else 0.0,
        source=source or "+".join(p.source for p in parts),
        dropped=dropped,
        sections=tuple(sorted({s for p in parts for s in p.sections})),
        dt_row_s=dt_row,
        release_pct=cat("release_pct"),
        launch_row=cat("launch_row").astype(bool),
    )


def speed_cell(speed_kmh: np.ndarray | float, speed_step_kmh: float) -> np.ndarray:
    """車速のマス番号 k = floor(v / step)。マスは [k*step, (k+1)*step)（下端を含む）。"""
    return np.floor(np.asarray(speed_kmh) / speed_step_kmh + 1e-9).astype(int)


# 車速の境界（不均一）: 15 km/h 以下は 2.5 km/h、それより上は 5 km/h（ユーザー決定 2026-10-03）
FINE_SPEED_STEP_KMH = 2.5
FINE_SPEED_MAX_KMH = 15.0
COARSE_SPEED_STEP_KMH = 5.0


def make_speed_edges(
    max_kmh: float,
    fine_step_kmh: float = FINE_SPEED_STEP_KMH,
    fine_max_kmh: float = FINE_SPEED_MAX_KMH,
    coarse_step_kmh: float = COARSE_SPEED_STEP_KMH,
) -> np.ndarray:
    """車速のマスの境界の列 [0, 2.5, 5, ..., 15, 20, 25, ...]（max_kmh を含む最初の境界まで）。

    fine_max_kmh 以下は fine_step_kmh、それより上は coarse_step_kmh。
    全部同じ刻みにしたいときは fine_step_kmh = coarse_step_kmh を渡す。
    """
    fine = np.arange(0.0, fine_max_kmh + 1e-9, fine_step_kmh)
    n = max(int(math.ceil((max_kmh - fine_max_kmh) / coarse_step_kmh - 1e-9)), 1)
    coarse = fine_max_kmh + coarse_step_kmh * np.arange(1, n + 1)
    return np.concatenate([fine, coarse])


def speed_cell_edges(speed_kmh: np.ndarray | float, edges: np.ndarray) -> np.ndarray:
    """境界の列 edges で引いた車速のマス番号。マスは [edges[k], edges[k+1])（下端を含む）。

    最初の境界より下は 0、最後の境界以上は最後のマスに丸める。
    """
    k = np.searchsorted(edges, np.asarray(speed_kmh, dtype=float) + 1e-9, side="right") - 1
    return np.clip(k, 0, len(edges) - 2).astype(int)


def u_cell(u_pct: np.ndarray | float, u_step_pct: float) -> np.ndarray:
    """u のマス番号。0 は惰行（u = 0 ちょうど）だけの独立のマス。

    u > 0: k = ceil(u / step) ≥ 1 → マスは ((k-1)*step, k*step]（0 を含まず上端を含む）
    u < 0: k = −ceil(−u / step) ≤ −1 → マスは [−|k|*step, −(|k|−1)*step)（0 を含まず下端を含む）
    例: step=2 のとき u=2.0 は番号 1（0〜2）、u=2.1 は番号 2（2〜4）、u=−2.0 は番号 −1（−2〜0）。
    """
    u = np.asarray(u_pct, dtype=float)
    mag = np.ceil(np.abs(u) / u_step_pct - 1e-9).astype(int)
    return np.where(u > 0.0, np.maximum(mag, 1), np.where(u < 0.0, -np.maximum(mag, 1), 0))


def cell_counts(
    rows: FwdRows,
    speed_step_kmh: float = 5.0,
    u_step_pct: float = 2.0,
    speed_edges: np.ndarray | None = None,
) -> dict[Cell, float]:
    """マスごとの秒数（行数 × 刻み）。行の無いマスは含まない。

    speed_edges（車速のマスの境界の列）を渡すと speed_step_kmh は使わず不均一なマスで数える。
    """
    out: dict[Cell, float] = {}
    if len(rows) == 0:
        return out
    if speed_edges is not None:
        sc = speed_cell_edges(rows.speed_kmh, speed_edges)
    else:
        sc = speed_cell(rows.speed_kmh, speed_step_kmh)
    uc = u_cell(rows.u_pct, u_step_pct)
    for s, u, d in zip(sc.tolist(), uc.tolist(), rows.dt_row_s.tolist(), strict=True):
        out[(s, u)] = out.get((s, u), 0.0) + d
    return out


@dataclass(frozen=True)
class ThinCell:
    """モードが通るのに学習が薄いマス 1 つ。"""

    speed_idx: int
    u_idx: int
    check_s: float  # 検証ログがそこに居た秒数
    train_s: float  # 学習ログの秒数（無ければ 0）


def thin_cells(
    train: dict[Cell, float], check: dict[Cell, float], min_s: float = 1.0
) -> list[ThinCell]:
    """check が通ったマスのうち、学習側の秒数が min_s 未満のマス（車速・u の順に並べる）。"""
    out = [
        ThinCell(s, u, sec, train.get((s, u), 0.0))
        for (s, u), sec in check.items()
        if train.get((s, u), 0.0) < min_s
    ]
    out.sort(key=lambda c: (c.speed_idx, c.u_idx))
    return out
