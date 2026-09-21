"""段4: クリープ域ブレーキの下限（停止境界）の推定（ProblemReport_20260916 段4 改訂）。

用語:
    停止境界 … クリープで転がっている車を、そのブレーキ開度のまま停車まで持っていける
        最小の開度。速度依存はほとんど見えない（後述の実測参照）ため、カーブではなく
        「不感帯からの超過 [%]」という定数 1 個で持つ。
    停止ブレーキの下限（`ff_candidate._apply_brake_trim`）… 基準が止まりかけている（または
        止まりから出ていく）あいだ、ブレーキ開度をこの境界より浅くしない下限。

背景（2026-09-19 04:06 の手順2 実機ログ `drive_log_real_20260919_040613.csv` の実測。
ProblemReport_20260916 段4）: クリープ平衡 4.77 km/h からブレーキを「不感帯 + オフセット」で
保持した実測で、+6.0%（開度 17.79%）は 0.33 km/h に浮き、**+8.0%（19.79%）で停止する**。
境界は +6.0〜+8.0% の間。

**旧同定コード `stop_brake_curve.py`（削除済み）は別のものを測っていた。** 保存値
`stop_brake_speeds_kmh: [0.0, 0.25, 0.75]` / `openings_pct: [28.32, 25.558, 26.551]` は使えない。
理由は 2 つ、どちらも旧仕様の設計ミス:

1. **速度の取り方が構造的に誤り。** 「停止する 1 行前の車速」を速度軸に取ったが、車は減速し
   ながら止まるのでその車速は必ず 0 付近になる。この走行の停止イベント 45 件すべてで
   0.03〜1.19 km/h。**定義上 0〜1.25 km/h より上には点が出ない。** 4 km/h 側の情報は
   「その保持が停止に到達した」という事実の側にあり、旧仕様はそれを速度軸に載せられず
   捨てていた。
2. **開度が高めに偏る。** 45 件のうち 43 件は停止確認の刻み送り（0.5mm/1s）中の停止で
   23.35〜28.32% と記録される。これは「止まるのに必要な開度」ではなく「停止判定の瞬間に
   刻みがどこまで進んでいたか」であり、境界（17.8〜19.8%）より 5〜10% 深い。

**そこでカーブをやめ、定数 1 個（不感帯からの超過 [%]）に置き換える。** 停止側と発進側が
1 本の下限規則で書けることが実測で確認できている（`ff_candidate._apply_brake_trim` 参照）。

同定のしかた（`estimate_stop_brake_floor` 参照）:
    `phase` 列は `tests.research.drive_log.read_drive_logs` が `DriveLog` に読み戻さない
    （CSV には残るが読み込み時に捨てられ常に None）ため、走行時の意図（保持中か刻み送り中か）
    をログから復元できない。そこで代わりに、**手順2 が指令した候補開度**
    （`learning.creep_brake_hold_offsets_pct` から作った開度列）を呼び出し側から渡す設計にし、
    候補開度ごとに実際に保持できていた区間（プラトー）だけを抽出する。

    各候補開度の保持プラトーの**先頭車速**がクリープ平衡（`start_speed_kmh`）付近であることを
    条件にするのが要点（`start_tol_kmh` の窓）。これが以下の 3 種類の混入を排除する唯一の
    条件になっている:
        - A5 の高ブレーキ停車（120 km/h からの制動）
        - A4 のトリム階段（19 km/h からの制動）
        - ペダル探索の停止確認刻み送り（保持ではなく 0.5mm/1s で開度を上げ続ける区間）
    刻み送り中の各開度は 1 秒未満しか続かないため、`min_float_s`（既定 5.0s）未満の浮いた
    区間は「浮いた」に数えない（探索の刻みを「その開度で浮いた」と誤って記録しないため）。

**ビン内で最小を採るのは今回は正しい。** 旧仕様（`stop_brake_curve.py`。速度ビンで中央値）で
最小を採ると誤りだったのは、高速から momentum を持って突っ込んだ停車と低速からの停車が
混ざった集団だったため（惰行減速がすでに仕事をしているぶん浅い開度で止まれる外れ値が
最小に選ばれる）。ここは**同じクリープ平衡速度から開度だけを振った意図的な掃引**なので、
「止まった最小開度」が定義どおり境界になる（二分探索と同じ理屈）。上の先頭車速条件が
この前提（同じ平衡速度からの掃引であること）を保証している。

CLI:
    .venv/bin/python -m tests.research.stop_brake_floor \
        tests/research/results/drive_log_real_<日時>.csv

    止まった開度・浮いた開度と、得られたオフセットを表で出す。ファイルは書かない（図も無い）。
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from src.domain.model_training import STOP_SPEED_KMH, _group_by_session
from src.models.drive_log import DriveLog
from src.models.profile import FeedforwardParams
from tests.research.config import DEFAULT_CONFIG_PATH, load_config
from tests.research.debug_process23 import md_table
from tests.research.drive_log import read_drive_logs
from tests.research.vehicle import build_vehicle_profile


@dataclass(frozen=True)
class StopBrakeFloor:
    offset_pct: float  # 不感帯からの超過 [%]。0.0 で未同定
    stopped_pct: tuple[float, ...]  # 停止した保持開度（昇順・重複無し）
    floated_pct: tuple[float, ...]  # 浮いた保持開度（昇順・重複無し）

    @property
    def identified(self) -> bool:
        return self.offset_pct > 0.0


def _contiguous_true_runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """`mask` 内で True が連続する区間を `[start, end)` のリストで返す。"""
    runs: list[tuple[int, int]] = []
    n = len(mask)
    i = 0
    while i < n:
        if not mask[i]:
            i += 1
            continue
        start = i
        while i < n and mask[i]:
            i += 1
        runs.append((start, i))
    return runs


def estimate_stop_brake_floor(
    logs: list[DriveLog],
    params: FeedforwardParams,
    *,
    candidate_openings_pct: Sequence[float],
    start_speed_kmh: float,
    start_tol_kmh: float,
    min_float_s: float,
    opening_tol_pct: float,
) -> StopBrakeFloor:
    """走行ログからクリープ域ブレーキの下限（不感帯からの超過 [%]）を推定する。

    サンプル条件・アルゴリズムはモジュール docstring 参照。手順:
        1. セッション分割（`_group_by_session`）
        2. 各セッションで `accel_opening <= accel_deadband_pct and
           brake_opening > brake_deadband_pct` の連続区間（run）を作る
        3. 各 run の中で、各候補開度について `abs(brake_opening - o) <= opening_tol_pct` の
           極大連続部分区間（保持のプラトー）を列挙する
        4. プラトー先頭行の車速が `start_speed_kmh ± start_tol_kmh` に入るものだけ残す
        5. 残ったプラトーに `STOP_SPEED_KMH` 以下の行があれば stopped、無くて
           `min_float_s` 以上続けば floated に候補開度を記録する
        6. stopped が空、または `max(floated) >= min(stopped)`（矛盾）なら未同定
           （`offset_pct=0.0`。ただし `stopped_pct`/`floated_pct` は観測どおり返す）
        7. それ以外は `offset_pct = min(stopped) - brake_deadband_pct`
           （0 以下になった場合も未同定）
    """
    accel_db = params.accel_deadband_pct
    brake_db = params.brake_deadband_pct

    stopped: set[float] = set()
    floated: set[float] = set()

    for session_logs in _group_by_session(logs):
        if len(session_logs) < 1:
            continue
        speed = np.clip(
            np.array([lg.actual_speed_kmh for lg in session_logs], dtype=float), 0.0, None
        )
        accel = np.array([lg.accel_opening for lg in session_logs], dtype=float)
        brake = np.array([lg.brake_opening for lg in session_logs], dtype=float)
        pedal_mask = (accel <= accel_db) & (brake > brake_db)

        for run_start, run_end in _contiguous_true_runs(pedal_mask):
            run_brake = brake[run_start:run_end]
            for opening in candidate_openings_pct:
                hold_mask = np.abs(run_brake - opening) <= opening_tol_pct
                for sub_start_rel, sub_end_rel in _contiguous_true_runs(hold_mask):
                    sub_start = run_start + sub_start_rel
                    sub_end = run_start + sub_end_rel  # プラトーは [sub_start, sub_end)

                    lead_speed = float(speed[sub_start])
                    if not (
                        start_speed_kmh - start_tol_kmh
                        <= lead_speed
                        <= start_speed_kmh + start_tol_kmh
                    ):
                        continue  # A4/A5・刻み送りなど、平衡からの掃引ではないプラトーを除外

                    window_speed = speed[sub_start:sub_end]
                    if np.any(window_speed <= STOP_SPEED_KMH):
                        stopped.add(float(opening))
                        continue

                    duration_s = (
                        session_logs[sub_end - 1].timestamp - session_logs[sub_start].timestamp
                    ).total_seconds()
                    if duration_s >= min_float_s:
                        floated.add(float(opening))

    stopped_pct = tuple(sorted(stopped))
    floated_pct = tuple(sorted(floated))

    if not stopped_pct:
        return StopBrakeFloor(offset_pct=0.0, stopped_pct=stopped_pct, floated_pct=floated_pct)
    if floated_pct and max(floated_pct) >= min(stopped_pct):
        return StopBrakeFloor(offset_pct=0.0, stopped_pct=stopped_pct, floated_pct=floated_pct)

    offset_pct = min(stopped_pct) - brake_db
    if offset_pct <= 0.0:
        return StopBrakeFloor(offset_pct=0.0, stopped_pct=stopped_pct, floated_pct=floated_pct)

    return StopBrakeFloor(offset_pct=offset_pct, stopped_pct=stopped_pct, floated_pct=floated_pct)


__all__ = ["StopBrakeFloor", "estimate_stop_brake_floor"]


# ─────────────────────────────────────────────────────────────────────
# 点検 CLI
# ─────────────────────────────────────────────────────────────────────


def run(csv_path: Path, config_path: Path) -> int:
    cfg = load_config(config_path)
    params = build_vehicle_profile(cfg).feedforward_params
    logs = read_drive_logs(csv_path)
    lr = cfg.learning
    ff = cfg.feedforward

    candidate_openings_pct = [
        round(ff.brake_deadband_pct + offset, 2) for offset in lr.creep_brake_hold_offsets_pct
    ]

    floor = estimate_stop_brake_floor(
        logs, params,
        candidate_openings_pct=candidate_openings_pct,
        start_speed_kmh=params.creep_speed_kmh,
        start_tol_kmh=lr.stop_brake_floor_start_tol_kmh,
        min_float_s=lr.stop_brake_floor_min_float_s,
        opening_tol_pct=lr.stop_brake_floor_opening_tol_pct,
    )

    print(f"候補開度: {candidate_openings_pct}")
    print(
        f"クリープ平衡: {params.creep_speed_kmh:.2f} km/h"
        f"（± {lr.stop_brake_floor_start_tol_kmh:g}）"
    )
    header = ["開度 [%]", "結果"]
    rows = [[f"{o:.2f}", "停止"] for o in floor.stopped_pct]
    rows += [[f"{o:.2f}", "浮く"] for o in floor.floated_pct]
    rows.sort(key=lambda r: float(r[0]))
    print(md_table(header, rows))
    print(f"\noffset_pct = {floor.offset_pct:.2f}（identified={floor.identified}）")
    return 0 if floor.identified else 1


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("csv", type=Path, help="停止ブレーキ下限の元にする走行ログ CSV（手順2）")
    ap.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    args = ap.parse_args(argv)
    return run(args.csv, args.config)


if __name__ == "__main__":
    raise SystemExit(main())
