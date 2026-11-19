# ruff: noqa: E501
"""core の SessionLog の派生。record() でライブ値（sample イベント）を 5 Hz に間引いて流し、
close() で保存した CSV を csv イベントで知らせる。

RPD_Phase_20261115_PortResearch.md §3 段2。core の main / pattern_drive が SessionLog を作る所を
この派生に差し替える 1 か所は patches.install_streaming_log()。CSV の書き方・列は SessionLog のまま
（tests と同じ約 100 列）。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

from src.robot import events
from src.robot.core.drive_log import SECTION_MODE_DRIVE, DriveSample, SessionLog
from src.robot.core.vehicle import pulse_to_opening

#: sample イベントを出す最短間隔 [s]（5 Hz）
SAMPLE_INTERVAL_S = 0.2

#: タイムスケジュール再生（開ループ。src/robot/schedule_job.py）の CSV の section 列
SECTION_SCHEDULE_DRIVE = "SCHEDULE_DRIVE"

#: csv イベントの kind
KIND_LEARNING = "learning"
KIND_DRIVE = "drive"


def csv_kind(sections: set[str]) -> str:
    """CSV の種類。モード走行（MODE_DRIVE）・スケジュール再生（SCHEDULE_DRIVE）の行を含めば drive（Web が DB へ入れて消す走行ログ）、
    それ以外（ペダル探索・G ランプ・FEL・走行前チェックだけ）は learning（手順 2・12〜14 が後で
    読み直す学習データ。vehicles/<車ID>/data/ に残す）。"""
    return (
        KIND_DRIVE if sections & {SECTION_MODE_DRIVE, SECTION_SCHEDULE_DRIVE} else KIND_LEARNING
    )


def sample_fields(sample: DriveSample) -> dict[str, Any]:
    """DriveSample → sample イベントの項目（単位は README の表）。"""
    d = sample.data

    def actual_pct(monitor: Any) -> float | None:
        if monitor is None:
            return None
        return round(pulse_to_opening(monitor.position_pulse), 3)

    return {
        "t_s": round(sample.elapsed_s, 3),
        "mode_time_s": None if sample.mode_time_s is None else round(sample.mode_time_s, 3),
        "ref_kmh": None if d.ref_speed_kmh is None else round(d.ref_speed_kmh, 3),
        "speed_kmh": round(d.actual_speed_kmh, 3),
        "accel_cmd_pct": round(d.accel_opening, 3),
        "brake_cmd_pct": round(d.brake_opening, 3),
        "accel_actual_pct": actual_pct(sample.monitor_accel),
        "brake_actual_pct": actual_pct(sample.monitor_brake),
        "section": sample.section,
        "pattern": sample.pattern,  # モードの区間名（WLTP は Low/Mid/High/ExHi。画面の「区間」）
        "phase": sample.phase,
    }


class StreamingSessionLog(SessionLog):
    """SessionLog ＋ ライブ値の配信 ＋ CSV 保存の通知。"""

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._last_emit_s = -1e9

    def record(self, data: Any, **kwargs: Any) -> DriveSample:
        sample = super().record(data, **kwargs)
        if sample.elapsed_s - self._last_emit_s >= SAMPLE_INTERVAL_S:
            self._last_emit_s = sample.elapsed_s
            events.emit("sample", **sample_fields(sample))
        return sample

    async def close(self) -> Path | None:
        already_closed = self.closed
        path = await super().close()
        if not already_closed and path is not None:
            events.emit(
                "csv",
                path=str(path.resolve()),
                kind=csv_kind({s.section for s in self.samples}),
                rows=len(self.samples),
            )
        return path
