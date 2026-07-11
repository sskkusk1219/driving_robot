"""100ms 周期で PostgreSQL に走行ログを書き込むインフラコンポーネント。"""

import json
import uuid
from datetime import UTC, datetime
from typing import Any

import asyncpg

from src.models.drive_log import DriveLogData

# drive_logs バッファのフラッシュ間隔 [s]。1サンプル=1INSERT=1プール接続取得だと
# 10件/s・10時間走行で36万往復のDBラウンドトリップが発生するため、この間隔ごとに
# executemany でまとめて書き込む（E5 レビュー指摘）。
LOG_FLUSH_INTERVAL_S: float = 0.5

_INSERT_LOG_SQL = """
    INSERT INTO drive_logs
        (session_id, timestamp, ref_speed_kmh, actual_speed_kmh,
         accel_opening, brake_opening, accel_pos, brake_pos,
         accel_current, brake_current)
    VALUES
        ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10)
"""


class LogWriter:
    """走行セッションとログデータを PostgreSQL に永続化する。

    asyncpg.Connection または asyncpg.Pool を受け取る。Pool を渡した場合は
    各操作（start_session / write_log / end_session）ごとに Pool.execute が
    接続を都度取得・解放するため、Web 駆動で寿命が不定な走行セッションに適合する。
    単一 Connection を渡した場合は呼び出し元が接続寿命を管理する。

    write_log は即座に INSERT せず、LOG_FLUSH_INTERVAL_S 毎にバッファをまとめて
    executemany する（E5 レビュー指摘）。timestamp はサンプル取得時刻をアプリ側
    （write_log 呼び出し時）で記録する。バッチ内で SQL の NOW() を使うと、同一
    トランザクション内の全行が同じ時刻になり時系列としての意味を失うため。
    """

    def __init__(self, conn: asyncpg.Connection | asyncpg.Pool) -> None:
        self._conn = conn
        self._log_buffer: list[tuple[Any, ...]] = []
        self._last_flush_at: float | None = None

    async def start_session(
        self,
        profile_id: str,
        mode_id: str | None,
        run_type: str,
        cycle_id: str | None = None,
    ) -> str:
        """drive_sessions に INSERT し、生成したセッション ID (UUID 文字列) を返す。

        Args:
            profile_id: 使用する車両プロファイルの UUID 文字列
            mode_id: 走行モードの UUID 文字列（手動運転・学習運転時は None）
            run_type: 'auto' | 'manual' | 'learning' | 'tuning'
            cycle_id: 参加する学習サイクルの UUID 文字列（manual・通常autoは None）
        """
        session_id = str(uuid.uuid4())
        await self._conn.execute(
            """
            INSERT INTO drive_sessions
                (id, profile_id, mode_id, run_type, started_at, status, cycle_id)
            VALUES
                ($1, $2, $3, $4, NOW(), 'running', $5)
            """,
            session_id,
            profile_id,
            mode_id,
            run_type,
            cycle_id,
        )
        return session_id

    async def start_cycle(self, profile_id: str) -> str:
        """learning_cycles に INSERT し、生成したサイクル ID (UUID 文字列) を返す。"""
        cycle_id = str(uuid.uuid4())
        await self._conn.execute(
            """
            INSERT INTO learning_cycles
                (id, profile_id, status, started_at, detail)
            VALUES
                ($1, $2, 'running', NOW(), '{}'::jsonb)
            """,
            cycle_id,
            profile_id,
        )
        return cycle_id

    async def end_cycle(
        self, cycle_id: str, status: str, detail: dict[str, Any] | None = None
    ) -> None:
        """learning_cycles.ended_at / status / detail を UPDATE してサイクルを終了する。

        Args:
            cycle_id: 終了するサイクルの UUID 文字列
            status: 'completed' | 'error' | 'aborted'
            detail: 段階別ゲイン/コスト/モデルパス等（JSONB として保存）
        """
        await self._conn.execute(
            """
            UPDATE learning_cycles
            SET ended_at = NOW(), status = $2, detail = $3::jsonb
            WHERE id = $1
            """,
            cycle_id,
            status,
            json.dumps(detail if detail is not None else {}),
        )

    async def write_log(self, session_id: str, data: DriveLogData) -> None:
        """drive_logs へ書き込むサンプルをバッファに追加する。

        100ms 周期で呼ばれることを前提とする。即座に INSERT せず、
        LOG_FLUSH_INTERVAL_S 毎にバッファをまとめて executemany する（E5 レビュー指摘）。
        """
        now = datetime.now(tz=UTC)
        self._log_buffer.append(
            (
                session_id,
                now,
                data.ref_speed_kmh,
                data.actual_speed_kmh,
                data.accel_opening,
                data.brake_opening,
                data.accel_pos,
                data.brake_pos,
                data.accel_current,
                data.brake_current,
            )
        )
        if self._last_flush_at is None:
            self._last_flush_at = now.timestamp()
        elif now.timestamp() - self._last_flush_at >= LOG_FLUSH_INTERVAL_S:
            await self._flush_log_buffer()

    async def _flush_log_buffer(self) -> None:
        """バッファ中の drive_logs サンプルをまとめて INSERT する。空なら何もしない。"""
        self._last_flush_at = datetime.now(tz=UTC).timestamp()
        if not self._log_buffer:
            return
        rows = self._log_buffer
        self._log_buffer = []
        await self._conn.executemany(_INSERT_LOG_SQL, rows)

    async def end_session(self, session_id: str, status: str) -> None:
        """drive_sessions.ended_at と status を UPDATE してセッションを終了する。

        バッファに残っている drive_logs サンプルを先にフラッシュしてから終了する
        （フラッシュ間隔未達のまま走行終了しても最後のサンプルが失われないようにする）。

        Args:
            session_id: 終了するセッションの UUID 文字列
            status: 'completed' | 'error' | 'emergency'
        """
        await self._flush_log_buffer()
        await self._conn.execute(
            """
            UPDATE drive_sessions
            SET ended_at = NOW(), status = $2
            WHERE id = $1
            """,
            session_id,
            status,
        )

    async def reap_interrupted_sessions(self) -> int:
        """起動時に取り残された未終了セッション・学習サイクルを 'error' で閉じる。

        サーバ起動直後に status='running'（または ended_at IS NULL）のセッションが
        残っているのは、前回のプロセス異常終了（強制 kill / クラッシュ）で
        end_session が呼ばれなかった孤児セッションのみ。実行中の走行は存在し得ない
        ため、これらを終了済みに是正する。ended_at は記録された最後のログ時刻
        （なければ started_at）に設定し、走行時間表示を妥当にする。
        同様に status='running' のまま残った孤児 learning_cycles も 'error' で回収する
        （オーケストレータが走行中にプロセスが落ちたケース）。

        Returns:
            是正したセッション件数。
        """
        result = await self._conn.execute(
            """
            UPDATE drive_sessions s
            SET status = 'error',
                ended_at = COALESCE(
                    (SELECT MAX(l.timestamp) FROM drive_logs l WHERE l.session_id = s.id),
                    s.started_at
                )
            WHERE s.ended_at IS NULL OR s.status = 'running'
            """
        )
        await self._conn.execute(
            """
            UPDATE learning_cycles
            SET status = 'error', ended_at = NOW()
            WHERE status = 'running'
            """
        )
        # asyncpg は "UPDATE <n>" を返す
        return int(result.split()[-1]) if result else 0
