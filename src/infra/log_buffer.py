"""サーバーログのリングバッファ（WebUI の「現在ステータス」表示用）。

``logging.Handler`` のサブクラス。直近 ``maxlen`` 行を ``deque`` に保持し、
``seq``（連番）で差分取得できる。ログを出すスレッドと API を処理するスレッドが
別になり得るため ``threading.Lock`` で保護する。
"""

import logging
import threading
import time
from collections import deque
from datetime import datetime
from typing import Any


class LogBufferHandler(logging.Handler):
    """直近 N 行のログをメモリに保持するハンドラ。"""

    def __init__(self, maxlen: int = 500, level: int = logging.INFO) -> None:
        super().__init__(level=level)
        self._lines: deque[dict[str, Any]] = deque(maxlen=maxlen)
        self._buf_lock = threading.Lock()
        self._seq = 0
        self._t0 = time.monotonic()

    def emit(self, record: logging.LogRecord) -> None:
        try:
            message = record.getMessage()
            if record.exc_info:
                message = f"{message}: {record.exc_info[1]!r}"
            with self._buf_lock:
                self._seq += 1
                self._lines.append(
                    {
                        "seq": self._seq,
                        "t": round(time.monotonic() - self._t0, 1),
                        "time": datetime.fromtimestamp(record.created).isoformat(
                            timespec="milliseconds"
                        ),
                        "level": record.levelname,
                        "logger": record.name,
                        "message": message,
                    }
                )
        except Exception:
            self.handleError(record)

    @property
    def last_seq(self) -> int:
        with self._buf_lock:
            return self._seq

    def get_lines(self, after: int | None = None, limit: int = 200) -> list[dict[str, Any]]:
        """``seq > after`` の行を古い順に最大 ``limit`` 行返す。

        ``after`` 省略時は直近 ``limit`` 行。``after`` 指定時は ``after`` の直後から
        古い順に ``limit`` 行（取りこぼさないよう、続きは次回の ``after`` で取る）。
        """
        with self._buf_lock:
            if after is None:
                return list(self._lines)[-limit:]
            return [ln for ln in self._lines if ln["seq"] > after][:limit]
