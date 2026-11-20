# ruff: noqa: E501
"""サーバーのログ（``src``・``uvicorn.error``）の WARNING 以上をエラーのできごとにするハンドラ。

``LogBufferHandler``（src/infra/log_buffer.py）と同じく、ロガーへ付けて使う。次は記録しない。
- ``extra={"error_recorded": True}`` の印がある行（JobManager が自分の経路でもう記録した行）
- 記録係・リポジトリ・このハンドラ自身のログ（無限ループ防止）
"""

from __future__ import annotations

import logging
import threading
from collections.abc import Callable

from src.models.error_event import LEVEL_ERROR, LEVEL_WARNING, SOURCE_SERVER, NewErrorEvent

#: 記録済みの行に付ける印（``logger.warning(..., extra=RECORDED)``）
RECORDED = {"error_recorded": True}

#: これらのロガー（と子）の行は拾わない
IGNORED_LOGGERS = (
    "src.app.error_recorder",
    "src.infra.error_repository",
    "src.infra.error_log_handler",
)

TITLE_MAX = 120


class ErrorLogHandler(logging.Handler):
    """WARNING 以上のログ行を ``submit``（ErrorRecorder.record）へ渡す。"""

    def __init__(self, submit: Callable[[NewErrorEvent], None], level: int = logging.WARNING) -> None:
        super().__init__(level=level)
        self._submit = submit
        self._guard = threading.local()

    def emit(self, record: logging.LogRecord) -> None:
        if record.levelno < logging.WARNING or getattr(record, "error_recorded", False):
            return
        if any(record.name == n or record.name.startswith(n + ".") for n in IGNORED_LOGGERS):
            return
        if getattr(self._guard, "busy", False):  # 記録の途中で出たログは拾わない
            return
        self._guard.busy = True
        try:
            message = record.getMessage()
            detail = message
            if record.exc_info:
                formatter = self.formatter or logging.Formatter()
                detail = f"{message}\n{formatter.formatException(record.exc_info)}"
            first = message.strip().splitlines()[0] if message.strip() else record.name
            self._submit(
                NewErrorEvent(
                    level=LEVEL_ERROR if record.levelno >= logging.ERROR else LEVEL_WARNING,
                    source=SOURCE_SERVER,
                    title=first[:TITLE_MAX],
                    detail=detail,
                    context={"logger": record.name},
                    dedupe=True,
                )
            )
        except Exception:  # noqa: BLE001 - ログの記録で例外を出さない
            pass
        finally:
            self._guard.busy = False
