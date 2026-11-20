"""cancel の最中に起きた通信の例外を、CancelledError として投げ直す小さな道具。

理由: 利用者の「走行終了」（stop コマンド）や SIGINT で内側のタスクを ``cancel()`` したとき、
Modbus の要求の最中に cancel が届くと、pymodbus は CancelledError を
``ModbusIOException("Request cancelled outside library")`` に変えて投げる。それを制御ループの
``except Exception`` が「アクチュエータ通信に失敗しました」の DriveError にすると、停止のはずなのに
通信失敗の ERROR が出る（ProblemReport_20261120_src全動作チェック B-2）。cancel が sleep の最中に
届けば CancelledError がそのまま上へ抜けて停止の流れに入る（こちらが正しい道）。
"""

from __future__ import annotations

import asyncio


def raise_if_cancelling(exc: BaseException) -> None:
    """今のタスクが cancel されている最中なら、通信の例外ではなく CancelledError として投げ直す。

    ``except Exception as exc:`` の先頭で呼ぶ。cancel されていなければ何もしない（呼び出し側が
    そのまま通信失敗として扱う）。pymodbus が cancel を ModbusIOException に変えた場合に、
    停止の流れ（CancelledError）へ戻すために使う。
    """
    task = asyncio.current_task()
    if task is not None and task.cancelling() > 0:
        raise asyncio.CancelledError() from exc
