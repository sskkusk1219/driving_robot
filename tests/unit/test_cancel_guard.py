"""cancel_guard.raise_if_cancelling: cancel の最中の通信例外を CancelledError に戻す。"""

from __future__ import annotations

import asyncio
import contextlib

import pytest

from src.robot.cancel_guard import raise_if_cancelling


async def test_cancelling_task_raises_cancelled_error() -> None:
    # cancel されている最中（cancelling() > 0）のタスクでは CancelledError になる
    async def body() -> None:
        asyncio.current_task().cancel()  # type: ignore[union-attr]
        exc = Exception("Request cancelled outside library")
        with pytest.raises(asyncio.CancelledError) as info:
            raise_if_cancelling(exc)
        assert info.value.__cause__ is exc

    # 最後まで進んだタスクは cancel 済みとして終わる（本体の中の確認は済んでいる）
    with contextlib.suppress(asyncio.CancelledError):
        await asyncio.create_task(body())


async def test_not_cancelling_does_nothing() -> None:
    raise_if_cancelling(Exception("通信失敗"))


def test_install_cancel_guard_wraps_three_methods(monkeypatch: pytest.MonkeyPatch) -> None:
    from src.robot import patches
    from src.robot.core import excite, mode_drive, pattern_loop

    targets = [
        (mode_drive._ModeRun, "drive_axis"),
        (pattern_loop.PatternLoop, "_drive_or_sense"),
        (excite._ExciteRun, "_drive_axis"),
    ]
    origs = {}
    for cls, name in targets:
        origs[(cls, name)] = cls.__dict__[name]
        monkeypatch.setattr(cls, name, cls.__dict__[name])  # テスト後に元へ戻す
    patches.install_cancel_guard()
    for cls, name in targets:
        assert cls.__dict__[name].__wrapped__ is origs[(cls, name)]
    # 2 回呼んでも二重に包まない
    patches.install_cancel_guard()
    for cls, name in targets:
        assert cls.__dict__[name].__wrapped__ is origs[(cls, name)]


async def test_install_cancel_guard_converts_in_cancelling_task(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    from src.robot import patches
    from src.robot.core import mode_drive

    async def fake(self: object, *args: object, **kwargs: object) -> None:
        raise Exception("Request cancelled outside library")  # noqa: TRY002

    monkeypatch.setattr(mode_drive._ModeRun, "drive_axis", fake)
    patches.install_cancel_guard()
    method = mode_drive._ModeRun.drive_axis

    async def body_cancelled() -> None:
        asyncio.current_task().cancel()  # type: ignore[union-attr]
        with pytest.raises(asyncio.CancelledError):
            await method(object())

    async def body_normal() -> None:
        with pytest.raises(Exception, match="Request cancelled") as info:
            await method(object())
        assert not isinstance(info.value, asyncio.CancelledError)

    with contextlib.suppress(asyncio.CancelledError):
        await asyncio.create_task(body_cancelled())
    await asyncio.create_task(body_normal())
