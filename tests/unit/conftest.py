# ruff: noqa: E501
"""tests/unit 共通の fixture。"""

from __future__ import annotations

import sys
from collections.abc import Iterator
from typing import Any

import pytest

#: patches.install_all（core の差し替え）をプロセスに残すテストファイル。終わりに元へ戻す
_PATCHING_MODULES = {
    "tests.unit.test_emergency_stop",
    "tests.unit.test_robot_serve",
    "tests.unit.test_robot_runner",
}


@pytest.fixture(autouse=True)
def _restore_robot_patches(request: pytest.FixtureRequest) -> Iterator[None]:
    """``patches.install_all`` などの差し替えを、テストの終わりに元へ戻す（ProblemReport_20261120 O-8）。

    ``patches.install_all`` / ``install_state_events`` はプロセスに差し替えを残し、``install_state_events`` は
    呼ぶたびに終了処理（core_main.shutdown）を包み重ねる。残ると、後に走るテスト（test_robot_serve の
    「shutdown の state イベントが 1 回」・test_cancel_guard の「包み済みなら二重に包まない」）が、
    重なった分だけ数えて落ちた（単独なら通る）。差し替えを行うテストファイルでだけ、src.robot.* の全モジュールの属性と、
    差し替える 4 つのクラスの属性を、テスト前の値へ戻す。
    """
    if request.module.__name__ not in _PATCHING_MODULES:
        yield
        return
    from src.robot import (
        schedule_job,  # noqa: F401, PLC0415 - 遅延 import のモジュールを先に読み込んでおく
    )
    from src.robot.core import excite, pattern_loop  # noqa: PLC0415
    from src.robot.core import hardware as core_hardware  # noqa: PLC0415
    from src.robot.core import mode_drive as core_mode_drive  # noqa: PLC0415

    owners: list[Any] = [m for n, m in list(sys.modules.items()) if n.startswith("src.robot.") and m is not None]
    owners += [
        core_hardware.InitReport,
        core_mode_drive._ModeRun,  # noqa: SLF001
        pattern_loop.PatternLoop,
        excite._ExciteRun,  # noqa: SLF001
    ]
    saved = [(o, dict(vars(o))) for o in owners]
    yield
    for owner, before in saved:
        for key in [k for k in vars(owner) if k not in before and not k.startswith("__")]:
            try:
                delattr(owner, key)
            except (AttributeError, TypeError):
                pass
        for key, value in before.items():
            if key.startswith("__") or vars(owner).get(key) is value:
                continue
            try:
                setattr(owner, key, value)
            except (AttributeError, TypeError):
                pass
