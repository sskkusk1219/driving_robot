"""src/ 配下の .py が tests を import していないことを確かめる（本番とテストの隔離）。

src/robot/core は tests/research から写したもの（scripts/port_research.py）。写すときに
`tests.research` → `src.robot.core` へ置き換えるので、残っていれば置き換え漏れ。
"""

from __future__ import annotations

import ast
from pathlib import Path

SRC_DIR = Path(__file__).resolve().parents[2] / "src"


def _tests_imports(path: Path) -> list[str]:
    bad: list[str] = []
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"), filename=str(path))):
        if isinstance(node, ast.Import):
            bad += [
                f"{path}:{node.lineno}: import {a.name}"
                for a in node.names
                if a.name == "tests" or a.name.startswith("tests.")
            ]
        elif isinstance(node, ast.ImportFrom) and node.level == 0:
            mod = node.module or ""
            if mod == "tests" or mod.startswith("tests."):
                bad.append(f"{path}:{node.lineno}: from {mod} import ...")
    return bad


def test_src_does_not_import_tests() -> None:
    files = sorted(SRC_DIR.rglob("*.py"))
    assert files, "src/ に .py が見つかりません"
    bad = [line for f in files for line in _tests_imports(f)]
    assert not bad, "src/ が tests を import しています:\n" + "\n".join(bad)
