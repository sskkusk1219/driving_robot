"""tests/research が `src/` に依存しないことの回帰テスト（ProblemReport_20260924）。

`tests/` は `tests/` だけで完結し、`src/` を丸ごと削除しても動かなければならない。
tests/research 配下の全 .py を AST で走査し、`src` パッケージの import が1つも無いことを確かめる
（関数内の遅延 import・`importlib.import_module("src...")`・`__import__("src...")` も対象）。
"""

from __future__ import annotations

import ast
from pathlib import Path

RESEARCH_DIR = Path(__file__).parent


def _src_references(path: Path) -> list[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    found: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            if node.level == 0 and node.module and node.module.split(".")[0] == "src":
                found.append(f"{path.name}:{node.lineno} from {node.module} import ...")
        elif isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name.split(".")[0] == "src":
                    found.append(f"{path.name}:{node.lineno} import {alias.name}")
        elif isinstance(node, ast.Call):
            func = node.func
            name = func.attr if isinstance(func, ast.Attribute) else getattr(func, "id", "")
            if name in {"import_module", "__import__"} and node.args:
                arg = node.args[0]
                if (
                    isinstance(arg, ast.Constant)
                    and isinstance(arg.value, str)
                    and arg.value.split(".")[0] == "src"
                ):
                    found.append(f"{path.name}:{node.lineno} {name}({arg.value!r})")
    return found


def test_research_does_not_import_src() -> None:
    files = sorted(RESEARCH_DIR.glob("*.py"))
    assert files  # 走査対象が空でないこと
    offenders = [ref for f in files for ref in _src_references(f)]
    assert not offenders, "tests/research が src を参照している:\n" + "\n".join(offenders)


def test_scanner_detects_src_imports(tmp_path: Path) -> None:
    """走査自体が効いていること（すり抜けて常に空になる事故を防ぐ）。"""
    sample = tmp_path / "sample.py"
    sample.write_text(
        "import src.models.profile\n"
        "from src.domain import pre_check\n"
        "import importlib\n"
        "def f():\n"
        "    from src.infra.settings import load_settings\n"
        "    importlib.import_module('src.utils.time')\n"
        "from tests.research import config\n",
        encoding="utf-8",
    )
    assert len(_src_references(sample)) == 4
