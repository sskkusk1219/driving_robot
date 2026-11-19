#!/usr/bin/env python3
"""tests/research から src/robot/core/ へ研究ハーネスを写す道具（段0）。

使い方:
    .venv/bin/python scripts/port_research.py            # 写す（上書き・消えたものは削除）
    .venv/bin/python scripts/port_research.py --report   # 写した後 "tests/" を含む文字列を出す
    .venv/bin/python scripts/port_research.py --check    # 写さず差だけ確かめる（差があれば 1）

仕様（docs/Phase/RPD_Phase_20261115_PortResearch.md §3 段0）:
- 写す対象 = tests/research/main.py から import で辿れる全モジュール（関数内の遅延 import も含める。
  ast で辿る）。test_*.py・results/・private/ は辿れても写さない（辿れたら警告）。
  辿れないものは写さない。
- `tests.research` → `src.robot.core` に置き換える（import 文・ロガー名などの文字列中も）。
- 各ファイルの先頭に「移植元」の 1 行コメントを足す（shebang の次の行。docstring より前のコメントは
  ruff の E402 等に触れない。行が長いので E501 だけ noqa）。
- core は手で直さない。直すのは tests 側 → この道具を再実行。

注意: この道具は tests/ を読むだけで、1 文字も書き換えない。
"""

from __future__ import annotations

import argparse
import ast
import subprocess
import sys
from datetime import datetime
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_PKG = "tests.research"
SRC_DIR = REPO_ROOT / "tests" / "research"
DST_PKG = "src.robot.core"
DST_DIR = REPO_ROOT / "src" / "robot" / "core"
ENTRY = "main"
PORTED_MD = DST_DIR / "PORTED.md"
INIT_TEXT = '''"""tests/research から写した制御（研究ハーネス）。

このフォルダは scripts/port_research.py が写したもの。手で直さない
（直すのは tests/research 側 → port_research.py を再実行）。一覧は PORTED.md。
"""
'''
# 写さない（辿れても）ファイル名のパターン
_EXCLUDE_PREFIX = ("test_",)


def _mod_path(name: str) -> Path:
    return SRC_DIR / f"{name}.py"


def _imported_modules(path: Path) -> tuple[set[str], list[str]]:
    """ファイル中（関数内も含む）の tests.research.<module> への import 先と、問題の一覧を返す。"""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    found: set[str] = set()
    problems: list[str] = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                _take(alias.name, None, found, problems, path, node.lineno)
        elif isinstance(node, ast.ImportFrom):
            if node.level:
                problems.append(f"{path.name}:{node.lineno}: 相対 import は未対応")
                continue
            mod = node.module or ""
            for alias in node.names:
                _take(mod, alias.name, found, problems, path, node.lineno)
    return found, problems


def _take(
    module: str, name: str | None, found: set[str], problems: list[str], path: Path, lineno: int
) -> None:
    if module == "tests" or (module.startswith("tests.") and not module.startswith(SRC_PKG)):
        # `from tests import x` / `tests.hardware` など。tests.research の外は写せない
        if module != "tests" or name != "research":
            problems.append(f"{path.name}:{lineno}: {SRC_PKG} の外への import: {module}")
        return
    if module == SRC_PKG:
        # from tests.research import a, b  → a・b がモジュールなら辿る
        if name is not None and _mod_path(name).exists():
            found.add(name)
        return
    if module.startswith(SRC_PKG + "."):
        found.add(module[len(SRC_PKG) + 1 :].split(".")[0])


def trace_modules() -> tuple[list[str], list[str]]:
    """main から辿れるモジュール名（ソート済み）と警告の一覧。"""
    seen: set[str] = set()
    warnings: list[str] = []
    todo = [ENTRY]
    while todo:
        name = todo.pop()
        if name in seen:
            continue
        path = _mod_path(name)
        if not path.exists():
            warnings.append(f"{name}: {path} がありません（パッケージ属性かもしれません）")
            continue
        seen.add(name)
        found, problems = _imported_modules(path)
        warnings.extend(problems)
        todo.extend(sorted(found - seen))
    excluded = sorted(n for n in seen if n.startswith(_EXCLUDE_PREFIX))
    if excluded:
        warnings.append(f"写さないはずの test_* に辿り着きました: {excluded}")
    return sorted(n for n in seen if not n.startswith(_EXCLUDE_PREFIX)), warnings


def tests_commit() -> str:
    """tests/research を最後に変えた commit の短いハッシュ。未コミットの変更があれば印を付ける。"""
    def git(*args: str) -> str:
        return subprocess.run(
            ["git", *args], cwd=REPO_ROOT, capture_output=True, text=True, check=True
        ).stdout.strip()

    commit = git("log", "-1", "--format=%h", "--", "tests/research")
    dirty = git("status", "--porcelain", "--", "tests/research")
    # private/ は git 管理外で status に出ない。未追跡の test_ などが無いかは気にしない
    return commit + ("+未コミットの変更あり" if dirty else "")


def convert(text: str, name: str, date: str, commit: str) -> str:
    """1 ファイル分の変換: パッケージ名を置き換え、先頭に移植元コメントを足す。"""
    body = text.replace(SRC_PKG, DST_PKG)
    header = (
        f"# 移植元: tests/research/{name}.py（scripts/port_research.py が {date} に写した。"
        f"tests の commit {commit}）。手で直さない  # noqa: E501\n"
    )
    if body.startswith("#!"):
        first, _, rest = body.partition("\n")
        return f"{first}\n{header}{rest}"
    return header + body


def port(names: list[str], date: str, commit: str) -> list[tuple[str, int]]:
    DST_DIR.mkdir(parents=True, exist_ok=True)
    keep = {f"{n}.py" for n in names} | {"__init__.py", "PORTED.md"}
    for old in DST_DIR.iterdir():
        if old.is_file() and old.name not in keep:
            old.unlink()
    (DST_DIR / "__init__.py").write_text(INIT_TEXT, encoding="utf-8")
    rows: list[tuple[str, int]] = []
    for name in names:
        src_text = _mod_path(name).read_text(encoding="utf-8")
        out = convert(src_text, name, date, commit)
        (DST_DIR / f"{name}.py").write_text(out, encoding="utf-8")
        rows.append((name, src_text.count("\n") + 1))
    return rows


def write_ported_md(rows: list[tuple[str, int]], date: str, commit: str) -> None:
    total = sum(n for _, n in rows)
    lines = [
        "# 写したファイル一覧（scripts/port_research.py が作る。手で直さない）",
        "",
        f"- 写した日: {date}",
        f"- tests の commit: {commit}",
        f"- ファイル数: {len(rows)}（__init__.py を除く）/ 合計 {total} 行（移植元の行数）",
        "- 写す対象: tests/research/main.py から import で辿れるモジュール全部"
        "（関数内の遅延 import も含む）",
        "- 取り込み直し: tests を直したら `.venv/bin/python scripts/port_research.py` を再実行する",
        "",
        "| ファイル | 移植元 | 行数 | commit |",
        "|---|---|---:|---|",
    ]
    lines += [f"| {n}.py | tests/research/{n}.py | {c} | {commit} |" for n, c in rows]
    PORTED_MD.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _docstring_nodes(tree: ast.AST) -> set[int]:
    ids: set[int] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Module | ast.ClassDef | ast.FunctionDef | ast.AsyncFunctionDef):
            body = node.body
            if body and isinstance(body[0], ast.Expr):
                v = body[0].value
                if isinstance(v, ast.Constant) and isinstance(v.value, str):
                    ids.add(id(v))
    return ids


def report_strings(names: list[str]) -> list[tuple[str, int, str]]:
    """写したファイルの文字列定数（docstring 以外）のうち `tests/` を含むもの。"""
    out: list[tuple[str, int, str]] = []
    for name in names:
        path = DST_DIR / f"{name}.py"
        tree = ast.parse(path.read_text(encoding="utf-8"))
        docs = _docstring_nodes(tree)
        for node in ast.walk(tree):
            if (
                isinstance(node, ast.Constant)
                and isinstance(node.value, str)
                and id(node) not in docs
                and "tests/" in node.value
            ):
                text = node.value.replace("\n", "\\n")
                out.append((f"{name}.py", node.lineno, text[:140]))
    return sorted(out)


def check_only(names: list[str], date: str, commit: str) -> int:
    """写さずに、今ある core が写し元の変換結果と（先頭コメントの日付以外）同じか確かめる。"""
    bad = 0
    for name in names:
        dst = DST_DIR / f"{name}.py"
        if not dst.exists():
            print(f"無い: {dst}")
            bad += 1
            continue
        want = convert(_mod_path(name).read_text(encoding="utf-8"), name, "X", "X").splitlines()
        have = dst.read_text(encoding="utf-8").splitlines()
        # 移植元コメント行（日付・commit だけ違ってよい）を除いて比べる
        want = [ln for ln in want if not ln.startswith("# 移植元:")]
        have = [ln for ln in have if not ln.startswith("# 移植元:")]
        if want != have:
            print(f"差あり: {name}.py")
            bad += 1
    extra = sorted(
        p.name for p in DST_DIR.glob("*.py") if p.stem not in names and p.name != "__init__.py"
    )
    for e in extra:
        print(f"写し元から消えたファイルが残っています: {e}")
        bad += 1
    print("一致" if not bad else f"{bad} 件の差")
    return 1 if bad else 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("--report", action="store_true", help='"tests/" を含む文字列を一覧に出す')
    ap.add_argument("--check", action="store_true", help="写さずに写し元との差だけ確かめる")
    args = ap.parse_args()

    names, warnings = trace_modules()
    for w in warnings:
        print(f"警告: {w}", file=sys.stderr)
    date = datetime.now().strftime("%Y-%m-%d")
    commit = tests_commit()
    if args.check:
        return check_only(names, date, commit)

    rows = port(names, date, commit)
    write_ported_md(rows, date, commit)
    total = sum(n for _, n in rows)
    print(f"写した: {len(rows)} ファイル / {total} 行（commit {commit}）→ {DST_DIR}")
    if args.report:
        items = report_strings(names)
        print(f'\n"tests/" を含む文字列（docstring 以外）: {len(items)} 件')
        for f, ln, text in items:
            print(f"  {f}:{ln}: {text}")
    return 1 if any("の外への import" in w for w in warnings) else 0


if __name__ == "__main__":
    raise SystemExit(main())
