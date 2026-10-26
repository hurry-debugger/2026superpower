"""只读检查自维护文件；不改写文件、不安装工具、不操作 Git 索引。"""

import argparse
from collections.abc import Sequence
import os
from pathlib import Path
import shutil
import subprocess
import sys


ROOT_FILES = frozenset({"AGENTS.md", "README.md", ".editorconfig", ".clang-format"})
MANAGED_DIRECTORIES = ("docs", "User", "templates", "tools")
TEXT_SUFFIXES = frozenset({".md", ".c", ".h", ".py", ".json", ".yaml", ".yml", ".toml", ".txt"})
EXCLUDED_DIRECTORIES = frozenset({
    ".git", "core", "drivers", "mdk-arm", "references", "build", "debug", "release",
    "__pycache__", ".cache", "objects", "listings",
})


def is_managed(relative: Path) -> bool:
    """采用允许列表；目录保护优先于文件扩展名。"""
    if relative.is_absolute() or ".." in relative.parts:
        return False
    if any(part.casefold() in EXCLUDED_DIRECTORIES for part in relative.parts[:-1]):
        return False
    if relative.as_posix() in ROOT_FILES:
        return True
    return (
        len(relative.parts) > 1
        and relative.parts[0] in MANAGED_DIRECTORIES
        and relative.suffix.casefold() in TEXT_SUFFIXES
    )


def is_safe_file(path: Path, root: Path) -> bool:
    """不跟随链接到工程外；有明确生成标记的 C/H 文件也不纳入检查。"""
    try:
        if not path.is_file() or not path.resolve().is_relative_to(root.resolve()):
            return False
        current = path
        while current != root:
            if current.is_symlink() or getattr(current, "is_junction", lambda: False)():
                return False
            current = current.parent
        if path.suffix.casefold() in {".c", ".h"}:
            with path.open("rb") as stream:
                header = stream.read(4096)
            if b"USER CODE BEGIN" in header or b"STMicroelectronics" in header:
                return False
        return True
    except OSError as error:
        raise RuntimeError(f"Cannot inspect {path}: {error}") from error


def git_output(root: Path, arguments: Sequence[str]) -> bytes:
    """读取 Git 文件清单；关闭可选索引刷新，不调用外部 diff。"""
    environment = os.environ.copy()
    environment["GIT_OPTIONAL_LOCKS"] = "0"
    try:
        result = subprocess.run(
            ["git", "-c", f"safe.directory={root.as_posix()}", "-C", str(root), *arguments],
            capture_output=True,
            env=environment,
            check=False,
        )
    except OSError as error:
        raise RuntimeError(f"Git unavailable: {error}") from error
    if result.returncode:
        message = result.stderr.decode("utf-8", errors="replace").strip()
        raise RuntimeError(f"Git read failed: {message}")
    return result.stdout


def collect_paths(root: Path, changed: bool = False) -> list[Path]:
    """收集全部受管文件，或工作区、暂存区和未跟踪文件的并集。"""
    root = root.resolve()
    candidates = set()
    if changed:
        commands = (
            ["diff", "--no-ext-diff", "--no-textconv", "--name-only", "--diff-filter=ACMRTUXB", "-z"],
            ["diff", "--cached", "--no-ext-diff", "--no-textconv", "--name-only", "--diff-filter=ACMRTUXB", "-z"],
            ["ls-files", "--others", "--exclude-standard", "-z"],
        )
        for arguments in commands:
            for raw in git_output(root, arguments).split(b"\0"):
                if raw:
                    relative = Path(raw.decode("utf-8", errors="surrogateescape"))
                    if is_managed(relative):
                        candidates.add(root / relative)
    else:
        candidates.update(root / name for name in ROOT_FILES if (root / name).exists())
        for name in MANAGED_DIRECTORIES:
            directory = root / name
            if directory.is_symlink() or getattr(directory, "is_junction", lambda: False)():
                continue
            for current, directories, filenames in os.walk(directory):
                directories[:] = sorted(
                    item for item in directories
                    if item.casefold() not in EXCLUDED_DIRECTORIES
                    and not (Path(current) / item).is_symlink()
                    and not getattr(Path(current) / item, "is_junction", lambda: False)()
                )
                for filename in filenames:
                    path = Path(current) / filename
                    if is_managed(path.relative_to(root)):
                        candidates.add(path)
    return sorted(
        (path for path in candidates if is_safe_file(path, root)),
        key=lambda path: path.relative_to(root).as_posix(),
    )


def check_file(path: Path, root: Path) -> list[str]:
    """返回 path:line:column 诊断；列号按解码字符计，编码错误按字节计。"""
    relative = path.relative_to(root).as_posix()
    messages = []

    def report(line: int, column: int, code: str, detail: str) -> None:
        messages.append(f"{relative}:{line}:{column}: {code}: {detail}")

    try:
        content = path.read_bytes()
    except OSError as error:
        report(1, 1, "READ_ERROR", str(error))
        return messages
    if content.startswith(b"\xef\xbb\xbf"):
        report(1, 1, "BOM", "Use UTF-8 without BOM")
    try:
        text = content.decode("utf-8", errors="strict")
    except UnicodeDecodeError as error:
        prefix = content[:error.start]
        report(prefix.count(b"\n") + 1, len(prefix.rsplit(b"\n", 1)[-1]) + 1,
               "ENCODING_UTF8", "Invalid UTF-8; do not auto-convert unknown encodings")
        return messages
    for line_number, line in enumerate(text.split("\n"), start=1):
        if "\r" in line:
            report(line_number, line.index("\r") + 1, "LINE_ENDING", "Use LF, not CRLF or CR")
        body = line.rstrip("\r")
        if body.endswith((" ", "\t")):
            column = len(body.rstrip(" \t")) + 1
            report(line_number, column, "TRAILING_WHITESPACE", "Remove trailing spaces or tabs")
        if path.suffix.casefold() in {".c", ".h"} and "\t" in line:
            report(line_number, line.index("\t") + 1, "TAB", "Use four spaces in managed C/H")
    if not content.endswith(b"\n"):
        report(text.count("\n") + 1, len(text.rsplit("\n", 1)[-1]) + 1,
               "FINAL_NEWLINE", "End the file with LF; empty managed files are not templates")
    return messages


def check_format(paths: Sequence[Path], root: Path) -> list[str]:
    """显式指定配置和文件，仅运行 dry-run；不使用 -i 或递归格式化。"""
    executable = shutil.which("clang-format")
    if executable is None:
        raise RuntimeError("NOT_RUN: clang-format is unavailable; no tool was installed")
    configuration = root / ".clang-format"
    if not configuration.is_file():
        raise RuntimeError("NOT_RUN: selected project has no .clang-format")
    messages = []
    for path in paths:
        if path.suffix.casefold() not in {".c", ".h"}:
            continue
        try:
            result = subprocess.run(
                [executable, f"--style=file:{configuration}", "--dry-run", "--Werror", str(path)],
                cwd=root,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                check=False,
            )
        except OSError as error:
            raise RuntimeError(f"NOT_RUN: clang-format cannot start: {error}") from error
        if result.returncode:
            detail = (result.stderr or result.stdout).strip() or "Formatter exited with an error"
            messages.append(f"{path.relative_to(root).as_posix()}:1:1: FORMAT: {detail}")
    return messages


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1],
                        help="Project root; defaults to this script's parent project")
    parser.add_argument("--changed", action="store_true",
                        help="Check worktree, staged and untracked managed files")
    parser.add_argument("--format", action="store_true",
                        help="Also run clang-format in read-only dry-run mode")
    arguments = parser.parse_args(argv)
    root = arguments.root.resolve()
    if not root.is_dir():
        print(f"ERROR: Project root does not exist: {root}", file=sys.stderr)
        return 2
    try:
        paths = collect_paths(root, changed=arguments.changed)
        messages = [message for path in paths for message in check_file(path, root)]
        format_error = None
        if arguments.format:
            try:
                messages.extend(check_format(paths, root))
            except RuntimeError as error:
                format_error = str(error)
        for message in messages:
            print(message)
        if format_error:
            print(format_error, file=sys.stderr)
        if messages:
            print(f"FAIL: {len(messages)} issue(s) in {len(paths)} selected managed file(s)")
            return 1
        if format_error:
            return 2
        if not paths:
            print("NO_FILES: 0 managed files selected; this is not a full-project verification")
            return 0
        mode = "changed" if arguments.changed else "all"
        scope = "text + C/H format" if arguments.format else "text only; C/H format NOT_RUN"
        print(f"PASS: {len(paths)} managed file(s); selection={mode}; {scope}")
        return 0
    except RuntimeError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
