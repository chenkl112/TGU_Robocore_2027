#!/usr/bin/env python3
"""使用固定版本 clang-format 检查或格式化项目自有 C++ 源码。"""

import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys


FORMAT_VERSION = "23.1.5"
SOURCE_ROOTS = ("app/auto_aim", "io", "tools", "task", "tests")
EXCLUDED_TREES = (
    "app/auto_aim/planner/tinympc",
    "tools/foxglove",
    "tools/BS_thread_pool",
    "tools/concurrentqueue",
)
EXCLUDED_FILES = {"tomlpp.hpp", "BS_thread_pool.hpp", "concurrentqueue.hpp"}


def collect_sources(repo):
    """只遍历指定业务目录；历史审查复现和第三方实现不进入格式检查。"""
    result = []
    excluded = tuple(repo / name for name in EXCLUDED_TREES)
    for directory in SOURCE_ROOTS:
        for path in (repo / directory).rglob("*"):
            if not path.is_file() or path.suffix not in {".cpp", ".hpp"}:
                continue
            if path.name in EXCLUDED_FILES or any(path.is_relative_to(tree) for tree in excluded):
                continue
            result.append(path)
    return sorted(result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    operation = parser.add_mutually_exclusive_group(required=True)
    operation.add_argument("--check", action="store_true", help="检查格式，不改写文件")
    operation.add_argument("--fix", action="store_true", help="按仓库 .clang-format 改写文件")
    parser.add_argument(
        "--clang-format", default=os.environ.get("CLANG_FORMAT", "clang-format"),
        help="clang-format 可执行文件；默认读取 CLANG_FORMAT 或 PATH",
    )
    args = parser.parse_args()
    executable = shutil.which(args.clang_format)
    if executable is None:
        parser.error(f"找不到 clang-format；请安装 clang-format=={FORMAT_VERSION}")

    version = subprocess.run([executable, "--version"], capture_output=True, text=True, check=True)
    match = re.search(r"\bversion\s+(\d+\.\d+\.\d+)\b", version.stdout)
    if match is None or match.group(1) != FORMAT_VERSION:
        parser.error(f"需要 clang-format {FORMAT_VERSION}，当前为：{version.stdout.strip()}")

    repo = Path(__file__).resolve().parents[1]
    style = repo / ".clang-format"
    if not style.is_file():
        parser.error(f"找不到格式配置：{style}")
    sources = collect_sources(repo)
    if not sources:
        parser.error("没有找到项目自有 C++ 源码")

    changed = []
    for path in sources:
        original = path.read_bytes()
        formatted = subprocess.run(
            [executable, f"--style=file:{style}", str(path)], capture_output=True, check=True
        ).stdout
        if original != formatted:
            changed.append(path.relative_to(repo))
            if args.fix:
                path.write_bytes(formatted)

    if changed:
        for path in changed:
            print(path)
        if args.check:
            print(f"{len(changed)} 个文件需要格式化；请运行 python3 tools/check_format.py --fix")
            return 1
        print(f"已格式化 {len(changed)} 个文件，共检查 {len(sources)} 个文件。")
    else:
        print(f"格式检查通过：{len(sources)} 个文件，clang-format {FORMAT_VERSION}。")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, subprocess.CalledProcessError) as error:
        print(f"格式工具执行失败：{error}", file=sys.stderr)
        if isinstance(error, subprocess.CalledProcessError) and error.stderr:
            details = error.stderr
            print(details.decode() if isinstance(details, bytes) else details, file=sys.stderr)
        sys.exit(2)
