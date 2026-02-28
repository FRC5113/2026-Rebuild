#!/usr/bin/env python3
"""Dump comprehensive information from a cProfile .prof file.

Usage:
    python profile_everything.py path/to/profile.prof
    python profile_everything.py src/teleop.prof --output profile_report.txt
"""

from __future__ import annotations

import argparse
import io
import os
import pstats
import sys


def _section(title: str) -> str:
    return f"\n{'=' * 100}\n{title}\n{'=' * 100}\n"


def build_report(profile_path: str, strip_dirs: bool = True) -> str:
    stream = io.StringIO()

    stats = pstats.Stats(profile_path, stream=stream)
    if strip_dirs:
        stats.strip_dirs()

    stream.write(_section("PROFILE SUMMARY"))
    stats.print_stats()

    stream.write(_section("SORTED BY CUMULATIVE TIME"))
    stats.sort_stats("cumulative")
    stats.print_stats()

    stream.write(_section("SORTED BY TOTAL INTERNAL TIME (TOTTIME)"))
    stats.sort_stats("tottime")
    stats.print_stats()

    stream.write(_section("SORTED BY NUMBER OF CALLS"))
    stats.sort_stats("ncalls")
    stats.print_stats()

    stream.write(_section("CALLERS (WHO CALLED EACH FUNCTION)"))
    stats.sort_stats("cumulative")
    stats.print_callers()

    stream.write(_section("CALLEES (WHAT EACH FUNCTION CALLED)"))
    stats.sort_stats("cumulative")
    stats.print_callees()

    return stream.getvalue()


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Print everything from a cProfile .prof file"
    )
    parser.add_argument("profile", help="Path to .prof file")
    parser.add_argument(
        "-o",
        "--output",
        help="Optional output text file. If omitted, prints to stdout.",
        default=None,
    )
    parser.add_argument(
        "--keep-dirs",
        action="store_true",
        help="Keep full directory paths in function names (default strips dirs)",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)

    profile_path = os.path.abspath(args.profile)
    if not os.path.exists(profile_path):
        print(f"Error: file not found: {profile_path}", file=sys.stderr)
        return 1

    report = build_report(profile_path, strip_dirs=not args.keep_dirs)

    if args.output:
        output_path = os.path.abspath(args.output)
        with open(output_path, "w", encoding="utf-8") as f:
            f.write(report)
        print(f"Wrote report to: {output_path}")
    else:
        print(report)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
