#!/usr/bin/env python3
"""Summarize MAPF benchmark result directories by failure cause.

The benchmark runner writes a ``summary.csv`` plus one log file per level.  This
script turns one or more such directories into a stable triage report so solver
changes can be compared without ad-hoc log scraping.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

MARKERS: tuple[str, ...] = (
    "scheduler_empty",
    "repair outcome",
    "all_repair_stages_failed",
    "cycle_detected",
    "repeated_task_local_trajectory",
    "competitive_wave",
    "MoveBlockingBoxToParking",
    "planning_deadline",
    "partial_plan_unsolved",
    "budget",
)

FAMILY_PATTERNS: tuple[tuple[str, str], ...] = (
    ("MAPFreorder*", r"^MAPFreorder"),
    ("MAPF*", r"^MAPF"),
    ("MAsimple*", r"^MAsimple"),
    ("MAthomasAppartment*", r"^MAthomasAppartment"),
    ("MAbispebjerg*", r"^MAbispebjerg"),
    ("SAtowers*", r"^SAtowers"),
    ("SAsoko*", r"^SAsoko"),
    ("SA*", r"^SA"),
    ("MA*", r"^MA"),
)

PARKING_RE = re.compile(r"MoveBlockingBoxToParking[^\n]*\bparking=\((\d+),(\d+)\)")
STOP_REASON_RE = re.compile(r"partial_plan_unsolved[^\n]*\bstop_reason=([^\s]+)")


@dataclass(frozen=True)
class Row:
    level: str
    status: str
    solved: str
    exit_code: str
    wall_time_s: float | None
    server_time_s: float | None
    solution_length: int | None
    log_file: Path | None
    family: str
    log_markers: dict[str, int]
    repeated_parking_targets: dict[str, int]
    partial_stop_reason: str

    @property
    def is_solved(self) -> bool:
        return self.solved.lower() == "yes" or self.status == "SOLVED"

    @property
    def is_timeout_or_crash(self) -> bool:
        return self.status == "TIMEOUT_OR_CRASH" or self.status.startswith("EXIT_")

    @property
    def is_zero_action_failure(self) -> bool:
        return (not self.is_solved) and (self.solution_length or 0) == 0

    @property
    def is_partial_progress_failure(self) -> bool:
        return (not self.is_solved) and (self.solution_length or 0) > 0


def parse_float(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    try:
        return float(value)
    except ValueError:
        return None


def parse_int(value: str | None) -> int | None:
    if value is None or value == "":
        return None
    try:
        return int(value)
    except ValueError:
        return None


def family_for(level_name: str, log_text: str) -> str:
    stem = Path(level_name).stem
    for family, pattern in FAMILY_PATTERNS:
        if re.search(pattern, stem):
            return family
    # Historical CSV rows only contain the basename.  The server log preserves
    # whether the benchmark came from levels/ or complevels/.
    if "/complevels/" in log_text or "./complevels/" in log_text:
        return "competition/other"
    return "other"


def read_log(log_file: Path | None) -> str:
    if log_file is None or not log_file.exists():
        return ""
    return log_file.read_text(errors="replace")


def summarize_log(text: str) -> tuple[dict[str, int], dict[str, int], str]:
    markers = {marker: text.count(marker) for marker in MARKERS}
    parking_counts = Counter(f"({row},{col})" for row, col in PARKING_RE.findall(text))
    repeated = {
        target: count for target, count in sorted(parking_counts.items()) if count > 1
    }
    stop_reasons = STOP_REASON_RE.findall(text)
    partial_stop_reason = stop_reasons[-1] if stop_reasons else ""
    return markers, repeated, partial_stop_reason


def read_result_dir(result_dir: Path) -> list[Row]:
    csv_path = result_dir / "summary.csv"
    if not csv_path.exists():
        raise FileNotFoundError(f"missing summary.csv in {result_dir}")

    rows: list[Row] = []
    with csv_path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        for raw in reader:
            level = raw.get("level", "")
            log_value = raw.get("log_file", "")
            log_file = Path(log_value) if log_value else None
            if log_file is not None and not log_file.is_absolute():
                # Historical benchmark CSVs store paths relative to the repo
                # root, usually including the result directory name.  Prefer
                # that path, but also tolerate absolute result directories and
                # hand-made fixtures whose logs are relative to result_dir.
                candidates = (
                    Path.cwd() / log_file,
                    result_dir.parent / log_file,
                    result_dir / log_file,
                )
                log_file = next(
                    (candidate for candidate in candidates if candidate.exists()),
                    candidates[-1],
                )
            log_text = read_log(log_file)
            markers, repeated_parking, partial_stop_reason = summarize_log(log_text)
            rows.append(
                Row(
                    level=level,
                    status=raw.get("status", ""),
                    solved=raw.get("solved", ""),
                    exit_code=raw.get("exit_code", ""),
                    wall_time_s=parse_float(raw.get("wall_time_s")),
                    server_time_s=parse_float(raw.get("server_time_s")),
                    solution_length=parse_int(raw.get("solution_length")),
                    log_file=log_file,
                    family=family_for(level, log_text),
                    log_markers=markers,
                    repeated_parking_targets=repeated_parking,
                    partial_stop_reason=partial_stop_reason,
                )
            )
    return rows


def count_rows(rows: Iterable[Row]) -> dict[str, Any]:
    materialized = list(rows)
    solved = sum(row.is_solved for row in materialized)
    timeout_or_crash = sum(row.is_timeout_or_crash for row in materialized)
    zero_action = sum(row.is_zero_action_failure for row in materialized)
    partial_progress = sum(row.is_partial_progress_failure for row in materialized)
    marker_counts = Counter()
    repeated_parking_levels = 0
    repeated_parking_targets = Counter()
    partial_stop_reasons = Counter(
        row.partial_stop_reason
        for row in materialized
        if row.is_partial_progress_failure and row.partial_stop_reason
    )
    for row in materialized:
        marker_counts.update(row.log_markers)
        if row.repeated_parking_targets:
            repeated_parking_levels += 1
            repeated_parking_targets.update(row.repeated_parking_targets)
    total = len(materialized)
    return {
        "tested": total,
        "solved": solved,
        "failed": total - solved,
        "timeouts_or_crashes": timeout_or_crash,
        "zero_action_failures": zero_action,
        "partial_progress_failures": partial_progress,
        "success_rate_percent": round(100.0 * solved / total, 2) if total else 0.0,
        "marker_counts": dict(sorted(marker_counts.items())),
        "levels_with_repeated_parking_targets": repeated_parking_levels,
        "repeated_parking_targets": dict(repeated_parking_targets.most_common(10)),
        "partial_stop_reasons": dict(partial_stop_reasons.most_common()),
    }


def slow_solved(rows: list[Row], limit: int) -> list[dict[str, Any]]:
    solved_rows = [row for row in rows if row.is_solved]
    solved_rows.sort(
        key=lambda row: (
            row.server_time_s
            if row.server_time_s is not None
            else row.wall_time_s or -1
        ),
        reverse=True,
    )
    return [
        {
            "level": row.level,
            "family": row.family,
            "server_time_s": row.server_time_s,
            "wall_time_s": row.wall_time_s,
            "solution_length": row.solution_length,
        }
        for row in solved_rows[:limit]
    ]


def row_failure_payload(row: Row) -> dict[str, Any]:
    active_markers = {k: v for k, v in row.log_markers.items() if v}
    return {
        "level": row.level,
        "family": row.family,
        "status": row.status,
        "wall_time_s": row.wall_time_s,
        "server_time_s": row.server_time_s,
        "solution_length": row.solution_length,
        "markers": active_markers,
        "repeated_parking_targets": row.repeated_parking_targets,
        "partial_stop_reason": row.partial_stop_reason,
        "log_file": str(row.log_file) if row.log_file is not None else "",
    }


def partial_progress_failures(rows: list[Row], limit: int) -> list[dict[str, Any]]:
    partial_rows = [row for row in rows if row.is_partial_progress_failure]
    partial_rows.sort(
        key=lambda row: (
            row.family,
            -(row.solution_length or 0),
            row.level,
        )
    )
    return [row_failure_payload(row) for row in partial_rows[:limit]]


def notable_failures(rows: list[Row], limit: int) -> list[dict[str, Any]]:
    failed_rows = [row for row in rows if not row.is_solved]
    failed_rows.sort(
        key=lambda row: (
            row.is_timeout_or_crash,
            row.solution_length or 0,
            row.wall_time_s or 0.0,
            sum(row.log_markers.values()),
        ),
        reverse=True,
    )
    return [row_failure_payload(row) for row in failed_rows[:limit]]


def build_report(result_dirs: list[Path], limit: int) -> dict[str, Any]:
    directories: dict[str, Any] = {}
    all_rows: list[Row] = []
    for result_dir in result_dirs:
        rows = read_result_dir(result_dir)
        all_rows.extend(rows)
        family_rows: dict[str, list[Row]] = defaultdict(list)
        for row in rows:
            family_rows[row.family].append(row)
        directories[str(result_dir)] = {
            **count_rows(rows),
            "families": {
                family: count_rows(frows)
                for family, frows in sorted(family_rows.items())
            },
            "slow_solved": slow_solved(rows, limit),
            "partial_progress_failures_detail": partial_progress_failures(rows, limit),
            "notable_failures": notable_failures(rows, limit),
        }

    combined_families: dict[str, list[Row]] = defaultdict(list)
    for row in all_rows:
        combined_families[row.family].append(row)

    return {
        "directories": directories,
        "combined": {
            **count_rows(all_rows),
            "families": {
                family: count_rows(rows)
                for family, rows in sorted(combined_families.items())
            },
            "slow_solved": slow_solved(all_rows, limit),
            "partial_progress_failures_detail": partial_progress_failures(all_rows, limit),
            "notable_failures": notable_failures(all_rows, limit),
        },
    }


def print_section(title: str, stats: dict[str, Any], limit: int) -> None:
    print(f"## {title}")
    print(
        "tested={tested} solved={solved} failed={failed} success={success_rate_percent:.2f}% "
        "timeouts_or_crashes={timeouts_or_crashes} zero_action_failures={zero_action_failures} "
        "partial_progress_failures={partial_progress_failures}".format(**stats)
    )
    print(
        f"levels_with_repeated_parking_targets={stats['levels_with_repeated_parking_targets']}"
    )

    print("\nMarker counts:")
    for marker, count in stats["marker_counts"].items():
        if count:
            print(f"  {marker}: {count}")

    if stats.get("partial_stop_reasons"):
        print("\nPartial-plan stop reasons:")
        for reason, count in stats["partial_stop_reasons"].items():
            print(f"  {reason}: {count}")

    print("\nFamilies:")
    for family, family_stats in stats.get("families", {}).items():
        print(
            f"  {family}: tested={family_stats['tested']} solved={family_stats['solved']} "
            f"failed={family_stats['failed']} zero_action={family_stats['zero_action_failures']} "
            f"partial={family_stats['partial_progress_failures']} timeouts={family_stats['timeouts_or_crashes']}"
        )

    print(f"\nSlow solved levels (top {limit}):")
    for row in stats.get("slow_solved", []):
        print(
            f"  {row['level']} [{row['family']}]: server={row['server_time_s']}s "
            f"wall={row['wall_time_s']}s length={row['solution_length']}"
        )

    print(f"\nUnsolved levels that returned plans (top {limit}):")
    for row in stats.get("partial_progress_failures_detail", []):
        markers = ", ".join(f"{k}={v}" for k, v in row["markers"].items()) or "none"
        stop_reason = row.get("partial_stop_reason") or "unknown"
        print(
            f"  {row['level']} [{row['family']}]: length={row['solution_length']} "
            f"stop={stop_reason} server={row['server_time_s']}s wall={row['wall_time_s']}s "
            f"markers=({markers}) log={row['log_file']}"
        )

    print(f"\nNotable failures (top {limit}):")
    for row in stats.get("notable_failures", []):
        markers = ", ".join(f"{k}={v}" for k, v in row["markers"].items()) or "none"
        parking = (
            ", ".join(f"{k}x{v}" for k, v in row["repeated_parking_targets"].items())
            or "none"
        )
        print(
            f"  {row['level']} [{row['family']}]: {row['status']} wall={row['wall_time_s']}s "
            f"server={row['server_time_s']}s length={row['solution_length']} markers=({markers}) "
            f"repeated_parking=({parking})"
        )
    print()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "result_dirs",
        nargs="+",
        type=Path,
        help="Benchmark result directories containing summary.csv",
    )
    parser.add_argument(
        "--json", action="store_true", help="Emit machine-readable JSON instead of text"
    )
    parser.add_argument(
        "--limit", type=int, default=10, help="Rows to show in slow/failure tables"
    )
    args = parser.parse_args(argv)

    try:
        report = build_report(args.result_dirs, args.limit)
    except FileNotFoundError as exc:
        print(f"triage_benchmarks.py: {exc}", file=sys.stderr)
        return 2

    if args.json:
        json.dump(report, sys.stdout, indent=2, sort_keys=True)
        print()
    else:
        for result_dir, stats in report["directories"].items():
            print_section(result_dir, stats, args.limit)
        if len(args.result_dirs) > 1:
            print_section("combined", report["combined"], args.limit)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
