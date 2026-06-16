#!/usr/bin/env python3
"""Append received RACE UART text into cumulative local CSV files.

The serial receiver may split one MCU log line into several timestamped chunks.
This script rebuilds full RACE_* records, validates seq/idx continuity, and
appends raw records plus run/segment/turn summaries for Task3/Task4 tuning.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import os
import re
import statistics
import sys
import tempfile
from datetime import datetime
from pathlib import Path
from typing import Iterable


TIMESTAMP_RE = re.compile(r"^\[[^\]]+\]:\s*")
KEYVAL_RE = re.compile(r"(\w+)=([^\s]+)")
CSV_ENCODING = "utf-8-sig"

PREFERRED_COLUMNS = [
    "run_id",
    "imported_at",
    "source_file",
    "source_name",
    "source_mtime",
    "log_hash",
    "record_type",
    "seq",
    "idx",
    "name",
    "count",
    "lap",
    "seg",
    "phase",
    "event",
    "reason",
    "t",
    "t_start",
    "t_end",
    "dist",
    "phase_dist",
    "yaw",
    "yaw_start",
    "yaw_end",
    "yaw_raw",
    "pyaw",
    "yprog",
    "ydelta",
    "exp",
    "herr",
    "line_turn",
    "nav_turn",
    "turn",
    "tdiff",
    "ff",
    "fb",
    "corr",
    "end_herr",
    "avg_herr",
    "max_herr",
    "avg_gz",
    "max_gz",
    "avg_gzlp",
    "max_gzlp",
    "avg_line",
    "avg_nav",
    "gz",
    "gz100",
    "gzlp",
    "roll",
    "pitch",
    "n",
    "nav_n",
    "nav_lost",
    "nav_fd",
    "nav_stale",
    "upd",
    "lost",
    "line_n",
    "line_first",
    "line_last",
    "line_span",
    "end_gap",
    "avg_turn",
    "lost_streak",
    "end_lost",
    "avg_abs_err",
    "max_err",
    "pmask",
    "pflags",
    "raw",
    "mask",
    "cnt",
    "flags",
    "err",
    "B",
    "A",
    "win",
    "win_ov",
    "ev",
    "ev_ov",
    "sum",
    "sum_ov",
    "max_laps",
    "line_base",
    "arc_base",
    "gyro_st",
    "ir_assist",
    "h_div",
    "h_max",
    "h_gd",
    "ac_tgt",
    "bd_tgt",
    "gyro_to",
    "win_pre",
    "win_start",
    "arc_yaw",
    "arc_div",
    "arc_max",
    "arc_gd",
    "arc_yaw_arm",
    "turn_slow",
    "turn_slow_yaw",
    "yaw_stop",
    "yaw_tol",
    "yaw_gz",
    "b_exit",
    "a_exit",
    "ff_gain",
]

RUN_COLUMNS = [
    "run_id",
    "imported_at",
    "source_file",
    "source_name",
    "source_mtime",
    "log_hash",
    "validation_ok",
    "records",
    "seq_count",
    "seq_first",
    "seq_last",
    "evt_count",
    "sum_count",
    "win_count",
    "win_ov",
    "ev_ov",
    "sum_ov",
    "complete_lap",
    "complete_phase",
    "complete_t",
    "complete_reason",
    "line_base",
    "arc_base",
    "ff_gain",
    "gyro_st",
    "arc_yaw",
    "arc_yaw_arm",
    "ac_tgt",
    "bd_tgt",
    "gyro_to",
    "win_pre",
    "win_start",
    "yaw_stop",
    "yaw_tol",
    "yaw_gz",
    "b_exit",
    "a_exit",
]

TURN_COLUMNS = [
    "run_id",
    "imported_at",
    "source_file",
    "log_hash",
    "lap",
    "seg",
    "phase",
    "phase_dist",
    "turn_t_start",
    "turn_t_stop",
    "yaw_before",
    "yaw_after",
    "yaw_delta",
    "yaw_target",
    "yaw_error_after",
    "yaw_stop_enabled",
    "turn_dist",
    "stop_reason",
    "stop_mask",
    "stop_err",
    "motor_b_total",
    "motor_a_total",
]


def strip_timestamp(line: str) -> str:
    return TIMESTAMP_RE.sub("", line.rstrip("\r\n"))


def rebuild_race_records(text: str) -> list[str]:
    """Rebuild complete RACE_* records from timestamped receiver chunks."""

    records: list[str] = []
    current: str | None = None

    for raw_line in text.splitlines():
        payload = strip_timestamp(raw_line)
        if "RACE_" in payload:
            if current is not None:
                records.append(current.strip())
            current = payload.lstrip()
        elif current is not None and payload.strip():
            current += payload

    if current is not None:
        records.append(current.strip())

    return records


def parse_keyvals(record: str) -> dict[str, str]:
    return {match.group(1): match.group(2) for match in KEYVAL_RE.finditer(record)}


def split_pair(row: dict[str, str], key: str, left: str, right: str) -> None:
    value = row.get(key, "")
    if "/" not in value:
        return
    first, second = value.split("/", 1)
    row[left] = first
    row[right] = second


def records_to_rows(records: Iterable[str]) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for record in records:
        if not record or not record.strip():
            continue
        record_type = record.split(None, 1)[0]
        row = {"record_type": record_type}
        row.update(parse_keyvals(record))
        split_pair(row, "t", "t_start", "t_end")
        split_pair(row, "yaw", "yaw_start", "yaw_end")
        row["raw_record"] = record
        rows.append(row)
    return rows


def int_value(row: dict[str, str], key: str, default: int = 0) -> int:
    try:
        return int(row.get(key, default))
    except (TypeError, ValueError):
        return default


def first_slash_int(value: str | None) -> int | None:
    if not value:
        return None
    try:
        return int(value.split("/", 1)[0])
    except ValueError:
        return None


def missing_in_range(values: Iterable[int]) -> list[int]:
    nums = sorted(set(values))
    if not nums:
        return []
    present = set(nums)
    return [num for num in range(nums[0], nums[-1] + 1) if num not in present]


def range_report(name: str, values: Iterable[int]) -> tuple[str, bool]:
    nums = list(values)
    if not nums:
        return f"{name}: count=0", False
    unique = sorted(set(nums))
    missing = missing_in_range(unique)
    ok = len(nums) == len(unique) and not missing
    return (
        f"{name}: count={len(nums)} unique={len(unique)} "
        f"range={unique[0]}..{unique[-1]} missing={len(missing)}",
        ok,
    )


def by_type(rows: list[dict[str, str]], record_type: str) -> list[dict[str, str]]:
    return [row for row in rows if row.get("record_type") == record_type]


def build_validation(rows: list[dict[str, str]]) -> tuple[list[str], bool]:
    lines: list[str] = []
    ok = True

    begin_rows = by_type(rows, "RACE_RAM_BEGIN")
    cfg_rows = by_type(rows, "RACE_CFG")
    event_rows = by_type(rows, "RACE_EVT")
    sum_rows = by_type(rows, "RACE_SUM")
    win_rows = by_type(rows, "RACE_WIN")
    end_rows = by_type(rows, "RACE_RAM_END")
    section_rows = by_type(rows, "RACE_DUMP_SECTION")
    section_end_rows = by_type(rows, "RACE_DUMP_SECTION_END")

    lines.append(f"records={len(rows)}")
    lines.append(f"ram_begin={len(begin_rows)} cfg={len(cfg_rows)} ram_end={len(end_rows)}")

    for line, is_ok in [
        range_report("seq", [int_value(row, "seq") for row in rows if "seq" in row]),
        range_report("evt_idx", [int_value(row, "idx") for row in event_rows]),
        range_report("sum_idx", [int_value(row, "idx") for row in sum_rows]),
        range_report("win_idx", [int_value(row, "idx") for row in win_rows]),
    ]:
        lines.append(line)
        ok = ok and is_ok

    begin = begin_rows[0] if begin_rows else {}
    expected_counts = {
        "EVT": first_slash_int(begin.get("ev")),
        "SUM": first_slash_int(begin.get("sum")),
        "WIN": first_slash_int(begin.get("win")),
    }
    actual_counts = {
        "EVT": len(event_rows),
        "SUM": len(sum_rows),
        "WIN": len(win_rows),
    }
    for name in ("EVT", "SUM", "WIN"):
        expected = expected_counts[name]
        actual = actual_counts[name]
        section_count = next(
            (int_value(row, "count") for row in section_rows if row.get("name") == name),
            None,
        )
        ended = any(row.get("name") == name for row in section_end_rows)
        name_ok = expected == actual and section_count == actual and ended
        ok = ok and name_ok
        lines.append(
            f"{name}: expected={expected} section={section_count} "
            f"actual={actual} end={int(ended)} ok={int(name_ok)}"
        )

    overflow_ok = all(int_value(begin, key) == 0 for key in ("win_ov", "ev_ov", "sum_ov"))
    ok = ok and overflow_ok
    lines.append(
        "overflow: "
        f"win_ov={begin.get('win_ov', '')} ev_ov={begin.get('ev_ov', '')} "
        f"sum_ov={begin.get('sum_ov', '')} ok={int(overflow_ok)}"
    )

    complete_rows = [
        row for row in event_rows if row.get("event") == "complete"
    ]
    complete_text = "; ".join(
        f"lap={row.get('lap')} phase={row.get('phase')} "
        f"t={row.get('t')} reason={row.get('reason')}"
        for row in complete_rows
    )
    lines.append(f"complete: {complete_text or 'none'}")

    lines.append("")
    lines.append("SUM_BY_SEG")
    for seg in ("AC", "CB", "BD", "DA"):
        seg_rows = [row for row in sum_rows if row.get("seg") == seg]
        dists = [int_value(row, "dist") for row in seg_rows]
        yprog = [int_value(row, "yprog") for row in seg_rows]
        avg_herr = [int_value(row, "avg_herr") for row in seg_rows]
        max_herr = [int_value(row, "max_herr") for row in seg_rows]
        avg_nav = [int_value(row, "avg_nav") for row in seg_rows]
        avg_line = [int_value(row, "avg_line") for row in seg_rows]
        avg_turn = [int_value(row, "avg_turn") for row in seg_rows]
        line_n = [int_value(row, "line_n") for row in seg_rows]
        line_first = [int_value(row, "line_first") for row in seg_rows]
        line_last = [int_value(row, "line_last") for row in seg_rows]
        line_span = [int_value(row, "line_span") for row in seg_rows]
        end_gap = [int_value(row, "end_gap") for row in seg_rows]
        lost_streak = [int_value(row, "lost_streak") for row in seg_rows]
        end_lost = [int_value(row, "end_lost") for row in seg_rows]
        pflags = [row.get("pflags", "") for row in seg_rows]
        lost_sum = sum(int_value(row, "lost") for row in seg_rows)
        nav_lost_sum = sum(int_value(row, "nav_lost") for row in seg_rows)
        dist_avg = statistics.mean(dists) if dists else 0.0
        lines.append(
            f"{seg}: n={len(seg_rows)} dist={dists} dist_avg={dist_avg:.1f} "
            f"yprog={yprog} avg_herr={avg_herr} max_herr={max_herr} "
            f"avg_line={avg_line} avg_nav={avg_nav} avg_turn={avg_turn} "
            f"line_n={line_n} line_first={line_first} line_last={line_last} "
            f"line_span={line_span} end_gap={end_gap} "
            f"lost_streak={lost_streak} end_lost={end_lost} pflags={pflags} "
            f"lost_sum={lost_sum} nav_lost_sum={nav_lost_sum}"
        )

    lines.append("")
    lines.append("TURN_START_STOP")
    turn_stops = {
        (row.get("lap"), row.get("seg")): row
        for row in event_rows
        if row.get("event") == "turn_stop"
    }
    for start in [row for row in event_rows if row.get("event") == "turn_start"]:
        stop = turn_stops.get((start.get("lap"), start.get("seg")), {})
        yaw_target = stop.get("exp", start.get("exp", ""))
        yaw_stop_enabled = int(int_value({"target": yaw_target}, "target") != 0)
        lines.append(
            f"lap={start.get('lap')} seg={start.get('seg')} "
            f"phase_dist={start.get('phase_dist')} yaw0={start.get('yaw')} "
            f"yaw1={stop.get('yaw', '')} ydelta={stop.get('ydelta', '')} "
            f"target={yaw_target} yerr={stop.get('herr', '')} "
            f"yaw_stop={yaw_stop_enabled} "
            f"reason={stop.get('reason', '')} turn_dist={stop.get('dist', '')}"
        )

    return lines, ok


def csv_columns(rows: list[dict[str, str]]) -> list[str]:
    all_keys: set[str] = set()
    for row in rows:
        all_keys.update(row.keys())
    preferred = [key for key in PREFERRED_COLUMNS if key in all_keys]
    remaining = sorted(key for key in all_keys if key not in preferred and key != "raw_record")
    return preferred + remaining + ["raw_record"]


def read_csv_rows(path: Path) -> tuple[list[dict[str, str]], list[str]]:
    if not path.exists():
        return [], []
    try:
        with path.open("r", newline="", encoding=CSV_ENCODING) as file:
            reader = csv.DictReader(file)
            return list(reader), list(reader.fieldnames or [])
    except OSError as exc:
        raise OSError(f"failed to read CSV file {path}: {exc}") from exc
    except csv.Error as exc:
        raise ValueError(f"failed to parse CSV file {path}: {exc}") from exc


def sanitize_csv_cell(value: object) -> object:
    """Prevent spreadsheet formula execution while keeping numeric negatives usable."""

    if not isinstance(value, str) or not value:
        return value
    if value[0] in {"=", "+", "@"}:
        return "'" + value
    if value[0] == "-" and (len(value) == 1 or not value[1].isdigit()):
        return "'" + value
    return value


def sanitize_csv_row(row: dict[str, str], columns: list[str]) -> dict[str, object]:
    return {key: sanitize_csv_cell(row.get(key, "")) for key in columns}


def unique_backup_path(path: Path) -> Path:
    stem = path.with_suffix(path.suffix + ".bak")
    try:
        if not stem.exists():
            return stem
    except OSError:
        return stem
    index = 1
    while True:
        candidate = path.with_suffix(path.suffix + f".bak{index}")
        try:
            if not candidate.exists():
                return candidate
        except OSError:
            return candidate
        index += 1


def write_csv_rows(rows: list[dict[str, str]], output_path: Path, columns: list[str]) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temp_name = ""
    try:
        with tempfile.NamedTemporaryFile(
            "w",
            newline="",
            encoding=CSV_ENCODING,
            dir=output_path.parent,
            delete=False,
        ) as file:
            temp_name = file.name
            writer = csv.DictWriter(file, fieldnames=columns, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(sanitize_csv_row(row, columns) for row in rows)
        os.replace(temp_name, output_path)
    except OSError as exc:
        raise OSError(f"failed to write CSV file {output_path}: {exc}") from exc
    except csv.Error as exc:
        raise ValueError(f"failed to format CSV file {output_path}: {exc}") from exc
    finally:
        if temp_name:
            try:
                Path(temp_name).unlink(missing_ok=True)
            except OSError:
                pass


def backup_csv_file(path: Path) -> Path:
    backup_path = unique_backup_path(path)
    try:
        backup_path.write_bytes(path.read_bytes())
    except OSError as exc:
        raise OSError(f"failed to back up CSV file {path} to {backup_path}: {exc}") from exc
    return backup_path


def append_csv_rows(
    rows: list[dict[str, str]],
    output_path: Path,
    preferred_columns: list[str],
    replace: bool = False,
) -> None:
    if not rows:
        return

    output_path.parent.mkdir(parents=True, exist_ok=True)
    new_keys: set[str] = set()
    for row in rows:
        new_keys.update(row.keys())

    if replace or not output_path.exists():
        columns = [key for key in preferred_columns if key in new_keys]
        columns += sorted(key for key in new_keys if key not in columns)
        write_csv_rows(rows, output_path, columns)
        return

    existing_rows, existing_columns = read_csv_rows(output_path)
    columns = list(existing_columns)
    for key in preferred_columns:
        if key in new_keys and key not in columns:
            columns.append(key)
    for key in sorted(new_keys):
        if key not in columns:
            columns.append(key)

    if columns != existing_columns:
        backup_csv_file(output_path)
        write_csv_rows(existing_rows + rows, output_path, columns)
        return

    write_csv_rows(existing_rows + rows, output_path, columns)


def existing_values(path: Path, column: str) -> set[str]:
    rows, _ = read_csv_rows(path)
    return {row[column] for row in rows if row.get(column)}


def log_hash(records: list[str]) -> str:
    digest = hashlib.sha1("\n".join(records).encode("utf-8", errors="replace")).hexdigest()
    return digest[:12]


def source_mtime(path: Path) -> str:
    try:
        return datetime.fromtimestamp(path.stat().st_mtime).isoformat(timespec="seconds")
    except OSError as exc:
        print(f"warning: failed to read source mtime for {path}: {exc}", file=sys.stderr)
        return ""


def add_metadata(rows: list[dict[str, str]], metadata: dict[str, str]) -> list[dict[str, str]]:
    return [{**metadata, **row} for row in rows]


def build_run_summary_row(
    rows: list[dict[str, str]],
    metadata: dict[str, str],
    validation_ok: bool,
) -> dict[str, str]:
    begin = by_type(rows, "RACE_RAM_BEGIN")
    cfg = by_type(rows, "RACE_CFG")
    events = by_type(rows, "RACE_EVT")
    sums = by_type(rows, "RACE_SUM")
    wins = by_type(rows, "RACE_WIN")
    complete = next((row for row in events if row.get("event") == "complete"), {})
    seq_values = [int_value(row, "seq") for row in rows if "seq" in row]
    begin_row = begin[0] if begin else {}
    cfg_row = cfg[0] if cfg else {}

    row = {
        **metadata,
        "validation_ok": str(int(validation_ok)),
        "records": str(len(rows)),
        "seq_count": str(len(seq_values)),
        "seq_first": str(min(seq_values)) if seq_values else "",
        "seq_last": str(max(seq_values)) if seq_values else "",
        "evt_count": str(len(events)),
        "sum_count": str(len(sums)),
        "win_count": str(len(wins)),
        "win_ov": begin_row.get("win_ov", ""),
        "ev_ov": begin_row.get("ev_ov", ""),
        "sum_ov": begin_row.get("sum_ov", ""),
        "complete_lap": complete.get("lap", ""),
        "complete_phase": complete.get("phase", ""),
        "complete_t": complete.get("t", ""),
        "complete_reason": complete.get("reason", ""),
        "line_base": cfg_row.get("line_base", ""),
        "arc_base": cfg_row.get("arc_base", ""),
        "ff_gain": cfg_row.get("ff_gain", ""),
        "gyro_st": cfg_row.get("gyro_st", ""),
        "arc_yaw": cfg_row.get("arc_yaw", ""),
        "arc_yaw_arm": cfg_row.get("arc_yaw_arm", ""),
        "ac_tgt": cfg_row.get("ac_tgt", ""),
        "bd_tgt": cfg_row.get("bd_tgt", ""),
        "gyro_to": cfg_row.get("gyro_to", ""),
        "win_pre": cfg_row.get("win_pre", ""),
        "win_start": cfg_row.get("win_start", ""),
        "yaw_stop": cfg_row.get("yaw_stop", ""),
        "yaw_tol": cfg_row.get("yaw_tol", ""),
        "yaw_gz": cfg_row.get("yaw_gz", ""),
        "b_exit": cfg_row.get("b_exit", ""),
        "a_exit": cfg_row.get("a_exit", ""),
    }
    return row


def build_turn_rows(rows: list[dict[str, str]], metadata: dict[str, str]) -> list[dict[str, str]]:
    events = by_type(rows, "RACE_EVT")
    turn_stops = {
        (row.get("lap"), row.get("seg")): row
        for row in events
        if row.get("event") == "turn_stop"
    }
    turn_rows: list[dict[str, str]] = []
    for start in [row for row in events if row.get("event") == "turn_start"]:
        stop = turn_stops.get((start.get("lap"), start.get("seg")), {})
        yaw_target = stop.get("exp", start.get("exp", ""))
        turn_rows.append(
            {
                **metadata,
                "lap": start.get("lap", ""),
                "seg": start.get("seg", ""),
                "phase": start.get("phase", ""),
                "phase_dist": start.get("phase_dist", ""),
                "turn_t_start": start.get("t", ""),
                "turn_t_stop": stop.get("t", ""),
                "yaw_before": start.get("yaw", ""),
                "yaw_after": stop.get("yaw", ""),
                "yaw_delta": stop.get("ydelta", ""),
                "yaw_target": yaw_target,
                "yaw_error_after": stop.get("herr", ""),
                "yaw_stop_enabled": str(int(int_value({"target": yaw_target}, "target") != 0)),
                "turn_dist": stop.get("dist", ""),
                "stop_reason": stop.get("reason", ""),
                "stop_mask": stop.get("mask", ""),
                "stop_err": stop.get("err", ""),
                "motor_b_total": stop.get("B", ""),
                "motor_a_total": stop.get("A", ""),
            }
        )
    return turn_rows


def append_summary_text(
    summary_path: Path | None,
    summary_lines: list[str],
    metadata: dict[str, str],
    validation_ok: bool,
    replace: bool,
) -> None:
    if summary_path is None:
        return
    try:
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        mode = "w" if replace or not summary_path.exists() else "a"
        with summary_path.open(mode, encoding="utf-8") as file:
            if mode == "a":
                file.write("\n")
            file.write(
                f"=== RACE_RUN run_id={metadata['run_id']} "
                f"imported_at={metadata['imported_at']} "
                f"source={metadata['source_file']} "
                f"validation_ok={int(validation_ok)} ===\n"
            )
            file.write("\n".join(summary_lines))
            file.write("\n")
    except OSError as exc:
        print(f"Warning: failed to write summary {summary_path}: {exc}")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(
        description="Rebuild received RACE UART logs and append structured CSV data."
    )
    parser.add_argument("log_path", help="Received UART text log file.")
    parser.add_argument(
        "-o",
        "--output",
        default="data/task11_experience_data.csv",
        help="Cumulative raw-record CSV path. Default: data/task11_experience_data.csv",
    )
    parser.add_argument(
        "--runs-output",
        default="data/task11_experience_runs.csv",
        help="Cumulative one-row-per-run summary CSV.",
    )
    parser.add_argument(
        "--segments-output",
        default="data/task11_experience_segments.csv",
        help="Cumulative RACE_SUM segment CSV.",
    )
    parser.add_argument(
        "--turns-output",
        default="data/task11_experience_turns.csv",
        help="Cumulative turn-start/turn-stop summary CSV.",
    )
    parser.add_argument(
        "--summary",
        default="data/task11_experience_summary.txt",
        help="Cumulative human-readable summary log.",
    )
    parser.add_argument(
        "--no-summary",
        action="store_true",
        help="Do not write a summary text file.",
    )
    parser.add_argument(
        "--run-id",
        default=None,
        help="Run id to store. Default: race_<content-hash>.",
    )
    parser.add_argument(
        "--replace",
        action="store_true",
        help="Clear output files first. Default is append-only accumulation.",
    )
    parser.add_argument(
        "--allow-duplicate",
        action="store_true",
        help="Allow importing a log whose run_id already exists.",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Return non-zero if validation finds missing rows or overflow.",
    )
    args = parser.parse_args(argv)

    log_path = Path(args.log_path)
    output_path = Path(args.output)
    runs_output_path = Path(args.runs_output)
    segments_output_path = Path(args.segments_output)
    turns_output_path = Path(args.turns_output)
    summary_path = (
        None
        if args.no_summary
        else Path(args.summary)
    )

    try:
        text = log_path.read_text(encoding="utf-8", errors="replace")
    except OSError as exc:
        print(f"error: failed to read input log {log_path}: {exc}", file=sys.stderr)
        return 2

    records = rebuild_race_records(text)
    rows = records_to_rows(records)
    summary_lines, validation_ok = build_validation(rows)
    digest = log_hash(records)
    run_id = args.run_id or f"race_{digest}"
    metadata = {
        "run_id": run_id,
        "imported_at": datetime.now().isoformat(timespec="seconds"),
        "source_file": str(log_path),
        "source_name": log_path.name,
        "source_mtime": source_mtime(log_path),
        "log_hash": digest,
    }

    try:
        already_imported = run_id in existing_values(runs_output_path, "run_id")
    except (OSError, ValueError, csv.Error) as exc:
        print(f"error: failed to check existing RACE runs: {exc}", file=sys.stderr)
        return 2

    if not args.replace and not args.allow_duplicate and already_imported:
        print(f"input={log_path}")
        print(f"run_id={run_id}")
        print("already_imported=1")
        print("No files changed. Use --allow-duplicate to append anyway.")
        return 0

    record_rows = add_metadata(rows, metadata)
    run_rows = [build_run_summary_row(rows, metadata, validation_ok)]
    segment_rows = add_metadata(by_type(rows, "RACE_SUM"), metadata)
    turn_rows = build_turn_rows(rows, metadata)

    try:
        append_csv_rows(record_rows, output_path, PREFERRED_COLUMNS, replace=args.replace)
        append_csv_rows(run_rows, runs_output_path, RUN_COLUMNS, replace=args.replace)
        append_csv_rows(segment_rows, segments_output_path, PREFERRED_COLUMNS, replace=args.replace)
        append_csv_rows(turn_rows, turns_output_path, TURN_COLUMNS, replace=args.replace)
        append_summary_text(summary_path, summary_lines, metadata, validation_ok, args.replace)
    except (OSError, ValueError, csv.Error) as exc:
        print(f"error: failed to write RACE outputs: {exc}", file=sys.stderr)
        return 2

    print(f"input={log_path}")
    print(f"run_id={run_id}")
    print(f"records_csv={output_path} appended_rows={len(record_rows)}")
    print(f"runs_csv={runs_output_path} appended_rows={len(run_rows)}")
    print(f"segments_csv={segments_output_path} appended_rows={len(segment_rows)}")
    print(f"turns_csv={turns_output_path} appended_rows={len(turn_rows)}")
    if summary_path is not None:
        print(f"summary_log={summary_path}")
    print(f"validation_ok={int(validation_ok)}")

    if args.strict and not validation_ok:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
