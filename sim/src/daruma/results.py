"""The only writer of results/results.csv: an append-only log of every measured or simulated number.

Usage (from the repo root):
    uv run --project sim python -m daruma.results add --metric hw.motor.resistance_ohm --value 3.6 \
        --unit ohm --method "multimeter across motor A terminals" --evidence "manual:multimeter"
    uv run --project sim python -m daruma.results add ... --evidence logs/2026-12-01-imu.log
    uv run --project sim python -m daruma.results show --last 5

An evidence file is copied into results/raw/ (renamed with the date) so the public repo keeps the raw
data behind every number. Rows are never edited or deleted; a correction is a new row with --note.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import re
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
RESULTS = REPO / "results" / "results.csv"
RAW = REPO / "results" / "raw"
COLUMNS = ["date", "metric", "value", "unit", "method", "evidence", "commit", "note"]
METRIC_RE = re.compile(r"^(hw|sim)\.[a-z0-9_]+(\.[a-z0-9_]+)+$")


def current_commit() -> str:
    done = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=REPO, capture_output=True, text=True)
    commit = done.stdout.strip() or "uncommitted"
    dirty = subprocess.run(["git", "status", "--porcelain", "--untracked-files=no"], cwd=REPO, capture_output=True)
    return commit + ("+dirty" if dirty.stdout.strip() else "")


def store_evidence(evidence: str, metric: str, today: str) -> str:
    if evidence.startswith("manual:"):
        return evidence
    src = Path(evidence)
    if not src.is_absolute():
        src = REPO / src
    if not src.is_file():
        raise SystemExit(f"evidence file not found: {evidence}")
    RAW.mkdir(parents=True, exist_ok=True)
    dest = RAW / f"{today}_{metric}{src.suffix}"
    if dest.exists():
        raise SystemExit(f"{dest.relative_to(REPO)} already exists; raw evidence is never overwritten")
    shutil.copy2(src, dest)
    return dest.relative_to(REPO).as_posix()


def add(args: argparse.Namespace) -> None:
    if not METRIC_RE.match(args.metric):
        raise SystemExit("metric must look like hw.<area>.<name> or sim.<area>.<name> (lowercase)")
    float(args.value)  # must be numeric
    today = dt.date.today().isoformat()
    row = {
        "date": today,
        "metric": args.metric,
        "value": args.value,
        "unit": args.unit,
        "method": args.method,
        "evidence": store_evidence(args.evidence, args.metric, today),
        "commit": current_commit(),
        "note": args.note or "",
    }
    new_file = not RESULTS.exists()
    RESULTS.parent.mkdir(parents=True, exist_ok=True)
    with RESULTS.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS)
        if new_file:
            writer.writeheader()
        writer.writerow(row)
    print(",".join(row[c] for c in COLUMNS))


def show(args: argparse.Namespace) -> None:
    if not RESULTS.exists():
        print("no results yet")
        return
    rows = list(csv.DictReader(RESULTS.open(encoding="utf-8")))
    for row in rows[-args.last :]:
        print(f"{row['date']}  {row['metric']} = {row['value']} {row['unit']}  [{row['evidence']}]  {row['note']}")


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(prog="daruma.results")
    sub = parser.add_subparsers(required=True)
    p_add = sub.add_parser("add", help="append one result")
    p_add.add_argument("--metric", required=True)
    p_add.add_argument("--value", required=True)
    p_add.add_argument("--unit", required=True)
    p_add.add_argument("--method", required=True)
    p_add.add_argument("--evidence", required=True, help="path to a log/CSV, or 'manual:<who>, <instrument>'")
    p_add.add_argument("--note")
    p_add.set_defaults(func=add)
    p_show = sub.add_parser("show", help="print the newest rows")
    p_show.add_argument("--last", type=int, default=10)
    p_show.set_defaults(func=show)
    args = parser.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main(sys.argv[1:])
