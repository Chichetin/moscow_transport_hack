"""Serialize one jury run into a TSV row without leaking ROS storage log lines."""
from __future__ import annotations

import sys


def run_row(bag: str, status: str, note: str) -> str:
    clean = ' '.join(note.split())
    return f'{bag}\t{status}\t{clean}\n'


def read_runs(path: str) -> list[dict[str, str]]:
    rows = [line.rstrip('\n').split('\t') for line in open(path, encoding='utf-8') if line.strip()]
    return [{'bag': bag, 'status': status, 'note': note} for bag, status, note in rows]


if __name__ == '__main__':
    with open(sys.argv[1], 'a', encoding='utf-8') as target:
        target.write(run_row(sys.argv[2], sys.argv[3], sys.argv[4]))
