"""Copy a rosbag2 SQLite bag and inject the evaluator's wheel spike for a live stand run.

The source metadata, recording timestamps, connection QoS and message count stay intact.
Only serialized front-wheel messages in the event window change. This is a development
tool; rosbags is not needed by the ROS node at runtime.
"""
from __future__ import annotations

import argparse
import shutil
import sqlite3
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools' / 'eval'))

from tram_eval.bag import GNSS, INPUTS, FRONT, typestore  # noqa: E402
from tram_eval.stress import perturb  # noqa: E402


def make_spike_bag(source: Path, target: Path, gnss_window_s: float = 5.0) -> tuple[int, float, float]:
    """Write a copy with the same +25 km/h front-wheel spike used by eval --stress."""
    source, target = Path(source), Path(target)
    dbs = list(source.glob('*.db3'))
    if len(dbs) != 1 or not (source / 'metadata.yaml').is_file():
        raise ValueError('source must be one rosbag2 SQLite file with metadata.yaml')
    if target.exists() or target == source or source in target.parents:
        raise ValueError('target must be a new directory outside source')

    ts = typestore()
    placeholders = ','.join('?' for _ in INPUTS + GNSS)
    with sqlite3.connect(dbs[0]) as con:
        rows = con.execute(
            'SELECT m.id, t.name, t.type, m.data FROM messages m '
            'JOIN topics t ON t.id = m.topic_id '
            f'WHERE t.name IN ({placeholders}) ORDER BY m.timestamp, m.id',
            INPUTS + GNSS,
        ).fetchall()
    messages = [(topic, ts.deserialize_cdr(data, msgtype))
                for _, topic, msgtype, data in rows]
    event = perturb(messages, 'spike', gnss_window_s)
    if event is None:
        raise ValueError('bag is too short for a spike and recovery tail')
    changed_messages, start, end = event
    updates = []
    for (row_id, topic, msgtype, _), (_, old_msg), (_, new_msg) in zip(
            rows, messages, changed_messages):
        if new_msg is not old_msg:
            if topic != FRONT:
                raise ValueError('spike may only change front-wheel messages')
            updates.append((ts.serialize_cdr(new_msg, msgtype), row_id))
    if not updates:
        raise ValueError('spike did not change any wheel message')

    shutil.copytree(source, target)
    with sqlite3.connect(target / dbs[0].name) as con:
        con.executemany('UPDATE messages SET data = ? WHERE id = ?', updates)
    return len(updates), start, end


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path, help='original rosbag2 directory')
    parser.add_argument('target', type=Path, help='new rosbag2 directory')
    parser.add_argument('--gnss-window-s', type=float, default=5.0)
    args = parser.parse_args()
    changed, start, end = make_spike_bag(args.source, args.target, args.gnss_window_s)
    print(f'{args.target}: {changed} front-wheel messages changed, '
          f'stamp window {start:.3f}..{end:.3f} s')


if __name__ == '__main__':
    main()
