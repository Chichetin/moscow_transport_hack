"""Scenario bags for tools/submission/judge_runner.sh, cut from one train rosbag2 SQLite bag.

    .venv/bin/python tools/submission/make_scenario_bags.py SOURCE_BAG OUT_DIR NAME VARIANT [SECONDS]

VARIANT (the input header.stamp decides, not the record time, docs/data.md traps 5-6):
  crop        the first SECONDS of the bag (record time), every topic kept
  gnss_first  crop, and vehicle inputs stamped before the first master fix dropped: GNSS
              reaches the node before the first wheel
  no_gnss     crop without any /sensing/gnss/* topic: the node must publish odom, never an old anchor
  short_gnss  crop, /sensing/gnss/* only up to first vehicle header + 5 s (D-005)

Rows are deleted from a copy; metadata.yaml counts are rewritten so ros2 bag play agrees.
A development tool: rosbags is not needed by the ROS node at runtime.
"""
from __future__ import annotations

import shutil
import sqlite3
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools' / 'eval'))

from tram_eval.bag import GNSS, INPUTS, MASTER_FIX, typestore  # noqa: E402

VARIANTS = ('crop', 'gnss_first', 'no_gnss', 'short_gnss')
SHORT_GNSS_S = 5.0          # s after the first vehicle header, the GNSS window of D-005


def header_s(ts, data: bytes, msgtype: str) -> float:
    stamp = ts.deserialize_cdr(data, msgtype).header.stamp
    return stamp.sec + stamp.nanosec * 1e-9


def rows_to_drop(rows, variant: str, ts) -> set[int]:
    """Row ids to delete from `rows` = [(id, topic, msgtype, data)] in record order."""
    if variant == 'crop':
        return set()
    if variant == 'no_gnss':
        return {rid for rid, topic, _, _ in rows if topic.startswith('/sensing/gnss/')}
    if variant == 'gnss_first':
        first_fix = next((header_s(ts, d, k) for _, topic, k, d in rows if topic == MASTER_FIX), None)
        if first_fix is None:
            raise ValueError('no master fix in the cut: GNSS cannot come first')
        return {rid for rid, topic, k, d in rows
                if topic in INPUTS and header_s(ts, d, k) < first_fix}
    if variant == 'short_gnss':
        first = next((header_s(ts, d, k) for _, topic, k, d in rows if topic in INPUTS), None)
        if first is None:
            raise ValueError('no vehicle input in the cut')
        return {rid for rid, topic, k, d in rows
                if topic.startswith('/sensing/gnss/') and header_s(ts, d, k) > first + SHORT_GNSS_S}
    raise ValueError(f'unknown variant {variant!r}, one of {VARIANTS}')


def rewrite_metadata(target: Path, con: sqlite3.Connection) -> None:
    meta_path = target / 'metadata.yaml'
    meta = yaml.safe_load(meta_path.read_text(encoding='utf-8'))
    info = meta['rosbag2_bagfile_information']
    counts = dict(con.execute('SELECT t.name, COUNT(m.id) FROM topics t '
                              'LEFT JOIN messages m ON m.topic_id = t.id GROUP BY t.id'))
    t_min, t_max = con.execute('SELECT MIN(timestamp), MAX(timestamp) FROM messages').fetchone()
    for entry in info['topics_with_message_count']:
        entry['message_count'] = counts.get(entry['topic_metadata']['name'], 0)
    info['message_count'] = sum(counts.values())
    info['starting_time']['nanoseconds_since_epoch'] = t_min
    info['duration']['nanoseconds'] = t_max - t_min
    for f in info.get('files', []):
        f['message_count'] = info['message_count']
        f['starting_time']['nanoseconds_since_epoch'] = t_min
        f['duration']['nanoseconds'] = t_max - t_min
    meta_path.write_text(yaml.safe_dump(meta, sort_keys=False), encoding='utf-8')


def make_bag(source: Path, target: Path, variant: str, seconds: float) -> dict:
    """Write the scenario bag; returns message counts of vehicle and GNSS topics."""
    source, target = Path(source).resolve(), Path(target).resolve()
    dbs = list(source.glob('*.db3'))
    if len(dbs) != 1 or not (source / 'metadata.yaml').is_file():
        raise ValueError(f'{source}: need one rosbag2 SQLite file with metadata.yaml')
    if target.exists():
        raise ValueError(f'{target} exists')
    shutil.copytree(source, target)
    ts = typestore()
    with sqlite3.connect(target / dbs[0].name) as con:
        t0 = con.execute('SELECT MIN(timestamp) FROM messages').fetchone()[0]
        con.execute('DELETE FROM messages WHERE timestamp > ?', (t0 + int(seconds * 1e9),))
        rows = con.execute('SELECT m.id, t.name, t.type, m.data FROM messages m '
                           'JOIN topics t ON t.id = m.topic_id ORDER BY m.timestamp, m.id').fetchall()
        drop = rows_to_drop(rows, variant, ts)
        con.executemany('DELETE FROM messages WHERE id = ?', [(rid,) for rid in drop])
        con.commit()
        rewrite_metadata(target, con)
        counts = dict(con.execute('SELECT t.name, COUNT(m.id) FROM topics t '
                                  'LEFT JOIN messages m ON m.topic_id = t.id GROUP BY t.id'))
    con.close()
    with sqlite3.connect(target / dbs[0].name) as con:
        con.execute('VACUUM')
    con.close()
    return {topic: counts.get(topic, 0) for topic in INPUTS + GNSS}


def main(argv) -> int:
    if len(argv) not in (5, 6) or argv[4] not in VARIANTS:
        print(__doc__, file=sys.stderr)
        return 2
    seconds = float(argv[5]) if len(argv) == 6 else 90.0
    counts = make_bag(Path(argv[1]), Path(argv[2]) / argv[3], argv[4], seconds)
    print(f'{argv[3]}: {argv[4]} {seconds:g} s ' + ' '.join(f'{k}={v}' for k, v in counts.items()))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
