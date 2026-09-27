"""Scenario variants of make_scenario_bags decide by header.stamp, on serialized messages."""
import sqlite3
import sys
from pathlib import Path

import pytest
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import make_scenario_bags as msb  # noqa: E402

FIX, VEL, FRONT = '/sensing/gnss/master/fix', '/sensing/gnss/master/vel', '/vehicle/front_bogie_velocity'
REAR, CMD = '/vehicle/rear_bogie_velocity', '/vehicle/driver_position_cmd'
ROVER_FIX, ROVER_VEL = '/sensing/gnss/rover/fix', '/sensing/gnss/rover/vel'
TYPES = {FIX: 'sensor_msgs/msg/NavSatFix', ROVER_FIX: 'sensor_msgs/msg/NavSatFix',
         VEL: 'geometry_msgs/msg/TwistStamped', ROVER_VEL: 'geometry_msgs/msg/TwistStamped',
         FRONT: 'tram_vehicle_msgs/msg/VelocitySensor',
         REAR: 'tram_vehicle_msgs/msg/VelocitySensor',
         CMD: 'tram_vehicle_msgs/msg/DriverControllerCommand'}
DB_NAME = 'synthetic_0.db3'


def row(ts, rid, topic, t):
    kind = TYPES[topic]
    obj = build(ts, kind)
    obj.header.stamp.sec, obj.header.stamp.nanosec = int(t), int(round((t - int(t)) * 1e9))
    return rid, topic, kind, bytes(ts.serialize_cdr(obj, kind))


def build(ts, kind):
    """A message of `kind` with every field zero (rosbags classes need all fields)."""
    import numpy as np
    cls = ts.types[kind]
    _, fields = ts.fielddefs[kind]
    vals = []
    for _, (node, desc) in fields:
        if node == 1:          # base type
            vals.append('' if desc[0] == 'string' else 0)
        elif node == 2:        # nested message
            vals.append(build(ts, desc))
        else:                  # arrays / sequences
            base, size = desc[0], desc[1] if node == 3 else 0
            vals.append(np.zeros(size, float) if base[0] == 1 else [])
    return cls(*vals)


@pytest.fixture(scope='module')
def ts():
    return msb.typestore()


def rows(ts, spec):
    return [row(ts, i, topic, t) for i, (topic, t) in enumerate(spec)]


def make_synthetic_bag(path, ts, specs):
    """Write a minimal rosbag2 Humble SQLite bag from (topic, header time, record time) rows."""
    path.mkdir(parents=True)
    topics = msb.INPUTS + msb.GNSS + (ROVER_VEL,)
    topic_ids = {topic: i + 1 for i, topic in enumerate(topics)}
    qos_profiles = ''
    db_path = path / DB_NAME
    serialized = [(row(ts, i + 1, topic, header_time), record_time)
                  for i, (topic, header_time, record_time) in enumerate(specs)]

    with sqlite3.connect(db_path) as con:
        con.executescript(
            'CREATE TABLE topics('
            'id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL, '
            'serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL);'
            'CREATE TABLE messages('
            'id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, timestamp INTEGER NOT NULL, '
            'data BLOB NOT NULL);'
        )
        con.executemany(
            'INSERT INTO topics(id, name, type, serialization_format, offered_qos_profiles) '
            'VALUES (?, ?, ?, ?, ?)',
            [(topic_ids[topic], topic, TYPES[topic], 'cdr', qos_profiles) for topic in topics],
        )
        con.executemany(
            'INSERT INTO messages(id, topic_id, timestamp, data) VALUES (?, ?, ?, ?)',
            [(message[0], topic_ids[message[1]], record_time, message[3])
             for message, record_time in serialized],
        )

    record_times = [record_time for _, record_time in serialized]
    starting_time = min(record_times)
    duration = max(record_times) - starting_time
    topic_counts = {topic: sum(message[1] == topic for message, _ in serialized)
                    for topic in topics}
    message_count = len(serialized)
    metadata = {
        'rosbag2_bagfile_information': {
            'version': 5,
            'storage_identifier': 'sqlite3',
            'duration': {'nanoseconds': duration},
            'starting_time': {'nanoseconds_since_epoch': starting_time},
            'message_count': message_count,
            'topics_with_message_count': [
                {
                    'topic_metadata': {
                        'name': topic,
                        'type': TYPES[topic],
                        'serialization_format': 'cdr',
                        'offered_qos_profiles': qos_profiles,
                    },
                    'message_count': topic_counts[topic],
                }
                for topic in topics
            ],
            'files': [{
                'path': DB_NAME,
                'starting_time': {'nanoseconds_since_epoch': starting_time},
                'duration': {'nanoseconds': duration},
                'message_count': message_count,
            }],
        },
    }
    (path / 'metadata.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False), encoding='utf-8')
    return db_path


def bag_files_snapshot(path):
    return {file.relative_to(path): file.read_bytes() for file in path.rglob('*') if file.is_file()}


def bag_database_summary(db_path):
    with sqlite3.connect(db_path) as con:
        counts = dict(con.execute(
            'SELECT t.name, COUNT(m.id) FROM topics t '
            'LEFT JOIN messages m ON m.topic_id = t.id GROUP BY t.id'
        ))
        timestamps = [timestamp for (timestamp,) in con.execute(
            'SELECT timestamp FROM messages ORDER BY timestamp, id'
        )]
        starting_time, ending_time = con.execute(
            'SELECT MIN(timestamp), MAX(timestamp) FROM messages'
        ).fetchone()
    return counts, timestamps, starting_time, ending_time - starting_time


def read_metadata(path):
    return yaml.safe_load((path / 'metadata.yaml').read_text(encoding='utf-8'))[
        'rosbag2_bagfile_information'
    ]


def test_gnss_first_drops_vehicle_before_first_fix(ts):
    r = rows(ts, [(FRONT, 100.0), (FRONT, 100.1), (FIX, 100.2), (FRONT, 100.15), (FRONT, 100.3)])
    assert msb.rows_to_drop(r, 'gnss_first', ts) == {0, 1, 3}


def test_gnss_first_drops_earlier_record_order_even_with_later_header(ts):
    r = rows(ts, [(FRONT, 100.3), (FIX, 100.2), (FRONT, 100.4)])
    assert msb.rows_to_drop(r, 'gnss_first', ts) == {0}


def test_gnss_first_bag_delivers_fix_before_vehicle(tmp_path, ts):
    source = tmp_path / 'source'
    start = 8_000_000_000
    make_synthetic_bag(source, ts, [
        (FRONT, 100.3, start),
        (FIX, 100.2, start + 100_000_000),
        (FRONT, 100.4, start + 200_000_000),
    ])
    target = tmp_path / 'gnss_first'
    msb.make_bag(source, target, 'gnss_first', 2.0)
    with sqlite3.connect(target / DB_NAME) as con:
        topics = [topic for (topic,) in con.execute(
            'SELECT t.name FROM messages m JOIN topics t ON t.id = m.topic_id '
            'ORDER BY m.timestamp, m.id')]
    assert topics == [FIX, FRONT]


def test_short_gnss_keeps_window_inclusive(ts):
    r = rows(ts, [(FIX, 99.0), (FRONT, 100.0), (ROVER_VEL, 105.0),
                  (VEL, 105.5), (FIX, 110.0), (ROVER_VEL, 110.5)])
    assert msb.rows_to_drop(r, 'short_gnss', ts) == {3, 4, 5}


def test_no_gnss_and_crop(ts):
    r = rows(ts, [(FIX, 99.0), (FRONT, 100.0), (VEL, 101.0), (ROVER_VEL, 102.0)])
    assert msb.rows_to_drop(r, 'no_gnss', ts) == {0, 2, 3}
    assert msb.rows_to_drop(r, 'crop', ts) == set()


def test_gnss_first_without_fix_is_an_error(ts):
    with pytest.raises(ValueError):
        msb.rows_to_drop(rows(ts, [(FRONT, 100.0)]), 'gnss_first', ts)


def test_make_bag_crop_trims_by_record_time_and_rewrites_metadata(tmp_path, ts):
    source = tmp_path / 'source'
    start = 8_000_000_000
    make_synthetic_bag(source, ts, [
        (FRONT, 100.0, start),
        (FIX, 100.5, start + 500_000_000),
        (REAR, 101.0, start + 1_500_000_000),
        (VEL, 101.5, start + 2_000_000_000),
        (CMD, 102.0, start + 2_000_000_001),
        (ROVER_FIX, 103.0, start + 3_000_000_000),
    ])
    before = bag_files_snapshot(source)
    target = tmp_path / 'crop'

    msb.make_bag(source, target, 'crop', 2.0)

    counts, timestamps, db_start, db_duration = bag_database_summary(target / DB_NAME)
    assert timestamps == [start, start + 500_000_000, start + 1_500_000_000,
                          start + 2_000_000_000]
    assert all(timestamp <= start + 2_000_000_000 for timestamp in timestamps)
    expected_counts = {topic: int(topic in (FRONT, FIX, REAR, VEL))
                       for topic in msb.INPUTS + msb.GNSS + (ROVER_VEL,)}
    assert counts == expected_counts

    metadata = read_metadata(target)
    metadata_counts = {entry['topic_metadata']['name']: entry['message_count']
                       for entry in metadata['topics_with_message_count']}
    assert metadata_counts == counts
    assert metadata['message_count'] == len(timestamps)
    assert metadata['files'][0]['message_count'] == len(timestamps)
    assert metadata['starting_time']['nanoseconds_since_epoch'] == db_start
    assert metadata['duration']['nanoseconds'] == db_duration
    assert metadata['files'][0]['starting_time']['nanoseconds_since_epoch'] == db_start
    assert metadata['files'][0]['duration']['nanoseconds'] == db_duration
    assert bag_files_snapshot(source) == before


def test_make_bag_no_gnss_removes_gnss_rows_and_keeps_vehicle_metadata(tmp_path, ts):
    source = tmp_path / 'source'
    start = 12_000_000_000
    make_synthetic_bag(source, ts, [
        (FIX, 50.0, start),
        (FRONT, 50.1, start + 100_000_000),
        (ROVER_FIX, 50.2, start + 200_000_000),
        (ROVER_VEL, 50.25, start + 250_000_000),
        (REAR, 50.3, start + 300_000_000),
        (VEL, 50.4, start + 400_000_000),
        (CMD, 50.5, start + 500_000_000),
    ])
    target = tmp_path / 'no_gnss'

    returned_counts = msb.make_bag(source, target, 'no_gnss', 10.0)

    counts, _, _, _ = bag_database_summary(target / DB_NAME)
    assert all(counts[topic] == 0 for topic in msb.GNSS + (ROVER_VEL,))
    assert all(counts[topic] > 0 for topic in msb.INPUTS)
    with sqlite3.connect(target / DB_NAME) as con:
        remaining_topics = {topic for (topic,) in con.execute(
            'SELECT DISTINCT t.name FROM topics t JOIN messages m ON m.topic_id = t.id'
        )}
    assert remaining_topics.isdisjoint(msb.GNSS + (ROVER_VEL,))
    assert all(returned_counts[topic] == 0 for topic in msb.GNSS)
    assert all(returned_counts[topic] > 0 for topic in msb.INPUTS)

    metadata_counts = {entry['topic_metadata']['name']: entry['message_count']
                       for entry in read_metadata(target)['topics_with_message_count']}
    assert all(metadata_counts[topic] == 0 for topic in msb.GNSS + (ROVER_VEL,))
    assert all(metadata_counts[topic] > 0 for topic in msb.INPUTS)


def test_make_bag_rejects_existing_target(tmp_path, ts):
    source = tmp_path / 'source'
    make_synthetic_bag(source, ts, [(FRONT, 100.0, 1_000_000_000)])
    target = tmp_path / 'existing'
    target.mkdir()

    with pytest.raises(ValueError):
        msb.make_bag(source, target, 'crop', 1.0)


@pytest.mark.parametrize('db_count', [0, 2])
def test_make_bag_requires_exactly_one_db3(tmp_path, ts, db_count):
    source = tmp_path / f'source-{db_count}'
    db_path = make_synthetic_bag(source, ts, [(FRONT, 100.0, 1_000_000_000)])
    if db_count == 0:
        db_path.unlink()
    else:
        (source / 'extra.db3').write_bytes(db_path.read_bytes())

    with pytest.raises(ValueError):
        msb.make_bag(source, tmp_path / f'target-{db_count}', 'crop', 1.0)
