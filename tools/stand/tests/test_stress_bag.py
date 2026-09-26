"""A live-node stress bag must preserve every recording timestamp and topic."""

import sqlite3

import pytest

from tram_eval.bag import typestore


def test_spike_bag_changes_only_front_velocity_in_event_window(tmp_path):
    from make_stress_bag import make_spike_bag

    source = tmp_path / 'source'
    source.mkdir()
    (source / 'metadata.yaml').write_text('source metadata\n', encoding='utf-8')
    db = source / 'source_0.db3'
    ts = typestore()
    time_type = ts.types['builtin_interfaces/msg/Time']
    header_type = ts.types['std_msgs/msg/Header']
    wheel_type = ts.types['tram_vehicle_msgs/msg/VelocitySensor']
    with sqlite3.connect(db) as con:
        con.execute('CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT, type TEXT, '
                    'serialization_format TEXT, offered_qos_profiles TEXT)')
        con.execute('CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER, '
                    'timestamp INTEGER, data BLOB)')
        for topic_id, name in enumerate(('/vehicle/front_bogie_velocity',
                                         '/vehicle/rear_bogie_velocity'), 1):
            con.execute('INSERT INTO topics VALUES (?, ?, ?, ?, ?)',
                        (topic_id, name, 'tram_vehicle_msgs/msg/VelocitySensor', 'cdr', ''))
        for sec in range(120):
            for topic_id in (1, 2):
                msg = wheel_type(header_type(time_type(sec, 0), ''), 10.0)
                con.execute('INSERT INTO messages(topic_id, timestamp, data) VALUES (?, ?, ?)',
                            (topic_id, sec * 10**9, ts.serialize_cdr(msg, msg.__msgtype__)))

    target = tmp_path / 'spike'
    changed, start, end = make_spike_bag(source, target, gnss_window_s=5.0)

    assert changed == 5
    assert (start, end) == pytest.approx((47.6, 52.6))
    assert (target / 'metadata.yaml').read_bytes() == (source / 'metadata.yaml').read_bytes()
    with sqlite3.connect(db) as clean, sqlite3.connect(target / db.name) as dirty:
        old = clean.execute('SELECT topic_id, timestamp, data FROM messages ORDER BY id').fetchall()
        new = dirty.execute('SELECT topic_id, timestamp, data FROM messages ORDER BY id').fetchall()
    assert len(old) == len(new) == 240
    for (old_topic, old_time, old_data), (topic, time, data) in zip(old, new):
        assert (topic, time) == (old_topic, old_time)
        speed = ts.deserialize_cdr(data, 'tram_vehicle_msgs/msg/VelocitySensor').velocity
        if topic == 1 and 48 <= time // 10**9 <= 52:
            assert speed == 35.0
            assert data != old_data
        else:
            assert speed == 10.0
            assert data == old_data
