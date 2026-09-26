"""Check recorded result types and input-derived header stamps in ROS 2 Humble."""
from __future__ import annotations

import sys
from functools import lru_cache


OUTPUT_TYPES = {'/result/velocity': 'tram_vehicle_msgs/msg/VelocitySensor',
                '/result/position': 'nav_msgs/msg/Odometry'}
INPUT_TOPICS = {'/vehicle/front_bogie_velocity', '/vehicle/rear_bogie_velocity',
                '/vehicle/driver_position_cmd'}


def stamp_of(msg):
    stamp = msg.header.stamp
    sec, nanosec = int(stamp.sec), int(stamp.nanosec)
    if sec < 0 or not 0 <= nanosec < 1_000_000_000:
        raise ValueError(f'invalid stamp {sec}.{nanosec}')
    return sec, nanosec


def display(stamp):
    return f'{stamp[0]}.{stamp[1]:09d}'


def validate(input_bag, result_bag, open_reader, deserialize):
    """Return (pass, reason); inspect all output messages, then match their stamps to inputs."""
    try:
        result = open_reader(result_bag)
        result_types = {topic.name: topic.type for topic in result.get_all_topics_and_types()}
        for topic, expected in OUTPUT_TYPES.items():
            actual = result_types.get(topic)
            if actual != expected:
                return False, f'{topic}: тип {actual or "отсутствует"}, нужен {expected}'

        pending = set()
        counts = {topic: 0 for topic in OUTPUT_TYPES}
        first_topic = {}
        while result.has_next():
            topic, raw, _ = result.read_next()
            if topic not in OUTPUT_TYPES:
                continue
            counts[topic] += 1
            stamp = stamp_of(deserialize(raw, result_types[topic]))
            if stamp == (0, 0):
                return False, f'{topic}: нулевой header.stamp'
            pending.add(stamp)
            first_topic.setdefault(stamp, topic)
        for topic, count in counts.items():
            if count == 0:
                return False, f'{topic}: нет записанных сообщений'

        source = open_reader(input_bag)
        source_types = {topic.name: topic.type for topic in source.get_all_topics_and_types()}
        while source.has_next() and pending:
            topic, raw, _ = source.read_next()
            if topic in INPUT_TOPICS:
                pending.discard(stamp_of(deserialize(raw, source_types[topic])))
        if pending:
            stamp = min(pending)
            return False, f'{first_topic[stamp]}: header.stamp {display(stamp)} отсутствует во входном bag'
        return True, (f'типы ok; stamp из входного bag: /result/velocity {counts["/result/velocity"]}, '
                      f'/result/position {counts["/result/position"]}')
    except (OSError, RuntimeError, KeyError, TypeError, ValueError, AttributeError) as exc:
        return False, f'не удалось проверить типы/stamp: {type(exc).__name__}: {exc}'


def main(argv):
    if len(argv) != 3:
        print('usage: check_recording.py INPUT_BAG RESULT_BAG', file=sys.stderr)
        return 2

    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    def open_reader(path):
        reader = rosbag2_py.SequentialReader()
        reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                    rosbag2_py.ConverterOptions('cdr', 'cdr'))
        return reader

    @lru_cache(maxsize=8)
    def message_class(type_name):
        return get_message(type_name)

    ok, note = validate(argv[1], argv[2], open_reader,
                        lambda raw, kind: deserialize_message(raw, message_class(kind)))
    print(('ok: ' if ok else 'fail: ') + note)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main(sys.argv))
