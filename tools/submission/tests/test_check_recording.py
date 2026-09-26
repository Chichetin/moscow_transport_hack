"""The jury recording gate checks every published message against input stamps."""
from types import SimpleNamespace

import check_recording as gate


def message(sec, nanosec=0):
    return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=nanosec)))


class Reader:
    def __init__(self, topics, records):
        self.topics = [SimpleNamespace(name=name, type=kind) for name, kind in topics.items()]
        self.records = iter(records)
        self.next_record = next(self.records, None)

    def get_all_topics_and_types(self):
        return self.topics

    def has_next(self):
        return self.next_record is not None

    def read_next(self):
        current = self.next_record
        self.next_record = next(self.records, None)
        return current


INPUT = {'/vehicle/front_bogie_velocity': 'tram_vehicle_msgs/msg/VelocitySensor',
         '/vehicle/rear_bogie_velocity': 'tram_vehicle_msgs/msg/VelocitySensor',
         '/vehicle/driver_position_cmd': 'tram_vehicle_msgs/msg/DriverControllerCommand'}
OUTPUT = {'/result/velocity': 'tram_vehicle_msgs/msg/VelocitySensor',
          '/result/position': 'nav_msgs/msg/Odometry'}


def check(inputs, outputs, output_types=None):
    readers = {'input': Reader(INPUT, inputs), 'output': Reader(output_types or OUTPUT, outputs)}
    return gate.validate('input', 'output', lambda path: readers[path], lambda raw, kind: raw)


def test_valid_recording_passes_with_stamps_from_any_allowed_input():
    inputs = [('/vehicle/front_bogie_velocity', message(17, 3), 0),
              ('/vehicle/driver_position_cmd', message(18, 4), 1)]
    outputs = [('/result/velocity', message(17, 3), 9),
               ('/result/position', message(18, 4), 10)]
    assert check(inputs, outputs)[0] is True


def test_wrong_type_fails_with_topic_and_type():
    types = dict(OUTPUT, **{'/result/position': 'geometry_msgs/msg/PoseStamped'})
    ok, note = check([], [], types)
    assert not ok and '/result/position' in note and 'PoseStamped' in note


def test_foreign_stamp_fails_with_topic_and_stamp():
    inputs = [('/vehicle/rear_bogie_velocity', message(17, 3), 0)]
    outputs = [('/result/velocity', message(17, 3), 9),
               ('/result/position', message(18, 4), 10)]
    ok, note = check(inputs, outputs)
    assert not ok and '/result/position' in note and '18.000000004' in note


def test_zero_stamp_fails_even_if_present_in_input():
    inputs = [('/vehicle/front_bogie_velocity', message(0), 0)]
    outputs = [('/result/velocity', message(0), 9),
               ('/result/position', message(0), 10)]
    ok, note = check(inputs, outputs)
    assert not ok and 'нулевой' in note


def test_each_output_message_is_checked():
    inputs = [('/vehicle/front_bogie_velocity', message(17), 0)]
    outputs = [('/result/velocity', message(17), 9),
               ('/result/velocity', message(99), 10),
               ('/result/position', message(17), 11)]
    ok, note = check(inputs, outputs)
    assert not ok and '/result/velocity' in note and '99.000000000' in note


def test_gnss_stamp_does_not_authorize_result_publication():
    inputs = [('/sensing/gnss/master/fix', message(99), 0),
              ('/vehicle/front_bogie_velocity', message(17), 1)]
    outputs = [('/result/velocity', message(99), 9),
               ('/result/position', message(99), 10)]
    ok, note = check(inputs, outputs)
    assert not ok and '/result/velocity' in note and '99.000000000' in note
