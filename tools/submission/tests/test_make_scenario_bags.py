"""Scenario variants of make_scenario_bags decide by header.stamp, on serialized messages."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import make_scenario_bags as msb  # noqa: E402

FIX, VEL, FRONT = '/sensing/gnss/master/fix', '/sensing/gnss/master/vel', '/vehicle/front_bogie_velocity'
TYPES = {FIX: 'sensor_msgs/msg/NavSatFix', VEL: 'geometry_msgs/msg/TwistStamped',
         FRONT: 'tram_vehicle_msgs/msg/VelocitySensor'}


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


def test_gnss_first_drops_vehicle_before_first_fix(ts):
    r = rows(ts, [(FRONT, 100.0), (FRONT, 100.1), (FIX, 100.2), (FRONT, 100.15), (FRONT, 100.3)])
    assert msb.rows_to_drop(r, 'gnss_first', ts) == {0, 1, 3}


def test_short_gnss_keeps_window_inclusive(ts):
    r = rows(ts, [(FIX, 99.0), (FRONT, 100.0), (FIX, 105.0), (VEL, 105.5), (FIX, 110.0)])
    assert msb.rows_to_drop(r, 'short_gnss', ts) == {3, 4}


def test_no_gnss_and_crop(ts):
    r = rows(ts, [(FIX, 99.0), (FRONT, 100.0), (VEL, 101.0)])
    assert msb.rows_to_drop(r, 'no_gnss', ts) == {0, 2}
    assert msb.rows_to_drop(r, 'crop', ts) == set()


def test_gnss_first_without_fix_is_an_error(ts):
    with pytest.raises(ValueError):
        msb.rows_to_drop(rows(ts, [(FRONT, 100.0)]), 'gnss_first', ts)
