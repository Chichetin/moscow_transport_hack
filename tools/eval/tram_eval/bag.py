"""Bag reading and one evaluation run: bag -> Odometry.step -> estimates -> metrics.

Messages go to the pipeline in recording order, exactly as `ros2 bag play` publishes them
to the node, including the non-monotonic and late header.stamp (docs/data.md, traps 5-6).
GNSS is passed only while its header.stamp is within the init window from the stamp of the
first message (D-005); the reference is built from the whole GNSS record.
"""
from __future__ import annotations

import math
import os
import subprocess
import sys
import traceback
from pathlib import Path

import numpy as np
import yaml

from .metrics import Estimates, bag_metrics
from .reference import build_reference

REPO = Path(__file__).resolve().parents[3]
MSG_DIR = REPO / 'src' / 'tram_vehicle_msgs' / 'msg'
PARAMS_YAML = REPO / 'src' / 'tram_odometry' / 'config' / 'params.yaml'
SPLITS_YAML = REPO / 'tools' / 'eval' / 'splits.yaml'

FRONT = '/vehicle/front_bogie_velocity'
REAR = '/vehicle/rear_bogie_velocity'
CMD = '/vehicle/driver_position_cmd'
MASTER_FIX = '/sensing/gnss/master/fix'
ROVER_FIX = '/sensing/gnss/rover/fix'
MASTER_VEL = '/sensing/gnss/master/vel'
INPUTS = (FRONT, REAR, CMD)
GNSS = (MASTER_FIX, ROVER_FIX, MASTER_VEL)     # model inputs inside the window (contracts §1)


def main_checkout() -> Path:
    common = subprocess.run(['git', '-C', str(REPO), 'rev-parse', '--path-format=absolute',
                             '--git-common-dir'], capture_output=True, text=True).stdout.strip()
    return Path(common).parent if common else REPO


def read_dotenv(path: Path) -> dict[str, str]:
    env = {}
    if path.exists():
        for line in path.read_text(encoding='utf-8').splitlines():
            line = line.split('#', 1)[0].strip()
            if '=' in line:
                k, v = line.split('=', 1)
                env[k.strip()] = v.strip()
    return env


def env_path(name: str, default: Path) -> Path:
    """Environment, else .env of this worktree, else default; relative paths are from REPO."""
    value = os.environ.get(name) or read_dotenv(REPO / '.env').get(name)
    if not value:
        return default
    p = Path(value)
    return p if p.is_absolute() else REPO / p


def data_dir() -> Path:
    return env_path('TRAM_DATA_DIR', main_checkout() / 'dataset' / 'data')


def out_dir() -> Path:
    return env_path('TRAM_OUT_DIR', REPO / 'out')


def load_splits() -> dict:
    return yaml.safe_load(SPLITS_YAML.read_text(encoding='utf-8'))


def default_gnss_window() -> float:
    params = yaml.safe_load(PARAMS_YAML.read_text(encoding='utf-8'))
    return float(params['/**']['ros__parameters']['gnss']['init_window_s'])


def stamp(msg) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def typestore():
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    types = {}
    for name in ('VelocitySensor', 'DriverControllerCommand'):
        types.update(get_types_from_msg((MSG_DIR / f'{name}.msg').read_text(encoding='utf-8'),
                                        f'tram_vehicle_msgs/msg/{name}'))
    ts.register(types)
    return ts


def read_bag(path: Path) -> list[tuple[str, object]]:
    """(topic, message) of the input and GNSS topics in recording order."""
    from rosbags.highlevel import AnyReader
    out = []
    with AnyReader([path], default_typestore=typestore()) as reader:
        conns = [c for c in reader.connections if c.topic in INPUTS + GNSS]
        for conn, _, raw in reader.messages(connections=conns):
            out.append((conn.topic, reader.deserialize(raw, conn.msgtype)))
    return out


def to_raw(topic: str, msg):
    """The one place where a bag message becomes the `raw` of Odometry.step.

    The node passes the same (topic, ROS message) pair: rosbags and rclpy messages have the
    same fields. Follows `types.py` once contract v1 (#1) is in main.
    """
    return topic, msg


def default_odometry():
    """pipeline.Odometry with params.yaml, the object tram_odometry node runs (D-001)."""
    sys.path.insert(0, str(REPO / 'src' / 'tram_odometry_core'))
    try:
        from tram_odometry_core.pipeline import Odometry
        from tram_odometry_core.types import load_params
    except ImportError as e:
        raise SystemExit(f'tram_odometry_core.pipeline is not available yet ({e}); '
                         'contract v1 #1 and baseline #3 must be in main') from e
    return Odometry(load_params(str(PARAMS_YAML)))


def reference_inputs(msgs):
    fix_t, fix, vel_t, vel = [], [], [], []
    for topic, m in msgs:
        if topic == MASTER_FIX:
            fix_t.append(stamp(m))
            fix.append((m.latitude, m.longitude, m.altitude, m.status.status))
        elif topic == MASTER_VEL:
            vel_t.append(stamp(m))
            vel.append((m.twist.linear.x, m.twist.linear.y))
    return fix_t, fix, vel_t, vel


def run_pipeline(msgs, odometry, gnss_window_s: float):
    """Feed messages; returns Estimates, crash text or None, count of Estimate.t != input stamp."""
    t, speed, pos, slip = [], [], [], []
    crash, stamp_mismatch = None, 0
    window_end = stamp(msgs[0][1]) + gnss_window_s if msgs else 0.0
    for topic, msg in msgs:
        s = stamp(msg)
        if topic in GNSS and s > window_end:
            continue
        try:
            est = odometry.step(to_raw(topic, msg))
        except Exception:
            crash = traceback.format_exc(limit=3)
            break
        if est is None:
            continue
        stamp_mismatch += abs(est.t - s) > 1e-6
        t.append(est.t)
        speed.append(est.speed)
        pos.append((est.x, est.y, getattr(est, 'z', 0.0)))
        slip.append(bool(est.slip.slip_front or est.slip.slip_rear))
    est = Estimates(np.asarray(t, float), np.asarray(speed, float),
                    np.asarray(pos, float).reshape(-1, 3), np.asarray(slip, bool))
    return est, crash, stamp_mismatch


def evaluate_bag(path: Path, gnss_window_s: float, make_odometry=default_odometry,
                 msgs=None) -> dict:
    """Metrics dict of one bag (docs/contracts.md §4). `msgs` skips reading, for tests."""
    msgs = read_bag(path) if msgs is None else msgs
    stamps = [stamp(m) for _, m in msgs]
    window_end = stamps[0] + gnss_window_s if stamps else 0.0
    ref = build_reference(*reference_inputs(msgs), window_end)
    est, crash, mismatch = run_pipeline(msgs, make_odometry(), gnss_window_s)
    m = {'duration_s': float(max(stamps) - min(stamps)) if stamps else 0.0}
    m.update(bag_metrics(ref, est))
    m['crashed'] = crash is not None
    if crash:
        print(f'{Path(path).name}: pipeline crashed\n{crash}', file=sys.stderr)
    if mismatch:
        print(f'{Path(path).name}: {mismatch} estimates with t != input stamp (contract §1, D-015)',
              file=sys.stderr)
    return {k: (round(v, 4) if isinstance(v, float) and math.isfinite(v) else v) for k, v in m.items()}
