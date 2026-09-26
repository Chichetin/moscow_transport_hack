"""One evaluation run on fake messages: GNSS window, recording order, crash, stamp contract."""
from types import SimpleNamespace as NS

import pytest

from tram_eval import bag
from tram_eval.bag import CMD, FRONT, MASTER_FIX, MASTER_VEL, REAR, ROVER_FIX, evaluate_bag, run_pipeline

LAT0, LON0 = 55.75, 37.60
M_PER_DEG_E = 62_860.0           # approx at 55.75 N, enough for a synthetic track


def header(t):
    return NS(stamp=NS(sec=int(t), nanosec=int(round((t - int(t)) * 1e9))))


def wheel(t, kmh):
    return NS(header=header(t), velocity=kmh)


def fix(t, x_east):
    return NS(header=header(t), latitude=LAT0, longitude=LON0 + x_east / M_PER_DEG_E, altitude=150.0,
              status=NS(status=0))


def vel(t, ve):
    return NS(header=header(t), twist=NS(linear=NS(x=ve, y=0.0, z=0.0)))


def drive(duration=60.0, speed=10.0):
    """Tram moving east at `speed` m/s: wheels, controller, master/rover GNSS at 10 Hz."""
    msgs = []
    for k in range(int(duration * 10)):
        t = 100.0 + 0.1 * k
        msgs += [(FRONT, wheel(t, speed * 3.6)), (REAR, wheel(t + 0.01, speed * 3.6)),
                 (CMD, NS(header=header(t + 0.02), position=3)),
                 (MASTER_FIX, fix(t + 0.03, speed * 0.1 * k)), (ROVER_FIX, fix(t + 0.03, speed * 0.1 * k - 12.4)),
                 (MASTER_VEL, vel(t + 0.04, speed))]
    return msgs


class DeadReckoning:
    """Test double of pipeline.Odometry: wheel mean / 3.6 integrated east from x = 0."""

    def __init__(self, crash_at=None, stamp_offset=0.0):
        self.seen, self.x, self.t, self.v = [], 0.0, None, 0.0
        self.crash_at, self.stamp_offset = crash_at, stamp_offset

    def step(self, raw):
        topic, msg = raw
        t = bag.stamp(msg)
        self.seen.append((topic, t))
        if self.crash_at is not None and t >= self.crash_at:
            raise ValueError('boom')
        if topic in (FRONT, REAR):
            self.v = msg.velocity / 3.6
        if self.t is not None and t > self.t:
            self.x += self.v * (t - self.t)
        self.t = t if self.t is None else max(self.t, t)
        return NS(t=t + self.stamp_offset, speed=self.v, x=self.x, y=0.0,
                  slip=NS(slip_front=False, slip_rear=topic == REAR))


def test_gnss_after_window_is_not_fed_but_used_as_reference():
    odo = DeadReckoning()
    msgs = drive()
    est, crash, mismatch = run_pipeline(msgs, odo, bag.gnss_window_end(msgs, 5.0))
    gnss_seen = [t for topic, t in odo.seen if topic in bag.GNSS]
    assert crash is None and mismatch == 0
    assert gnss_seen and max(gnss_seen) <= 105.0
    assert sum(topic == FRONT for topic, _ in odo.seen) == 600   # every wheel message fed
    m = evaluate_bag('fake', 5.0, make_odometry=DeadReckoning, msgs=msgs, ref_point='master')
    assert m['n_matched'] == 600                                 # reference: whole GNSS record
    assert m['speed_rmse'] == pytest.approx(0.0, abs=1e-9)
    assert m['along_rmse'] < 1.0 and m['drift_pct'] < 0.2      # the double lags the fix stamps by 10-30 ms
    assert m['slip_flag_frac'] == pytest.approx(600 / (1800 + 150), abs=1e-4)  # rear / all fed: 3 per 0.1 s + GNSS in 5 s
    assert m['crashed'] is False


def test_zero_window_feeds_no_gnss():
    odo = DeadReckoning()
    run_pipeline(drive(), odo, bag.gnss_window_end(drive(), 0.0))
    assert not [t for topic, t in odo.seen if topic in bag.GNSS and t > 100.0]


def test_messages_fed_in_recording_order_with_stamps_going_back():
    msgs = drive(5.0)
    msgs.insert(10, (FRONT, wheel(99.0, 36.0)))                  # late message, stamp from the past
    odo = DeadReckoning()
    run_pipeline(msgs, odo, bag.gnss_window_end(msgs, 5.0))
    assert [t for _, t in odo.seen][:12] == [bag.stamp(m) for _, m in msgs[:12]]


def test_crash_is_reported_not_raised():
    m = evaluate_bag('fake', 5.0, make_odometry=lambda: DeadReckoning(crash_at=130.0), msgs=drive())
    assert m['crashed'] is True
    assert m['n_matched'] > 0


def test_estimate_stamp_other_than_input_is_counted(capsys):
    _, _, mismatch = run_pipeline(drive(1.0), DeadReckoning(stamp_offset=0.02), bag.gnss_window_end(drive(1.0), 5.0))
    assert mismatch == 60
    evaluate_bag('fake', 5.0, make_odometry=lambda: DeadReckoning(stamp_offset=0.02), msgs=drive(1.0))
    assert 't != input stamp' in capsys.readouterr().err


def test_dotenv_relative_path_is_from_repo(tmp_path, monkeypatch):
    monkeypatch.delenv('TRAM_DATA_DIR', raising=False)
    monkeypatch.setattr(bag, 'REPO', tmp_path)
    (tmp_path / '.env').write_text('TRAM_DATA_DIR=./dataset/data  # comment\n')
    assert bag.data_dir() == tmp_path / 'dataset' / 'data'


def test_splits_and_window_from_contract_files():
    splits = bag.load_splits()
    assert set(splits['quick']) <= set(splits['holdout'])
    assert not set(splits['holdout']) & set(splits['train'])
    assert bag.default_gnss_window() == 5.0


def test_real_short_bag_is_read_in_recording_order():
    path = bag.data_dir() / '30618_082f1d65'                     # 20 s holdout bag
    if not (path / 'metadata.yaml').exists():
        pytest.skip('dataset not unpacked (docs/data.md)')
    msgs = bag.read_bag(path)
    topics = {t for t, _ in msgs}
    assert {FRONT, REAR, CMD} <= topics
    m = evaluate_bag(path, 5.0, make_odometry=DeadReckoning, msgs=msgs)
    assert m['crashed'] is False and m['duration_s'] > 10


class Glitchy(DeadReckoning):
    """Returns NaN position on every 50th step and inf speed on every 70th."""

    def step(self, raw):
        est = super().step(raw)
        n = len(self.seen)
        if n % 50 == 0:
            est.x = float('nan')
        if n % 70 == 0:
            est.speed = float('inf')
        return est


def test_non_finite_estimates_are_counted_not_scored(capsys):
    m = evaluate_bag('fake', 5.0, make_odometry=Glitchy, msgs=drive())
    assert m['crashed'] is False
    assert m[bag.NOTES]['nonfinite'] == len(range(50, 1950 + 1, 50)) + len(range(70, 1950 + 1, 70)) \
        - len(range(350, 1950 + 1, 350))     # every 50th and every 70th of the 1950 fed, both counted once
    assert all(m[k] is not None and m[k] == m[k] for k in ('speed_rmse', 'along_rmse', 'drift_pct'))
    assert 'non-finite' in capsys.readouterr().err


def test_metrics_failure_marks_one_bag_crashed(monkeypatch, capsys):
    def broken(ref, est):
        raise ValueError('metrics bug')
    monkeypatch.setattr(bag, 'bag_metrics', broken)
    m = evaluate_bag('fake', 5.0, make_odometry=DeadReckoning, msgs=drive(1.0))
    assert m['crashed'] is True
    assert 'metrics bug' in capsys.readouterr().err


def test_bag_without_gnss_has_no_metrics_and_does_not_crash():
    msgs = [x for x in drive(10.0) if x[0] not in bag.GNSS]
    m = evaluate_bag('fake', 5.0, make_odometry=DeadReckoning, msgs=msgs)
    assert m['crashed'] is False and m['n_matched'] == 0
    assert m['speed_rmse'] is None and m['along_rmse'] is None and m['drift_pct'] is None
