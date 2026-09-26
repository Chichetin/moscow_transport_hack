"""Drive model inside Odometry.step (#10): response delay, prediction on sensor gaps, and
accel_model handed to the slip detector. Messages are minimal stand-ins for the ROS ones."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from tram_odometry_core.dynamics import model_accel
from tram_odometry_core.pipeline import Odometry
from tram_odometry_core.types import load_params

ROOT = Path(__file__).resolve().parents[3]
PARAMS = load_params(ROOT / 'src/tram_odometry/config/params.yaml')
FRONT, REAR, CMD = ('/vehicle/front_bogie_velocity', '/vehicle/rear_bogie_velocity',
                    '/vehicle/driver_position_cmd')
T0 = 1_700_000_000.0


def _stamp(t):
    sec = int(t)
    return SimpleNamespace(sec=sec, nanosec=int(round((t - sec) * 1e9)))


def _wheel(t, kmh):
    return SimpleNamespace(header=SimpleNamespace(stamp=_stamp(t)), velocity=float(kmh))


def _cmd(t, notch):
    return SimpleNamespace(header=SimpleNamespace(stamp=_stamp(t)), position=int(notch))


def _run(odo, events):
    """events: (t, topic, value); returns the last Estimate per event (None for dropped)."""
    out = []
    for t, topic, value in events:
        msg = _cmd(t, value) if topic == CMD else _wheel(t, value)
        out.append(odo.step((topic, msg)))
    return out


def _cruise(odo, kmh, t_from, t_to, notch, step=0.1):
    """Both bogies at `kmh`, controller at `notch`, every `step` s."""
    t = t_from
    while t < t_to - 1e-9:
        _run(odo, [(t, FRONT, kmh), (t, REAR, kmh), (t, CMD, notch), (t + step / 2, CMD, notch)])
        t += step
    return t


def test_accel_model_follows_the_delayed_controller_position():
    odo = Odometry(PARAMS)
    _cruise(odo, 18.0, T0, T0 + 1.0, notch=0)          # 5 m/s, neutral for 1 s
    # controller goes to 10 at T0+1.0; the drive reacts response_delay_s later
    delay = PARAMS.drive.response_delay_s
    events = [(T0 + 1.0, CMD, 10)]
    for k in range(1, 8):
        t = T0 + 1.0 + 0.05 * k
        events.append((t, CMD, 10))
        events.append((t, FRONT, 18.0))
    ests = [e for e in _run(odo, events) if e is not None]
    before = [e for e in ests if e.t < T0 + 1.0 + delay - 1e-6]
    after = [e for e in ests if e.t >= T0 + 1.0 + delay + 0.05]
    assert before and after
    assert all(e.accel_model == pytest.approx(model_accel(0, e.speed, PARAMS), abs=0.005)
               for e in before)
    assert all(e.accel_model == pytest.approx(model_accel(10, e.speed, PARAMS), abs=0.005)
               for e in after)
    assert after[-1].accel_model > 0.3                   # traction, not drag


def test_speed_is_predicted_by_the_model_when_both_bogies_are_silent():
    odo = Odometry(PARAMS)
    t = _cruise(odo, 18.0, T0, T0 + 2.0, notch=10)      # 5 m/s, traction 10
    v0 = odo.step((CMD, _cmd(t, 10))).speed
    assert v0 == pytest.approx(5.0, abs=0.1)
    # both wheels silent for 3 s, only the controller keeps coming at 20 Hz
    last = None
    for k in range(1, 61):
        last = odo.step((CMD, _cmd(t + 0.05 * k, 10)))
    assert last.speed > v0 + 0.5                        # accelerating on the model
    assert last.speed < v0 + 3.0 * PARAMS.drive.adhesion_accel_mps2
    assert last.slip.front_trust == 0.0 and last.slip.rear_trust == 0.0
    assert last.distance > 2.0 * 5.0 + 3.0 * v0         # path integrates the predicted speed


def test_without_the_model_the_speed_is_held_on_a_gap():
    params = replace(PARAMS, drive=replace(PARAMS.drive, use_model=False))
    odo = Odometry(params)
    t = _cruise(odo, 18.0, T0, T0 + 2.0, notch=10)
    last = None
    for k in range(1, 61):
        last = odo.step((CMD, _cmd(t + 0.05 * k, 10)))
    assert last.speed == pytest.approx(5.0, abs=0.01)
    assert last.accel_model == 0.0


def test_braking_prediction_stops_at_zero_and_never_reverses():
    odo = Odometry(PARAMS)
    t = _cruise(odo, 3.6, T0, T0 + 2.0, notch=-15)      # 1 m/s, full brake
    last = None
    for k in range(1, 201):                             # 10 s without wheels
        last = odo.step((CMD, _cmd(t + 0.05 * k, -15)))
        assert last.speed >= 0.0
    assert last.speed == 0.0
    assert last.accel_model < 0.0                        # brake + drag still reported


def test_neutral_before_any_command_is_drag_only():
    odo = Odometry(PARAMS)
    est = odo.step((FRONT, _wheel(T0, 18.0)))
    assert est.accel_model == pytest.approx(model_accel(0, est.speed, PARAMS))


def test_standing_tram_stays_at_zero_on_a_gap():
    odo = Odometry(PARAMS)
    t = _cruise(odo, 0.0, T0, T0 + 1.0, notch=0)
    last = None
    for k in range(1, 41):
        last = odo.step((CMD, _cmd(t + 0.05 * k, 0)))
    assert last.speed == 0.0 and last.distance == 0.0


def test_command_history_is_bounded():
    odo = Odometry(PARAMS)
    for k in range(500):
        odo.step((CMD, _cmd(T0 + 0.05 * k, 3)))
    assert len(odo._cmd) == odo._cmd.maxlen


def test_wheels_lagging_behind_the_controller_are_still_used():
    """docs/data.md trap 5: wheel stamps can trail the controller by seconds; a late wheel
    sample is a measurement, not silence — on wheel and on controller events alike."""
    for use_model in (True, False):
        params = replace(PARAMS, drive=replace(PARAMS.drive, use_model=use_model))
        odo = Odometry(params)
        on_cmd, on_wheel = [], []
        for k in range(40):                              # 4 s, controller 1.0 s ahead
            t = T0 + 0.1 * k
            on_cmd.append(odo.step((CMD, _cmd(t + 1.0, 10))))
            on_wheel.append(odo.step((FRONT, _wheel(t, 36.0))))
            odo.step((REAR, _wheel(t, 36.0)))
        for est in on_cmd[10:] + on_wheel[10:]:
            assert 9.5 < est.speed < 11.5, use_model
            assert (est.slip.front_trust, est.slip.rear_trust) == (1.0, 1.0), use_model


@pytest.mark.parametrize('alive', [FRONT, REAR])
def test_one_silent_bogie_with_controller_events_in_between(alive):
    """30639: one bogie is silent for up to 73 s while the other talks; controller events
    between wheel samples must not switch to prediction (trust 1/0 or 0/1, never 0/0)."""
    odo = Odometry(PARAMS)
    _cruise(odo, 36.0, T0, T0 + 1.0, notch=10)
    for k in range(50):                                  # 5 s, only one bogie talks
        t = T0 + 1.0 + 0.1 * k
        odo.step((alive, _wheel(t, 36.0)))
        est = odo.step((CMD, _cmd(t + 0.05, 10)))
        if k > 6:
            assert est.speed == pytest.approx(10.0, abs=0.2)
            trusts = (est.slip.front_trust, est.slip.rear_trust)
            assert trusts == ((1.0, 0.0) if alive == FRONT else (0.0, 1.0))
            assert est.slip.slip_rear if alive == FRONT else est.slip.slip_front


def test_without_the_model_lagging_wheels_and_gaps_behave_like_the_baseline():
    params = replace(PARAMS, drive=replace(PARAMS.drive, use_model=False))
    odo = Odometry(params)
    t = _cruise(odo, 36.0, T0, T0 + 2.0, notch=10)
    # controller 2 s ahead of the wheels, then wheels stop: speed is held, no prediction
    last = None
    for k in range(1, 61):
        last = odo.step((CMD, _cmd(t + 2.0 + 0.05 * k, 10)))
    assert last.speed == pytest.approx(10.0, abs=1e-6)
    assert last.accel_model == 0.0
    assert (last.slip.front_trust, last.slip.rear_trust) == (1.0, 1.0)   # as on main


def test_accel_model_changes_which_bogie_the_detector_blames():
    """Disagreeing bogies, neither at 0: the detector blames the one further from its
    prediction `est + accel_model * dt`; with full brake the prediction drops, so the slower
    bogie becomes the plausible one. Steps stay inside the 3 m/s^2 outlier gate."""
    results = {}
    for use_model in (True, False):
        params = replace(PARAMS, drive=replace(PARAMS.drive, use_model=use_model))
        odo = Odometry(params)
        t = T0
        for _ in range(20):                               # 2 s: front 5.0, rear 5.2 (agree)
            _run(odo, [(t, CMD, -15), (t, FRONT, 5.0 * 3.6), (t, REAR, 5.2 * 3.6),
                       (t + 0.05, CMD, -15)])
            t += 0.1
        t += 0.3                                          # 0.4 s after the last wheels
        odo.step((CMD, _cmd(t - 0.2, -15)))
        est = odo.step((FRONT, _wheel(t, 4.1 * 3.6)))    # front drops 0.9 m/s, rear is 5.2
        assert est is not None
        results[use_model] = (est.slip.front_trust, est.slip.rear_trust)
    assert results[False] == (0.0, 1.0)                  # a = 0: front is the outlier
    assert results[True] == (1.0, 0.0)                   # brake model: rear is the outlier
