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


# an exact non-zero repeat while the drive model changes the speed by more than
# slip.freeze_dv_mps is a frozen sensor (#144): real readings under traction or brake never do
# that, so steady speeds here are dithered by this share on every other sample (0 stays 0)
DITHER = 1e-9


def _cruise(odo, kmh, t_from, t_to, notch, step=0.1):
    """Both bogies at `kmh`, controller at `notch`, every `step` s."""
    t, k = t_from, 0
    while t < t_to - 1e-9:
        v = kmh * (1.0 + DITHER * (k % 2))
        _run(odo, [(t, FRONT, v), (t, REAR, v), (t, CMD, notch), (t + step / 2, CMD, notch)])
        t, k = t + step, k + 1
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
    sample is a measurement, not silence — on wheel and on controller events alike. The
    state runs at the controller's time; an output stamped with the late wheel carries the
    speed at the wheel's stamp (#105), not the speed 1 s later."""
    for use_model in (True, False):
        params = replace(PARAMS, drive=replace(PARAMS.drive, use_model=use_model))
        odo = Odometry(params)
        on_cmd, on_wheel = [], []
        for k in range(40):                              # 4 s, controller 1.0 s ahead
            t = T0 + 0.1 * k
            v = 5.0 + 0.05 * k                           # accelerating at 0.5 m/s^2
            on_cmd.append((v + 0.5, odo.step((CMD, _cmd(t + 1.0, 10)))))
            on_wheel.append((t, v, odo.step((FRONT, _wheel(t, v * 3.6)))))
            odo.step((REAR, _wheel(t, v * 3.6)))
        for t, v, est in on_wheel[20:]:
            assert est.t == pytest.approx(t, abs=1e-6)
            assert est.speed == pytest.approx(v, abs=0.1), use_model
        for v, est in on_cmd[20:]:
            assert est.speed == pytest.approx(v, abs=0.5), use_model
            assert (est.slip.front_trust, est.slip.rear_trust) == (1.0, 1.0), use_model


def test_speed_at_the_stamp_of_a_late_wheel_is_never_negative():
    # trap 5 at a start from rest: the controller 3.7 s ahead at full traction, the wheels
    # just starting; the speed moved back 3.7 s along the acceleration would be below 0
    odo = Odometry(PARAMS)
    for k in range(60):
        t = T0 + 0.1 * k
        v = max(0.0, 1.0 * (0.1 * k - 3.0))
        odo.step((CMD, _cmd(t + 3.7, 15)))
        for topic in (FRONT, REAR):
            est = odo.step((topic, _wheel(t, v * 3.6)))
            assert est is not None and est.speed >= 0.0


@pytest.mark.parametrize('alive', [FRONT, REAR])
def test_one_silent_bogie_with_controller_events_in_between(alive):
    """30639: one bogie is silent for up to 73 s while the other talks; controller events
    between wheel samples must not switch to prediction (trust 1/0 or 0/1, never 0/0)."""
    odo = Odometry(PARAMS)
    _cruise(odo, 36.0, T0, T0 + 1.0, notch=10)
    for k in range(50):                                  # 5 s, only one bogie talks
        t = T0 + 1.0 + 0.1 * k
        odo.step((alive, _wheel(t, 36.0 * (1.0 + DITHER * (k % 2)))))
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


@pytest.mark.parametrize('notch', [-5, -15])
@pytest.mark.parametrize('lag', [1.0, 3.7])
def test_late_wheel_at_rest_under_brake_reads_standstill(notch, lag):
    """#105 review: at rest under brake the filter acceleration is negative (the drive model
    brakes at v = 0) while the state is held at 0. Moving a late wheel's output back along
    that acceleration would publish |a| * lag of phantom speed; the car was standing."""
    assert model_accel(notch, 0.0, PARAMS) < 0.0
    odo = Odometry(PARAMS)
    worst = 0.0
    for k in range(200):
        t = T0 + 0.1 * k
        odo.step((CMD, _cmd(t + lag, notch)))
        for topic in (FRONT, REAR):
            est = odo.step((topic, _wheel(t, 0.0)))
            assert est is not None
            worst = max(worst, est.speed)    # from the first message: standing since start
    assert worst < PARAMS.position.stop_speed_mps
    assert worst == 0.0                      # not even the neutral drag moved back


@pytest.mark.parametrize('lag', [1.0, 3.7])
def test_late_wheel_braking_to_a_stop_keeps_its_own_speed(lag):
    """Braking to a stop with the controller ahead: before the state reaches 0 the late
    wheel still carries the speed at its own stamp, and after the car stands it reads 0."""
    odo = Odometry(PARAMS)
    a = model_accel(-15, 5.0, PARAMS)
    for k in range(120):
        t = T0 + 0.1 * k
        v = max(0.0, 8.0 + a * 0.1 * k)
        odo.step((CMD, _cmd(t + lag, -15)))
        for topic in (FRONT, REAR):
            est = odo.step((topic, _wheel(t, v * 3.6)))
        if k > 40:
            assert est.speed == pytest.approx(v, abs=0.6), (k, v)
    assert est.speed < PARAMS.position.stop_speed_mps



def test_late_wheel_after_the_state_braked_to_zero_on_prediction():
    """The wheels fall silent while braking at 2 m/s, the controller runs 8 s ahead and the
    state reaches 0 on the prediction in between. A late zero wheel stamped before that
    moment is published on the state's braking line, moved back from where it crossed 0
    (t + v / -a), not from the latest state time 8 s later."""
    odo = Odometry(PARAMS)
    t = _cruise(odo, 7.2, T0, T0 + 2.0, notch=-15)     # 2 m/s under brake
    v0, _, accel = odo._filter.state()
    assert accel < 0.0
    assert odo.step((CMD, _cmd(t + 8.0, -15))).speed == 0.0
    t_cross = t + v0 / -accel
    est = odo.step((FRONT, _wheel(t_cross - 1.0, 0.0)))
    assert est.speed == pytest.approx(-accel, abs=0.1)
    assert odo.step((REAR, _wheel(t_cross + 0.1, 0.0))).speed == 0.0


def test_frozen_bogies_under_traction_hand_the_speed_to_the_model_and_come_back():
    """#144 through Odometry.step: both bogies repeat 18 km/h exactly while traction 10 pulls:
    no trust, both flagged, the speed follows the drive model; the first changed reading
    brings both back."""
    odo = Odometry(PARAMS)
    t = _cruise(odo, 18.0, T0, T0 + 2.0, notch=10)       # 5 m/s, honest (dithered) readings
    v0 = odo.step((CMD, _cmd(t, 10))).speed
    last = None
    for k in range(20):                                   # 2 s frozen at exactly 18 km/h
        last = _run(odo, [(t + 0.1 * k, FRONT, 18.0), (t + 0.1 * k, REAR, 18.0),
                          (t + 0.1 * k + 0.05, CMD, 10)])[-1]
    assert (last.slip.front_trust, last.slip.rear_trust) == (0.0, 0.0)
    assert last.slip.slip_front and last.slip.slip_rear
    assert last.speed > v0 + 0.2              # accelerating on the model, not held at 5 m/s
    t += 2.0
    back = _run(odo, [(t, FRONT, 18.1), (t, REAR, 18.1)])[-1]
    assert (back.slip.front_trust, back.slip.rear_trust) == (1.0, 1.0)
    assert not back.slip.slip_front and not back.slip.slip_rear
