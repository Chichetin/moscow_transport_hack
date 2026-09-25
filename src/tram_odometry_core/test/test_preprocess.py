import dataclasses
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from tram_odometry_core.preprocess import CMD_TOPIC, FRONT_TOPIC, REAR_TOPIC, Preprocessor
from tram_odometry_core.types import CommandSample, GnssFix, GnssVel, WheelSample, load_params

ROOT = Path(__file__).resolve().parents[3]
PARAMS = load_params(ROOT / 'src' / 'tram_odometry' / 'config' / 'params.yaml')
MASTER_FIX = PARAMS.gnss.topic_fix
MASTER_VEL = PARAMS.gnss.topic_vel
KMH_36 = 36.0  # = 10 m/s
ACCEL_LIMIT = PARAMS.input.max_wheel_accel_mps2


def _hdr(t):
    sec = math.floor(t)
    return SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=round((t - sec) * 1e9)))


def wheel(topic, t, kmh):
    return topic, SimpleNamespace(header=_hdr(t), velocity=kmh)


def cmd(t, notch):
    return CMD_TOPIC, SimpleNamespace(header=_hdr(t), position=notch)


def gnss_fix(t, status=0, lat=55.7, lon=37.6, alt=150.0):
    return MASTER_FIX, SimpleNamespace(header=_hdr(t), latitude=lat, longitude=lon,
                                       altitude=alt, status=SimpleNamespace(status=status))


def gnss_vel(t, ve, vn):
    return MASTER_VEL, SimpleNamespace(
        header=_hdr(t), twist=SimpleNamespace(linear=SimpleNamespace(x=ve, y=vn, z=0.0)))


def test_kmh_to_mps_is_the_only_conversion():
    sample = Preprocessor(PARAMS).accept(wheel(FRONT_TOPIC, 1.0, KMH_36))
    assert isinstance(sample, WheelSample)
    assert sample.speed == pytest.approx(10.0)
    assert sample.bogie == 'front'
    assert sample.t == pytest.approx(1.0)


def test_wheel_scale_applied_per_bogie():
    p = dataclasses.replace(PARAMS, vehicle=dataclasses.replace(
        PARAMS.vehicle, wheel_scale_front=1.1, wheel_scale_rear=0.9))
    pre = Preprocessor(p)
    front = pre.accept(wheel(FRONT_TOPIC, 1.0, KMH_36))
    rear = pre.accept(wheel(REAR_TOPIC, 1.0, KMH_36))
    assert front.speed == pytest.approx(11.0)
    assert rear.speed == pytest.approx(9.0)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf'), -5.0, 'text', True])
def test_bad_wheel_value_dropped(bad):
    assert Preprocessor(PARAMS).accept(wheel(FRONT_TOPIC, 1.0, bad)) is None


def test_non_monotonic_wheel_stamp_dropped_repeat_and_rollback():
    pre = Preprocessor(PARAMS)
    assert pre.accept(wheel(FRONT_TOPIC, 5.0, KMH_36)) is not None
    assert pre.accept(wheel(FRONT_TOPIC, 5.0, KMH_36)) is None    # repeated stamp
    assert pre.accept(wheel(FRONT_TOPIC, 4.0, KMH_36)) is None    # rollback
    assert pre.accept(wheel(FRONT_TOPIC, 5.1, KMH_36)) is not None


def test_streams_gate_independently():
    """A rollback on one bogie must not block the other stream or the controller (trap 6)."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 5.0, KMH_36))
    assert pre.accept(wheel(FRONT_TOPIC, 1.0, KMH_36)) is None      # front rollback dropped
    assert pre.accept(wheel(REAR_TOPIC, 1.0, KMH_36)) is not None   # rear stream unaffected
    assert pre.accept(cmd(1.0, 0)) is not None                      # controller unaffected


def test_controller_20hz_rollback_dropped():
    """Trap 9: controller at 20 Hz can roll back up to 20 times in a bag."""
    pre = Preprocessor(PARAMS)
    assert pre.accept(cmd(1.0, 5)) is not None
    assert pre.accept(cmd(1.0, 5)) is None       # repeated stamp, two publishers (trap 9)
    assert pre.accept(cmd(0.9, 5)) is None       # rollback
    assert pre.accept(cmd(1.05, 5)) is not None


@pytest.mark.parametrize('notch', [3.0, '5', None, True])
def test_bad_command_value_dropped(notch):
    assert Preprocessor(PARAMS).accept(cmd(1.0, notch)) is None


BASE_MPS = 5.0   # non-zero: the stuck-at-zero exemption must not apply to these cases


def test_wheel_acceleration_outlier_dropped():
    """A jump faster than `input.max_wheel_accel_mps2` is a sensor glitch, not real driving."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, BASE_MPS * 3.6))
    jump_mps = BASE_MPS + ACCEL_LIMIT * 5.0                           # far beyond the limit over 1 s
    assert pre.accept(wheel(FRONT_TOPIC, 1.0, jump_mps * 3.6)) is None


def test_wheel_acceleration_outlier_does_not_poison_state():
    """A rejected sample must not become the new reference for the next comparison."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, BASE_MPS * 3.6))
    jump_mps = BASE_MPS + ACCEL_LIMIT * 5.0
    assert pre.accept(wheel(FRONT_TOPIC, 1.0, jump_mps * 3.6)) is None   # rejected glitch
    within_mps = BASE_MPS + (ACCEL_LIMIT - 0.1) * 1.0                 # still close to the real 5 m/s
    sample = pre.accept(wheel(FRONT_TOPIC, 1.0 + 1e-6, within_mps * 3.6))
    assert sample is not None


def test_wheel_acceleration_within_limit_accepted():
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, BASE_MPS * 3.6))
    ok_mps = BASE_MPS + (ACCEL_LIMIT - 0.1) * 1.0                     # dt=1 s, just under the limit
    assert pre.accept(wheel(FRONT_TOPIC, 1.0, ok_mps * 3.6)) is not None


def test_wheel_deceleration_outlier_dropped_too():
    """The physical limit is on `|dv/dt|`, braking spikes are outliers just like traction ones."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, 100.0))
    assert pre.accept(wheel(FRONT_TOPIC, 1.0, 20.0)) is None    # not exactly 0: gate applies


def test_stuck_at_zero_recovery_is_not_an_outlier():
    """Trap 8: a bogie stuck exactly at 0 can jump straight back to the real speed."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, 0.0))
    sample = pre.accept(wheel(FRONT_TOPIC, 0.1, KMH_36))    # 10 m/s in 0.1 s: unstuck, not a glitch
    assert sample is not None and sample.speed == pytest.approx(10.0)


def test_drop_to_zero_is_not_an_outlier():
    """The mirror direction: getting stuck must reach the detector too, not be filtered out."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, KMH_36))
    sample = pre.accept(wheel(FRONT_TOPIC, 0.1, 0.0))
    assert sample is not None and sample.speed == 0.0


def test_first_wheel_sample_never_outlier_rejected():
    """No prior sample for the bogie: any non-negative finite speed is accepted."""
    assert Preprocessor(PARAMS).accept(wheel(FRONT_TOPIC, 1.0, 190.0)) is not None


def test_long_silent_bogie_resume_not_flagged_as_outlier():
    """Trap 7: a bogie silent up to 73 s; the large dt keeps the implied accel small."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(REAR_TOPIC, 0.0, BASE_MPS * 3.6))              # non-zero: not the trap-8 exemption
    # the controller keeps the input clock running while the bogie is silent (every bag)
    for k in range(1, 1461):
        pre.accept(cmd(0.05 * k, 0))
    sample = pre.accept(wheel(REAR_TOPIC, 73.0, KMH_36))            # 10 m/s over 73 s
    assert sample is not None and sample.speed == pytest.approx(10.0)


def test_gnss_fix_within_window_accepted_status_not_filtered():
    """Preprocess only enforces the window (D-005); status filtering is pipeline's job."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, KMH_36))                     # sets t0
    sample = pre.accept(gnss_fix(1.0, status=-1))
    assert isinstance(sample, GnssFix) and sample.status == -1 and sample.antenna == 'master'


def test_gnss_fix_after_window_dropped():
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, KMH_36))
    t = PARAMS.gnss.init_window_s + 0.1
    assert pre.accept(gnss_fix(t)) is None


def test_gnss_vel_after_window_dropped():
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, KMH_36))
    t = PARAMS.gnss.init_window_s + 0.1
    assert pre.accept(gnss_vel(t, 1.0, 0.0)) is None


def test_gnss_vel_within_window_accepted():
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, KMH_36))
    sample = pre.accept(gnss_vel(1.0, 3.0, 4.0))
    assert isinstance(sample, GnssVel)
    assert sample.ve == pytest.approx(3.0) and sample.vn == pytest.approx(4.0)


@pytest.mark.parametrize('bad', [float('nan'), float('inf')])
def test_gnss_vel_non_finite_dropped(bad):
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, KMH_36))
    assert pre.accept(gnss_vel(1.0, bad, 0.0)) is None


def test_unknown_topic_dropped():
    assert Preprocessor(PARAMS).accept(('/unknown', SimpleNamespace(header=_hdr(0.0)))) is None


def test_t0_is_stamp_of_first_input_regardless_of_topic():
    pre = Preprocessor(PARAMS)
    assert pre.t0 is None
    pre.accept(cmd(3.5, 0))
    assert pre.t0 == pytest.approx(3.5)


# --- #77: a stamp far in the future must not freeze the stream -------------------------

JUMP = PARAMS.input.max_stamp_jump_s


def _feed(pp, events):
    return [pp.accept(e) for e in events]


def test_one_future_command_does_not_block_the_normal_stream():
    pp = Preprocessor(PARAMS)
    t0 = 1000.0
    _feed(pp, [cmd(t0 + 0.05 * k, 3) for k in range(20)])
    assert pp.accept(cmd(t0 + 86400.0, 3)) is None                # glitch: +1 day
    out = _feed(pp, [cmd(t0 + 1.0 + 0.05 * k, 3) for k in range(200)])
    assert all(isinstance(s, CommandSample) for s in out)          # nothing dropped after it
    out = _feed(pp, [wheel(FRONT_TOPIC, t0 + 1.0 + 0.1 * k, 36.0) for k in range(20)])
    assert all(isinstance(s, WheelSample) for s in out)


def test_one_future_wheel_does_not_block_its_bogie():
    pp = Preprocessor(PARAMS)
    t0 = 1000.0
    _feed(pp, [wheel(REAR_TOPIC, t0 + 0.1 * k, 36.0) for k in range(10)])
    assert pp.accept(wheel(REAR_TOPIC, t0 + 5e6, 36.0)) is None
    assert isinstance(pp.accept(wheel(REAR_TOPIC, t0 + 1.0, 36.0)), WheelSample)


def test_a_real_clock_jump_on_all_streams_is_accepted():
    """The whole bag clock jumps by more than the limit: the second stream confirms it."""
    pp = Preprocessor(PARAMS)
    t0, big = 1000.0, JUMP + 30.0
    _feed(pp, [cmd(t0, 0), wheel(FRONT_TOPIC, t0, 18.0), wheel(REAR_TOPIC, t0, 18.0)])
    assert pp.accept(cmd(t0 + big, 0)) is None                     # alone: not yet trusted
    assert isinstance(pp.accept(wheel(FRONT_TOPIC, t0 + big, 18.0)), WheelSample)   # confirmed
    assert isinstance(pp.accept(cmd(t0 + big + 0.05, 0)), CommandSample)
    assert isinstance(pp.accept(wheel(REAR_TOPIC, t0 + big + 0.1, 18.0)), WheelSample)


def test_jumps_within_the_limit_are_normal():
    """Data: the largest forward jump over all 122 bags is 2.6 s (a bogie after a pause)."""
    pp = Preprocessor(PARAMS)
    t0 = 1000.0
    _feed(pp, [cmd(t0, 0), wheel(FRONT_TOPIC, t0, 18.0)])
    assert isinstance(pp.accept(wheel(REAR_TOPIC, t0 + JUMP - 0.5, 18.0)), WheelSample)


def test_a_lone_stream_coming_back_after_a_long_silence_confirms_itself():
    """Only one stream talks, silent longer than the limit, then continues: the second
    sample moving on from the jumped stamp confirms it (one sample lost, not the stream)."""
    pp = Preprocessor(PARAMS)
    t0 = 1000.0
    _feed(pp, [cmd(t0, 0)])
    assert pp.accept(cmd(t0 + 86400.0, 0)) is None
    assert isinstance(pp.accept(cmd(t0 + 86400.05, 0)), CommandSample)


def test_a_lone_glitch_followed_by_normal_stamps_stays_rejected():
    pp = Preprocessor(PARAMS)
    t0 = 1000.0
    _feed(pp, [cmd(t0, 0)])
    assert pp.accept(cmd(t0 + 86400.0, 0)) is None
    out = _feed(pp, [cmd(t0 + 0.05 * k, 0) for k in range(1, 50)])
    assert all(isinstance(s, CommandSample) for s in out)


def test_nonpositive_stamp_jump_limit_is_rejected(tmp_path):
    text = (ROOT / 'src' / 'tram_odometry' / 'config' / 'params.yaml').read_text(encoding='utf-8')
    bad = tmp_path / 'params.yaml'
    bad.write_text(text.replace('max_stamp_jump_s: 10.0', 'max_stamp_jump_s: 0.0'), encoding='utf-8')
    with pytest.raises(ValueError):
        load_params(bad)


def test_two_unrelated_glitches_on_different_streams_stay_rejected():
    """A second glitch far from the first one does not confirm it: the clock stays."""
    pp = Preprocessor(PARAMS)
    t0 = 1000.0
    _feed(pp, [cmd(t0, 0), wheel(FRONT_TOPIC, t0, 18.0)])
    assert pp.accept(cmd(t0 + 86400.0, 0)) is None
    assert pp.accept(wheel(FRONT_TOPIC, t0 + 5e6, 18.0)) is None
    assert isinstance(pp.accept(wheel(FRONT_TOPIC, t0 + 0.1, 18.0)), WheelSample)
    assert isinstance(pp.accept(cmd(t0 + 0.05, 0)), CommandSample)


def test_a_glitch_does_not_move_the_clock_for_later_real_samples():
    """After a rejected glitch, a normal sample 3 s ahead (a bogie after a pause) passes."""
    pp = Preprocessor(PARAMS)
    t0 = 1000.0
    _feed(pp, [cmd(t0, 0), wheel(REAR_TOPIC, t0, 18.0)])
    assert pp.accept(cmd(t0 + 86400.0, 0)) is None
    assert isinstance(pp.accept(wheel(REAR_TOPIC, t0 + 3.0, 18.0)), WheelSample)


def test_an_earlier_future_stamp_of_the_same_stream_does_not_confirm():
    """Same stream, second future stamp behind the first one: not a clock moving on."""
    pp = Preprocessor(PARAMS)
    t0 = 1000.0
    _feed(pp, [cmd(t0, 0)])
    assert pp.accept(cmd(t0 + 86400.0, 0)) is None
    assert pp.accept(cmd(t0 + 86395.0, 0)) is None


def test_a_lagging_stream_does_not_pull_the_clock_back():
    """A stream far behind (accepted: it is not ahead) must not make the leading stream's
    next sample look like a jump."""
    pp = Preprocessor(PARAMS)
    t0 = 1000.0
    _feed(pp, [cmd(t0 + 100.0, 0)])
    assert isinstance(pp.accept(wheel(FRONT_TOPIC, t0 + 100.0 - 2 * JUMP, 18.0)), WheelSample)
    assert isinstance(pp.accept(cmd(t0 + 100.05, 0)), CommandSample)


def test_odometry_keeps_publishing_after_a_future_command():
    """#77 acceptance: through Odometry.step, after a +1 day command every normal input of
    the next 10 s still produces an Estimate (the node publishes /result/* for each)."""
    from tram_odometry_core.pipeline import Odometry
    odo = Odometry(PARAMS)
    t0 = 1000.0
    for k in range(20):
        odo.step(cmd(t0 + 0.05 * k, 3))
        odo.step(wheel(FRONT_TOPIC, t0 + 0.05 * k, 36.0))
    assert odo.step(cmd(t0 + 86400.0, 3)) is None
    outs = []
    for k in range(200):                                           # 10 s at 20 Hz
        t = t0 + 1.0 + 0.05 * k
        outs.append(odo.step(cmd(t, 3)))
        outs.append(odo.step(wheel(FRONT_TOPIC, t, 36.0)))
    assert all(e is not None for e in outs)
    assert outs[-1].t == pytest.approx(t0 + 1.0 + 0.05 * 199)
    assert outs[-1].speed == pytest.approx(10.0, abs=1e-6)


def test_two_future_glitches_in_a_row_do_not_freeze_the_stream():
    """A 100 ms clock glitch at 20 Hz: two future stamps in a row get accepted as a jump;
    the stream then resyncs back on the normal clock, losing one sample."""
    pp = Preprocessor(PARAMS)
    t0 = 1000.0
    _feed(pp, [cmd(t0 + 0.05 * k, 3) for k in range(20)])
    _feed(pp, [cmd(t0 + 86400.0, 3), cmd(t0 + 86400.05, 3)])
    out = _feed(pp, [cmd(t0 + 1.0 + 0.05 * k, 3) for k in range(200)])
    assert out[0] is None
    assert all(isinstance(s, CommandSample) for s in out[1:])


def test_glitches_on_two_streams_do_not_freeze_the_second_one():
    pp = Preprocessor(PARAMS)
    t0 = 1000.0
    _feed(pp, [cmd(t0, 0), wheel(FRONT_TOPIC, t0, 18.0), wheel(REAR_TOPIC, t0, 18.0)])
    _feed(pp, [cmd(t0 + 86400.0, 0), wheel(FRONT_TOPIC, t0 + 86401.0, 18.0)])
    out = _feed(pp, [wheel(FRONT_TOPIC, t0 + 0.1 * k, 18.0) for k in range(1, 100)])
    assert sum(isinstance(s, WheelSample) for s in out) >= 97
    assert isinstance(out[-1], WheelSample)


def test_first_sample_of_the_bag_from_the_future_does_not_freeze_the_stream():
    pp = Preprocessor(PARAMS)
    t0 = 1000.0
    assert pp.accept(cmd(t0 + 86400.0, 0)) is not None          # nothing to compare yet
    out = _feed(pp, [cmd(t0 + 0.05 * k, 0) for k in range(100)])
    assert sum(isinstance(s, CommandSample) for s in out) >= 98
    assert isinstance(out[-1], CommandSample)


def test_ordinary_rollbacks_are_still_dropped_after_a_resync_is_possible():
    """Trap 6: small rollbacks (well within the limit) are never resynced to."""
    pp = Preprocessor(PARAMS)
    t0 = 1000.0
    _feed(pp, [cmd(t0 + 0.05 * k, 0) for k in range(100)])
    assert pp.accept(cmd(t0 + 2.0, 0)) is None
    assert pp.accept(cmd(t0 + 2.05, 0)) is None
    assert isinstance(pp.accept(cmd(t0 + 5.0, 0)), CommandSample)


def test_a_stale_pending_jump_does_not_confirm_a_later_unrelated_glitch():
    pp = Preprocessor(PARAMS)
    t0 = 1000.0
    _feed(pp, [cmd(t0, 0), wheel(REAR_TOPIC, t0, 18.0)])
    assert pp.accept(cmd(t0 + 15.0, 0)) is None                    # glitch, pending
    for k in range(1, 61):                                          # 6 s of normal flow
        pp.accept(cmd(t0 + 0.1 * k, 0))
        pp.accept(wheel(REAR_TOPIC, t0 + 0.1 * k, 18.0))
    assert pp.accept(wheel(REAR_TOPIC, t0 + 6.0 + 11.0, 18.0)) is None   # unrelated glitch
    assert isinstance(pp.accept(wheel(REAR_TOPIC, t0 + 6.1, 18.0)), WheelSample)


def test_resync_needs_the_stream_to_move_on_not_back():
    pp = Preprocessor(PARAMS)
    t0 = 1000.0
    _feed(pp, [cmd(t0 + 86400.0, 0)])                            # gate in the future
    assert pp.accept(cmd(t0 + 1.0, 0)) is None                    # far behind: pending
    assert pp.accept(cmd(t0 + 0.5, 0)) is None                    # behind the pending one
    assert isinstance(pp.accept(cmd(t0 + 0.55, 0)), CommandSample)


def test_after_a_resync_the_clock_is_back_and_a_new_glitch_is_rejected():
    pp = Preprocessor(PARAMS)
    t0 = 1000.0
    _feed(pp, [cmd(t0 + 86400.0, 0), cmd(t0, 0), cmd(t0 + 0.05, 0)])    # resynced
    assert pp.accept(cmd(t0 + 0.1, 0)) is not None
    assert pp.accept(cmd(t0 + 20.0, 0)) is None                   # +20 s over the real clock
    assert isinstance(pp.accept(cmd(t0 + 0.15, 0)), CommandSample)

