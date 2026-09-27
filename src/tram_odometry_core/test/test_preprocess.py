import dataclasses
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from tram_odometry_core.preprocess import (CMD_TOPIC, FRONT_TOPIC, REAR_TOPIC, ROVER_FIX_TOPIC,
                                          Preprocessor)
from tram_odometry_core.types import CommandSample, GnssFix, GnssVel, WheelSample, load_params

ROOT = Path(__file__).resolve().parents[3]
PARAMS = load_params(ROOT / 'src' / 'tram_odometry' / 'config' / 'params.yaml')
MASTER_FIX = PARAMS.gnss.topic_fix
MASTER_VEL = PARAMS.gnss.topic_vel
KMH_36 = 36.0  # = 10 m/s
ACCEL_LIMIT = PARAMS.input.max_wheel_accel_mps2
# km/h: an exact repeat while the drive model changes the speed by more than slip.freeze_dv_mps
# is a frozen sensor (#144), so steady speeds through Odometry are dithered by this
DITHER_KMH = 1e-9


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


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf'), 'text', True])
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
    pre.accept(wheel(FRONT_TOPIC, 0.0, 80.0))
    assert pre.accept(wheel(FRONT_TOPIC, 1.0, 20.0)) is None    # not exactly 0: gate applies


def test_stuck_at_zero_recovery_is_not_an_outlier():
    """Trap 8: a bogie stuck exactly at 0 can jump straight back to the real speed -- the
    other bogie reads that speed, so it is recovery, not a glitch (#73)."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, 0.0))
    pre.accept(wheel(REAR_TOPIC, 0.05, KMH_36))
    sample = pre.accept(wheel(FRONT_TOPIC, 0.1, KMH_36))    # 10 m/s in 0.1 s: unstuck, not a glitch
    assert sample is not None and sample.speed == pytest.approx(10.0)


def test_drop_to_zero_is_not_an_outlier():
    """The mirror direction: getting stuck must reach the detector too, not be filtered out."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, KMH_36))
    sample = pre.accept(wheel(FRONT_TOPIC, 0.1, 0.0))
    assert sample is not None and sample.speed == 0.0


@pytest.mark.parametrize('kmh', [-0.16, -0.39, -3.0, -5.0])
def test_negative_wheel_at_standstill_reads_zero(kmh):
    """#111: a negative reading at standstill -- real slow roll-back (-0.16..-0.39 km/h, both
    bogies) or noise around 0 -- is 0 m/s, not dropped: dropping keeps only the positive half
    of zero-mean noise and biases the speed up."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, 0.5))
    sample = pre.accept(wheel(FRONT_TOPIC, 0.1, kmh))
    assert isinstance(sample, WheelSample) and sample.speed == 0.0


def test_negative_wheel_as_first_sample_reads_zero():
    sample = Preprocessor(PARAMS).accept(wheel(REAR_TOPIC, 1.0, -0.3))
    assert isinstance(sample, WheelSample) and sample.speed == 0.0


def test_negative_wheel_in_motion_is_gated_not_stuck_at_zero():
    """#111: a negative reading is clamped to 0 but is not a bogie stuck at exactly 0 (trap 8):
    in motion the acceleration gate still drops it as a glitch."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, KMH_36))
    assert pre.accept(wheel(FRONT_TOPIC, 0.1, -5.0)) is None
    sample = pre.accept(wheel(FRONT_TOPIC, 0.2, KMH_36))
    assert sample is not None and sample.speed == pytest.approx(10.0)


def test_first_wheel_sample_is_only_bounded_by_the_speed_limit():
    """No prior sample for the bogie: any speed up to input.max_wheel_speed_mps is accepted,
    above it is a glitch (#73: 1e9 km/h as the first sample reached /result/velocity)."""
    limit_kmh = PARAMS.input.max_wheel_speed_mps * 3.6
    assert Preprocessor(PARAMS).accept(wheel(FRONT_TOPIC, 1.0, limit_kmh - 1.0)) is not None
    assert Preprocessor(PARAMS).accept(wheel(FRONT_TOPIC, 1.0, limit_kmh + 1.0)) is None
    assert Preprocessor(PARAMS).accept(wheel(FRONT_TOPIC, 1.0, 1e9)) is None


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


def test_rover_fix_is_a_rover_sample_in_the_window_only():
    """The rover gives the heading at a standstill (docs/data.md, trap 15), window only (D-005)."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, KMH_36))
    topic, msg = gnss_fix(1.0, status=2)
    sample = pre.accept((ROVER_FIX_TOPIC, msg))
    assert isinstance(sample, GnssFix) and sample.antenna == 'rover' and sample.status == 2
    topic, msg = gnss_fix(PARAMS.gnss.init_window_s + 0.1)
    assert pre.accept((ROVER_FIX_TOPIC, msg)) is None


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
        odo.step(wheel(FRONT_TOPIC, t0 + 0.05 * k, 36.0 + DITHER_KMH * (k % 2)))
    assert odo.step(cmd(t0 + 86400.0, 3)) is None
    outs = []
    for k in range(200):                                           # 10 s at 20 Hz
        t = t0 + 1.0 + 0.05 * k
        outs.append(odo.step(cmd(t, 3)))
        outs.append(odo.step(wheel(FRONT_TOPIC, t, 36.0 + DITHER_KMH * (k % 2))))
    assert all(e is not None for e in outs)
    assert outs[-1].t == pytest.approx(t0 + 1.0 + 0.05 * 199)
    assert outs[-1].speed == pytest.approx(10.0, abs=0.001)


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


def _run_odometry(glitches):
    """10 m/s: 1 s of normal flow, the glitch inputs, then 10 s of normal flow (cmd + front
    at 20 Hz). Returns the distance before the glitch and the last Estimate."""
    from tram_odometry_core.pipeline import Odometry
    odo = Odometry(PARAMS)
    t0 = 1000.0
    start = [cmd(t0 + 86400.0, 3)] if glitches == 'first' else []
    for e in start:
        odo.step(e)
    before = None
    for k in range(20):
        odo.step(cmd(t0 + 0.05 * k, 3))
        before = odo.step(wheel(FRONT_TOPIC, t0 + 0.05 * k, 36.0 + DITHER_KMH * (k % 2))) or before
    for e in {'first': [], 'two_in_a_row': [cmd(t0 + 86400.0, 3), cmd(t0 + 86400.05, 3)],
              'two_streams': [cmd(t0 + 86400.0, 3), wheel(FRONT_TOPIC, t0 + 86401.0, 36.0)]}[glitches]:
        odo.step(e)
    last = None
    for k in range(200):
        t = t0 + 1.0 + 0.05 * k
        odo.step(cmd(t, 3))
        last = odo.step(wheel(FRONT_TOPIC, t, 36.0 + DITHER_KMH * (k % 2))) or last
    return before, last


@pytest.mark.parametrize('glitches', ['first', 'two_in_a_row', 'two_streams'])
def test_odometry_distance_keeps_growing_after_an_accepted_clock_glitch(glitches):
    """Review #77: the pipeline state time must follow a resync, not freeze in the future or
    integrate the glitch (10 m/s over the last 10 s: about 100 m)."""
    before, last = _run_odometry(glitches)
    assert last is not None and last.t == pytest.approx(1000.0 + 1.0 + 0.05 * 199)
    grown = last.distance - (before.distance if before is not None else 0.0)
    assert 90.0 < grown < 120.0, grown
    assert last.speed == pytest.approx(10.0, abs=0.001)



# --- #80: the GNSS window start follows the clock back from a future first stamp -------------

WINDOW = PARAMS.gnss.init_window_s


@pytest.mark.parametrize('first', [
    cmd(1000.0 + 86400.0, 0),
    wheel(FRONT_TOPIC, 1000.0 + 86400.0, 0.0),
    gnss_fix(1000.0 + 86400.0, status=2),
])
def test_future_first_stamp_does_not_keep_the_gnss_window_open(first):
    """The first input of the bag comes from the future (+1 day), the stream then runs from
    the real start: GNSS after the real window is dropped (D-005), the window start is back."""
    pp = Preprocessor(PARAMS)
    start = 1000.0
    assert pp.accept(first) is None or pp.t0 == pytest.approx(start + 86400.0)
    _feed(pp, [cmd(start + 0.05 * k, 0) for k in range(4)]
          + [wheel(FRONT_TOPIC, start + 0.1 * k, 0.0) for k in range(4)])
    assert pp.t0 == pytest.approx(start)
    assert isinstance(pp.accept(gnss_fix(start + 1.0, status=2)), GnssFix)
    assert pp.accept(gnss_fix(start + WINDOW + 5.0, status=2)) is None
    assert pp.accept(gnss_vel(start + WINDOW + 5.0, 1.0, 0.0)) is None


def test_a_slightly_earlier_stamp_moves_the_window_start_at_once():
    """Data: 4 of 122 bags carry a stamp up to 51 ms older than the first message."""
    pp = Preprocessor(PARAMS)
    pp.accept(cmd(1000.05, 0))
    pp.accept(wheel(FRONT_TOPIC, 1000.0, 0.0))
    assert pp.t0 == pytest.approx(1000.0)


def test_one_stamp_from_the_past_does_not_close_the_gnss_window():
    """A single stamp a day behind is a glitch: the window stays open for the real start."""
    pp = Preprocessor(PARAMS)
    start = 1000.0
    _feed(pp, [cmd(start + 0.05 * k, 0) for k in range(4)])
    pp.accept(wheel(FRONT_TOPIC, start - 86400.0, 0.0))            # glitch: -1 day
    _feed(pp, [cmd(start + 0.2 + 0.05 * k, 0) for k in range(4)])
    assert pp.t0 == pytest.approx(start)
    assert isinstance(pp.accept(gnss_fix(start + 1.0, status=2)), GnssFix)


def test_a_confirmed_earlier_clock_moves_the_window_start():
    """Two inputs in a row far behind the window start agree: the clock really is there."""
    pp = Preprocessor(PARAMS)
    start = 1000.0
    pp.accept(cmd(start, 0))
    pp.accept(cmd(start - 3600.0, 0))
    assert pp.t0 == pytest.approx(start)                           # alone: not yet trusted
    pp.accept(wheel(FRONT_TOPIC, start - 3600.0 + 0.1, 0.0))
    assert pp.t0 == pytest.approx(start - 3600.0)


def test_a_small_step_back_does_not_take_a_pending_glitch_along():
    pp = Preprocessor(PARAMS)
    pp.accept(cmd(1000.0, 0))
    pp.accept(wheel(FRONT_TOPIC, 1000.0 - 86400.0, 0.0))           # glitch: -1 day, pending
    pp.accept(wheel(REAR_TOPIC, 999.95, 0.0))
    assert pp.t0 == pytest.approx(999.95)


def test_normal_input_between_two_past_glitches_cancels_the_first():
    """Two -1 day glitches 2 s apart with the normal stream between them: not a real clock."""
    pp = Preprocessor(PARAMS)
    start = 1000.0
    pp.accept(cmd(start, 0))
    pp.accept(wheel(FRONT_TOPIC, start - 86400.0, 0.0))
    _feed(pp, [cmd(start + 0.05 * k, 0) for k in range(1, 40)])
    pp.accept(wheel(FRONT_TOPIC, start + 2.0 - 86400.0, 0.0))
    assert pp.t0 == pytest.approx(start)


def test_a_gnss_epoch_from_the_past_does_not_close_the_window():
    """Review #100: fix and vel of one epoch share a stamp; they must not confirm each other."""
    pp = Preprocessor(PARAMS)
    start = 1000.0
    _feed(pp, [cmd(start, 0), wheel(FRONT_TOPIC, start, 0.0)])
    pp.accept(gnss_fix(start - 86400.0, status=2))
    pp.accept(gnss_vel(start - 86400.0, 1.0, 0.0))
    assert pp.t0 == pytest.approx(start)
    assert isinstance(pp.accept(gnss_fix(start + 1.0, status=2)), GnssFix)


def test_gnss_never_moves_the_window_start_back():
    pp = Preprocessor(PARAMS)
    start = 1000.0
    pp.accept(cmd(start, 0))
    pp.accept(gnss_fix(start - 9.0, status=2))
    assert pp.t0 == pytest.approx(start)


@pytest.mark.parametrize('make', [lambda t: gnss_fix(t, status=2), lambda t: gnss_vel(t, 1.0, 0.0)])
def test_gnss_far_behind_the_window_start_is_dropped(make):
    """A GNSS stamp a day in the past is not inside the window, whenever it arrives (D-005)."""
    pp = Preprocessor(PARAMS)
    start = 1000.0
    _feed(pp, [cmd(start, 0), wheel(FRONT_TOPIC, start, 0.0)])
    assert pp.accept(make(start - 86400.0)) is None
    assert pp.accept(make(start - JUMP - 0.5)) is None
    assert pp.accept(make(start - 0.05)) is not None               # buffered, trap 5


# --- #73: an outlier at standstill must not reach the estimate --------------------------

def test_outlier_at_standstill_is_dropped():
    """Both bogies at exactly 0, one reads 180 km/h for a sample: a glitch, not unsticking."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, 0.0))
    pre.accept(wheel(REAR_TOPIC, 0.0, 0.0))
    assert pre.accept(wheel(FRONT_TOPIC, 0.1, 180.0)) is None                 # above the limit
    assert pre.accept(wheel(FRONT_TOPIC, 0.2, 60.0)) is None                  # 16.7 m/s in 0.2 s
    assert isinstance(pre.accept(wheel(FRONT_TOPIC, 0.3, 0.0)), WheelSample)  # back to 0


def test_jump_from_zero_needs_the_other_bogie_fresh():
    """A stale other bogie cannot vouch for the jump."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(REAR_TOPIC, 0.0, KMH_36))
    pre.accept(wheel(FRONT_TOPIC, 0.0, 0.0))
    late = PARAMS.input.stale_timeout_s + 0.5
    pre.accept(wheel(FRONT_TOPIC, late - 0.1, 0.0))
    assert pre.accept(wheel(FRONT_TOPIC, late, KMH_36)) is None


def test_jump_from_zero_to_a_different_speed_than_the_other_bogie_is_dropped():
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, 0.0))
    pre.accept(wheel(REAR_TOPIC, 0.05, KMH_36))
    assert pre.accept(wheel(FRONT_TOPIC, 0.1, 2 * KMH_36)) is None


def test_start_of_motion_from_zero_within_physics_passes_without_the_other_bogie():
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, 0.0))
    assert isinstance(pre.accept(wheel(FRONT_TOPIC, 0.1, 0.5)), WheelSample)   # 0.14 m/s


def test_odometry_speed_stays_near_zero_on_a_standstill_outlier():
    """#73 end to end: the stress `outlier` (180 km/h once on one bogie at standstill) does
    not reach the published speed."""
    from tram_odometry_core.pipeline import Odometry
    odo = Odometry(PARAMS)
    worst = 0.0
    for k in range(50):
        t = 1000.0 + 0.1 * k
        for topic in (FRONT_TOPIC, REAR_TOPIC):
            kmh = 180.0 if (k == 25 and topic == FRONT_TOPIC) else 0.0
            e = odo.step(wheel(topic, t, kmh))
            if e is not None:
                worst = max(worst, e.speed)
    assert worst < 1.0


def _worst_speed(events):
    from tram_odometry_core.pipeline import Odometry
    odo = Odometry(PARAMS)
    return max((e.speed for e in map(odo.step, events) if e is not None), default=0.0)


def test_odometry_first_wheel_sample_above_the_limit_never_published():
    """#73 comment: 1e9 km/h as the very first sample went to /result/velocity as 2.8e8 m/s."""
    events = [wheel(FRONT_TOPIC, 1000.0, 1e9)]
    events += [wheel(b, 1000.0 + 0.1 * k, 0.0) for k in range(1, 20) for b in (FRONT_TOPIC, REAR_TOPIC)]
    assert _worst_speed(events) < 1.0


def test_odometry_standstill_outlier_with_the_other_bogie_silent():
    """The detector cannot outvote a lone bogie: the preprocess gate has to drop the jump."""
    events = [wheel(FRONT_TOPIC, 1000.0 + 0.1 * k, 180.0 if k == 25 else 0.0) for k in range(50)]
    assert _worst_speed(events) < 1.0


def test_a_rejected_jump_from_zero_comes_back_once_within_the_gate():
    """Review #101: a rejected sample does not move the gate's reference, so a bogie that
    really left 0 alone is accepted again after at most max_wheel_speed_mps / max_accel."""
    pre = Preprocessor(PARAMS)
    pre.accept(wheel(FRONT_TOPIC, 0.0, 0.0))
    accepted = [t for t in (0.1 * k for k in range(1, 60))
                if pre.accept(wheel(FRONT_TOPIC, t, KMH_36)) is not None]
    assert accepted, 'the bogie never came back'
    assert accepted[0] <= 10.0 / ACCEL_LIMIT + 0.1 + 1e-9
    assert accepted[0] <= PARAMS.input.max_wheel_speed_mps / ACCEL_LIMIT + 0.1
    assert accepted == [t for t in (0.1 * k for k in range(1, 60)) if t >= accepted[0] - 1e-9]
