"""Min/max wheel selector (#59): experiment against the speed filter, not wired in pipeline."""
import math
from pathlib import Path

import pytest

from tram_odometry_core.estimator.selector import WheelSelector
from tram_odometry_core.types import WheelSample, load_params

PARAMS = load_params(Path(__file__).resolve().parents[3] / 'src/tram_odometry/config/params.yaml')
STALE = PARAMS.input.stale_timeout_s


def step(sel, t, accel, front=None, rear=None, trust=(1.0, 1.0)):
    sel.predict(t, accel)
    if front is not None:
        sel.update(WheelSample(t, 'front', front), trust[0])
    if rear is not None:
        sel.update(WheelSample(t, 'rear', rear), trust[1])
    return sel.state()[0]


@pytest.mark.parametrize('accel, expected', [(0.5, 9.0), (-0.5, 11.0), (0.0, 10.0)])
def test_traction_takes_min_braking_max_cruise_mean(accel, expected):
    """A spinning bogie reads high under traction, a sliding one reads low under braking."""
    assert step(WheelSelector(PARAMS), 1.0, accel, 9.0, 11.0) == pytest.approx(expected)


def test_untrusted_bogie_is_not_selected():
    assert step(WheelSelector(PARAMS), 1.0, 0.5, 9.0, 11.0, trust=(0.0, 1.0)) == pytest.approx(11.0)


def test_silent_bogie_drops_out_after_stale_timeout():
    sel = WheelSelector(PARAMS)
    step(sel, 1.0, 0.5, 9.0, 11.0)
    t = 1.0 + STALE + 0.1
    assert step(sel, t, 0.5, rear=11.0) == pytest.approx(11.0)


def test_both_silent_extrapolates_with_the_drive_model():
    sel = WheelSelector(PARAMS)
    step(sel, 1.0, 0.0, 10.0, 10.0)
    sel.predict(1.0 + STALE, 0.0)
    assert sel.state()[0] == pytest.approx(10.0)
    sel.predict(1.0 + STALE + 2.0, -1.0)
    assert sel.state()[0] == pytest.approx(8.0)
    sel.predict(1.0 + STALE + 20.0, -1.0)
    assert sel.state()[0] == 0.0


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -1.0])
def test_non_finite_or_negative_input_is_ignored(bad):
    sel = WheelSelector(PARAMS)
    step(sel, 1.0, 0.0, 10.0, 10.0)
    sel.update(WheelSample(1.1, 'front', bad), 1.0)
    sel.update(WheelSample(bad, 'rear', 3.0), 1.0)
    sel.predict(bad, 0.0)
    v, var, accel = sel.state()
    assert v == pytest.approx(10.0) and math.isfinite(var) and math.isfinite(accel)


def test_repeated_or_past_stamp_of_a_bogie_is_ignored():
    sel = WheelSelector(PARAMS)
    step(sel, 1.0, 0.0, 10.0, 10.0)
    sel.update(WheelSample(1.0, 'front', 20.0), 1.0)
    sel.update(WheelSample(0.5, 'front', 20.0), 1.0)
    assert sel.state()[0] == pytest.approx(10.0)


def test_rebase_time_forgets_the_old_clock():
    sel = WheelSelector(PARAMS)
    step(sel, 1000.0, 0.0, 10.0, 10.0)
    sel.rebase_time(1.0)
    assert step(sel, 1.1, 0.0, 12.0, 12.0) == pytest.approx(12.0)


def test_accepted_update_reports_diagnostics():
    """pipeline takes the last measured speed for the slip detector from accepted updates."""
    sel = WheelSelector(PARAMS)
    sel.predict(1.0, 0.0)
    sel.update(WheelSample(1.0, 'front', 10.0), 1.0)
    d = sel.diagnostics()
    assert d is not None and d.accepted and d.bogie == 'front'


def test_live_bogies_are_not_moved_by_the_drive_model_between_samples():
    sel = WheelSelector(PARAMS)
    step(sel, 1.0, 0.0, 10.0, 10.0)
    sel.predict(1.0 + STALE / 2, 2.0)
    assert sel.state()[0] == pytest.approx(10.0)
