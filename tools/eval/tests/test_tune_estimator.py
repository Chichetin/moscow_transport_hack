"""tune_estimator (#18): overrides in memory, the score, the D-012 guard, train only."""
import pytest

import tune_estimator as tune
from tram_eval import bag


def params():
    from tram_odometry_core.types import load_params
    return load_params(str(bag.PARAMS_YAML))


def test_override_changes_only_the_named_fields():
    base = params()
    new = tune.override(base, {'filter.q_accel': 1.25, 'slip.noise_hold_s': 0.5})
    assert new.filter.q_accel == 1.25
    assert new.slip.noise_hold_s == 0.5
    assert new.filter.r_wheel == base.filter.r_wheel
    assert new.slip.noise_accel_mps2 == base.slip.noise_accel_mps2
    assert new.drive == base.drive and new.input == base.input
    assert base.filter.q_accel != 1.25   # the loaded params are not mutated


@pytest.mark.parametrize('key', ['filter.no_such_key', 'input.max_wheel_accel_mps2',
                                 'gnss.init_window_s', 'q_accel'])
def test_override_rejects_keys_outside_the_tuned_set(key):
    with pytest.raises(KeyError):
        tune.override(params(), {key: 1.0})


def test_override_rejects_unsafe_filter_values():
    with pytest.raises(ValueError):
        tune.override(params(), {'filter.r_wheel': 0.0})


def summary(speed, along, drift):
    return {'speed_rmse': speed, 'along_rmse': along, 'drift_pct': drift}


def test_score_is_mean_ratio_of_main_metrics():
    base = summary(0.04, 2.0, 0.02)
    assert tune.score(base, base) == pytest.approx(1.0)
    assert tune.score(summary(0.02, 2.0, 0.02), base) == pytest.approx((0.5 + 1 + 1) / 3)
    assert tune.score(summary(None, 2.0, 0.02), base) == float('inf')


def test_d012_guard_rejects_a_main_metric_worse_by_more_than_2_percent():
    base = summary(0.04, 2.0, 0.02)
    assert tune.d012_ok(summary(0.0408, 2.0, 0.02), base)          # +2 %: allowed
    assert not tune.d012_ok(summary(0.02, 2.05, 0.02), base)       # along +2.5 %
    assert not tune.d012_ok(summary(0.02, 2.0, None), base)        # lost metric is worse


def test_candidate_must_beat_the_best_by_the_margin_and_pass_d012():
    start = summary(0.04, 2.0, 0.02)
    best = summary(0.04, 2.0, 0.02)
    better = summary(0.036, 2.0, 0.02)                  # score 0.967
    tiny = summary(0.0399, 2.0, 0.02)                   # score 0.99917, below the margin
    trade = summary(0.02, 2.1, 0.02)                    # better score, along +5 %
    assert tune.better(better, best, start, margin=0.005)
    assert not tune.better(tiny, best, start, margin=0.005)
    assert not tune.better(trade, best, start, margin=0.005)


@pytest.mark.parametrize('split', ['holdout', 'quick'])
def test_holdout_bags_are_refused(split):
    with pytest.raises(SystemExit):
        tune.main(['--split', split])


def test_bags_outside_the_split_are_refused():
    with pytest.raises(SystemExit):
        tune.main(['--split', 'train', '--bag', '30618_27e994fc'])   # a holdout bag


def test_random_samples_are_reproducible_new_and_on_the_grid():
    start = {k: float(v[0]) for k, v in tune.GRID.items()}
    a = tune.sample(start, 20, seed=18)
    assert a == tune.sample(start, 20, seed=18)
    assert a != tune.sample(start, 20, seed=19)
    keys = [tune.key_of(c) for c in a]
    assert len(set(keys)) == 20 and tune.key_of(start) not in keys
    assert all(c[k] in tune.GRID[k] for c in a for k in tune.GRID)
