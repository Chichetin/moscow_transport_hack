"""Deterministic input disturbances for evaluator diagnostics, separate from metrics.json."""
from __future__ import annotations

import copy
import math

import numpy as np

from .bag import FRONT, REAR, gnss_window_end, stamp

SCENARIOS = ('outlier', 'gap_1', 'gap_10', 'gap_70', 'spike', 'noise', 'jitter', 'rollback',
             'freeze')
DURATION_S = {'outlier': 0.2, 'gap_1': 1.0, 'gap_10': 10.0, 'gap_70': 70.0,
              'spike': 5.0, 'noise': 10.0, 'jitter': 10.0, 'rollback': 10.0, 'freeze': 10.0}
MAX_RECOVERY_GAP_S = 0.3


def _set_stamp(msg, t):
    sec = math.floor(t)
    nsec = round((t - sec) * 1e9)
    if nsec == 1_000_000_000:
        sec, nsec = sec + 1, 0
    msg.header.stamp.sec, msg.header.stamp.nanosec = sec, nsec


def perturb(msgs, scenario: str, gnss_window_s: float):
    """Return (new messages, anomaly start, end), or None if no recovery tail fits.

    Only wheel messages are changed. GNSS objects and their order remain identical, so the
    reference built from the original recording stays valid. All amplitudes are fixed.
    """
    if scenario not in DURATION_S:
        raise ValueError(f'unknown stress scenario: {scenario}')
    wheel_t = [stamp(m) for topic, m in msgs if topic in (FRONT, REAR)]
    if not wheel_t:
        return None
    first, last = min(wheel_t), max(wheel_t)
    duration = DURATION_S[scenario]
    # after the same GNSS window run_pipeline applies, and a 3 s recovery tail before the end
    begin = max(first, gnss_window_end(msgs, gnss_window_s)) + 1.0
    if last - begin < duration + 3.0:
        return None
    start = max(begin, min(first + (last - first) * 0.4, last - duration - 3.0))
    end = start + duration
    result = []
    front_n = rear_n = 0
    outlier_done = False
    held = {}           # freeze: bogie -> (stamp, reading) newest by stamp before the event (#144)
    changed_count = removed_count = 0
    for topic, msg in msgs:
        t = stamp(msg)
        inside = start <= t < end
        if topic in (FRONT, REAR) and t < start and t > held.get(topic, (-math.inf, None))[0]:
            held[topic] = (t, msg.velocity)
        if topic == FRONT and inside and scenario.startswith('gap_'):
            removed_count += 1
            continue
        if not inside or topic not in (FRONT, REAR):
            result.append((topic, msg))
            continue
        if topic == REAR and scenario not in ('noise', 'freeze'):
            result.append((topic, msg))
            continue
        new = copy.deepcopy(msg)
        if scenario == 'freeze':
            # both sensors repeat their last reading: they still agree, only the model can tell
            new.velocity = held.get(topic, (None, new.velocity))[1]
        elif topic == FRONT:
            front_n += 1
            if scenario == 'outlier' and not outlier_done:
                new.velocity = 180.0
                outlier_done = True
            elif scenario == 'spike':
                new.velocity += 25.0
            elif scenario == 'noise':
                new.velocity += 3.0 * math.sin(2 * math.pi * front_n / 10)
            elif scenario == 'jitter':
                _set_stamp(new, t + (0.04 if front_n % 2 else -0.04))
            elif scenario == 'rollback' and front_n % 10 == 0:
                _set_stamp(new, t - 0.35)
        else:
            rear_n += 1
            new.velocity += 3.0 * math.sin(2 * math.pi * rear_n / 10 + math.pi)
        if stamp(new) != t or new.velocity != msg.velocity:
            changed_count += 1
        result.append((topic, new))
    if changed_count + removed_count == 0:
        return None
    return result, start, end


def recovery_seconds(times: np.ndarray, excess: np.ndarray, event_end: float,
                     threshold: float, sustain_s: float):
    """First post-event time with a continuous sustain window below excess threshold."""
    for i in np.flatnonzero(times >= event_end):
        last = int(np.searchsorted(times, times[i] + sustain_s, side='left'))
        if last < len(times) and np.all(np.isfinite(excess[i:last + 1])) \
                and np.all(excess[i:last + 1] <= threshold) \
                and np.all(np.diff(times[i:last + 1]) <= MAX_RECOVERY_GAP_S):
            return float(times[i] - event_end)
    return None


def _errors(ref_t, ref_value, clean, dirty, value_name, event_start, event_end, threshold):
    """Align clean/dirty estimates, then compare both with unchanged GNSS."""
    from .metrics import match_nearest

    if not len(ref_t):
        return None, None, None, None, 0, 0
    # build_reference sorts and deduplicates both GNSS time axes before interpolation.
    di, ce = match_nearest(dirty.t, clean.t)
    keep = (dirty.t[di] >= ref_t[0]) & (dirty.t[di] <= ref_t[-1])
    di, ce = di[keep], ce[keep]
    if not len(di):
        return None, None, None, None, 0, 0
    times = dirty.t[di]
    old = getattr(clean, value_name)[ce]
    new = getattr(dirty, value_name)[di]
    order = np.argsort(times, kind='stable')
    times, old, new = times[order], old[order], new[order]
    if value_name == 'speed':
        truth = np.interp(times, ref_t, ref_value)
        old_error, new_error = np.abs(old - truth), np.abs(new - truth)
    else:
        truth = np.column_stack([np.interp(times, ref_t, ref_value[:, k]) for k in range(3)])
        old_error = np.linalg.norm(old - truth, axis=1)
        new_error = np.linalg.norm(new - truth, axis=1)
    excess = new_error - old_error
    during = (times >= event_start) & (times < event_end)
    peak = float(np.max(new_error[during])) if during.any() else None
    clean_peak = float(np.max(old_error[during])) if during.any() else None
    excess_peak = float(max(0.0, np.max(excess[during]))) if during.any() else None
    # Keep the worst publication at each timestamp: selecting the first duplicate
    # can hide an error spike and falsely satisfy the recovery sustain window.
    group_starts = np.r_[0, np.flatnonzero(np.diff(times) > 0) + 1]
    unique_times = times[group_starts]
    worst_excess = np.maximum.reduceat(excess, group_starts)
    recovery = recovery_seconds(unique_times, worst_excess, event_end, threshold, 2.0)
    return round(peak, 4) if peak is not None else None, \
        round(clean_peak, 4) if clean_peak is not None else None, \
        round(excess_peak, 4) if excess_peak is not None else None, \
        round(recovery, 4) if recovery is not None else None, len(times), int(during.sum())


def evaluate_stress_bag(path, gnss_window_s: float, make_odometry=None, msgs=None) -> dict:
    """Per-scenario diagnostics; never writes into contractual metrics.json."""
    from . import bag
    from .reference import build_reference

    make_odometry = make_odometry or bag.default_odometry
    msgs = bag.read_bag(path) if msgs is None else msgs
    window_end = bag.gnss_window_end(msgs, gnss_window_s)
    ref = build_reference(*bag.reference_inputs(msgs), window_end)
    clean, clean_crash, _ = bag.run_pipeline(msgs, make_odometry(), window_end)
    clean, _ = bag.finite_only(clean)
    result = {}
    for scenario in SCENARIOS:
        event = perturb(msgs, scenario, gnss_window_s)
        if event is None:
            result[scenario] = {
                'skipped': True,
                'reason': 'bag too short for event/recovery tail or no target wheel sample changed',
            }
            continue
        changed, start, end = event
        dirty, crash, mismatch = bag.run_pipeline(changed, make_odometry(), window_end)
        dirty, nonfinite = bag.finite_only(dirty)
        speed_peak, clean_speed_peak, speed_excess, speed_recovery, n_speed, n_speed_during = _errors(
            ref.vel_t, ref.speed, clean, dirty, 'speed', start, end, 0.2)
        pos_peak, clean_pos_peak, pos_excess, pos_recovery, n_pos, n_pos_during = _errors(
            ref.pos_t, ref.pos, clean, dirty, 'pos', start, end, 2.0)
        result[scenario] = {
            'skipped': False, 'event_start_s': round(start, 4), 'event_end_s': round(end, 4),
            'peak_speed_error_mps': speed_peak, 'clean_peak_speed_error_mps': clean_speed_peak,
            'peak_speed_excess_mps': speed_excess,
            'peak_pos3d_error_m': pos_peak, 'clean_peak_pos3d_error_m': clean_pos_peak,
            'peak_pos3d_excess_m': pos_excess,
            'speed_recovery_s': speed_recovery if clean_crash is None and crash is None else None,
            'pos_recovery_s': pos_recovery if clean_crash is None and crash is None else None,
            'n_speed': n_speed, 'n_pos': n_pos,
            'n_speed_during': n_speed_during, 'n_pos_during': n_pos_during,
            'crashed': bool(clean_crash or crash), 'nonfinite': nonfinite,
            'stamp_mismatch': mismatch,
        }
    return result
