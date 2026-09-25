"""Stand measurements from a recording: latency, rate, CPU, RSS. Pure functions, no ROS.

Thresholds are criterion 4 of task.md; RSS-growth limit is ours (D-023).
"""
from __future__ import annotations

from collections import defaultdict, deque

import numpy as np

LATENCY_LIMIT_MS = 100.0        # task.md, criterion 4
LATENCY_PEAK_LIMIT_MS = 250.0
RATE_LIMIT_HZ = 10.0
CORES_LIMIT = 2.0
RSS_LIMIT_MB = 512.0
RSS_GROWTH_LIMIT_MB_MIN = 1.0   # slope of total RSS after warm-up; a leak shows as steady growth
WARMUP_FRACTION = 1 / 3         # of the run, excluded from the RSS slope


def match_latency(inputs, outputs):
    """Latency in ms, output matched to input by header.stamp.

    inputs, outputs: iterables of (recv_ns, stamp_ns) in recording order. Repeated stamps
    are matched first-in first-out; an output without a free input counts as unmatched.
    Returns (latencies_ms ndarray, n_unmatched).
    """
    pending = defaultdict(deque)
    for recv, stamp in inputs:
        pending[stamp].append(recv)
    lat, unmatched = [], 0
    for recv, stamp in outputs:
        q = pending.get(stamp)
        if q:
            lat.append((recv - q.popleft()) / 1e6)
        else:
            unmatched += 1
    return np.asarray(lat, dtype=float), unmatched


def latency_summary(lat_ms) -> dict:
    if len(lat_ms) == 0:
        return {'n': 0, 'p50_ms': None, 'p99_ms': None, 'max_ms': None}
    return {'n': int(len(lat_ms)), 'p50_ms': float(np.percentile(lat_ms, 50)),
            'p99_ms': float(np.percentile(lat_ms, 99)), 'max_ms': float(np.max(lat_ms))}


def rate_summary(recv_ns) -> dict:
    """Mean rate over the run and the slowest full 1 s window (Hz)."""
    t = np.sort(np.asarray(recv_ns, dtype=float)) / 1e9
    if len(t) == 0:
        return {'n': 0, 'mean_hz': 0.0, 'min_window_hz': 0.0}     # topic never published
    if len(t) < 2 or t[-1] <= t[0]:
        return {'n': int(len(t)), 'mean_hz': None, 'min_window_hz': None}
    edges = np.arange(t[0], t[-1], 1.0)
    counts = np.histogram(t, bins=np.append(edges, t[-1]))[0][:-1] if len(edges) > 1 else []
    return {'n': int(len(t)), 'mean_hz': float((len(t) - 1) / (t[-1] - t[0])),
            'min_window_hz': float(np.min(counts)) if len(counts) else None}


def parse_samples(text: str):
    """Sampler CSV `t,pid,ticks,rss_kb` -> rows (float t, int pid, int ticks, int rss_kb)."""
    rows = []
    for line in text.splitlines()[1:]:
        p = line.split(',')
        if len(p) == 4:
            rows.append((float(p[0]), int(p[1]), int(p[2]), int(p[3])))
    return rows


def resource_summary(rows, clk_tck: int) -> dict:
    """CPU (cores, all node processes summed) and RSS (MB, summed) per sampling round."""
    rounds = defaultdict(list)
    for t, pid, ticks, rss in rows:
        rounds[t].append((pid, ticks, rss))
    ts = sorted(rounds)
    if len(ts) < 3:
        return {'n_samples': len(ts), 'cpu_mean_cores': None, 'cpu_max_cores': None,
                'rss_peak_mb': None, 'rss_growth_mb_min': None}
    cpu, rss = [], []
    last = {pid: ticks for pid, ticks, _ in rounds[ts[0]]}
    for t0, t1 in zip(ts, ts[1:]):
        used = 0
        for pid, ticks, _ in rounds[t1]:
            used += ticks - last.get(pid, ticks)
            last[pid] = ticks
        cpu.append(used / clk_tck / (t1 - t0))
    for t in ts:
        rss.append(sum(r for _, _, r in rounds[t]) / 1024.0)
    start = int(len(ts) * WARMUP_FRACTION)
    slope = np.polyfit(np.asarray(ts[start:]) / 60.0, rss[start:], 1)[0]
    return {'n_samples': len(ts), 'cpu_mean_cores': float(np.mean(cpu)),
            'cpu_max_cores': float(np.max(cpu)), 'rss_peak_mb': float(np.max(rss)),
            'rss_growth_mb_min': float(slope)}


def verdict(latency: dict, rates: dict, res: dict) -> dict:
    """Pass/fail per criterion-4 threshold; None means not measured."""
    def le(v, lim):
        return None if v is None else v <= lim
    return {
        'latency_p99': le(latency['p99_ms'], LATENCY_LIMIT_MS),
        'latency_peak': le(latency['max_ms'], LATENCY_PEAK_LIMIT_MS),
        'rate': None if rates['mean_hz'] is None else rates['mean_hz'] >= RATE_LIMIT_HZ,
        'cpu': le(res['cpu_max_cores'], CORES_LIMIT),
        'rss': le(res['rss_peak_mb'], RSS_LIMIT_MB),
        'rss_growth': le(res['rss_growth_mb_min'], RSS_GROWTH_LIMIT_MB_MIN),
    }
