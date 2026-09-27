import pytest

import io

from tram_stand import analyze as A
from tram_stand.cli import ensure_utf8_stdout, measure, table

MS = 10**6


def test_latency_matched_by_stamp_fifo():
    inputs = [(0, 100), (10 * MS, 200), (20 * MS, 200)]
    outputs = [(5 * MS, 100), (37 * MS, 200), (60 * MS, 200)]
    lat, unmatched = A.match_latency(inputs, outputs)
    assert list(lat) == pytest.approx([5.0, 27.0, 40.0]) and unmatched == 0


def test_latency_unmatched_output_counted():
    lat, unmatched = A.match_latency([(0, 1)], [(MS, 1), (2 * MS, 1), (3 * MS, 9)])
    assert list(lat) == [1.0] and unmatched == 2


def test_latency_summary_percentiles():
    s = A.latency_summary([float(i) for i in range(1, 101)])
    assert s['p50_ms'] == pytest.approx(50.5) and s['max_ms'] == 100.0
    assert s['p99_ms'] == pytest.approx(99.01)
    assert A.latency_summary([])['p99_ms'] is None


def test_rate_mean_and_slowest_window():
    t = [int(i * 0.05 * 1e9) for i in range(200)]          # 20 Hz for ~10 s
    r = A.rate_summary(t)
    assert r['mean_hz'] == pytest.approx(20.0, rel=0.01) and r['min_window_hz'] == 20
    gap = [x for x in t if not 3e9 <= x < 5e9]             # 2 s silence
    assert A.rate_summary(gap)['min_window_hz'] == 0


def _rows(n, ticks_per_s, rss0_kb, rss_growth_kb_s, pids=(1,)):
    text = ['t,pid,ticks,rss_kb']
    for i in range(n):
        for p in pids:
            text.append(f'{1000 + i}.0,{p},{i * ticks_per_s},{int(rss0_kb + i * rss_growth_kb_s)}')
    return A.parse_samples('\n'.join(text))


def test_cpu_cores_from_ticks_summed_over_processes():
    res = A.resource_summary(_rows(20, 150, 50_000, 0, pids=(1, 2)), clk_tck=100)
    assert res['cpu_mean_cores'] == pytest.approx(3.0)      # 1.5 core per process, two of them
    assert res['rss_peak_mb'] == pytest.approx(2 * 50_000 / 1024)


def test_rss_growth_detects_leak_and_flat_is_ok():
    leak = A.resource_summary(_rows(600, 10, 50_000, 1024 / 6, ), clk_tck=100)   # +10 MB/min
    assert leak['rss_growth_mb_min'] == pytest.approx(10.0, rel=0.05)
    flat = A.resource_summary(_rows(600, 10, 50_000, 0), clk_tck=100)
    assert flat['rss_growth_mb_min'] == pytest.approx(0.0, abs=1e-6)


def test_new_process_not_counted_from_zero():
    rows = A.parse_samples('t,pid,ticks,rss_kb\n1.0,1,0,1\n2.0,1,100,1\n3.0,1,200,1\n'
                           '3.0,2,999999,1\n4.0,1,300,1\n4.0,2,1000099,1\n')
    res = A.resource_summary(rows, clk_tck=100)
    assert res['cpu_max_cores'] == pytest.approx(2.0)       # pid 2 appeared with a large counter


def test_verdict_thresholds():
    lat = {'p99_ms': 99.0, 'max_ms': 251.0}
    v = A.verdict(lat, {'mean_hz': 9.0}, {'cpu_max_cores': 2.0, 'rss_peak_mb': 513.0,
                                          'rss_growth_mb_min': 0.5})
    assert v == {'latency_p99': True, 'latency_peak': False, 'rate': False, 'cpu': True,
                 'rss': False, 'rss_growth': True}
    empty = A.verdict({'p99_ms': None, 'max_ms': None}, {'mean_hz': None},
                      {'cpu_max_cores': None, 'rss_peak_mb': None, 'rss_growth_mb_min': None})
    assert set(empty.values()) == {None}


def test_measure_and_table_end_to_end():
    msgs = []
    for i in range(400):                                    # 10 Hz front, output 20 ms later
        stamp, recv = i * 100 * MS, i * 100 * MS
        msgs += [('/vehicle/front_bogie_velocity', recv, stamp),
                 ('/result/velocity', recv + 20 * MS, stamp),
                 ('/result/position', recv + 30 * MS, stamp)]
    res = measure(msgs, _rows(60, 50, 40_000, 0), 100)
    assert res['latency']['p99_ms'] == pytest.approx(30.0)
    assert res['topics']['/result/velocity']['unmatched'] == 0
    assert res['verdict']['latency_p99'] and res['verdict']['rate'] and res['verdict']['cpu']
    assert '| задержка p99, мс | 30.0 | ≤ 100 | ✅ |' in table(res)


def test_missing_output_topic_fails_rate():
    msgs = [('/vehicle/front_bogie_velocity', i * 100 * MS, i * 100 * MS) for i in range(50)]
    msgs += [('/result/velocity', i * 100 * MS + MS, i * 100 * MS) for i in range(50)]
    res = measure(msgs, [], 100)               # /result/position never published
    assert res['rate']['mean_hz'] == 0.0 and res['verdict']['rate'] is False


def test_ensure_utf8_stdout_survives_a_narrow_console_encoding(monkeypatch):
    """A '✅'/'❌' in table() must not crash the stand report on a console whose default
    codepage is not UTF-8 (Windows, cp1251) -- same class of bug as #124 (#126)."""
    import tram_stand.cli as cli
    narrow = io.TextIOWrapper(io.BytesIO(), encoding='ascii')
    monkeypatch.setattr(cli.sys, 'stdout', narrow)
    ensure_utf8_stdout()
    print('✅ ❌')  # raises UnicodeEncodeError on the original ascii-encoded stream
    narrow.flush()
    assert narrow.buffer.getvalue()
