"""judge_runner_report must fail a scenario on every broken expectation, never mask it."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import judge_runner_report as rep  # noqa: E402

OK_EXIT = 'ready_exit\t0\nrecord_ready_exit\t0\nplay_exit\t0\nrecord_exit\t0\nnode_exit\t0\ncheck_recording_exit\t0\n'


def stream(frame, n=200, hz=40.0):
    pos = [(100 + i / hz, 1000 + i / hz, frame, 1.0, 2.0) for i in range(n)]
    vel = [(100 + i / hz, 1000 + i / hz) for i in range(n)]
    return pos, vel


def make(tmp_path, monkeypatch, frames, exit_tsv=OK_EXIT, alive='node running\nrecord running\n',
         errors=None):
    d = tmp_path / 's'
    (d / 'record').mkdir(parents=True)
    (d / 'exit.tsv').write_text(exit_tsv)
    (d / 'alive.txt').write_text(alive)
    (d / 'times.tsv').write_text('node_launch\t10.0\nready\t11.0\nplay\t11.1\n')
    if errors:
        (d / 'errors.txt').write_text(errors)
    pos, vel = [], []
    for frame, n in frames:
        p, v = stream(frame, n)
        pos += p
        vel += v
    monkeypatch.setattr(rep, 'read_result', lambda record: (pos, vel))
    return tmp_path


@pytest.mark.parametrize('expect,frames', [('map', [('map', 200)]), ('odom', [('odom', 200)])])
def test_expected_frames_pass(tmp_path, monkeypatch, expect, frames):
    run = make(tmp_path, monkeypatch, frames)
    r = rep.scenario(run, ['s', 'bag', 'protocol', '0', expect], {})
    assert r['status'] == 'ok', r['fails']
    assert r['launch_to_ready_s'] == 1.0


def test_odom_where_map_expected_fails(tmp_path, monkeypatch):
    run = make(tmp_path, monkeypatch, [('odom', 100), ('map', 100)])
    r = rep.scenario(run, ['s', 'bag', 'protocol', '0', 'map'], {})
    assert r['status'] == 'fail' and r['n_odom'] == 100


def test_old_anchor_in_bag_without_gnss_fails(tmp_path, monkeypatch):
    run = make(tmp_path, monkeypatch, [('map', 5), ('odom', 195)])
    r = rep.scenario(run, ['s', 'bag', 'protocol', '0', 'odom'], {})
    assert r['status'] == 'fail'


def test_node_dead_before_stop_fails(tmp_path, monkeypatch):
    run = make(tmp_path, monkeypatch, [('map', 200)], alive='node exited before stop\nrecord running\n',
               exit_tsv=OK_EXIT.replace('node_exit\t0', 'node_exit\t1'))
    r = rep.scenario(run, ['s', 'bag', 'protocol', '0', 'map'], {})
    assert r['status'] == 'fail'


@pytest.mark.parametrize('key', ['play_exit', 'check_recording_exit', 'ready_exit'])
def test_nonzero_exit_fails(tmp_path, monkeypatch, key):
    run = make(tmp_path, monkeypatch, [('map', 200)], exit_tsv=OK_EXIT.replace(f'{key}\t0', f'{key}\t1'))
    r = rep.scenario(run, ['s', 'bag', 'protocol', '0', 'map'], {})
    assert r['status'] == 'fail'


def test_leftover_node_and_low_rate_fail(tmp_path, monkeypatch):
    run = make(tmp_path, monkeypatch, [('map', 200)], errors='перед запуском жив процесс ноды')
    assert rep.scenario(run, ['s', 'bag', 'protocol', '0', 'map'], {})['status'] == 'fail'
    monkeypatch.setattr(rep, 'read_result', lambda record: stream('map', 20, hz=5.0))
    (tmp_path / 's' / 'errors.txt').unlink()
    assert rep.scenario(run, ['s', 'bag', 'protocol', '0', 'map'], {})['status'] == 'fail'


def test_no_output_fails(tmp_path, monkeypatch):
    run = make(tmp_path, monkeypatch, [])
    assert rep.scenario(run, ['s', 'bag', 'protocol', '0', 'odom'], {})['status'] == 'fail'
