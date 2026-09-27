"""judge_runner_report must fail a scenario on every broken expectation, never mask it."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import judge_runner_report as rep  # noqa: E402

OK_EXIT = 'ready_exit\t0\nrecord_ready_exit\t0\nplay_exit\t0\nrecord_exit\t0\nnode_exit\t0\ncheck_recording_exit\t0\n'


def stream(frame, n=200, hz=40.0):
    pos = [(100 + i / hz, 1000 + i / hz, frame, 1.0, 2.0) for i in range(n)]
    vel = [(100 + i / hz, 1000 + i / hz) for i in range(n)]
    return pos, vel


def make(tmp_path, monkeypatch, frames, exit_tsv=OK_EXIT, alive='node running\nrecord running\n',
         errors=None, node_log=''):
    d = tmp_path / 's'
    (d / 'record').mkdir(parents=True)
    (d / 'exit.tsv').write_text(exit_tsv)
    (d / 'alive.txt').write_text(alive)
    (d / 'times.tsv').write_text('node_launch\t10.0\nready\t11.0\nplay\t11.1\n')
    (d / 'node.log').write_text(node_log)
    if errors:
        (d / 'errors.txt').write_text(errors)
    pos, vel = [], []
    for frame, n in frames:
        p, v = stream(frame, n)
        pos += p
        vel += v
    monkeypatch.setattr(rep, 'read_result', lambda record: (pos, vel))
    monkeypatch.setattr(rep, 'read_input_span', lambda bag: (100.0, 104.9))
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


def test_output_covering_only_start_fails(tmp_path, monkeypatch):
    run = make(tmp_path, monkeypatch, [('map', 200)])
    monkeypatch.setattr(rep, 'read_input_span', lambda bag: (100.0, 190.0))
    r = rep.scenario(run, ['s', 'bag', 'protocol', '0', 'map'], {})
    assert r['status'] == 'fail'
    assert any('покрытие' in f for f in r['fails'])


@pytest.mark.parametrize('topic', ['position', 'velocity'])
def test_each_output_must_reach_last_input(tmp_path, monkeypatch, topic):
    run = make(tmp_path, monkeypatch, [('map', 200)])
    pos, vel = stream('map', 400)
    short_pos, short_vel = stream('map', 200)
    if topic == 'position':
        pos = short_pos
    else:
        vel = short_vel
    monkeypatch.setattr(rep, 'read_result', lambda record: (pos, vel))
    monkeypatch.setattr(rep, 'read_input_span', lambda bag: (100.0, 109.0))
    r = rep.scenario(run, ['s', 'bag', 'protocol', '0', 'map'], {})
    assert r['status'] == 'fail'
    assert any(f'покрытие /result/{topic}: конец' in f for f in r['fails'])


def test_full_output_coverage_passes(tmp_path, monkeypatch):
    run = make(tmp_path, monkeypatch, [('map', 200)])
    r = rep.scenario(run, ['s', 'bag', 'protocol', '0', 'map'], {})
    assert r['status'] == 'ok', r['fails']
    assert r['coverage']['position_first_delta_s'] == 0.0
    assert r['coverage']['velocity_last_delta_s'] == pytest.approx(0.075)


def test_late_mode_does_not_check_start_coverage(tmp_path, monkeypatch):
    run = make(tmp_path, monkeypatch, [('odom', 200)])
    pos, vel = stream('odom')
    monkeypatch.setattr(rep, 'read_result',
                        lambda record: ([(p[0] + 5, *p[1:]) for p in pos],
                                        [(v[0] + 5, v[1]) for v in vel]))
    monkeypatch.setattr(rep, 'read_input_span', lambda bag: (100.0, 109.0))
    r = rep.scenario(run, ['s', 'bag', 'late', '8', 'odom'], {})
    assert r['status'] == 'ok', r['fails']
    assert r['coverage']['velocity_first_delta_s'] == 5.0


def test_protocol_requires_velocity_from_start(tmp_path, monkeypatch):
    run = make(tmp_path, monkeypatch, [('map', 200)])
    pos, vel = stream('map', 400)
    monkeypatch.setattr(rep, 'read_result',
                        lambda record: (pos, [(v[0] + 2, v[1]) for v in vel]))
    monkeypatch.setattr(rep, 'read_input_span', lambda bag: (100.0, 109.0))
    r = rep.scenario(run, ['s', 'bag', 'protocol', '0', 'map'], {})
    assert r['status'] == 'fail'
    assert any('покрытие /result/velocity: начало' in f for f in r['fails'])


def test_child_exit_after_runner_sigint_is_warning(tmp_path, monkeypatch, capsys):
    log = '[odometry_node-1] process has died [pid 12, exit code 1, cmd /tmp/odometry_node]\n'
    run = make(tmp_path, monkeypatch, [('map', 200)], node_log=log)
    r = rep.scenario(run, ['s', 'bag', 'protocol', '0', 'map'], {})
    assert r['status'] == 'ok', r['fails']
    assert r['node_child_exit'] == 1
    assert r['warnings'] == ['odometry_node exit 1 после SIGINT runner']
    plan = run / 'plan.tsv'
    plan.write_text('s\tbag\tprotocol\t0\tmap\n')
    sources = run / 'sources.json'
    sources.write_text('{}')
    assert rep.main(['report', str(run), str(plan), str(sources)]) == 0
    printed = capsys.readouterr().out
    assert 'child exit' in printed
    assert '  ! s: odometry_node exit 1 после SIGINT runner' in printed
    assert json.loads((run / 'report.json').read_text())[0]['node_child_exit'] == 1


def test_child_exit_before_stop_fails(tmp_path, monkeypatch):
    log = '[odometry_node-1] process has died [pid 12, exit code 1, cmd /tmp/odometry_node]\n'
    run = make(tmp_path, monkeypatch, [('map', 200)],
               alive='node exited before stop\nrecord running\n', node_log=log)
    r = rep.scenario(run, ['s', 'bag', 'protocol', '0', 'map'], {})
    assert r['status'] == 'fail'
    assert r['node_child_exit'] == 1


def test_read_input_span_uses_only_vehicle_header_stamps(monkeypatch, tmp_path):
    def msg(seconds):
        return SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(sec=seconds, nanosec=0)))

    monkeypatch.setattr(rep, 'read_bag', lambda bag: [
        ('/sensing/gnss/master/fix', msg(1)),
        (rep.INPUTS[0], msg(12)),
        (rep.INPUTS[1], msg(10)),
        (rep.INPUTS[2], msg(20)),
        ('/sensing/gnss/master/vel', msg(99)),
    ])
    assert rep.read_input_span(tmp_path / 'bag') == (10.0, 20.0)


def test_clean_child_exit_and_missing_exit(tmp_path):
    log = tmp_path / 'node.log'
    assert rep.node_child_exit(log) is None
    log.write_text('[odometry_node-1] process has finished cleanly [pid 12]\n')
    assert rep.node_child_exit(log) == 0
