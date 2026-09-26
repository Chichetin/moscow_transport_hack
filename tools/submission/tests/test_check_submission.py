"""check_submission: every check says OK only on evidence; anything unknown is red."""
import io
import json
from pathlib import Path

import pytest

import check_submission as cs

OK, FAIL, UNVERIFIED = cs.OK, cs.FAIL, cs.UNVERIFIED


def write(root: Path, rel: str, text: str) -> Path:
    p = root / rel
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(text, encoding='utf-8')
    return p


def report(root: Path, commit: str, layouts=('ok', 'ok'), runs=('ok',), dirty=0) -> None:
    write(root, f'out/submission/layouts-{commit}.json', json.dumps({
        'commit': commit, 'dirty': dirty,
        'layouts': [{'name': f'l{i}', 'what': '', 'build': s, 'note': ''} for i, s in enumerate(layouts)],
        'runs': [{'bag': f'b{i}', 'status': s, 'note': ''} for i, s in enumerate(runs)],
    }))


# --- blockers

def test_open_blockers_fail_and_are_named():
    status, detail = cs.check_blockers([{'number': 38, 'title': 'дубликат tram_vehicle_msgs'}])
    assert status == FAIL and '#38' in detail


def test_no_open_blockers_is_ok():
    assert cs.check_blockers([])[0] == OK


def test_blockers_unknown_when_gh_fails():
    assert cs.check_blockers(None)[0] == UNVERIFIED


def gate_issue(number=96):
    return {'number': number, 'title': 'blocker: check_submission.py fails on main',
            'labels': [{'name': 'blocker'}, {'name': cs.GATE_LABEL}]}


def test_gate_umbrella_alone_does_not_block_itself():
    """#117: the umbrella issue closes when the gate is green; counting it would deadlock."""
    status, detail = cs.check_blockers([gate_issue()])
    assert status == OK and '#96' in detail


def test_real_blocker_next_to_gate_umbrella_fails():
    other = {'number': 41, 'title': 'ссылки формы', 'labels': [{'name': 'blocker'}]}
    status, detail = cs.check_blockers([gate_issue(), other])
    assert status == FAIL and '#41' in detail and '#96' not in detail


def test_blocker_without_labels_field_still_fails():
    assert cs.check_blockers([{'number': 38, 'title': 'x'}])[0] == FAIL


def test_gate_label_on_another_issue_still_fails():
    """Review of #118: only the pinned umbrella is exempt; a real blocker about the gate
    labelled `gate` by mistake must not turn the gate green."""
    status, detail = cs.check_blockers([gate_issue(), gate_issue(number=120)])
    assert status == FAIL and '#120' in detail


def test_umbrella_number_without_gate_label_still_fails():
    issue = {'number': cs.GATE_ISSUE, 'title': 'x', 'labels': [{'name': 'blocker'}]}
    assert cs.check_blockers([issue])[0] == FAIL


@pytest.mark.parametrize('labels', [None, ['gate'], 'gate'])
def test_odd_labels_format_fails_safe(labels):
    issue = {'number': cs.GATE_ISSUE, 'title': 'x', 'labels': labels}
    assert cs.check_blockers([issue])[0] == FAIL


def test_open_blockers_asks_gh_for_labels(monkeypatch):
    """Without `labels` in the gh query the gate umbrella can't be told apart (#117)."""
    seen = {}

    def fake_run(cmd, **kwargs):
        seen['cmd'] = cmd
        return type('R', (), {'returncode': 0, 'stdout': json.dumps([gate_issue()])})()

    monkeypatch.setattr(cs.subprocess, 'run', fake_run)
    issues = cs.open_blockers()
    assert 'labels' in seen['cmd'][seen['cmd'].index('--json') + 1].split(',')
    assert cs.check_blockers(issues)[0] == OK


def test_open_blockers_decodes_gh_output_as_utf8(monkeypatch):
    """`gh` emits UTF-8 JSON regardless of the console codepage (#124): decoding it via the
    locale's preferred encoding (Python's default for `text=True`) mangles Cyrillic titles
    on a non-UTF-8 Windows console, even though `check_blockers` itself still returns FAIL."""
    seen = {}

    def fake_run(cmd, **kwargs):
        seen['kwargs'] = kwargs
        return type('R', (), {'returncode': 0, 'stdout': json.dumps([gate_issue()])})()

    monkeypatch.setattr(cs.subprocess, 'run', fake_run)
    cs.open_blockers()
    assert seen['kwargs'].get('encoding') == 'utf-8'


# --- layouts report

def test_missing_report_for_head_fails(tmp_path):
    report(tmp_path, 'old')
    status, detail = cs.check_layouts(tmp_path / 'out', 'head')
    assert status == FAIL and 'jury_layouts.sh' in detail


def test_all_layouts_and_runs_ok(tmp_path):
    report(tmp_path, 'head')
    assert cs.check_layouts(tmp_path / 'out', 'head')[0] == OK


def test_failed_layout_fails(tmp_path):
    report(tmp_path, 'head', layouts=('ok', 'fail'))
    status, detail = cs.check_layouts(tmp_path / 'out', 'head')
    assert status == FAIL and 'l1' in detail


def test_build_only_report_is_not_enough(tmp_path):
    report(tmp_path, 'head', runs=())
    status, detail = cs.check_layouts(tmp_path / 'out', 'head')
    assert status == FAIL and '--run' in detail


def test_skipped_run_fails(tmp_path):
    report(tmp_path, 'head', runs=('ok', 'skipped'))
    assert cs.check_layouts(tmp_path / 'out', 'head')[0] == FAIL


def test_report_from_dirty_tree_fails(tmp_path):
    report(tmp_path, 'head', dirty=2)
    assert cs.check_layouts(tmp_path / 'out', 'head')[0] == FAIL


# --- artifacts

def test_missing_artifacts_are_listed(tmp_path):
    write(tmp_path, 'README.md', 'x')
    status, detail = cs.check_artifacts(tmp_path)
    assert status == FAIL and 'docs/model.md' in detail and 'launch' in detail


def test_all_artifacts_present(tmp_path):
    for rel in cs.ARTIFACTS:
        write(tmp_path, rel, 'x')
    write(tmp_path, 'src/tram_odometry/launch/odometry.launch.py', 'x')
    assert cs.check_artifacts(tmp_path)[0] == OK


# --- markers

def test_markers_in_submission_docs_fail_with_file_and_line(tmp_path):
    write(tmp_path, 'README.md', 'ok\nЧастота: TBD Гц\n')
    write(tmp_path, 'docs/model.md', '# TODO: уравнения\n')
    status, detail = cs.check_markers(tmp_path)
    assert status == FAIL and 'README.md:2' in detail and 'docs/model.md:1' in detail


def test_markers_inside_words_do_not_count(tmp_path):
    write(tmp_path, 'README.md', 'TBDX и todolist не маркеры\n')
    assert cs.check_markers(tmp_path)[0] == OK


# --- compliance matrix

MATRIX = """| # | Пункт | Артефакт | Проверка | Статус |
|---|---|---|---|---|
| T1 | colcon | src | стенд | ✅ собирается |
| T2 | входы | нода | grep | ⚠️ частично |
| S1 | RMSE | eval | holdout | — |
| A1 | пакеты | src | T1–T8 | — |
"""


def test_compliance_requires_every_T_and_A_row_closed(tmp_path):
    write(tmp_path, 'docs/tz-compliance.md', MATRIX)
    assert cs.check_compliance(tmp_path) == (FAIL, 'не ✅: T2, A1')


def test_compliance_all_closed(tmp_path):
    write(tmp_path, 'docs/tz-compliance.md', MATRIX.replace('⚠️ частично', '✅ да').replace('| — |\n', '| ✅ да |\n'))
    assert cs.check_compliance(tmp_path)[0] == OK


# --- params described

PARAMS = """/**:
  ros__parameters:
    frames:
      map: map   # frame
    gnss:
      init_window_s: 5.0   # s
"""


def test_every_param_key_must_be_described(tmp_path):
    write(tmp_path, 'src/tram_odometry/config/params.yaml', PARAMS)
    write(tmp_path, 'docs/parameters.md', '`frames.map` — frame_id\n')
    status, detail = cs.check_params(tmp_path)
    assert status == FAIL and 'gnss.init_window_s' in detail and 'frames.map' not in detail


def test_params_described(tmp_path):
    write(tmp_path, 'src/tram_odometry/config/params.yaml', PARAMS)
    write(tmp_path, 'docs/parameters.md', '`frames.map`, `gnss.init_window_s`\n')
    assert cs.check_params(tmp_path)[0] == OK


# --- verdict

def test_verdict_is_red_on_any_unverified():
    rows = [('a', OK, ''), ('b', UNVERIFIED, 'gh')]
    assert cs.exit_code(rows) == 1
    assert cs.exit_code([('a', OK, '')]) == 0


# --- console encoding (#124)

def test_ensure_utf8_stdout_survives_a_narrow_console_encoding(monkeypatch):
    """A '✅' in a FAIL detail (check_compliance) must not crash the gate on a console
    whose default codepage is not UTF-8 (Windows, cp1251) -- it must print something,
    not raise UnicodeEncodeError."""
    narrow = io.TextIOWrapper(io.BytesIO(), encoding='ascii')
    monkeypatch.setattr(cs.sys, 'stdout', narrow)
    cs.ensure_utf8_stdout()
    print('не ✅: A2')  # raises UnicodeEncodeError on the original ascii-encoded stream
    narrow.flush()
    assert narrow.buffer.getvalue()  # something was written, not silently dropped


def test_ensure_utf8_stdout_is_a_noop_without_reconfigure(monkeypatch):
    """A stream with no `reconfigure` (e.g. some test/CI capture shims) must not crash either."""
    monkeypatch.setattr(cs.sys, 'stdout', object())
    cs.ensure_utf8_stdout()  # must not raise AttributeError
