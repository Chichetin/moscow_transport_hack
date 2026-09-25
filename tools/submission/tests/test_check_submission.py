"""check_submission: every check says OK only on evidence; anything unknown is red."""
import json
from pathlib import Path

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
