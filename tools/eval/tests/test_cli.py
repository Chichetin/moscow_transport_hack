"""Command line end to end on two short real bags with a test double of the pipeline."""
import json

import pytest

from tram_eval import bag, cli
from tram_eval.metrics import METRIC_KEYS

from test_bag import DeadReckoning

SHORT = ['30618_082f1d65', '30639_0ab96c59']     # 20 s and 79 s, holdout


@pytest.fixture
def short_bags(monkeypatch):
    if not all((bag.data_dir() / b / 'metadata.yaml').exists() for b in SHORT):
        pytest.skip('dataset not unpacked (docs/data.md)')
    monkeypatch.setattr(bag, 'default_odometry', DeadReckoning)
    return SHORT


def test_run_writes_contract_json_and_tables(short_bags, tmp_path, capsys):
    args = [a for b in short_bags for a in ('--bag', b)] + ['--jobs', '1']
    assert cli.main(args + ['--out', str(tmp_path / 'base')]) == 0
    out = capsys.readouterr().out
    res = json.loads((tmp_path / 'base' / 'metrics.json').read_text(encoding='utf-8'))
    assert {'commit', 'split', 'gnss_window_s', 'ref_point', 'created', 'bags', 'summary'} <= set(res)
    assert res['ref_point'] == 'base_link'
    assert res['gnss_window_s'] == 5.0 and res['split'] == 'bags'
    for b in short_bags:
        m = res['bags'][b]
        assert set(METRIC_KEYS) | {'duration_s', 'distance_m', 'n_matched', 'crashed'} <= set(m)
        assert m['crashed'] is False
    assert set(res['summary']) == {'median', 'worst_bag'}
    assert not any(bag.NOTES in m for m in res['bags'].values())      # format of §4 unchanged
    assert '**speed_rmse**' in out and short_bags[0] in out
    assert 'NaN/inf (вне метрик): 0; t != stamp входа: 0' in out

    assert cli.main(args + ['--out', str(tmp_path / 'new'), '--compare',
                            str(tmp_path / 'base' / 'metrics.json')]) == 0
    out = capsys.readouterr().out
    assert '| было | стало | Δ, % |' in out
    assert 'D-012: главные метрики не хуже' in out


def test_compare_with_a_base_of_another_reference_point_is_an_error(short_bags, tmp_path):
    """A metrics.json without ref_point is from before D-076: its reference is master."""
    base = tmp_path / 'old.json'
    base.write_text(json.dumps({'commit': 'old', 'bags': {}, 'summary': {'median': {}}}), encoding='utf-8')
    args = ['--bag', short_bags[0], '--jobs', '1', '--out', str(tmp_path / 'run'), '--compare', str(base)]
    with pytest.raises(SystemExit):
        cli.main(args)
    assert not (tmp_path / 'run' / 'metrics.json').exists()


def test_unknown_split_and_missing_bag_are_errors(capsys):
    with pytest.raises(SystemExit):
        cli.main(['--split', 'nope'])
    with pytest.raises(SystemExit):
        cli.main(['--bag', '30618_does_not_exist'])


def test_d012_missing_main_metric_is_not_silently_ok():
    base = {'commit': 'a', 'split': 's', 'bags': {'b': {}},
            'summary': {'median': {k: 1.0 for k in METRIC_KEYS}, 'worst_bag': {}}}
    new = {'commit': 'b', 'split': 's', 'bags': {'b': {}},
           'summary': {'median': dict(base['summary']['median'], speed_rmse=None), 'worst_bag': {}}}
    line = [x for x in cli.summary_table(new, base) if x.startswith('D-012')][0]
    assert 'speed_rmse' in line and 'не хуже' not in line


def test_entry_point_with_process_pool(tmp_path):
    # Windows starts pool workers with spawn: they re-import the entry script, which must not
    # run main() again (BrokenProcessPool). Uses the real pipeline.Odometry from main.
    import subprocess
    import sys
    if not all((bag.data_dir() / b / 'metadata.yaml').exists() for b in SHORT):
        pytest.skip('dataset not unpacked (docs/data.md)')
    cmd = [sys.executable, str(bag.REPO / 'tools' / 'eval' / 'run_eval.py'), '--jobs', '2',
           '--out', str(tmp_path)] + [a for b in SHORT for a in ('--bag', b)]
    run = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8', timeout=300)
    assert run.returncode == 0, run.stderr[-2000:]
    res = json.loads((tmp_path / 'metrics.json').read_text(encoding='utf-8'))
    assert set(res['bags']) == set(SHORT)
    assert not any(bag.NOTES in m for m in res['bags'].values())      # pool path keeps §4 format too


def test_stress_writes_separate_diagnostics_without_changing_metrics_schema(short_bags, tmp_path, capsys):
    out = tmp_path / 'stress'
    assert cli.main(['--bag', short_bags[0], '--jobs', '1', '--stress', '--out', str(out)]) == 0
    normal = json.loads((out / 'metrics.json').read_text(encoding='utf-8'))
    stress = json.loads((out / 'stress.json').read_text(encoding='utf-8'))
    assert set(normal) == {'commit', 'split', 'gnss_window_s', 'ref_point', 'created', 'bags', 'summary'}
    assert set(normal['bags'][short_bags[0]]) == set(METRIC_KEYS) | {'duration_s', 'distance_m', 'n_matched', 'crashed'}
    assert set(stress['bags'][short_bags[0]]) == set(cli.SCENARIOS)
    assert stress['bags'][short_bags[0]]['gap_70']['skipped'] is True
    assert 'пик ошибки скорости, м/с' in capsys.readouterr().out
