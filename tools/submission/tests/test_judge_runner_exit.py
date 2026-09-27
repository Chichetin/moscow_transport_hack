"""The outer runner must preserve a failed container even when reporting succeeds."""
import os
import subprocess
from pathlib import Path


RUNNER = Path(__file__).resolve().parents[1] / 'judge_runner.sh'


def executable(path, body):
    path.write_text('#!/usr/bin/env bash\n' + body)
    path.chmod(0o755)


def test_failed_container_cannot_be_masked_by_successful_report(tmp_path):
    root = tmp_path / 'repo'
    (root / '.git').mkdir(parents=True)
    data = tmp_path / 'data'
    for bag in ('bag_a', 'bag_b'):
        (data / bag).mkdir(parents=True)
    archive = tmp_path / 'archive'
    (archive / 'tools' / 'submission').mkdir(parents=True)
    python = root / '.venv' / 'bin' / 'python'
    python.parent.mkdir(parents=True)
    executable(python, '[[ "$1" == */judge_runner_report.py ]] && echo "report succeeded"\nexit 0\n')

    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    executable(fake_bin / 'git', '''
if [[ "$1" == rev-parse && "$2" == --show-toplevel ]]; then
  echo "$FAKE_ROOT"
elif [[ "$1" == rev-parse && "$2" == --path-format=absolute ]]; then
  echo "$FAKE_ROOT/.git"
elif [[ "$1" == -C && "$3" == rev-parse ]]; then
  echo deadbeef
elif [[ "$1" == -C && "$3" == archive ]]; then
  tar -cf - -C "$FAKE_ARCHIVE" .
fi
''')
    executable(fake_bin / 'docker', '''
if [[ "$1" == build ]]; then
  echo image
elif [[ "$1" == run ]]; then
  echo 'container failed'
  exit 7
fi
''')
    out = tmp_path / 'out'
    env = dict(os.environ, PATH=f'{fake_bin}:{os.environ["PATH"]}',
               FAKE_ROOT=str(root), FAKE_ARCHIVE=str(archive),
               TRAM_DATA_DIR=str(data), TRAM_OUT_DIR=str(out))
    result = subprocess.run(['bash', str(RUNNER), 'bag_a', 'bag_b'],
                            env=env, text=True, capture_output=True, check=False)

    runs = list((out / 'submission').iterdir())
    assert len(runs) == 1
    assert (runs[0] / 'run.txt').read_text().endswith('container_exit 7\nreport_exit 0\n')
    assert (runs[0] / 'report.md').read_text() == 'report succeeded\n'
    assert result.returncode == 1
