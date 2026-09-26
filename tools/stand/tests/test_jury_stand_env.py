"""Explicit stand paths must win over defaults copied into a worktree .env."""

import os
import subprocess
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[3] / 'docker' / 'jury-stand.sh'


def test_explicit_data_dir_overrides_dotenv(tmp_path):
    subprocess.run(['git', 'init', '-q'], cwd=tmp_path, check=True)
    (tmp_path / '.env').write_text('TRAM_DATA_DIR=/bag-that-does-not-exist\n', encoding='utf-8')
    data = tmp_path / 'data'
    (data / 'stress_bag').mkdir(parents=True)
    fake_bin = tmp_path / 'bin'
    fake_bin.mkdir()
    docker = fake_bin / 'docker'
    docker.write_text('#!/bin/sh\n: > "$FAKE_DOCKER_MARKER"\nexit 17\n', encoding='utf-8')
    docker.chmod(0o755)
    marker = tmp_path / 'docker-reached'
    env = dict(os.environ, TRAM_DATA_DIR=str(data), FAKE_DOCKER_MARKER=str(marker),
               PATH=f'{fake_bin}:{os.environ["PATH"]}')

    result = subprocess.run(['bash', str(SCRIPT), 'stress_bag', '1.0'], cwd=tmp_path,
                            env=env, capture_output=True, text=True)

    assert result.returncode == 17
    assert marker.is_file()
    assert 'Нет bag' not in result.stderr
