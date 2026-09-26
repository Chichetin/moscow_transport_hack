"""Explicit stand paths must win over defaults copied into a worktree .env."""

import os
import subprocess
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[3] / 'docker' / 'jury-stand.sh'


def _bash_path(path):
    """A path bash's own `case $DATA in /*)` sees as absolute (#122).

    On Linux `str(path)` already starts with '/'. On Windows it starts with a drive
    letter ('C:\\...'), which that textual check does not recognise as absolute, so
    jury-stand.sh silently prepends its own root and looks for a mangled path. Only
    a POSIX-style path (e.g. '/c/Users/...', as Git Bash's own `cygpath -u` would give)
    passes the check; this test does not run in the jury's Linux environment, so the
    conversion belongs here, not in the script.
    """
    s = str(path)
    if os.name == 'nt' and len(s) > 1 and s[1] == ':':
        return '/' + s[0].lower() + s[2:].replace('\\', '/')
    return s


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
    # os.pathsep, not a literal ':' -- on Windows the native PATH is ';'-joined; a
    # hardcoded ':' merged fake_bin into the next real entry (#122).
    env = dict(os.environ, TRAM_DATA_DIR=_bash_path(data), FAKE_DOCKER_MARKER=str(marker),
               PATH=os.pathsep.join([str(fake_bin), os.environ['PATH']]))

    result = subprocess.run(['bash', str(SCRIPT), 'stress_bag', '1.0'], cwd=tmp_path,
                            env=env, capture_output=True, text=True)

    assert result.returncode == 17
    assert marker.is_file()
    assert 'Нет bag' not in result.stderr
