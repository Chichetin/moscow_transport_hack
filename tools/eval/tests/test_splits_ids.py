"""splits.yaml contract: every bag id reads back as the '<vehicle>_<hash>' string it names.

YAML 1.1 treats '_' as a digit separator, so an unquoted all-digit hash such as
30618_68847170 loads as the integer 3061868847170 and the bag silently drops out of a split.
"""
import os
import re
import subprocess
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
SPLITS = ROOT / 'tools' / 'eval' / 'splits.yaml'
BAG_ID = re.compile(r'^306(18|39)_[0-9a-f]{8}$')


def all_ids(splits: dict) -> list:
    ids = []
    for name, value in splits.items():
        if isinstance(value, dict):
            ids += list(value.keys()) + list(value.values())
        else:
            ids += list(value or [])
    return ids


def data_dir() -> Path:
    env = os.environ.get('TRAM_DATA_DIR')
    if env:
        return Path(env)
    common = subprocess.run(['git', '-C', str(ROOT), 'rev-parse', '--path-format=absolute',
                             '--git-common-dir'], capture_output=True, text=True).stdout.strip()
    return (Path(common).parent if common else ROOT) / 'dataset' / 'data'


def test_every_bag_id_loads_as_a_string():
    ids = all_ids(yaml.safe_load(SPLITS.read_text(encoding='utf-8')))
    bad = [x for x in ids if not isinstance(x, str) or not BAG_ID.match(x)]
    assert ids and bad == []


def test_every_bag_id_exists_in_the_dataset_when_present():
    data = data_dir()
    if not data.is_dir():
        pytest.skip(f'no dataset at {data}')
    ids = all_ids(yaml.safe_load(SPLITS.read_text(encoding='utf-8')))
    assert [x for x in ids if not (data / str(x)).is_dir()] == []
