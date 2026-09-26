"""Machine gate for the submission (D-025): green only on evidence, anything unknown is red.

Usage (from the repo root or a worktree):
    .venv/bin/python tools/submission/check_submission.py

Checks, each OK / FAIL / НЕ ПРОВЕРЕНО; exit code 0 only when every check is OK:
  blockers    open GitHub issues labelled `blocker`, except the gate's own umbrella issue
              (#96, also labelled `gate`; #117, D-065): it closes when this gate is green
  layouts     out/submission/layouts-<HEAD>.json from tools/submission/jury_layouts.sh --run
  tree        tracked files are committed: the layouts report proves HEAD, nothing else
  artifacts   the six submission artifacts and a launch file exist
  markers     TBD / TODO / FIXME / XXX in the submission documents
  compliance  every T* and A* row of docs/tz-compliance.md is ✅
  params      every params.yaml key is described in docs/parameters.md
"""
from __future__ import annotations

import json
import os
import re
import subprocess
import sys
from pathlib import Path

import yaml

OK, FAIL, UNVERIFIED = 'OK', 'FAIL', 'НЕ ПРОВЕРЕНО'
# The umbrella blocker that tracks this gate itself (D-065): exempt only by number AND label,
# so a real blocker labelled `gate` by mistake still fails the gate.
GATE_ISSUE = 96
GATE_LABEL = 'gate'
ROOT = Path(__file__).resolve().parents[2]
ARTIFACTS = ('README.md', 'docs/model.md', 'docs/parameters.md', 'docs/accuracy.md',
             'docs/roadmap.md', 'src/tram_odometry/package.xml',
             'src/tram_odometry_core/package.xml', 'src/tram_odometry/config/params.yaml')
DOCS = ('README.md', 'docs/model.md', 'docs/parameters.md', 'docs/accuracy.md', 'docs/roadmap.md')
MARKER = re.compile(r'\b(TBD|TODO|FIXME|XXX)\b')
ROW = re.compile(r'^\|\s*([TA]\d+)\s*\|')


def ensure_utf8_stdout() -> None:
    """Console codepage must not crash the gate on its own Cyrillic/emoji output (#124).

    `print()` writes through `sys.stdout`, whose encoding on Windows defaults to the
    console codepage (e.g. cp1251), not UTF-8; `check_compliance`'s FAIL detail contains
    U+2705 and raises `UnicodeEncodeError` there without this. `errors='replace'` keeps
    the verdict itself readable even if a few glyphs render as '?'.
    """
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')


def _is_gate_umbrella(issue: dict) -> bool:
    labels = issue.get('labels') or []
    return (issue.get('number') == GATE_ISSUE
            and any(isinstance(lb, dict) and lb.get('name') == GATE_LABEL for lb in labels))


def check_blockers(issues: list | None) -> tuple[str, str]:
    if issues is None:
        return UNVERIFIED, 'gh не ответил: открытые blocker не видны (gh auth status)'
    gate = [i for i in issues if _is_gate_umbrella(i)]
    real = [i for i in issues if i not in gate]
    if real:
        return FAIL, 'открыты: ' + '; '.join(f"#{i['number']} {i['title']}" for i in real)
    if gate:
        return OK, 'открыта только зонтичная issue гейта: ' + ', '.join(f"#{i['number']}" for i in gate)
    return OK, 'открытых нет'


def check_layouts(out: Path, commit: str) -> tuple[str, str]:
    path = out / 'submission' / f'layouts-{commit}.json'
    if not path.is_file():
        return FAIL, f'нет отчёта на HEAD {commit[:7]}: bash tools/submission/jury_layouts.sh --run'
    rep = json.loads(path.read_text(encoding='utf-8'))
    if rep.get('dirty'):
        return FAIL, 'отчёт снят с незакоммиченными изменениями'
    bad = [x['name'] for x in rep['layouts'] if x['build'] != 'ok']
    if bad:
        return FAIL, 'не собирается: ' + ', '.join(bad)
    if not rep['runs']:
        return FAIL, 'нода не запускалась: нужен jury_layouts.sh --run'
    bad = [f"{x['bag']} ({x['note']})" for x in rep['runs'] if x['status'] != 'ok']
    if bad:
        return FAIL, 'запуск не ok: ' + '; '.join(bad)
    return OK, f"{len(rep['layouts'])} раскладок, {len(rep['runs'])} bag"


def check_tree(root: Path) -> tuple[str, str]:
    res = subprocess.run(['git', '-C', str(root), 'status', '--porcelain', '--untracked-files=no'],
                         capture_output=True, text=True, encoding='utf-8')
    if res.returncode != 0:
        return UNVERIFIED, 'git status не отработал'
    changed = res.stdout.split('\n')
    changed = [line for line in changed if line.strip()]
    return (FAIL, f'незакоммичено: {len(changed)} путей') if changed else (OK, 'чисто')


def check_artifacts(root: Path) -> tuple[str, str]:
    missing = [rel for rel in ARTIFACTS if not (root / rel).is_file()]
    if not list((root / 'src/tram_odometry/launch').glob('*.launch.py')):
        missing.append('src/tram_odometry/launch/*.launch.py')
    return (FAIL, 'нет: ' + ', '.join(missing)) if missing else (OK, 'все на месте')


def check_markers(root: Path) -> tuple[str, str]:
    hits = []
    for rel in DOCS:
        path = root / rel
        if path.is_file():
            for n, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
                if MARKER.search(line):
                    hits.append(f'{rel}:{n}')
    return (FAIL, ', '.join(hits[:10]) + (f' и ещё {len(hits) - 10}' if len(hits) > 10 else '')) \
        if hits else (OK, 'нет')


def check_compliance(root: Path) -> tuple[str, str]:
    path = root / 'docs/tz-compliance.md'
    if not path.is_file():
        return FAIL, 'нет docs/tz-compliance.md'
    open_rows = []
    for line in path.read_text(encoding='utf-8').splitlines():
        m = ROW.match(line)
        if m and not line.rstrip(' |').split('|')[-1].strip().startswith('✅'):
            open_rows.append(m.group(1))
    return (FAIL, 'не ✅: ' + ', '.join(open_rows)) if open_rows else (OK, 'T* и A* закрыты')


def param_keys(node: dict, prefix: str = '') -> list[str]:
    keys = []
    for k, v in node.items():
        keys += param_keys(v, f'{prefix}{k}.') if isinstance(v, dict) else [f'{prefix}{k}']
    return keys


def check_params(root: Path) -> tuple[str, str]:
    doc = root / 'docs/parameters.md'
    if not doc.is_file():
        return FAIL, 'нет docs/parameters.md'
    params = yaml.safe_load((root / 'src/tram_odometry/config/params.yaml').read_text(encoding='utf-8'))
    text = doc.read_text(encoding='utf-8')
    missing = [k for k in param_keys(params['/**']['ros__parameters']) if k not in text]
    return (FAIL, 'не описаны: ' + ', '.join(missing)) if missing else (OK, 'все ключи описаны')


def open_blockers() -> list | None:
    try:
        res = subprocess.run(['gh', 'issue', 'list', '--label', 'blocker', '--state', 'open',
                              '--limit', '100', '--json', 'number,title,labels'],
                             capture_output=True, text=True, encoding='utf-8', timeout=30, cwd=ROOT)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return json.loads(res.stdout) if res.returncode == 0 else None


def out_dir(root: Path) -> Path:
    """TRAM_OUT_DIR from the environment or .env, else <root>/out (as jury_layouts.sh)."""
    value = os.environ.get('TRAM_OUT_DIR')
    env = root / '.env'
    if value is None and env.is_file():
        for line in env.read_text(encoding='utf-8').splitlines():
            if line.startswith('TRAM_OUT_DIR='):
                value = line.split('=', 1)[1].split('#', 1)[0].strip()
    path = Path(value) if value else root / 'out'
    return path if path.is_absolute() else root / path


def exit_code(rows: list[tuple[str, str, str]]) -> int:
    return 0 if all(status == OK for _, status, _ in rows) else 1


def main() -> int:
    ensure_utf8_stdout()
    root = Path(subprocess.run(['git', 'rev-parse', '--show-toplevel'], capture_output=True,
                               text=True, encoding='utf-8', check=True).stdout.strip())
    commit = subprocess.run(['git', '-C', str(root), 'rev-parse', 'HEAD'], capture_output=True,
                            text=True, encoding='utf-8', check=True).stdout.strip()
    rows = [('blockers', *check_blockers(open_blockers())),
            ('layouts', *check_layouts(out_dir(root), commit)),
            ('tree', *check_tree(root)),
            ('artifacts', *check_artifacts(root)),
            ('markers', *check_markers(root)),
            ('compliance', *check_compliance(root)),
            ('params', *check_params(root))]
    print(f'Проверка сдачи на {commit[:7]}\n')
    print('| проверка | итог | подробности |\n|---|---|---|')
    for name, status, detail in rows:
        print(f'| {name} | {status} | {detail} |')
    code = exit_code(rows)
    print('\nСдавать можно.' if code == 0 else '\nСдавать нельзя: всё, что не OK, — работа до сдачи.')
    return code


if __name__ == '__main__':
    sys.exit(main())
