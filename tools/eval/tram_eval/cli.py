"""Command line: a split or bags -> markdown table in stdout + out/eval/<run>/metrics.json."""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime
from pathlib import Path

from . import bag as bagmod
from .metrics import MAIN_METRICS, METRIC_KEYS, summarize

D012_TOL = 0.02   # main metrics may not get worse by more than 2 % (D-012)


def git_commit() -> str:
    def git(*a):
        return subprocess.run(['git', '-C', str(bagmod.REPO), *a], capture_output=True, text=True).stdout.strip()
    commit = git('rev-parse', '--short', 'HEAD') or 'nogit'
    return commit + ('-dirty' if git('status', '--porcelain', '--untracked-files=no') else '')


def fmt(v) -> str:
    if v is None:
        return '—'
    if isinstance(v, bool):
        return 'да' if v else ''
    return f'{v:.3f}' if abs(v) < 10 else f'{v:.1f}'


def summary_table(result: dict, base: dict | None) -> list[str]:
    med, worst = result['summary']['median'], result['summary']['worst_bag']
    if base is None:
        lines = ['| Метрика (медиана по bag) | медиана | худший bag |', '|---|---|---|']
        for k in METRIC_KEYS:
            w = worst.get(k)
            name = f'**{k}**' if k in MAIN_METRICS else k
            lines.append(f"| {name} | {fmt(med[k])} | {fmt(result['bags'][w][k]) + ' `' + w + '`' if w else '—'} |")
        return lines
    bmed = base['summary']['median']
    lines = [f"Было: `{base['commit']}` ({base['split']}), стало: `{result['commit']}` ({result['split']})", '',
             '| Метрика (медиана по bag) | было | стало | Δ, % |', '|---|---|---|---|']
    for k in METRIC_KEYS:
        old, new = bmed.get(k), med[k]
        delta = fmt((new - old) / abs(old) * 100) if old not in (None, 0) and new is not None else '—'
        name = f'**{k}**' if k in MAIN_METRICS else k
        lines.append(f'| {name} | {fmt(old)} | {fmt(new)} | {delta} |')
    # a main metric the base has and the branch lost (None) is worse, not "no change"
    worse = [k for k in MAIN_METRICS if bmed.get(k) is not None
             and (med[k] is None or med[k] > bmed[k] * (1 + D012_TOL))]
    lines += ['', f"D-012: {'хуже больше чем на 2 %: ' + ', '.join(worse) if worse else 'главные метрики не хуже'}"]
    if set(base['bags']) != set(result['bags']):
        lines.append('Внимание: наборы bag в сравнении разные — медианы несравнимы.')
    return lines


def bag_table(result: dict) -> list[str]:
    cols = ('distance_m', 'speed_rmse', 'along_rmse', 'drift_pct', 'cross_rmse', 'pos3d_rmse', 'n_matched', 'crashed')
    lines = ['| bag | ' + ' | '.join(cols) + ' |', '|---' * (len(cols) + 1) + '|']
    for b, m in sorted(result['bags'].items()):
        lines.append(f'| {b} | ' + ' | '.join(str(m[c]) if c == 'n_matched' else fmt(m[c]) for c in cols) + ' |')
    return lines


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog='tram_eval', description=__doc__)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--split', help='набор из tools/eval/splits.yaml: quick, holdout, train')
    src.add_argument('--bag', action='append', help='имя каталога bag (можно несколько раз)')
    ap.add_argument('--gnss-window', type=float, default=None,
                    help='секунд GNSS для модели, по умолчанию gnss.init_window_s из params.yaml (D-005)')
    ap.add_argument('--compare', type=Path, help='metrics.json базы (например, прогон origin/main)')
    ap.add_argument('--jobs', type=int, default=max(1, min(8, (os.cpu_count() or 2) - 1)))
    ap.add_argument('--out', type=Path, default=None, help='каталог прогона, по умолчанию out/eval/<commit>-<набор>')
    args = ap.parse_args(argv)
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')   # markdown with Cyrillic on a Windows console

    if args.split:
        splits = bagmod.load_splits()
        if args.split not in splits or not isinstance(splits[args.split], list):
            ap.error(f'нет набора {args.split!r} в {bagmod.SPLITS_YAML}')
        names = list(splits[args.split])
    else:
        names = args.bag
    data = bagmod.data_dir()
    paths = [data / n for n in names]
    missing = [p.name for p in paths if not (p / 'metadata.yaml').exists()]
    if missing:
        ap.error(f'нет bag в {data}: {", ".join(missing)} (docs/data.md, TRAM_DATA_DIR)')
    window = bagmod.default_gnss_window() if args.gnss_window is None else args.gnss_window
    base = json.loads(args.compare.read_text(encoding='utf-8')) if args.compare else None
    bagmod.default_odometry()   # fail fast if the pipeline cannot be built

    t0 = time.monotonic()
    if args.jobs > 1 and len(paths) > 1:
        with ProcessPoolExecutor(min(args.jobs, len(paths))) as ex:
            results = list(ex.map(bagmod.evaluate_bag, paths, [window] * len(paths)))
    else:
        results = [bagmod.evaluate_bag(p, window) for p in paths]
    notes = [r.pop(bagmod.NOTES, {}) for r in results]
    nonfinite = sum(n.get('nonfinite', 0) for n in notes)
    mismatch = sum(n.get('stamp_mismatch', 0) for n in notes)
    commit = git_commit()
    label = args.split or (names[0] if len(names) == 1 else 'bags')
    result = {'commit': commit, 'split': label, 'gnss_window_s': window,
              'created': datetime.now().astimezone().isoformat(timespec='seconds'),
              'bags': dict(zip(names, results))}
    result['summary'] = summarize(result['bags'])

    out = args.out or bagmod.out_dir() / 'eval' / f'{commit}-{label}'
    out.mkdir(parents=True, exist_ok=True)
    (out / 'metrics.json').write_text(json.dumps(result, ensure_ascii=False, indent=1), encoding='utf-8')

    print('\n'.join(summary_table(result, base)))
    print()
    print('\n'.join(bag_table(result)))
    crashed = sorted(b for b, m in result['bags'].items() if m['crashed'])
    print(f"\n{len(paths)} bag, окно GNSS {window} с, {time.monotonic() - t0:.0f} с; "
          f"упали: {', '.join(crashed) if crashed else 'нет'}; "
          f"оценок NaN/inf (вне метрик): {nonfinite}; t != stamp входа: {mismatch}; -> {out / 'metrics.json'}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
