"""Command line: a split or bags -> markdown table in stdout + out/eval/<run>/metrics.json."""
from __future__ import annotations

import argparse
import json
import os
import statistics
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from datetime import datetime
from pathlib import Path

from . import bag as bagmod
from .metrics import MAIN_METRICS, METRIC_KEYS, summarize
from .plot import plot_bag
from .stress import SCENARIOS, evaluate_stress_bag

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


def stress_table(bags: dict) -> list[str]:
    lines = ['| Сценарий | bag | пик ошибки скорости, м/с median/max | добавка к пику скорости, м/с median | восстановление скорости, с median (нет) | пик ошибки позиции, м median/max | добавка к пику позиции, м median | восстановление позиции, с median (нет) | нет оценок в событии | падения |',
             '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|']

    def vals(rows, key):
        return [r[key] for r in rows if r[key] is not None]

    def medmax(rows, key):
        v = vals(rows, key)
        return f'{fmt(statistics.median(v))}/{fmt(max(v))}' if v else '—'

    for scenario in SCENARIOS:
        rows = [cases[scenario] for cases in bags.values() if not cases[scenario]['skipped']]
        speed_recovery = vals(rows, 'speed_recovery_s')
        pos_recovery = vals(rows, 'pos_recovery_s')
        recover_speed = f"{fmt(statistics.median(speed_recovery))} ({len(rows) - len(speed_recovery)})" if speed_recovery else f'— ({len(rows)})'
        recover_pos = f"{fmt(statistics.median(pos_recovery))} ({len(rows) - len(pos_recovery)})" if pos_recovery else f'— ({len(rows)})'
        excess = vals(rows, 'peak_speed_excess_mps')
        pos_excess = vals(rows, 'peak_pos3d_excess_m')
        lines.append(f"| {scenario} | {len(rows)}/{len(bags)} | {medmax(rows, 'peak_speed_error_mps')} | "
                     f"{fmt(statistics.median(excess)) if excess else '—'} | {recover_speed} | "
                     f"{medmax(rows, 'peak_pos3d_error_m')} | "
                     f"{fmt(statistics.median(pos_excess)) if pos_excess else '—'} | {recover_pos} | "
                     f"{sum(r['n_speed_during'] == 0 for r in rows)} | {sum(r['crashed'] for r in rows)} |")
    return lines


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog='tram_eval', description=__doc__)
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument('--split', help='набор из tools/eval/splits.yaml: quick, holdout, train')
    src.add_argument('--bag', action='append', help='имя каталога bag (можно несколько раз)')
    ap.add_argument('--gnss-window', type=float, default=None,
                    help='секунд GNSS для модели, по умолчанию gnss.init_window_s из params.yaml (D-005)')
    ap.add_argument('--compare', type=Path, help='metrics.json базы (например, прогон origin/main)')
    ap.add_argument('--stress', action='store_true', help='детерминированные сбои входа; отдельный stress.json')
    ap.add_argument('--plot', action='store_true', help='PNG по каждому bag в <out>/plots (нужен matplotlib)')
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
    if args.stress:
        if args.jobs > 1 and len(paths) > 1:
            with ProcessPoolExecutor(min(args.jobs, len(paths))) as ex:
                diagnostics = list(ex.map(evaluate_stress_bag, paths, [window] * len(paths)))
        else:
            diagnostics = [evaluate_stress_bag(p, window) for p in paths]
        stress = {'commit': commit, 'split': label, 'gnss_window_s': window,
                  'created': datetime.now().astimezone().isoformat(timespec='seconds'),
                  'bags': dict(zip(names, diagnostics))}
        (out / 'stress.json').write_text(json.dumps(stress, ensure_ascii=False, indent=1), encoding='utf-8')
        print('\n'.join(stress_table(stress['bags'])))
        print(f"\nСтресс: -> {out / 'stress.json'}")
    if args.plot:
        plots = out / 'plots'
        if args.jobs > 1 and len(paths) > 1:
            with ProcessPoolExecutor(min(args.jobs, len(paths))) as ex:
                files = list(ex.map(plot_bag, paths, [window] * len(paths), [plots] * len(paths)))
        else:
            files = [plot_bag(p, window, plots) for p in paths]
        print(f"\nГрафики: {sum(len(f) for f in files)} PNG -> {plots}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
