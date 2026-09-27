"""tram_stand: python tools/stand/run_stand.py <out/stand/BAG> -> table + stand.json."""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

from . import analyze

REPO = Path(__file__).resolve().parents[3]
MSG_DIR = REPO / 'src' / 'tram_vehicle_msgs' / 'msg'
INPUTS = ('/vehicle/front_bogie_velocity', '/vehicle/rear_bogie_velocity',
          '/vehicle/driver_position_cmd')
OUTPUTS = ('/result/velocity', '/result/position')


def read_record(path: Path):
    """[(topic, recv_ns, stamp_ns)] for inputs and outputs, in recording order."""
    from rosbags.highlevel import AnyReader
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore
    ts = get_typestore(Stores.ROS2_HUMBLE)
    for name in ('VelocitySensor', 'DriverControllerCommand'):
        ts.register(get_types_from_msg((MSG_DIR / f'{name}.msg').read_text(encoding='utf-8'),
                                       f'tram_vehicle_msgs/msg/{name}'))
    out = []
    with AnyReader([path], default_typestore=ts) as reader:
        conns = [c for c in reader.connections if c.topic in INPUTS + OUTPUTS]
        for conn, recv, raw in reader.messages(connections=conns):
            s = reader.deserialize(raw, conn.msgtype).header.stamp
            out.append((conn.topic, recv, s.sec * 10**9 + s.nanosec))
    return out


def measure(msgs, resource_rows, clk_tck: int) -> dict:
    inputs = [(r, s) for t, r, s in msgs if t in INPUTS]
    res = {'topics': {}}
    lat_all = []
    for topic in OUTPUTS:
        outs = [(r, s) for t, r, s in msgs if t == topic]
        lat, unmatched = analyze.match_latency(inputs, outs)
        lat_all.append(lat)
        res['topics'][topic] = {'latency': analyze.latency_summary(lat), 'unmatched': unmatched,
                                'rate': analyze.rate_summary([r for r, _ in outs])}
    import numpy as np
    res['latency'] = analyze.latency_summary(np.concatenate(lat_all))
    res['rate'] = min((v['rate'] for v in res['topics'].values()),
                      key=lambda r: float('inf') if r['mean_hz'] is None else r['mean_hz'])
    res['resources'] = analyze.resource_summary(resource_rows, clk_tck)
    res['verdict'] = analyze.verdict(res['latency'], res['rate'], res['resources'])
    return res


def _f(v, fmt='{:.1f}'):
    return '—' if v is None else fmt.format(v)


def ensure_utf8_stdout() -> None:
    """Console codepage must not crash the report on its own ✅/❌ marks (#126, same
    class of bug as #124 in tools/submission/check_submission.py)."""
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')


def table(res: dict) -> str:
    lat, rate, r, v = res['latency'], res['rate'], res['resources'], res['verdict']
    mark = {True: '✅', False: '❌', None: '—'}
    rows = [
        ('задержка p50, мс', _f(lat['p50_ms']), '', ''),
        ('задержка p99, мс', _f(lat['p99_ms']), f'≤ {analyze.LATENCY_LIMIT_MS:g}', v['latency_p99']),
        ('задержка max, мс', _f(lat['max_ms']), f'≤ {analyze.LATENCY_PEAK_LIMIT_MS:g}', v['latency_peak']),
        ('частота /result/*, Гц (меньшая из двух)', _f(rate['mean_hz']), f'≥ {analyze.RATE_LIMIT_HZ:g}', v['rate']),
        ('CPU max, ядер', _f(r['cpu_max_cores'], '{:.2f}'), f'≤ {analyze.CORES_LIMIT:g}', v['cpu']),
        ('CPU среднее, ядер', _f(r['cpu_mean_cores'], '{:.2f}'), '', ''),
        ('RSS пик, МБ', _f(r['rss_peak_mb']), f'≤ {analyze.RSS_LIMIT_MB:g}', v['rss']),
        ('рост RSS, МБ/мин', _f(r['rss_growth_mb_min'], '{:.2f}'),
         f'≤ {analyze.RSS_GROWTH_LIMIT_MB_MIN:g}', v['rss_growth']),
    ]
    lines = ['| Показатель | Значение | Порог | |', '|---|---|---|---|']
    for name, val, lim, ok in rows:
        lines.append(f'| {name} | {val} | {lim} | {mark.get(ok, "") if lim else ""} |')
    return '\n'.join(lines)


def main(argv=None) -> int:
    ensure_utf8_stdout()
    ap = argparse.ArgumentParser(prog='tram_stand')
    ap.add_argument('stand_dir', type=Path, help='out/stand/<bag>: record/, resources.csv')
    args = ap.parse_args(argv)
    record = args.stand_dir / 'record'
    if not record.exists():
        print(f'нет записи {record}: нода не запускалась (стенд проверил только сборку)', file=sys.stderr)
        return 2
    res_file = args.stand_dir / 'resources.csv'
    rows = analyze.parse_samples(res_file.read_text(encoding='utf-8')) if res_file.exists() else []
    res = measure(read_record(record), rows, os.sysconf('SC_CLK_TCK'))
    (args.stand_dir / 'stand.json').write_text(json.dumps(res, indent=2, ensure_ascii=False), encoding='utf-8')
    print(table(res))
    return 0
