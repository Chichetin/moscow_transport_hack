"""Samples CPU ticks and RSS of the node processes from /proc once a second (stdlib only).

Runs inside the stand container: python3 sampler.py <cmdline-substring> > resources.csv
Columns: t (epoch s), pid, ticks (utime+stime), rss_kb. Stops on SIGINT/SIGTERM.
"""
import os
import signal
import sys
import time

PERIOD_S = 1.0


def read_proc(pid):
    with open(f'/proc/{pid}/stat') as f:
        stat = f.read()
    fields = stat[stat.rindex(')') + 2:].split()    # after "pid (comm) "
    ticks = int(fields[11]) + int(fields[12])       # utime, stime
    with open(f'/proc/{pid}/statm') as f:
        rss_pages = int(f.read().split()[1])
    return ticks, rss_pages * os.sysconf('SC_PAGE_SIZE') // 1024


def matching(pattern):
    me = os.getpid()
    for name in os.listdir('/proc'):
        if name.isdigit() and int(name) != me:
            try:
                with open(f'/proc/{name}/cmdline', 'rb') as f:
                    if pattern.encode() in f.read():
                        yield int(name)
            except OSError:
                pass


def main(pattern):
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    print('t,pid,ticks,rss_kb', flush=True)
    while not stop:
        t = time.time()
        for pid in matching(pattern):
            try:
                ticks, rss = read_proc(pid)
            except (OSError, ValueError, IndexError):
                continue
            print(f'{t:.3f},{pid},{ticks},{rss}')
        sys.stdout.flush()
        time.sleep(PERIOD_S)


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'odometry')
