#!/usr/bin/env python3
"""fifo_stats.py A.vcd [B.vcd]: per HLS FIFO of vcd_fifo.tcl's dump, % of the middle half of the run it was full
(full_n = 0: its producer is blocked) and empty (empty_n = 0); with two dumps, side by side, sorted by the change in
% full. 2026-10-28 (Loom both-way limit: which stack stage slows WRITE_ONLY packets when the node also receives)."""
import sys


def load(path):
    ids, val, last, acc, t, tend = {}, {}, {}, {}, 0, 0
    events = []
    with open(path) as f:
        for line in f:
            if line.startswith('$var'):
                p = line.split()
                ids[p[3]] = p[4]
            elif line.startswith('#'):
                t = int(line[1:])
                tend = t
            elif line and line[0] in '01xz' and line[1:].strip() in ids:
                events.append((t, line[1:].strip(), line[0]))
    return ids, events, tend


def stats(path):
    ids, events, tend = load(path)
    t0, t1 = tend // 4, 3 * tend // 4
    cur = {i: 'x' for i in ids}
    low = {i: 0 for i in ids}
    tp = 0
    for t, i, v in events + [(t1, None, None)]:
        a, b = max(tp, t0), min(t, t1)
        if b > a:
            for k, w in cur.items():
                if w == '0':
                    low[k] += b - a
        tp = t
        if i is None or t >= t1:
            break
        cur[i] = v
    span = t1 - t0
    return {ids[i]: 100.0 * low[i] / span for i in ids}


a = stats(sys.argv[1])
b = stats(sys.argv[2]) if len(sys.argv) > 2 else {}
names = sorted({n[:-7] if n.endswith('_full_n') else n[:-8] for n in a})
rows = []
for n in names:
    fa, ea = a.get(n + '_full_n', 0), a.get(n + '_empty_n', 0)
    fb, eb = b.get(n + '_full_n', 0), b.get(n + '_empty_n', 0)
    rows.append((fa - fb, n, fa, ea, fb, eb))
rows.sort(key=lambda r: -abs(r[0]))
print(f'{"fifo":45s} {"A full%":>8} {"A empty%":>9} {"B full%":>8} {"B empty%":>9}')
for d, n, fa, ea, fb, eb in rows:
    if fa > 1 or fb > 1 or abs(ea - eb) > 5:
        print(f'{n:45s} {fa:8.1f} {ea:9.1f} {fb:8.1f} {eb:9.1f}')
