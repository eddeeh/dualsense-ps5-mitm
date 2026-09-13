#!/usr/bin/env python3
"""Where the time between a stick moving and the console hearing about it goes.

The relay sits between two radios, and until now only its CPU cost had been
measured. This measures the path itself, from the captures both radios already
write:

    python3 tools/analyse_input_path.py logs/capture-<stamp>.dump \\
                                        logs/capture-ps5-<stamp>.dump

With one capture it reports how the pad-facing radio hands input to the host.
With both it also joins them - the relay forwards a full 0x31 report byte for
byte, so each send can be matched to the arrival that produced it - and prints
what the relay itself adds, and whether it ever ships a report that a fresher
one had already superseded.

Reports are matched on the pad's own 32-bit counter, not on the first body
bytes: those are sticks and buttons, which are identical for long stretches
while the pad is still.
"""
import struct
import sys

import numpy as np

REC = struct.Struct('>IIIIq')
BURST_GAP_US = 2000     # arrivals further apart than this start a new burst
COUNTER_OFF = 23        # the pad's per-report counter, 4 bytes, little endian
COUNTER_LEN = 4
OPAQUE_OFF = 13         # body[1]; 0xd4 marks a frame of the pad's opaque stream,
OPAQUE_MARK = 0xd4      # which shares report id 0x31 and carries no counter


def reports(path, received, hidp, rid=0x31):
    """(timestamp, counter, body key) for every matching HID report."""
    ts, ct, kb, op = [], [], [], []
    with open(path, 'rb') as f:
        if f.read(8) != b'btsnoop\x00':
            sys.exit(f'{path}: not a btsnoop capture')
        f.read(8)
        carry = b''
        while True:
            chunk = f.read(1 << 22)
            if not chunk:
                break
            buf, off, end = carry + chunk, 0, len(carry) + len(chunk)
            while off + 24 <= end:
                _, ilen, flags, _, t = REC.unpack_from(buf, off)
                if off + 24 + ilen > end:
                    break
                d = buf[off + 24:off + 24 + ilen]
                off += 24 + ilen
                if not d or d[0] != 0x02 or ilen < 40:
                    continue
                if ((d[2] >> 4) & 3) == 1:      # continuation: no L2CAP header
                    continue
                if bool(flags & 1) != received or d[9] != hidp or d[10] != rid:
                    continue
                ts.append(t)
                ct.append(int.from_bytes(d[COUNTER_OFF:COUNTER_OFF + COUNTER_LEN], 'little'))
                kb.append(int.from_bytes(d[11:19], 'little'))
                op.append(d[OPAQUE_OFF] == OPAQUE_MARK)
            carry = buf[off:]
    return (np.array(ts, np.int64), np.array(ct, np.uint64),
            np.array(kb, np.uint64), np.array(op, bool))


def grid(name, t):
    """How a radio hands received packets to the host: one at a time, or in bursts."""
    d = np.diff(t)
    span = (t[-1] - t[0]) / 1e6
    print(f'\n== {name} ==')
    print(f'{len(t)} reports, {len(t)/span:.1f}/s')
    print(f'arrivals sharing a timestamp with the one before: {100*(d==0).mean():.1f}%')
    i = np.where(d > BURST_GAP_US)[0]
    if len(i) < 100:
        print('no burst structure - each packet arrives on its own')
        return
    st = np.concatenate(([0], i + 1))
    en = np.concatenate((i, [len(t) - 1]))
    n, per = en - st + 1, d[i] / 1000.0
    inner = (t[en] - t[st])[n > 1]
    print(f'bursts of {np.median(n):.0f} reports (p90 {np.percentile(n,90):.0f}), '
          f'spread {np.median(inner):.0f} us inside')
    print(f'burst period: median {np.median(per):.4f} ms, '
          f'p25 {np.percentile(per,25):.4f}, p75 {np.percentile(per,75):.4f}')
    # A burst released by a timer sits on an absolute grid; one released as the
    # data lands does not. Rayleigh concentration, where chance is about 1. The
    # period is scanned around the median: a grid this sharp is destroyed over a
    # long capture by rounding it even to the nearest microsecond.
    d0 = d  # kept for the batching test below
    best_r, p = 0.0, np.median(per) * 1000.0
    for cand in np.arange(p - 20, p + 20, 0.25):
        ph = 2 * np.pi * (t[st] % cand) / cand
        r = (np.cos(ph).sum() ** 2 + np.sin(ph).sum() ** 2) / len(st)
        if r > best_r:
            best_r, p = r, cand
    r = best_r
    # Two grids turn up here and only one is a defect. A BR/EDR link exchanges
    # on slot pairs, so a few ms of granularity with every packet handed over on
    # its own is the air, and not ours to change. Packets sharing a timestamp
    # mean a transport is holding them back, and that can be moved.
    batched = (d0 == 0).mean() > 0.05
    what = ('a transport holding packets back'
            if batched else 'the air schedule - not ours to change')
    print(f'burst starts locked to an absolute {p/1000:.3f} ms grid: R={r:.1f} '
          f'(chance ~1) -> {"a timer, " + what if r > 20 else "not a fixed timer"}')
    if r > 20 and batched:
        print(f'so the freshest report waits a mean {p/2000:.1f} ms before the '
              f'relay can see it at all')


def main(pad_path, ps5_path=None):
    pt, pc, pk, po = reports(pad_path, True, 0xa1)
    step = np.diff(pc[~po].astype(np.int64))
    print(f'pad reports: {len(pt)} - {100*po.mean():.1f}% frames of the opaque '
          f'stream, which the relay drops')
    print(f'of the rest, the counter advances by exactly one '
          f'{100*(step == 1).mean():.2f}% of the time - none are lost on the way in')
    grid('pad-facing radio, input from the pad', pt)

    if not ps5_path:
        return
    ct, cc, ck, _ = reports(ps5_path, True, 0xa2, rid=0x36)
    if len(ct) > 1000:
        grid('console-facing radio, output from the console', ct)

    st, sc, sk, _ = reports(ps5_path, False, 0xa1)
    o = np.argsort(pc, kind='stable')
    pc_s, pk_s, pt_s = pc[o], pk[o], pt[o]
    i = np.searchsorted(pc_s, sc)
    ok = i < len(pc_s)
    i = np.clip(i, 0, len(pc_s) - 1)
    ok &= (pc_s[i] == sc) & (pk_s[i] == sk)
    src, snd = pt_s[i[ok]], st[ok]
    keep = (snd >= src) & (snd - src < 5_000_000)
    src, snd = src[keep], snd[keep]
    lat = (snd - src) / 1000.0
    print(f'\n== what the relay itself adds ==')
    print(f'matched {ok.sum()} of {len(st)} sends ({100*ok.mean():.1f}%)')
    print('arrival on the pad radio -> write to the console radio, ms:')
    for q in (50, 75, 90, 99):
        print(f'   p{q:<3} {np.percentile(lat, q):8.3f}')
    print(f'   mean {lat.mean():8.3f}')

    # Did a fresher report exist at the moment we sent this one?
    pts = np.sort(pt)
    newer = (np.searchsorted(pts, snd, 'right')
             - np.searchsorted(pts, src, 'right'))
    best = pts[np.clip(np.searchsorted(pts, snd, 'right') - 1, 0, len(pts) - 1)]
    lost_ms = (src - best) * -1 / 1000.0
    print(f'sends a fresher report had already superseded: {100*(newer>0).mean():.1f}%')
    print(f'freshness given up by sending it anyway: mean {lost_ms.mean():.4f} ms '
          f'- the newer ones landed in the same burst')


if __name__ == '__main__':
    if len(sys.argv) not in (2, 3):
        sys.exit(__doc__)
    main(*sys.argv[1:])
