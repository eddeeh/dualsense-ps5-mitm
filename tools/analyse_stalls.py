#!/usr/bin/env python3
"""Find the holes in the console link and say what causes them.

The relay's own 5-second counters are too coarse to see a 135 ms hole, so every
question about the gaps has to be asked of the console-side capture. This walks
one and prints the numbers PERFORMANCE.md quotes, in the order that document
makes its argument:

    python3 tools/analyse_stalls.py logs/capture-ps5-YYYYMMDD-HHMMSS.dump

A stall is a gap between two consecutive ACL packets we wrote to the play link.
Everything else here is measured against those gaps - what arrives inside them,
when the controller gives its buffers back, and which console report id is on
the air when it happens.
"""
import struct
import sys

import numpy as np

REC = struct.Struct('>IIIIq')
STALL_MS = 100          # a gap this long is a stall; the distribution is bimodal
                        # either side of it, so the exact cut changes nothing
AUDIO_GAP_MS = 150      # 0x39 packets further apart than this start a new episode

HID_REPORT_OFFSET = 10  # H4 type, handle, ACL len, L2CAP len, CID, HIDP header


def read(path):
    """Timestamps of what crossed the play link, split by direction and kind."""
    tx, rx, rx_id, ncp_t, ncp_n = [], [], [], [], []
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
                if not d:
                    continue
                if d[0] == 0x02 and ilen >= 3:
                    if (d[1] | (d[2] << 8)) & 0x0fff != PLAY_HANDLE:
                        continue
                    if not flags & 1:
                        tx.append(t)
                    elif ilen > HID_REPORT_OFFSET:
                        rx.append(t)
                        rx_id.append(d[HID_REPORT_OFFSET])
                elif d[0] == 0x04 and ilen >= 5 and d[1] == 0x13:
                    for k in range(d[3]):
                        o = 4 + k * 4
                        if o + 4 <= len(d) and (d[o] | (d[o + 1] << 8)) == PLAY_HANDLE:
                            ncp_t.append(t)
                            ncp_n.append(d[o + 2] | (d[o + 3] << 8))
            carry = buf[off:]
    return (np.array(tx, np.int64), np.array(rx, np.int64),
            np.array(rx_id, np.uint8), np.array(ncp_t, np.int64),
            np.array(ncp_n, np.uint16))


def play_handle(path):
    """The play link is the handle that carries the input stream, by volume."""
    seen = {}
    with open(path, 'rb') as f:
        f.read(16)
        buf = f.read(1 << 24)
        off = 0
        while off + 24 <= len(buf):
            _, ilen, _, _, _ = REC.unpack_from(buf, off)
            if off + 24 + ilen > len(buf):
                break
            d = buf[off + 24:off + 24 + ilen]
            off += 24 + ilen
            if d and d[0] == 0x02 and ilen >= 3:
                h = (d[1] | (d[2] << 8)) & 0x0fff
                seen[h] = seen.get(h, 0) + 1
    return max(seen, key=seen.get)


def audio_only(rx, rx_id, span):
    """How much haptic audio ran, for a session with no stalls to attribute."""
    t39 = rx[rx_id == 0x39]
    if len(t39) < 3:
        print(f'0x39 haptic audio: {len(t39)} packets in {span:.0f} s - '
              f'essentially none, so this session cannot test the stall rate')
        return
    g = np.diff(t39) / 1000.0
    brk = np.where(g > AUDIO_GAP_MS)[0]
    a = np.concatenate(([0], brk + 1))
    b = np.concatenate((brk, [len(t39) - 1]))
    ep = (t39[b] - t39[a]) / 1e6
    keep = ep > 0.05
    print(f'0x39 haptic audio: {len(t39)} packets, {int(keep.sum())} episodes, '
          f'{ep[keep].sum():.1f} s = {100*ep[keep].sum()/span:.1f}% of the session')


def pct(a, *qs):
    return tuple(np.percentile(a, q) for q in qs)


def main(path):
    global PLAY_HANDLE
    PLAY_HANDLE = play_handle(path)
    tx, rx, rx_id, ncp_t, ncp_n = read(path)
    span = (tx[-1] - tx[0]) / 1e6
    print(f'play link handle {PLAY_HANDLE}, {span/3600:.2f} h, '
          f'{len(tx)} sent ({len(tx)/span:.1f}/s), {len(rx)} received')

    dt = np.diff(tx) / 1000.0
    i = np.where(dt > STALL_MS)[0]
    s0, s1, dur = tx[i], tx[i + 1], dt[i]
    print(f'\n== gaps ==\nmedian {np.median(dt):.2f} ms, '
          f'p99 {np.percentile(dt,99):.2f}, p99.9 {np.percentile(dt,99.9):.2f}')
    print(f'{len(i)} stalls >{STALL_MS} ms ({len(i)/span:.3f}/s), '
          f'{100*dur.sum()/dt.sum():.2f}% of the session')
    if len(i) == 0:
        print('no stalls at all - if the session is short, check below whether '
              'any haptic audio ran: without it the rate is low anyway')
        audio_only(rx, rx_id, span)
        return
    print(f'duration: median {np.median(dur):.1f} ms, '
          f'{100*((dur>100)&(dur<150)).mean():.1f}% inside 100-150 ms')

    # Bursts. A stall on its own is not what a player feels; a run of them is.
    iv = np.diff(s0 / 1000.0)
    brk = np.where(iv >= 250)[0]
    a, b = np.concatenate(([0], brk + 1)), np.concatenate((brk, [len(s0) - 1]))
    n = b - a + 1
    burst = (s1[b] - s0[a]) / 1000.0
    stalled = np.array([dur[x:y + 1].sum() for x, y in zip(a, b)])
    m = n > 1
    print(f'\n== bursts ==\n{m.sum()} runs of 2+ stalls, holding {n[m].sum()} of '
          f'{len(s0)} stalls ({100*n[m].sum()/len(s0):.0f}%)')
    if m.sum():
        print(f'span: median {np.median(burst[m]):.0f} ms, p90 {pct(burst[m],90)[0]:.0f}, '
              f'max {burst[m].max():.0f}')
        print(f'dead inside a burst: median {100*np.median(stalled[m]/burst[m]):.1f}%')
    near = np.where(iv < 250)[0]
    if len(near):
        between = i[near + 1] - i[near]
        print(f'packets between back-to-back stalls: median {np.median(between):.0f} '
              f'({100*(between==2).mean():.1f}% are exactly 2)')

    # Who is on the air inside a stall, and where.
    lo, hi = np.searchsorted(rx, s0), np.searchsorted(rx, s1)
    inside, base = int((hi - lo).sum()), len(rx) / span
    print(f'\n== inside a stall ==\nstalls with no console traffic at all: '
          f'{int(((hi-lo)==0).sum())} of {len(s0)}')
    print(f'console packets inside stalls: {inside}, against {base*dur.sum()/1000:.0f} '
          f'at the baseline rate - {100*inside/(base*dur.sum()/1000):.0f}% of it')
    ph = np.concatenate([(rx[x:y] - t0) / (t1 - t0)
                         for x, y, t0, t1 in zip(lo, hi, s0, s1) if y > x])
    ids = np.concatenate([rx_id[x:y] for x, y in zip(lo, hi) if y > x])
    for name, sel in (('start', ph < 0.1), ('middle', (ph >= 0.4) & (ph < 0.6)),
                      ('end', ph > 0.9)):
        u, c = np.unique(ids[sel], return_counts=True)
        top = sorted(zip(u, c), key=lambda x: -x[1])[:2]
        share = '  '.join(f'0x{k:02x} {100*v/sel.sum():.0f}%' for k, v in top)
        print(f'  {name:7s} n={sel.sum():5d}   {share}')

    # Credits. The stall ends when the controller gives buffers back.
    u, c = np.unique(ncp_n, return_counts=True)
    print(f'\n== credits ==\nNumber of Completed Packets: '
          + ', '.join(f'{k} packet(s) x{v}' for k, v in zip(u, c)))
    j = np.searchsorted(ncp_t, s0)
    ends = ncp_t[np.clip(j, 0, len(ncp_t) - 1)]
    ends = ends[(j < len(ncp_t)) & (ends < s1)]
    print(f'completions inside a stall: {len(ends)} for {len(s0)} stalls '
          f'- the one that ends it')
    k = np.clip(np.searchsorted(rx, ends) - 1, 0, len(rx) - 1)
    print(f'lag from the console packet to that completion: '
          f'median {np.median((ends-rx[k])/1000.0):.2f} ms')

    # The cause.
    print('\n== report ids ==')
    u, c = np.unique(rx_id, return_counts=True)
    for k, v in sorted(zip(u, c), key=lambda x: -x[1])[:3]:
        t = rx[rx_id == k]
        g = dt[np.clip(np.searchsorted(tx, t), 1, len(tx) - 1) - 1]
        print(f'  0x{k:02x}: {100*v/len(rx_id):5.1f}% of console traffic, '
              f'gap spanning it median {np.median(g):6.1f} ms, '
              f'>{STALL_MS} ms in {100*(g>STALL_MS).mean():.1f}% of cases')

    t39 = rx[rx_id == 0x39]
    if len(t39) <= 2:
        print(f'\n== haptic audio ==\n{len(t39)} 0x39 packets in {span:.0f} s - '
              f'none ran, so this session cannot test the stall rate')
    if len(t39) > 2:
        k = np.clip(np.searchsorted(t39, s0) - 1, 0, len(t39) - 1)
        before = (s0 - t39[k]) / 1000.0
        print(f'\nstalls with a 0x39 in the 5 ms before them: '
              f'{100*(before<5).mean():.1f}% (within 20 ms: {100*(before<20).mean():.1f}%)')
        g = np.diff(t39) / 1000.0
        brk = np.where(g > AUDIO_GAP_MS)[0]
        a, b = np.concatenate(([0], brk + 1)), np.concatenate((brk, [len(t39) - 1]))
        ep = (t39[b] - t39[a]) / 1e6
        keep = ep > 0.05
        on = ep[keep].sum()
        lo39, hi39 = t39[a[keep]], t39[b[keep]]
        j = np.clip(np.searchsorted(hi39, s0), 0, len(lo39) - 1)
        during = s0 >= lo39[j] - 70000
        print(f'\n== haptic audio ==\n{keep.sum()} episodes of 0x39, {on:.0f} s '
              f'= {100*on/span:.1f}% of the session')
        print(f'episode length: median {np.median(ep[keep]):.2f} s, '
              f'p90 {pct(ep[keep],90)[0]:.2f}, max {ep[keep].max():.2f}')
        if on < 20 or during.sum() < 10:
            print('too little audio ran to compare stall rates against it - '
                  'this session cannot test the holes')
        else:
            print(f'stall rate with audio    {during.sum()/on:5.2f}/s')
            print(f'stall rate without audio {(~during).sum()/(span-on):5.2f}/s')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
