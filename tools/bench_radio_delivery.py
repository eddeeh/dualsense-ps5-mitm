#!/usr/bin/env python3
"""Measure how an adapter hands received ACL packets to the host.

`analyse_input_path.py` found that input from the pad arrives in bursts on a
fixed 8 ms grid, and that the onboard radio shows nothing of the kind. That
comparison was between two radios carrying different traffic at different rates,
which is not an experiment. This is: it drives a chosen adapter with an L2CAP
stream from another adapter at a chosen rate, captures the receiving side, and
reports the delivery pattern. No pad, no console, nothing paired, and no stored
key is touched.

    sudo python3 tools/bench_radio_delivery.py --recv hci1 --send hci0

Both adapters must be free - stop the relay first. They are left down
afterwards, which is the state scripts/run.sh expects.
"""
import argparse
import os
import re
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

import numpy as np

REC = struct.Struct('>IIIIq')
SOL_BLUETOOTH, BT_SECURITY, BT_SECURITY_LOW = 274, 4, 1
PSM = 0x1001
MONITOR_ACL_RX = 5          # btmon's opcode for a received ACL packet
BURST_GAP_US = 2000


def hci(*args):
    subprocess.run(['hciconfig', *args], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def addr_of(dev):
    out = subprocess.run(['hciconfig', dev], capture_output=True, text=True).stdout
    m = re.search(r'BD Address: ([0-9A-F:]{17})', out)
    if not m:
        sys.exit(f'{dev}: no such adapter')
    return m.group(1)


def blast(recv_addr, send_addr, rate, secs):
    """An L2CAP stream from one adapter to the other, paced at `rate`/s."""
    srv = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_SEQPACKET, socket.BTPROTO_L2CAP)
    srv.setsockopt(SOL_BLUETOOTH, BT_SECURITY, BT_SECURITY_LOW)
    srv.bind((recv_addr, PSM))
    srv.listen(1)
    seen = [0]

    def serve():
        c, _ = srv.accept()
        while c.recv(1024):
            seen[0] += 1

    threading.Thread(target=serve, daemon=True).start()
    time.sleep(0.3)
    cli = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_SEQPACKET, socket.BTPROTO_L2CAP)
    cli.setsockopt(SOL_BLUETOOTH, BT_SECURITY, BT_SECURITY_LOW)
    cli.bind((send_addr, 0))
    cli.connect((recv_addr, PSM))
    payload = bytes(range(88))
    n, t0, step = 0, time.perf_counter(), 1.0 / rate
    while (now := time.perf_counter() - t0) < secs:
        if now < n * step:
            time.sleep(min(n * step - now, 0.002))
            continue
        try:
            cli.send(payload)
        except OSError as e:
            print(f'send stopped after {n}: {e}')
            break
        n += 1
    cli.close()
    time.sleep(0.5)
    return n, seen[0]


def arrivals(path):
    t = []
    with open(path, 'rb') as f:
        f.read(16)
        buf, off = f.read(), 0
        while off + 24 <= len(buf):
            _, ilen, flags, _, ts = REC.unpack_from(buf, off)
            off += 24 + ilen
            if ilen >= 50 and (flags & 0xffff) == MONITOR_ACL_RX:
                t.append(ts)
    return np.array(t, np.int64)


def report(label, t):
    if len(t) < 500:
        print(f'{label}: only {len(t)} packets arrived - nothing to measure')
        return
    d = np.diff(t)
    span = (t[-1] - t[0]) / 1e6
    print(f'\n== {label} ==')
    print(f'   {len(t)} packets in, {len(t)/span:.1f}/s')
    print(f'   arrivals sharing a timestamp with the one before: {100*(d==0).mean():.1f}%')
    print(f'   inter-arrival p25/p50/p75/p90 us: '
          + ' / '.join(f'{np.percentile(d,q):.0f}' for q in (25, 50, 75, 90)))
    i = np.where(d > BURST_GAP_US)[0]
    if len(i) < 50:
        print('   no burst structure - each packet is handed over on its own')
        return
    st = np.concatenate(([0], i + 1))
    en = np.concatenate((i, [len(t) - 1]))
    n, per = en - st + 1, d[i] / 1000.0
    print(f'   bursts of {np.median(n):.0f} (p90 {np.percentile(n,90):.0f}), '
          f'period median {np.median(per):.4f} ms')
    best, p = 0.0, np.median(per) * 1000
    for cand in np.arange(p - 30, p + 30, 0.25):
        ph = 2 * np.pi * (t[st] % cand) / cand
        r = (np.cos(ph).sum() ** 2 + np.sin(ph).sum() ** 2) / len(st)
        if r > best:
            best, p = r, cand
    print(f'   locked to a {p/1000:.3f} ms grid: R={best:.1f} (chance ~1) -> '
          f'{"a timer holds them" if best > 20 else "not a timer"}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--recv', required=True, help='adapter under test, e.g. hci1')
    ap.add_argument('--send', required=True, help='adapter to drive it from')
    ap.add_argument('--rate', type=float, default=600.0, help='packets/s (pad does ~600)')
    ap.add_argument('--secs', type=float, default=20.0)
    ap.add_argument('--out', default='/tmp/radio_delivery.dump')
    a = ap.parse_args()
    if os.geteuid() != 0:
        sys.exit('needs root: the capture and the raw sockets both do')

    recv_addr, send_addr = addr_of(a.recv), addr_of(a.send)
    for d in (a.recv, a.send):
        hci(d, 'up')
    hci(a.recv, 'piscan')
    time.sleep(1)

    mon = subprocess.Popen(['btmon', '-i', a.recv, '-w', a.out],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)
    try:
        sent, got = blast(recv_addr, send_addr, a.rate, a.secs)
        print(f'{a.send} ({send_addr}) -> {a.recv} ({recv_addr}): '
              f'sent {sent}, delivered {got}')
    finally:
        mon.send_signal(signal.SIGINT)
        mon.wait(timeout=10)
        hci(a.recv, 'noscan')
        for d in (a.recv, a.send):
            hci(d, 'down')

    report(f'{a.recv} receiving', arrivals(a.out))


if __name__ == '__main__':
    main()
