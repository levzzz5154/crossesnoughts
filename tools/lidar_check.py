#!/usr/bin/env python3
"""Hardware health check for a YDLidar X3 on a serial port.

Pure standard library (termios + select) - no ydlidar SDK, no pyserial, and no
particular Python version required, so it runs even when the SWIG module is
broken or built for another interpreter.

Speaks the triangle protocol directly (YDLidar-SDK/core/common/ydlidar_protocol.h):

    packet = AA 55 | ct | count | firstAngle:u16 | lastAngle:u16 | cs:u16 | u16[count]
    angle  = (raw >> 1) / 64.0        degrees   (CYdLidar.cpp:659)
    range  = raw / 4000.0             metres    (CYdLidar.cpp:685)
    cs     = XOR of the four header u16s, then XOR of every node u16
             (YDlidarDriver.cpp:1472)

Usage:
    python3 tools/lidar_check.py [PORT] [BAUD] [SECONDS]
    python3 tools/lidar_check.py /dev/ttyUSB0 115200 4
"""
from __future__ import annotations

import math
import os
import select
import sys
import termios
import time

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
SECONDS = float(sys.argv[3]) if len(sys.argv) > 3 else 4.0

TRI_PACKHEADSIZE = 10
TRI_PACKMAXNODES = 80


def log(*a):
    print(*a, flush=True)


def open_port(port: str, baud: int) -> int:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0  # iflag
    attrs[1] = 0  # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag
    attrs[3] = 0  # lflag
    attrs[4] = attrs[5] = getattr(termios, f"B{baud}")
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def parse(buf: bytes):
    """Yield (ct, first_deg, last_deg, ranges_m); return (consumed, bad_cs)."""
    out, i, n, bad = [], 0, len(buf), 0
    while True:
        j = buf.find(b"\xaa\x55", i)
        if j < 0:
            return out, n, bad
        if j + TRI_PACKHEADSIZE > n:
            return out, j, bad
        count = buf[j + 3]
        if count == 0 or count > TRI_PACKMAXNODES:
            i = j + 1
            continue
        need = TRI_PACKHEADSIZE + count * 2
        if j + need > n:
            return out, j, bad
        pkt = buf[j:j + need]
        w = [pkt[k] | (pkt[k + 1] << 8) for k in range(0, need, 2)]
        ccs = 0
        for x in w[:4]:
            ccs ^= x
        for x in w[5:5 + count]:
            ccs ^= x
        if ccs != w[4]:
            bad += 1
            i = j + 1
            continue
        out.append((pkt[2], (w[2] >> 1) / 64.0, (w[3] >> 1) / 64.0,
                    [x / 4000.0 for x in w[5:5 + count]]))
        i = j + need


def median(xs):
    s = sorted(xs)
    return s[len(s) // 2] if s else float("nan")


def main() -> int:
    fd = open_port(PORT, BAUD)
    buf = bytearray()
    pts: list[tuple[float, float]] = []
    revs: list[tuple[int, float, float]] = []  # points, duration, angular span
    good = bad = 0
    cur_pts = 0
    cur_span = 0.0
    cur_t0 = None
    prev_first = None
    started = False
    t0 = time.time()

    while time.time() - t0 < SECONDS:
        r, _, _ = select.select([fd], [], [], 0.05)
        if not r:
            continue
        buf += os.read(fd, 65536)
        pkts, used, nbad = parse(bytes(buf))
        del buf[:used]
        bad += nbad
        for ct, first, last, ranges in pkts:
            good += 1
            now = time.time()
            adv = 0.0
            if prev_first is not None:
                adv = (first - prev_first) % 360.0
                if adv > 180.0:
                    adv -= 360.0
            if ct & 0x01:  # zero packet -> start of a new revolution
                if started:
                    revs.append((cur_pts, now - cur_t0, cur_span))
                started = True
                cur_pts, cur_span, cur_t0 = 0, 0.0, now
            cur_pts += len(ranges)
            cur_span += adv
            k = len(ranges)
            step = (((last - first) % 360.0) / (k - 1)) if k > 1 else 0.0
            for m in range(k):
                pts.append((first + step * m, ranges[m]))
            prev_first = first
    os.close(fd)
    elapsed = time.time() - t0

    log(f"port              : {PORT} @ {BAUD} baud")
    log(f"capture           : {elapsed:.2f}s")
    if not good:
        log("")
        log("NO VALID PACKETS - check the port, the baud rate, and power.")
        return 1
    log(f"packets ok / bad  : {good} / {bad}")
    log(f"checksum ok rate  : {100.0 * good / max(good + bad, 1):.2f}%")
    log(f"points            : {len(pts)} (~{len(pts) / elapsed:.0f} pts/s)")

    if len(revs) > 2:
        r = revs[1:]
        per, dur, span = (median([x[0] for x in r]),
                          median([x[1] for x in r]),
                          median([x[2] for x in r]))
        log(f"revolutions       : {len(r)}")
        log(f"period            : {dur * 1000:.1f} ms -> {1.0 / max(dur, 1e-9):.2f} Hz")
        log(f"points / rev      : {per:.0f}  (~{360.0 / max(per, 1):.2f} deg/point)")
        log(f"angular span / rev: {span:.1f} deg (near 360 = no dropped packets)")

    ok = [r for _, r in pts if r > 0]
    if ok:
        s = sorted(ok)
        p = lambda q: s[min(len(s) - 1, int(q * len(s)))]  # noqa: E731
        log(f"valid ranges      : {len(ok)}/{len(pts)} "
            f"({100.0 * len(ok) / len(pts):.1f}%)")
        log(f"range p5/med/p95  : {p(0.05):.3f} / {p(0.50):.3f} / {p(0.95):.3f} m")

        log("")
        log("coverage by 30 deg sector:")
        for sec in range(12):
            lo, hi = sec * 30, sec * 30 + 30
            sel = [r for a, r in pts if lo <= (a % 360) < hi and r > 0]
            n_all = sum(1 for a, _ in pts if lo <= (a % 360) < hi)
            if not n_all:
                continue
            mean = sum(sel) / len(sel) if sel else 0.0
            log(f"  {lo:>3}-{hi:>3} deg : {len(sel):>5}/{n_all:<5} mean {mean:5.2f} m "
               f"|{'#' * int(28 * len(sel) / n_all)}")

        full = max(2.0, p(0.95))
        W, H = 63, 27
        grid = [[" "] * W for _ in range(H)]
        cx, cy = W // 2, H // 2
        for a, r in pts:
            if r <= 0:
                continue
            rad = min(r / full, 1.0) * (min(cx, cy) - 1)
            x = int(round(cx + rad * math.cos(math.radians(a))))
            y = int(round(cy - rad * math.sin(math.radians(a)) * 0.5))
            if 0 <= x < W and 0 <= y < H:
                grid[y][x] = "*"
        grid[cy][cx] = "+"
        log("")
        log(f"polar view, {full:.1f} m full scale (+ = lidar, 0 deg = right):")
        for row in grid:
            log("  " + "".join(row))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
