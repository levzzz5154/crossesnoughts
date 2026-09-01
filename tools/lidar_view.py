#!/usr/bin/env python3
"""Minimal live visualisation of what the YDLidar X3 sees.

Left panel  : polar radar view (lidar at centre, 0 deg = up = "forward").
Right panel : top-down Cartesian map, same data, same orientation.
Points are coloured by distance; older revolutions fade out.

Reads the triangle protocol straight off the serial port, so it needs no
ydlidar SDK and no pyserial - just numpy + pygame and any Python 3.

    /home/levzzz/miniconda3/bin/python tools/lidar_view.py /dev/ttyUSB0 115200

Auto-zooms to fill the panel by default.

Keys:  Q / ESC quit   F flip   T trails   [ ] zoom (manual)   A auto-zoom
"""
from __future__ import annotations

import collections
import math
import os
import select
import sys
import termios
import time

import numpy as np
import pygame

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200

PANEL = 520
HUD_H = 46
W, H = PANEL * 2, PANEL + HUD_H
CX, CY = PANEL // 2, PANEL // 2
RAD = PANEL // 2 - 26

TRI_HEAD = 10
TRI_MAX_NODES = 80
HISTORY = 26

BG = (9, 11, 15)
GRID = (38, 46, 58)
GRID_HI = (60, 72, 90)
TEXT = (198, 208, 220)
DIM = (110, 122, 138)

# near -> far
STOPS = np.array([[255, 78, 48],
                  [255, 208, 74],
                  [96, 238, 152],
                  [66, 138, 255]], dtype=np.float32)


def ramp(t: np.ndarray) -> np.ndarray:
    """Distance [0,1] -> RGB uint8."""
    t = np.clip(t, 0.0, 1.0) * (len(STOPS) - 1)
    i = np.minimum(t.astype(int), len(STOPS) - 2)
    f = (t - i)[:, None]
    return (STOPS[i] * (1.0 - f) + STOPS[i + 1] * f).astype(np.uint8)


def open_port(port: str, baud: int) -> int:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = a[1] = a[3] = 0
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    a[4] = a[5] = getattr(termios, f"B{baud}")
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def parse(buf: bytes):
    """Yield (ct, first_deg, last_deg, ranges_m); return (consumed, bad_cs)."""
    out, i, n, bad = [], 0, len(buf), 0
    while True:
        j = buf.find(b"\xaa\x55", i)
        if j < 0:
            return out, n, bad
        if j + TRI_HEAD > n:
            return out, j, bad
        count = buf[j + 3]
        if count == 0 or count > TRI_MAX_NODES:
            i = j + 1
            continue
        need = TRI_HEAD + count * 2
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


def plot(img, xs, ys, cols):
    """Splat 2x2 points with max-combine so bright/newest wins."""
    m = (xs >= 1) & (xs < img.shape[0] - 1) & (ys >= 1) & (ys < img.shape[1] - 1)
    if not m.any():
        return
    xs, ys, cols = xs[m], ys[m], cols[m]
    img[xs, ys] = np.maximum(img[xs, ys], cols)
    img[xs + 1, ys] = np.maximum(img[xs + 1, ys], cols)
    img[xs, ys + 1] = np.maximum(img[xs, ys + 1], cols)
    img[xs + 1, ys + 1] = np.maximum(img[xs + 1, ys + 1], cols)


def axes(ox, oy, scale, flip):
    """Return (xs, ys) plotting helper for one panel."""
    def f(ang_deg, rng):
        a = np.radians(ang_deg)
        s = np.clip(rng / scale, 0.0, 1.0) * RAD
        sx = -1.0 if flip else 1.0
        return (ox + sx * s * np.sin(a)).astype(int), (oy - s * np.cos(a)).astype(int)
    return f


def main() -> int:
    try:
        fd = open_port(PORT, BAUD)
    except OSError as e:
        print(f"cannot open {PORT}: {e}", file=sys.stderr)
        return 1

    pygame.init()
    screen = pygame.display.set_mode((W, H))
    pygame.display.set_caption(f"YDLidar X3  -  {PORT} @ {BAUD}")
    font = pygame.font.SysFont("monospace", 15)
    small = pygame.font.SysFont("monospace", 12)
    clock = pygame.time.Clock()

    buf = bytearray()
    history = collections.deque(maxlen=HISTORY)
    cur_a: list[float] = []
    cur_r: list[float] = []
    partial_a = np.zeros(0)
    partial_r = np.zeros(0)

    good = bad = 0
    rev_times = collections.deque(maxlen=40)
    last_pts = 0
    scale = 4.0
    auto = True
    flip = False
    trails = True
    running = True
    t0 = time.time()

    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_ESCAPE, pygame.K_q):
                    running = False
                elif ev.key == pygame.K_f:
                    flip = not flip
                elif ev.key == pygame.K_t:
                    trails = not trails
                elif ev.key == pygame.K_a:
                    auto = True
                elif ev.key == pygame.K_LEFTBRACKET:
                    auto, scale = False, min(8.0, scale * 1.25)
                elif ev.key == pygame.K_RIGHTBRACKET:
                    auto, scale = False, max(0.4, scale / 1.25)

        # ---- serial ------------------------------------------------------
        r, _, _ = select.select([fd], [], [], 0.002)
        if r:
            buf += os.read(fd, 65536)
            pkts, used, nbad = parse(bytes(buf))
            del buf[:used]
            bad += nbad
            for ct, first, last, ranges in pkts:
                good += 1
                k = len(ranges)
                step = (((last - first) % 360.0) / (k - 1)) if k > 1 else 0.0
                for m in range(k):
                    cur_a.append(first + step * m)
                    cur_r.append(ranges[m])
                if ct & 0x01:  # zero packet: revolution complete
                    a = np.asarray(cur_a, dtype=np.float32)
                    rr = np.asarray(cur_r, dtype=np.float32)
                    if a.size:
                        history.append((a, rr))
                        rev_times.append(time.time())
                        last_pts = a.size
                    cur_a, cur_r = [], []
            if cur_a:
                partial_a = np.asarray(cur_a, dtype=np.float32)
                partial_r = np.asarray(cur_r, dtype=np.float32)

        # ---- newest scan drives the stats and the auto-zoom --------------
        latest_r = partial_r if partial_r.size else None
        if history:
            latest_r = history[-1][1]

        if auto and latest_r is not None and latest_r.size:
            v = latest_r[latest_r > 0]
            if v.size:
                target = float(np.percentile(v, 95)) * 1.20
                scale = float(np.clip(scale + 0.08 * (target - scale), 0.4, 8.0))

        # ---- render ------------------------------------------------------
        img = np.zeros((W, H, 3), dtype=np.uint8)
        img[:, :] = BG

        if trails:
            for i in range(len(history) - 1, -1, -1):
                a, rr = history[i]
                age = len(history) - 1 - i            # 0 = newest
                wgt = max(0.05, 0.86 ** age)
                ok = rr > 0
                if not ok.any():
                    continue
                cols = (ramp(rr[ok] / scale).astype(np.float32) * wgt).astype(np.uint8)
                for ox, oy in ((CX, CY), (PANEL + CX, CY)):
                    plot(img, *axes(ox, oy, scale, flip)(a[ok], rr[ok]), cols)

        if partial_r.size:
            ok = partial_r > 0
            if ok.any():
                cols = ramp(partial_r[ok] / scale)
                for ox, oy in ((CX, CY), (PANEL + CX, CY)):
                    plot(img, *axes(ox, oy, scale, flip)(partial_a[ok], partial_r[ok]), cols)

        pygame.surfarray.blit_array(screen, np.ascontiguousarray(img))

        # ---- overlay -----------------------------------------------------
        for ox, oy in ((CX, CY), (PANEL + CX, CY)):
            step_m = max(0.25, math.ceil(scale / 4.0 / 0.25) * 0.25)
            m = step_m
            while m <= scale + 1e-9:
                pygame.draw.circle(screen, GRID, (ox, oy),
                                   int(m / scale * RAD), 1)
                lbl = small.render(f"{m:g}m", True, DIM)
                screen.blit(lbl, (ox + 3, oy - int(m / scale * RAD) - 13))
                m += step_m
            pygame.draw.circle(screen, GRID_HI, (ox, oy), RAD, 1)
            pygame.draw.line(screen, GRID, (ox - RAD, oy), (ox + RAD, oy), 1)
            pygame.draw.line(screen, GRID, (ox, oy - RAD), (ox, oy + RAD), 1)
            pygame.draw.circle(screen, TEXT, (ox, oy), 2)
            for ang, name in ((0, "0"), (90, "90"), (180, "180"), (270, "270")):
                a = np.radians(ang)
                sx = -1.0 if flip else 1.0
                tx = ox + int(sx * (RAD + 12) * np.sin(a))
                ty = oy - int((RAD + 12) * np.cos(a))
                t = small.render(name, True, DIM)
                screen.blit(t, (tx - t.get_width() // 2, ty - t.get_height() // 2))

        screen.blit(font.render("POLAR", True, DIM), (14, 10))
        screen.blit(font.render("TOP-DOWN", True, DIM), (PANEL + 14, 10))

        # ---- HUD ---------------------------------------------------------
        pygame.draw.line(screen, GRID_HI, (0, PANEL), (W, PANEL), 1)
        hz = 0.0
        if len(rev_times) > 2:
            span = rev_times[-1] - rev_times[0]
            hz = (len(rev_times) - 1) / span if span > 0 else 0.0
        if latest_r is not None and latest_r.size:
            v = latest_r[latest_r > 0]
            med = float(np.median(v)) if v.size else 0.0
            mx = float(v.max()) if v.size else 0.0
            pct = 100.0 * v.size / latest_r.size
        else:
            med = mx = pct = 0.0
        elapsed = time.time() - t0
        stats = (f"{hz:5.2f} Hz   {last_pts:4d} pts/rev   "
                 f"{good / max(elapsed, 1e-9):6.0f} pts/s   "
                 f"valid {pct:4.1f}%   med {med:5.2f} m   max {mx:5.2f} m   "
                 f"{'auto' if auto else 'man '} {scale:4.1f} m")
        screen.blit(font.render(stats, True, TEXT), (14, PANEL + 8))
        screen.blit(small.render("Q/ESC quit    F flip    T trails    [ ] zoom    A auto",
                                 True, DIM), (14, PANEL + 28))
        if good + bad:
            cs = 100.0 * good / (good + bad)
            t = small.render(f"checksum {cs:.1f}%", True, DIM)
            screen.blit(t, (W - t.get_width() - 14, PANEL + 28))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()
    os.close(fd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
