"""Serial device detection + live probe for the settings screen.

Standard library only (termios + select) — no ydlidar SDK needed, so device
status works even when the SWIG module is missing or built for another
interpreter. The triangle-protocol parsing is the same verified logic as
tools/lidar_check.py:

    packet = AA 55 | ct | count | firstAngle:u16 | lastAngle:u16 | cs:u16 | u16[count]
    angle  = (raw >> 1) / 64.0   degrees  (CYdLidar.cpp:659)
    range  = raw / 4000.0        metres   (CYdLidar.cpp:685)
    cs     = XOR of the four header u16s, then XOR of every node u16
             (YDlidarDriver.cpp:1472)

Run probe() in a worker thread; it never raises (errors come back in the
result dict) and always closes the port.
"""
from __future__ import annotations

import glob
import os
import select
import termios
import time

TRI_PACKHEADSIZE = 10
TRI_PACKMAXNODES = 80
X3_BAUDRATE = 115200


def list_serial_ports() -> list[dict]:
    """Candidate serial devices, most-likely-lidar first.

    Prefers stable /dev/serial/by-id symlinks (survive replug order) over
    raw /dev/ttyUSB* names. Each entry: path, kind, exists, readable.
    """
    seen: set[str] = set()
    out: list[dict] = []
    for pat, kind in (
        ("/dev/serial/by-id/*", "by-id"),
        ("/dev/ttyUSB*", "usb"),
        ("/dev/ttyACM*", "usb"),
    ):
        for path in sorted(glob.glob(pat)):
            real = os.path.realpath(path)
            if real in seen:
                continue
            seen.add(real)
            exists = os.path.exists(path)
            readable = False
            if exists:
                try:
                    readable = os.access(path, os.R_OK | os.W_OK)
                except OSError:
                    readable = False
            out.append({"path": path, "kind": kind, "exists": exists,
                        "readable": readable})
    return out


def open_port(port: str, baud: int) -> int:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0  # iflag
    attrs[1] = 0  # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag
    attrs[3] = 0  # lflag
    speed = getattr(termios, f"B{baud}", None)
    if speed is None:
        os.close(fd)
        raise ValueError(f"unsupported baud {baud}")
    attrs[4] = attrs[5] = speed
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


def probe(port: str, baud: int = X3_BAUDRATE, seconds: float = 1.5) -> dict:
    """Sniff `port` for `seconds` and report whether a lidar is streaming.

    Never raises: any failure lands in result["error"]. Always closes the fd.
    """
    res: dict = {"port": port, "baud": baud, "ok": False, "error": None,
                 "packets": 0, "bad": 0, "points": 0, "pts_per_sec": 0.0,
                 "hz": None, "points_per_rev": None}
    fd = None
    try:
        fd = open_port(port, baud)
        buf = bytearray()
        good = bad = pts = 0
        revs: list[tuple[int, float]] = []  # (points, duration)
        cur_pts = 0
        cur_t0: float | None = None
        started = False
        t0 = time.time()
        while time.time() - t0 < seconds:
            r, _, _ = select.select([fd], [], [], 0.05)
            if not r:
                continue
            buf += os.read(fd, 65536)
            pkts, used, nbad = parse(bytes(buf))
            del buf[:used]
            bad += nbad
            for ct, first, last, ranges in pkts:
                good += 1
                pts += len(ranges)
                now = time.time()
                if ct & 0x01:  # zero packet -> new revolution
                    if started and cur_t0 is not None:
                        revs.append((cur_pts, now - cur_t0))
                    started, cur_pts, cur_t0 = True, 0, now
                cur_pts += len(ranges)
        elapsed = max(time.time() - t0, 1e-9)
        res.update(packets=good, bad=bad, points=pts,
                   pts_per_sec=round(pts / elapsed, 1))
        if good == 0:
            res["error"] = "no lidar packets on this port"
            return res
        if len(revs) > 1:
            # drop the first revolution (may be partial)
            durs = sorted(d for _, d in revs[1:])
            perr = sorted(p for p, _ in revs[1:])
            dur = durs[len(durs) // 2]
            per = perr[len(perr) // 2]
            if dur > 0:
                res["hz"] = round(1.0 / dur, 2)
                res["points_per_rev"] = int(round(per))
        res["ok"] = True
        return res
    except PermissionError:
        res["error"] = "permission denied (are you in the port's group?)"
        return res
    except FileNotFoundError:
        res["error"] = "device not found"
        return res
    except OSError as e:
        res["error"] = str(e)
        return res
    except Exception as e:  # never let a probe kill the UI
        res["error"] = f"{type(e).__name__}: {e}"
        return res
    finally:
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass
