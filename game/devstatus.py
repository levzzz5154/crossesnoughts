"""Serial device detection + live probe for the settings screen.

Uses PySerial, so device status works on Linux, Windows, and macOS without
loading the YDLidar SDK. Packets are parsed by core.x3_protocol:

    packet = AA 55 | ct | count | firstAngle:u16 | lastAngle:u16 | cs:u16 | u16[count]
    angle  = (raw >> 1) / 64.0   degrees  (CYdLidar.cpp:659)
    range  = raw / 4000.0        metres   (CYdLidar.cpp:685)
    cs     = XOR of the four header u16s, then XOR of every node u16
             (YDlidarDriver.cpp:1472)

Run probe() in a worker thread; it never raises (errors come back in the
result dict) and always closes the port.
"""
from __future__ import annotations

import time

import serial
from serial.tools import list_ports
from core.x3_protocol import parse

X3_BAUDRATE = 115200


def list_serial_ports() -> list[dict]:
    """Candidate serial devices, most-likely-lidar first.

    Each entry: path, kind, exists, readable, and description.
    """
    out: list[dict] = []
    ports = list(list_ports.comports())
    ports.sort(key=lambda item: (
        not (item.vid is not None or "USB" in (item.description or "").upper()),
        item.device))
    for info in ports:
        description = info.description or ""
        kind = "usb" if info.vid is not None or "USB" in description.upper() else "serial"
        out.append({"path": info.device, "kind": kind, "exists": True,
                    "readable": True, "description": description})
    return out


def open_port(port: str, baud: int):
    return serial.Serial(port=port, baudrate=baud, bytesize=serial.EIGHTBITS,
                         parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                         timeout=0.05)


def probe(port: str, baud: int = X3_BAUDRATE, seconds: float = 1.5) -> dict:
    """Sniff `port` for `seconds` and report whether a lidar is streaming.

    Never raises: any failure lands in result["error"]. Always closes the port.
    """
    res: dict = {"port": port, "baud": baud, "ok": False, "error": None,
                 "packets": 0, "bad": 0, "points": 0, "pts_per_sec": 0.0,
                 "hz": None, "points_per_rev": None}
    connection = None
    try:
        connection = open_port(port, baud)
        buf = bytearray()
        good = bad = pts = 0
        revs: list[tuple[int, float]] = []  # (points, duration)
        cur_pts = 0
        cur_t0: float | None = None
        started = False
        t0 = time.time()
        while time.time() - t0 < seconds:
            waiting = connection.in_waiting
            if not waiting:
                time.sleep(0.01)
                continue
            buf += connection.read(min(waiting, 65536))
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
        if connection is not None:
            try:
                connection.close()
            except (OSError, serial.SerialException):
                pass
