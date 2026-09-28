"""YDLidar X3 triangle packets, shared by acquisition and device diagnostics."""
from __future__ import annotations

import struct

TRI_PACKHEADSIZE = 10
TRI_PACKMAXNODES = 80


def parse(buf: bytes):
    """Return validated packets, consumed bytes, and bad checksum count.

    Packet tuples contain (ct, first angle, last angle, ranges in metres).
    Retain a trailing AA because the next serial read may begin with 55.
    """
    out, i, bad = [], 0, 0
    while True:
        j = buf.find(b"\xaa\x55", i)
        if j < 0:
            return out, max(i, len(buf) - int(buf.endswith(b"\xaa"))), bad
        if j + TRI_PACKHEADSIZE > len(buf):
            return out, j, bad
        count = buf[j + 3]
        if not 0 < count <= TRI_PACKMAXNODES:
            i = j + 1
            continue
        need = TRI_PACKHEADSIZE + count * 2
        if j + need > len(buf):
            return out, j, bad
        words = struct.unpack_from(f"<{5 + count}H", buf, j)
        checksum = words[0] ^ words[1] ^ words[2] ^ words[3]
        for word in words[5:]:
            checksum ^= word
        if checksum != words[4] or not (words[2] & words[3] & 1):
            bad += 1
            i = j + 1
            continue
        out.append((buf[j + 2], (words[2] >> 1) / 64.0,
                    (words[3] >> 1) / 64.0,
                    [word / 4000.0 for word in words[5:]]))
        i = j + need
