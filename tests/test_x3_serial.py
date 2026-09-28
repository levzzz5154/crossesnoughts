"""Exercise wire data through the Windows X3 driver without attached hardware."""
import math
import struct

import pytest

from core.config import NoiseConfig, SceneConfig
from core.real import RealLidarSource, canonical_option_block
from core.scan import LaserScan
from core.serial_x3 import SerialX3Driver
from core.x3_protocol import parse


def packet(ct, first, last, distances):
    raw = [round(d * 4000) for d in distances]
    words = [0x55AA, len(raw) << 8 | ct,
             round(first * 64) << 1 | 1, round(last * 64) << 1 | 1]
    checksum = 0
    for word in words + raw:
        checksum ^= word
    return struct.pack(f"<{5 + len(raw)}H", *words, checksum, *raw)


def driver():
    d = SerialX3Driver()
    d.opts = canonical_option_block("COM8")
    return d


def test_partial_headers_and_checksum_errors_resynchronize():
    good = packet(201, 359, 1, [1, 2, 3])
    broken = bytearray(good)
    broken[-1] ^= 1
    out, used, bad = parse(bytes(broken) + b"noise\xaa")
    assert out == [] and bad == 1
    remainder = (bytes(broken) + b"noise\xaa")[used:]
    assert remainder == b"\xaa"
    out, used, bad = parse(remainder + good[1:])
    assert out == [(201, 359.0, 1.0, [1.0, 2.0, 3.0])]
    assert used == len(good) and bad == 0


def test_fragmented_wire_produces_complete_scan_with_angle_correction():
    d = driver()
    stream = (packet(0, 200, 220, [5, 5]) +  # initial partial scan discarded
              packet(201, 359, 1, [1, 2, 0]) +
              packet(0, 40, 80, [0.04, 16.2]) +
              packet(201, 359, 1, [1, 2, 3]))
    for byte in stream:
        d._consume(bytes([byte]))  # includes splitting AA55 and checksum
    d._scanning = True
    scan = LaserScan()
    assert d.doProcessSimple(scan)
    assert scan.size == 5 and len(scan.points) == 5
    assert scan.stamp > 0
    assert scan.scanFreq == 10.0
    assert scan.sampleRate == pytest.approx(0.05)
    assert [p.range for p in scan.points] == [1, 2, 0, 0, 0]
    correction = math.degrees(math.atan(21.8 * (155.3 - 1000) / (155.3 * 1000)))
    expected = (math.radians(359 + correction) + math.pi) % (2 * math.pi) - math.pi
    assert scan.points[0].angle == pytest.approx(expected)


def test_bad_packets_never_create_a_successful_scan():
    d = driver()
    broken = bytearray(packet(201, 10, 20, [1, 2]))
    broken[8] ^= 1
    d._consume(bytes(broken) * 3)
    assert not d._frames and not d._started


def test_windows_real_source_starts_only_after_a_complete_scan(monkeypatch):
    stream = bytearray(packet(201, 0, 90, [1, 1]) + packet(201, 0, 90, [2, 2]))

    class Port:
        is_open = True
        dtr = True
        commands = []

        @property
        def in_waiting(self):
            return len(stream)

        def read(self, count):
            data = bytes(stream[:count])
            del stream[:count]
            return data

        def write(self, data):
            self.commands.append(data)
            return len(data)

        def close(self):
            self.is_open = False

    port = Port()
    monkeypatch.setattr("core.serial_x3.serial.Serial", lambda **kwargs: port)
    source = RealLidarSource(SceneConfig(), NoiseConfig(), port="COM8")
    # Use the Windows backend explicitly so this test also runs on Linux.
    source._laser = driver()
    source._swig_scan = LaserScan()
    assert source.initialize() and source.turnOn()
    scan = LaserScan()
    assert source.doProcessSimple(scan)
    assert scan.size == 2 and scan.scanFreq == 10
    assert port.commands[0] == b"\xa5\x60"
    source.turnOff()
    source.disconnecting()
    assert not port.is_open


def test_restart_discards_old_scans_and_closes_previous_port(monkeypatch):
    class Port:
        closed = False

        def close(self):
            self.closed = True

    old, new = Port(), Port()
    d = driver()
    d.connection = old
    d._frames.append((123, 10, []))
    d._buffer.extend(b"old data")
    d._started = True
    monkeypatch.setattr("core.serial_x3.serial.Serial", lambda **kwargs: new)
    assert d.initialize()
    assert old.closed and d.connection is new
    assert not d._frames and not d._buffer and not d._started


def test_serial_error_closes_port_and_raises_for_reconnect():
    import serial

    class Dead:
        in_waiting = 0

        def read(self, n):
            raise serial.SerialException("ClearCommError failed (PermissionError(13))")

        def close(self):
            closed.append(True)

    closed = []
    d = driver()
    d.connection, d._scanning = Dead(), True
    with pytest.raises(ConnectionError, match="ClearCommError"):
        d.doProcessSimple(LaserScan.blank())
    assert closed and d.connection is None
