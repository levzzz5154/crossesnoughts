"""Direct X3 serial acquisition for Windows, without the native SDK handshake.

Single-channel X3s stream AA55 packets without a scan-response descriptor.
Startup succeeds only after receiving a complete checksum-validated revolution.
Angle correction follows YDLidar-SDK's triangle parsePoints implementation.
"""
from __future__ import annotations

from collections import deque
import math
import time

import serial

from core import scan as s
from core.x3_protocol import parse


class SerialX3Driver:
    def __init__(self):
        self.opts = {}
        self.connection = None
        self._buffer = bytearray()
        self._points = []
        self._started = False
        self._frames = deque(maxlen=2)
        self._stamp = 0
        self._last_ring = None
        self._frequency = 0.0
        self._scanning = False
        self._error = ""

    def setlidaropt(self, prop, value):
        self.opts[prop] = value
        return True

    def getlidaropt_toInt(self, prop):
        return prop in self.opts, int(self.opts.get(prop, 0))

    def getlidaropt_toBool(self, prop):
        return prop in self.opts, bool(self.opts.get(prop, False))

    def getlidaropt_toFloat(self, prop):
        return prop in self.opts, float(self.opts.get(prop, 0.0))

    def getlidaropt_toString(self, prop):
        return prop in self.opts, str(self.opts.get(prop, ""))

    def DescribeError(self):
        return self._error

    def initialize(self):
        self.disconnecting()
        self._error = ""
        self._buffer.clear()
        self._points = []
        self._frames.clear()
        self._started = False
        self._last_ring = None
        self._frequency = 0.0
        self.connection = serial.Serial(
            port=self.opts[s.LidarPropSerialPort],
            baudrate=self.opts[s.LidarPropSerialBaudrate],
            timeout=0.05, write_timeout=0.5)
        return True

    def turnOn(self):
        if self.connection is None:
            self._error = "port is not open"
            return False
        self.connection.dtr = True
        self.connection.write(b"\xa5\x60")  # SCAN; no descriptor on X3
        self._scanning = True
        if not self._read_frame(3.0):
            self._scanning = False
            return False
        return True

    def _consume(self, data):
        self._buffer.extend(data)
        packets, used, _bad = parse(bytes(self._buffer))
        del self._buffer[:used]
        for ct, first, last, ranges in packets:
            if ct & 1:
                now = time.monotonic()
                # The CT byte encodes measured speed in tenths of Hz.
                if ct >> 1:
                    self._frequency = (ct >> 1) / 10.0
                elif self._last_ring is not None and now > self._last_ring:
                    self._frequency = 1.0 / (now - self._last_ring)
                if self._started and self._points:
                    self._frames.append((self._stamp, self._frequency, self._points))
                self._started = True
                self._points = []
                self._stamp = time.time_ns()
                self._last_ring = now
            if not self._started:
                continue  # discard the initial partial revolution
            step = (last - first) % 360.0 / (len(ranges) - 1) if len(ranges) > 1 else 0.0
            for index, distance in enumerate(ranges):
                angle = first + index * step
                if distance > 0:
                    mm = distance * 1000.0
                    angle += math.degrees(math.atan(21.8 * (155.3 - mm) / (155.3 * mm)))
                angle = math.radians(angle % 360.0)
                if self.opts.get(s.LidarPropReversion, False):
                    angle += math.pi
                if self.opts.get(s.LidarPropInverted, False):
                    angle = -angle
                angle = (angle + math.pi) % (2 * math.pi) - math.pi
                if not self.opts.get(s.LidarPropMinRange, 0.08) <= distance <= self.opts.get(s.LidarPropMaxRange, 16.0):
                    distance = 0.0
                self._points.append(s.LaserPoint(angle=angle, range=distance))
            if len(self._points) > 4096:
                # Missing revolution markers must not grow memory forever.
                self._points = []
                self._started = False

    def _read_frame(self, timeout):
        deadline = time.monotonic() + timeout
        while not self._frames and time.monotonic() < deadline:
            if self.connection is None:
                self._error = "port is not open"
                return False
            try:
                data = self.connection.read(min(max(self.connection.in_waiting, 1), 65536))
            except (OSError, serial.SerialException) as e:
                # Unplug/USB reset (Windows reports ClearCommError access
                # denied). Close the dead handle and let the pipeline
                # reconnect; returning False would stall on a closed port.
                self._error = f"serial read failed: {e}"
                self.disconnecting()
                raise ConnectionError(self._error) from e
            if data:
                self._consume(data)
        if not self._frames:
            self._error = "no complete X3 scans received; check power and USB cable"
            return False
        self._error = ""
        return True

    def doProcessSimple(self, scan):
        if not self._scanning or not self._read_frame(1.0):
            return False
        stamp, hz, points = self._frames.popleft()
        scan.stamp = stamp
        scan.scanFreq = hz
        scan.sampleRate = len(points) * hz / 1000.0
        scan.size = len(points)
        scan.points = points
        return True

    def turnOff(self):
        self._scanning = False
        if self.connection is not None and self.connection.is_open:
            try:
                self.connection.write(b"\xa5\x65")  # STOP
                self.connection.dtr = False
            except (OSError, serial.SerialException):
                pass
        return True

    def disconnecting(self):
        self._scanning = False
        if self.connection is not None:
            self.connection.close()
            self.connection = None
