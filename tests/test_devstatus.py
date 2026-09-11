from types import SimpleNamespace

from game import devstatus


def test_serial_port_listing_is_cross_platform_and_usb_first(monkeypatch):
    ports = [
        SimpleNamespace(device="COM8", description="Bluetooth link", vid=None),
        SimpleNamespace(device="COM4", description="USB Serial Device", vid=0x10C4),
    ]
    monkeypatch.setattr(devstatus.list_ports, "comports", lambda: ports)
    assert devstatus.list_serial_ports() == [
        {"path": "COM4", "kind": "usb", "exists": True, "readable": True,
         "description": "USB Serial Device"},
        {"path": "COM8", "kind": "serial", "exists": True, "readable": True,
         "description": "Bluetooth link"},
    ]


def test_probe_closes_port_when_no_data_arrives(monkeypatch):
    class EmptyPort:
        in_waiting = 0
        closed = False

        def close(self):
            self.closed = True

    port = EmptyPort()
    monkeypatch.setattr(devstatus, "open_port", lambda *_: port)
    result = devstatus.probe("COM4", seconds=0.01)
    assert result["ok"] is False
    assert result["error"] == "no lidar packets on this port"
    assert port.closed is True
