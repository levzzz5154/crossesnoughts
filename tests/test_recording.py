"""Record/replay round-trip tests (plan §17.6 real-mode path, no hardware).

The point of these: a recording captured from a real X3 and replayed later must
drive the SAME Tracker code as the sim, with zero consumer changes. Recording
from the sim here is a stand-in for recording from hardware.
"""
import json
import time

import numpy as np
import pytest

from core.config import NoiseConfig, SceneConfig
from core.lidar_source import BackgroundTable, make_lidar_source
from core.recording import RecordingLidarSource, ReplayLidarSource, record
from core.scan import LaserPoint, LaserScan, SCAN_POINT_COUNT
from core.sim import SimLidarSource
from core.tracking import Tracker

SWIG_METHODS = [
    "setlidaropt",
    "getlidaropt_toInt",
    "getlidaropt_toBool",
    "getlidaropt_toFloat",
    "getlidaropt_toString",
    "initialize",
    "turnOn",
    "doProcessSimple",
    "turnOff",
    "disconnecting",
]


def _fast(sim: SimLidarSource) -> SimLidarSource:
    """Skip the sim's 10 Hz pacing; these tests do not need wall-clock time."""
    sim._scan_freq = 500.0
    return sim


def _record_sim(tmp_path, n=40, board=2.0, pose=(1.2, 0.8), seed=11, noise=None):
    scene = SceneConfig(board_size=board)
    noise = noise or NoiseConfig()
    sim = _fast(SimLidarSource(scene, noise, seed=seed))
    sim.set_object_pose(pose)
    path = tmp_path / "rec.npz"
    record(sim, path, scans=n, note="unit test", kind="sim")
    return scene, noise, path, pose


# ------------------------------------------------------------------- recorder
def test_record_writes_expected_arrays(tmp_path):
    _, _, path, _ = _record_sim(tmp_path, n=25)
    with np.load(path, allow_pickle=False) as z:
        ranges = z["ranges"]
        stamps = z["stamps"]
        meta = json.loads(str(z["meta"].item()))
    assert ranges.shape == (25, SCAN_POINT_COUNT)
    assert stamps.shape == (25,)
    assert np.all(np.diff(stamps) >= 0)  # monotonic non-decreasing
    assert meta["n_scans"] == 25
    assert meta["point_count"] == SCAN_POINT_COUNT
    assert meta["source_kind"] == "sim"
    assert meta["note"] == "unit test"
    assert meta["format"] >= 1
    assert np.any(ranges > 0.0)  # not all dropout


def test_record_captures_background(tmp_path):
    scene = SceneConfig(board_size=2.0)
    sim = _fast(SimLidarSource(scene, NoiseConfig(), seed=3))
    expected = sim.calibrate_background()
    rec = RecordingLidarSource(sim, tmp_path / "bg.npz", kind="sim")
    got = rec.calibrate_background()
    assert np.allclose(got.ranges, expected.ranges)
    # the file only reaches disk once there is at least one scan to store
    rec.initialize()
    rec.turnOn()
    scan = LaserScan.blank()
    for _ in range(3):
        rec.doProcessSimple(scan)
    rec.close()
    with np.load(tmp_path / "bg.npz", allow_pickle=False) as z:
        assert "bg_angles" in z and "bg_ranges" in z
        assert np.allclose(z["bg_ranges"], expected.ranges)
        assert z["ranges"].shape[0] == 3


def test_recorder_delegates_sim_extras(tmp_path):
    """The wrapper must stay transparent: sim-only extras still work, so the
    web viz keeps its true_pose marker while recording."""
    scene = SceneConfig(board_size=2.0)
    sim = _fast(SimLidarSource(scene, NoiseConfig(), seed=4))
    rec = RecordingLidarSource(sim, tmp_path / "d.npz", kind="sim")
    assert rec.initialize() and rec.turnOn()
    rec.set_object_pose((0.5, 0.75))
    assert rec.object_pose == (0.5, 0.75)
    scan = LaserScan.blank()
    for _ in range(7):
        assert rec.doProcessSimple(scan)
    assert rec.n_scans == 7
    rec.close()
    assert (tmp_path / "d.npz").exists()


def test_recorder_flushes_and_is_idempotent_on_close(tmp_path):
    scene = SceneConfig(board_size=2.0)
    sim = _fast(SimLidarSource(scene, NoiseConfig(), seed=5))
    path = tmp_path / "f.npz"
    rec = RecordingLidarSource(sim, path, kind="sim", flush_every=5)
    rec.initialize()
    rec.turnOn()
    scan = LaserScan.blank()
    for _ in range(5):
        rec.doProcessSimple(scan)
    assert path.exists()  # periodic flush, not just on close
    rec.close()
    rec.close()  # idempotent
    with np.load(path, allow_pickle=False) as z:
        assert z["ranges"].shape[0] == 5


def test_angle_grid_collapsed_when_uniform(tmp_path):
    """A fixed-resolution recording stores one angle row instead of T rows."""
    noise = NoiseConfig(angular_jitter_std=0.0)
    _, _, path, _ = _record_sim(tmp_path, n=10, noise=noise)
    with np.load(path, allow_pickle=False) as z:
        assert z["angles"].ndim == 1
        assert z["angles"].shape == (SCAN_POINT_COUNT,)
    src = ReplayLidarSource(SceneConfig(), NoiseConfig(), path)
    src.initialize()
    src.turnOn()
    scan = LaserScan.blank()
    assert src.doProcessSimple(scan)
    assert np.allclose(
        [p.angle for p in scan.points], np.linspace(-np.pi, np.pi, SCAN_POINT_COUNT)
    )


def test_variable_resolution_recording_roundtrip(tmp_path):
    """Real-style 333/334/335-point scans flush and replay losslessly."""

    class VariableSource:
        def __init__(self):
            self.i = 0
            self.counts = [333, 335, 334]

        def doProcessSimple(self, scan):
            n = self.counts[self.i]
            scan.points = [
                LaserPoint(
                    angle=float(-np.pi + 2 * np.pi * j / (n - 1)),
                    range=float(1.0 + j / 10000),
                )
                for j in range(n)
            ]
            scan.size = n
            scan.stamp = 1_700_000_000_000_000_000 + self.i * 100_000_000
            self.i += 1
            return True

    path = tmp_path / "variable.npz"
    rec = RecordingLidarSource(VariableSource(), path, kind="real", flush_every=1)
    scan = LaserScan.blank()
    for _ in range(3):
        assert rec.doProcessSimple(scan)
    rec.close()

    with np.load(path, allow_pickle=False) as z:
        assert z["ranges"].shape == (3, 335)
        assert np.array_equal(z["sizes"], [333, 335, 334])
        assert json.loads(str(z["meta"].item()))["variable_point_count"] is True

    replay = ReplayLidarSource(SceneConfig(), NoiseConfig(), path, speed=0.0)
    replay.turnOn()
    for n in [333, 335, 334]:
        assert replay.doProcessSimple(scan)
        assert scan.size == n
        assert len(scan.points) == n


# --------------------------------------------------------------------- replay
def test_replay_reproduces_recorded_scans(tmp_path):
    _, _, path, _ = _record_sim(tmp_path, n=12)
    with np.load(path, allow_pickle=False) as z:
        ranges = z["ranges"]
        stamps = z["stamps"]
        angles = z["angles"]
    src = ReplayLidarSource(SceneConfig(), NoiseConfig(), path, speed=0.0)
    assert src.initialize() and src.turnOn()
    scan = LaserScan.blank()
    for i in range(12):
        assert src.doProcessSimple(scan)
        assert int(scan.stamp) == int(stamps[i])
        assert scan.size == SCAN_POINT_COUNT
        assert np.allclose([p.range for p in scan.points], ranges[i], atol=1e-6)
        expect_a = angles if angles.ndim == 1 else angles[i]
        assert np.allclose([p.angle for p in scan.points], expect_a, atol=1e-6)
    assert src.exhausted or src.index == 12


def test_replay_roundtrip_tracks_object(tmp_path):
    """Record from the sim, replay through the real Tracker path, and the
    tracked position must still land on the object."""
    scene, noise, path, pose = _record_sim(tmp_path, n=60, pose=(1.2, 0.8))
    src = ReplayLidarSource(scene, noise, path, speed=0.0)
    tracker = Tracker(scene, noise)
    assert src.initialize() and src.turnOn()
    tracker.set_background(src.calibrate_background())
    scan = LaserScan.blank()
    hits = []
    for _ in range(40):
        assert src.doProcessSimple(scan)
        res = tracker.process(scan)
        if res is not None:
            hits.append((res.x, res.y))
    assert len(hits) >= 35, "tracker lost the object during replay"
    tail = np.asarray(hits[-10:])
    err = np.hypot(tail[:, 0] - pose[0], tail[:, 1] - pose[1])
    assert float(np.median(err)) < 0.05, f"median replay error {np.median(err):.3f} m"


def test_replay_loops_wraps_to_start(tmp_path):
    _, _, path, _ = _record_sim(tmp_path, n=5)
    src = ReplayLidarSource(SceneConfig(), NoiseConfig(), path, loop=True, speed=0.0)
    src.initialize()
    src.turnOn()
    scan = LaserScan.blank()
    stamps = []
    for _ in range(12):
        assert src.doProcessSimple(scan)
        stamps.append(int(scan.stamp))
    with np.load(path, allow_pickle=False) as z:
        expect = list(z["stamps"])
    assert stamps[:5] == expect
    assert stamps[5:10] == expect  # wrapped
    assert not src.exhausted


def test_replay_no_loop_exhausts(tmp_path):
    _, _, path, _ = _record_sim(tmp_path, n=5)
    src = ReplayLidarSource(SceneConfig(), NoiseConfig(), path, loop=False, speed=0.0)
    src.initialize()
    src.turnOn()
    scan = LaserScan.blank()
    for _ in range(5):
        assert src.doProcessSimple(scan)
    assert src.exhausted
    assert src.doProcessSimple(scan) is False


def test_replay_seek_and_reset(tmp_path):
    _, _, path, _ = _record_sim(tmp_path, n=10)
    with np.load(path, allow_pickle=False) as z:
        stamps = list(z["stamps"])
    src = ReplayLidarSource(SceneConfig(), NoiseConfig(), path, speed=0.0)
    src.initialize()
    src.turnOn()
    scan = LaserScan.blank()
    src.seek(7)
    assert src.index == 7
    assert src.doProcessSimple(scan)
    assert int(scan.stamp) == stamps[7]
    src.reset()  # the web 'reset' control restarts playback
    assert src.index == 0
    assert src.doProcessSimple(scan)
    assert int(scan.stamp) == stamps[0]


def test_replay_pacing_is_skipped_when_free_running(tmp_path):
    """speed=0 must not sleep, so a long recording replays as fast as it can."""
    _, _, path, _ = _record_sim(tmp_path, n=60)
    src = ReplayLidarSource(SceneConfig(), NoiseConfig(), path, speed=0.0)
    src.initialize()
    src.turnOn()
    t0 = time.monotonic()
    scan = LaserScan.blank()
    for _ in range(60):
        src.doProcessSimple(scan)
    assert time.monotonic() - t0 < 2.0  # 6 s of 10 Hz data, replayed instantly


def test_replay_reports_duration_and_rate(tmp_path):
    scene = SceneConfig(board_size=2.0)
    sim = _fast(SimLidarSource(scene, NoiseConfig(), seed=9))
    path = tmp_path / "rate.npz"
    # 20 scans spaced 100 ms apart in the recorded stamps
    rec = RecordingLidarSource(sim, path, kind="sim")
    rec.initialize()
    rec.turnOn()
    scan = LaserScan.blank()
    base = 1_700_000_000_000_000_000
    for i in range(20):
        rec.doProcessSimple(scan)
        rec._stamps[-1] = base + i * 100_000_000
    rec.close()
    src = ReplayLidarSource(scene, NoiseConfig(), path)
    assert src.n_scans == 20
    assert abs(src.duration - 1.9) < 1e-6
    assert abs(src.scan_freq - 10.0) < 1e-6


def test_replay_duck_types_the_swig_surface(tmp_path):
    _, _, path, _ = _record_sim(tmp_path, n=3)
    src = ReplayLidarSource(SceneConfig(), NoiseConfig(), path)
    for m in SWIG_METHODS:
        assert callable(getattr(src, m, None)), f"ReplayLidarSource missing {m}"
    # sim-only affordances must be absent so the viz does not treat replay as sim
    assert not hasattr(src, "object_pose")


def test_replay_without_captured_background_warns(tmp_path):
    """A recording with no bg_ arrays still loads, deriving a median fallback."""
    path = tmp_path / "nobg.npz"
    angles = np.linspace(-np.pi, np.pi, SCAN_POINT_COUNT)
    ranges = np.tile(1.5 + np.abs(angles), (8, 1)).astype(np.float32)
    stamps = np.arange(8, dtype=np.int64) * 100_000_000
    np.savez_compressed(
        path,
        ranges=ranges,
        angles=angles.astype(np.float32),
        stamps=stamps,
        meta=np.array(json.dumps({"format": 1, "source_kind": "test"})),
    )
    src = ReplayLidarSource(SceneConfig(), NoiseConfig(), path)
    with pytest.warns(RuntimeWarning, match="no captured background"):
        bg = src.calibrate_background()
    assert isinstance(bg, BackgroundTable)
    assert np.allclose(bg.ranges, ranges[0].astype(float), atol=1e-6)


def test_replay_rejects_malformed_files(tmp_path):
    scene, noise = SceneConfig(), NoiseConfig()
    with pytest.raises(FileNotFoundError):
        ReplayLidarSource(scene, noise, tmp_path / "missing.npz")
    bad = tmp_path / "bad.npz"
    np.savez_compressed(
        bad,
        ranges=np.zeros((4, 10), dtype=np.float32),
        angles=np.zeros(10, dtype=np.float32),
        stamps=np.arange(3, dtype=np.int64),  # mismatch on purpose
        meta=np.array("{}"),
    )
    with pytest.raises(ValueError, match="mismatch"):
        ReplayLidarSource(scene, noise, bad)


# -------------------------------------------------------------------- factory
def test_factory_replay_requires_a_file():
    with pytest.raises(ValueError, match="--replay-file is required"):
        make_lidar_source("replay", scene=SceneConfig(), noise=NoiseConfig())


def test_factory_replay_and_record(tmp_path):
    scene, noise = SceneConfig(), NoiseConfig()
    _, _, path, _ = _record_sim(tmp_path, n=5)
    src = make_lidar_source("replay", scene=scene, noise=noise, replay_path=path)
    assert isinstance(src, ReplayLidarSource)
    assert src.n_scans == 5

    out = tmp_path / "wrapped.npz"
    wrapped = make_lidar_source(
        "sim", scene=scene, noise=noise, seed=2, record_path=out, record_note="via factory"
    )
    assert isinstance(wrapped, RecordingLidarSource)
    assert wrapped.initialize() and wrapped.turnOn()
    scan = LaserScan.blank()
    for _ in range(6):
        wrapped.doProcessSimple(scan)
    wrapped.close()
    with np.load(out, allow_pickle=False) as z:
        assert z["ranges"].shape[0] == 6
        assert json.loads(str(z["meta"].item()))["note"] == "via factory"


def test_replay_drives_web_scanloop(tmp_path):
    """The real two-thread ScanLoop must accept a replay source, emit frames,
    and stop cleanly when a non-looping recording ends."""
    from web.frames import SceneModel
    from web.main import ScanLoop

    scene, noise, path, _ = _record_sim(tmp_path, n=12)
    source = ReplayLidarSource(scene, noise, path, loop=False, speed=0.0)
    tracker = Tracker(scene, noise)
    model = SceneModel(scene)
    loop = ScanLoop(source, tracker, model, scene)
    loop.start()
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and model.current_seq() < 12:
        time.sleep(0.02)
    loop.stop()
    assert model.current_seq() == 12, "scan loop should emit exactly one frame per scan"
    frame = model.latest(0)
    assert frame is not None
    assert frame.seq == 12
