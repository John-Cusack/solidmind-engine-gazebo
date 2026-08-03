"""Real Gazebo: a headless world, the bridge in ``runtime="real"``, gz services.

Skipped unless the ``gz`` CLI is on PATH.  This launches an actual Gazebo
world once per class, points a real-runtime bridge at it, and drives
spawn/step/diagnose through the services Gazebo really exposes — the one
thing no stub can tell you.

This came from core's cross-engine ``test_sim_real_backends``.  Its Isaac and
Chrono halves went to those repos' own real-runtime suites, and the parts that
drove core's ``sim_engine_manager`` stayed in core, which still tests the
manager directly.  What is left is the Gazebo half, which belongs here.
"""

from __future__ import annotations

import json
import shutil
import socket
import subprocess
import threading
import time
import unittest
from pathlib import Path

from tests.conftest import unused_tcp_port

_PROJECT_ROOT = Path(__file__).resolve().parent.parent
_FIXTURE_DIR = Path(__file__).resolve().parent / "fixtures" / "simple_2body"
_URDF_PATH = _FIXTURE_DIR / "simple_2body.urdf"

requires_gazebo = unittest.skipUnless(
    shutil.which("gz") is not None,
    "Gazebo Harmonic not installed (gz CLI not found)",
)


def _send_command(host: str, port: int, cmd: str, args: dict | None = None) -> dict:
    """Send a single command to the bridge and return the parsed response."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(30.0)
    sock.connect((host, port))
    try:
        msg = json.dumps({"cmd": cmd, "args": args or {}}) + "\n"
        sock.sendall(msg.encode())
        data = b""
        while b"\n" not in data:
            chunk = sock.recv(65536)
            if not chunk:
                break
            data += chunk
        return json.loads(data.decode().strip())
    finally:
        sock.close()


class _GazeboWorld:
    """Launch a headless Gazebo world as a subprocess, tear it down on exit."""

    def __init__(self, world_name: str = "empty") -> None:
        self.world_name = world_name
        self._proc: subprocess.Popen | None = None

    def start(self, timeout_s: float = 15.0) -> None:
        self._proc = subprocess.Popen(
            ["gz", "sim", "-s", "-r", "--headless-rendering", f"{self.world_name}.sdf"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        # Wait for Gazebo services to appear
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._proc.poll() is not None:
                _, stderr = self._proc.communicate(timeout=2)
                raise RuntimeError(
                    f"Gazebo exited early (rc={self._proc.returncode}): "
                    f"{stderr.decode(errors='replace')[:500]}"
                )
            result = subprocess.run(
                ["gz", "service", "-l"],
                capture_output=True,
                text=True,
                timeout=5,
            )
            if f"/world/{self.world_name}/create" in result.stdout:
                return
            time.sleep(0.5)
        self.stop()
        raise RuntimeError(f"Gazebo world '{self.world_name}' did not start within {timeout_s}s")

    def stop(self) -> None:
        if self._proc is None:
            return
        self._proc.terminate()
        try:
            self._proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self._proc.kill()
            self._proc.wait(timeout=2)

    def __enter__(self) -> _GazeboWorld:
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()


class _RealGazeboBridge:
    """Launch a real GazeboBridgeServer with RealGazeboRuntime in a daemon thread."""

    def __init__(self, port: int, world_name: str = "empty") -> None:
        self.host = "127.0.0.1"
        self.port = port
        self.world_name = world_name
        self._server = None
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        from gazebo_bridge.bridge_server import GazeboBridgeServer

        self._server = GazeboBridgeServer(
            host=self.host,
            port=self.port,
            runtime_mode="real",
            world_name=self.world_name,
        )
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            daemon=True,
            name="gazebo-real-bridge",
        )
        self._thread.start()
        # Wait for bridge to accept connections
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(0.5)
                sock.connect((self.host, self.port))
                sock.close()
                return
            except (ConnectionRefusedError, OSError):
                time.sleep(0.05)
        raise RuntimeError(f"Real Gazebo bridge did not start on port {self.port}")

    def stop(self) -> None:
        if self._server is not None:
            self._server.shutdown()
        if self._thread is not None:
            self._thread.join(timeout=5.0)

    def __enter__(self) -> _RealGazeboBridge:
        self.start()
        return self

    def __exit__(self, *exc) -> None:
        self.stop()


@requires_gazebo
class TestRealGazebo(unittest.TestCase):
    """Real Gazebo backend tests — exercises actual gz CLI services."""

    _gz_world: _GazeboWorld | None = None

    @classmethod
    def setUpClass(cls) -> None:
        """Launch a headless Gazebo world once for all tests in this class."""
        cls._gz_world = _GazeboWorld("empty")
        cls._gz_world.start(timeout_s=20.0)

    @classmethod
    def tearDownClass(cls) -> None:
        if cls._gz_world is not None:
            cls._gz_world.stop()

    def test_diagnose_detects_real_world(self):
        """Bridge with runtime=real detects the running Gazebo world."""
        port = unused_tcp_port()
        with _RealGazeboBridge(port, world_name="empty") as bridge:
            resp = _send_command(bridge.host, bridge.port, "diagnose", {})

        self.assertTrue(resp["ok"], resp)
        result = resp["result"]
        self.assertEqual(result["runtime_mode"], "real")
        self.assertTrue(result["gz_available"])
        self.assertIn("worlds", result)
        self.assertIn("empty", result["worlds"])

    def test_spawn_urdf_into_real_world(self):
        """Spawn the test URDF into the running Gazebo world via gz service."""
        if not _URDF_PATH.exists():
            self.skipTest(f"Test fixture not found: {_URDF_PATH}")

        port = unused_tcp_port()
        with _RealGazeboBridge(port, world_name="empty") as bridge:
            resp = _send_command(
                bridge.host,
                bridge.port,
                "spawn_model",
                {
                    "urdf_path": str(_URDF_PATH),
                    "model_name": "test_2body_spawn",
                    "world_name": "empty",
                },
            )

        self.assertTrue(resp["ok"], resp)
        result = resp["result"]
        self.assertTrue(result["spawned"])
        self.assertEqual(result["model_name"], "test_2body_spawn")
        self.assertEqual(result["source_format"], "urdf")

    def test_simulate_spawns_and_steps_real_physics(self):
        """Run batch simulation: spawn URDF + step the real Gazebo world."""
        if not _URDF_PATH.exists():
            self.skipTest(f"Test fixture not found: {_URDF_PATH}")

        mech = {
            "name": "real_sim_test",
            "parts": [
                {"id": "chassis", "is_ground": True},
                {"id": "arm"},
            ],
            "joints": [
                {
                    "id": "shoulder",
                    "joint_type": "revolute",
                    "parent_part": "chassis",
                    "child_part": "arm",
                    "axis": [0, 0, 1],
                },
            ],
            "drives": [],
        }

        port = unused_tcp_port()
        with _RealGazeboBridge(port, world_name="empty") as bridge:
            resp = _send_command(
                bridge.host,
                bridge.port,
                "simulate",
                {
                    "mechanism": mech,
                    "urdf_path": str(_URDF_PATH),
                    "model_name": "test_2body_sim",
                    "duration_s": 0.5,
                    "dt_s": 0.01,
                    "output_interval": 0.1,
                    "world_name": "empty",
                },
            )

        self.assertTrue(resp["ok"], resp)
        result = resp["result"]

        # Time series from stub layer (real Gazebo doesn't generate custom telemetry yet)
        self.assertIn("time_series", result)
        self.assertGreater(len(result["time_series"]), 0)

        # Summary should indicate real Gazebo mode
        summary = result["summary"]
        self.assertEqual(summary["engine_mode"], "gazebo_real")

        # Spawn info should be present
        self.assertIn("spawn", summary)
        self.assertTrue(summary["spawn"]["spawned"])

    def test_health_check_through_real_bridge(self):
        """``ping`` answers while a real Gazebo world is attached.

        This used to call core's ``_health_check`` helper.  Over the wire that
        helper is a ``ping``, so sending one is the same check without the
        import — and it is the contract's own health verb, not core's idea of
        one.
        """
        port = unused_tcp_port()
        with _RealGazeboBridge(port, world_name="empty") as bridge:
            resp = _send_command(bridge.host, bridge.port, "ping")

        self.assertTrue(resp.get("ok"), f"Health check failed: {resp}")

    def test_list_worlds_via_diagnose(self):
        """Diagnose lists real Gazebo worlds via gz service -l."""
        port = unused_tcp_port()
        with _RealGazeboBridge(port, world_name="empty") as bridge:
            resp = _send_command(
                bridge.host,
                bridge.port,
                "diagnose",
                {
                    "world_name": "empty",
                },
            )

        self.assertTrue(resp["ok"], resp)
        worlds = resp["result"].get("worlds", [])
        self.assertIsInstance(worlds, list)
        # The empty world should be listed
        self.assertIn("empty", worlds)


if __name__ == "__main__":
    unittest.main()
