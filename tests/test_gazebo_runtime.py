"""Unit tests for Gazebo bridge runtime implementations."""

from __future__ import annotations

import os
import tempfile
import unittest
from unittest.mock import patch

from gazebo_bridge.runtime_gazebo import (
    GazeboRuntimeError,
    RealGazeboRuntime,
    StubGazeboRuntime,
    _angular_speeds_rpm,
    _parse_pose_message,
    create_runtime,
)


def _make_temp_model(suffix: str = ".sdf") -> str:
    with tempfile.NamedTemporaryFile(suffix=suffix, delete=False) as f:
        f.write(b"<sdf version='1.10'><model name='m'><link name='base'/></model></sdf>")
        return f.name


class TestCreateRuntime(unittest.TestCase):
    def test_env_selects_stub_runtime(self) -> None:
        with patch.dict(os.environ, {"SOLIDMIND_GAZEBO_RUNTIME": "stub"}, clear=False):
            rt = create_runtime()
        self.assertIsInstance(rt, StubGazeboRuntime)
        self.assertNotIsInstance(rt, RealGazeboRuntime)

    def test_explicit_real_runtime(self) -> None:
        with patch("gazebo_bridge.runtime_gazebo.shutil.which", return_value="/usr/bin/gz"):
            rt = create_runtime(runtime_mode="real")
        self.assertIsInstance(rt, RealGazeboRuntime)


class TestStubRuntime(unittest.TestCase):
    def setUp(self) -> None:
        self.runtime = StubGazeboRuntime(world_name="default", enable_px4=True)
        self.model_path = _make_temp_model()

    def tearDown(self) -> None:
        try:
            os.unlink(self.model_path)
        except OSError:
            pass

    def test_spawn_model_success(self) -> None:
        result = self.runtime.handle_spawn_model(
            {"sdf_path": self.model_path, "model_name": "drone"}
        )
        self.assertTrue(result["spawned"])
        self.assertEqual(result["model_name"], "drone")
        self.assertEqual(result["entity_id"], 1)

    def test_simulate_returns_time_series(self) -> None:
        result = self.runtime.handle_simulate(
            {
                "duration_s": 0.2,
                "dt_s": 0.01,
                "output_interval": 0.05,
                "mechanism": {"parts": [{"id": "frame"}]},
            }
        )
        self.assertIn("time_series", result)
        self.assertGreaterEqual(len(result["time_series"]), 2)
        self.assertEqual(result["summary"]["engine_mode"], "stub")

    def test_teleop_lifecycle_multirotor(self) -> None:
        started = self.runtime.handle_teleop_start(
            {
                "mechanism": {"parts": [{"id": "frame"}]},
                "profile": {"controller_type": "multirotor_direct"},
                "sdf_path": self.model_path,
            }
        )
        session_id = started["session_id"]
        self.assertEqual(started["controller_type"], "multirotor_direct")

        cmd = self.runtime.handle_teleop_command(
            {
                "session_id": session_id,
                "vx_mps": 0.4,
                "vy_mps": 0.1,
                "vz_mps": 0.2,
                "yaw_rate_rps": 0.3,
            }
        )
        self.assertTrue(cmd["applied"])
        self.assertIn("rotor_setpoints", cmd["state"])

        state = self.runtime.handle_teleop_state({"session_id": session_id})
        self.assertEqual(state["tick_count"], 1)
        self.assertAlmostEqual(state["state"]["vx_mps"], 0.4)

        stopped = self.runtime.handle_teleop_stop({"session_id": session_id})
        self.assertTrue(stopped["stopped"])

    def test_invalid_controller_type(self) -> None:
        with self.assertRaises(GazeboRuntimeError) as ctx:
            self.runtime.handle_teleop_start(
                {
                    "mechanism": {},
                    "profile": {"controller_type": "invalid"},
                    "sdf_path": self.model_path,
                }
            )
        self.assertEqual(ctx.exception.code, "INVALID_INPUT")


class TestRealRuntime(unittest.TestCase):
    def setUp(self) -> None:
        self.model_path = _make_temp_model()
        self.calls: list[list[str]] = []

        def _runner(cmd: list[str]) -> tuple[int, str, str]:
            self.calls.append(cmd)
            if cmd[:3] == ["gz", "service", "-l"]:
                return 0, "/world/default/control\n/world/default/create\n", ""
            if "/world/default/create" in cmd:
                return 0, "data: true", ""
            if "/world/default/control" in cmd:
                return 0, "data: true", ""
            return 1, "", "unsupported"

        self.runner = _runner

    def tearDown(self) -> None:
        try:
            os.unlink(self.model_path)
        except OSError:
            pass

    def test_real_runtime_uses_gz_services(self) -> None:
        with patch("gazebo_bridge.runtime_gazebo.shutil.which", return_value="/usr/bin/gz"):
            runtime = RealGazeboRuntime(world_name="default", command_runner=self.runner)

        spawned = runtime.handle_spawn_model({"sdf_path": self.model_path, "model_name": "quad"})
        self.assertTrue(spawned["spawned"])
        self.assertTrue(any("/world/default/create" in " ".join(c) for c in self.calls))

        sim = runtime.handle_simulate(
            {
                "duration_s": 0.2,
                "dt_s": 0.02,
                "output_interval": 0.1,
                "sdf_path": self.model_path,
                "mechanism": {"parts": [{"id": "frame"}]},
            }
        )
        self.assertEqual(sim["summary"]["engine_mode"], "gazebo_real")
        self.assertTrue(any("/world/default/control" in " ".join(c) for c in self.calls))

    def test_real_simulate_reports_nothing_it_did_not_measure(self) -> None:
        """Regression: a real run must not return the stub's invented numbers.

        ``RealGazeboRuntime.handle_simulate`` used to delegate to the stub and
        relabel the summary ``gazebo_real``, so every part came back at 120 rpm
        and every joint at 5 N.m whatever the world did.  Core cannot tell an
        invented number from a measured one, so the only safe answer when a
        quantity was not read back is to leave the field out.

        This runner answers the pose topic with an error, so nothing is
        measurable and nothing may be reported.
        """
        with patch("gazebo_bridge.runtime_gazebo.shutil.which", return_value="/usr/bin/gz"):
            runtime = RealGazeboRuntime(world_name="default", command_runner=self.runner)

        sim = runtime.handle_simulate(
            {
                "duration_s": 0.2,
                "dt_s": 0.02,
                "output_interval": 0.1,
                "sdf_path": self.model_path,
                "mechanism": {
                    "parts": [{"id": "frame"}, {"id": "gear_a"}],
                    "joints": [{"id": "rev_a"}],
                },
            }
        )
        summary = sim["summary"]
        self.assertNotIn("steady_state_speeds", summary)
        self.assertNotIn("peak_joint_forces", summary)
        for entry in sim["time_series"]:
            self.assertEqual(entry["parts"], {})
            self.assertNotIn("joint_efforts", entry)

    def test_real_simulate_reports_measured_poses(self) -> None:
        """With the pose topic answering, the series is what the world said."""
        pose_frames = [
            'pose {\n  name: "gear_a"\n  position {\n    z: 0.5\n  }\n'
            "  orientation {\n    w: 1\n  }\n}\n",
            # 45 degrees about z over one 0.1 s sample: 7.854 rad/s = 75 rpm.
            'pose {\n  name: "gear_a"\n  position {\n    z: 0.5\n  }\n'
            "  orientation {\n    z: 0.3826834\n    w: 0.9238795\n  }\n}\n",
        ]

        reads = iter(pose_frames)

        def _runner(cmd: list[str]) -> tuple[int, str, str]:
            if "dynamic_pose/info" in " ".join(cmd):
                return 0, next(reads, pose_frames[-1]), ""
            return self.runner(cmd)

        with patch("gazebo_bridge.runtime_gazebo.shutil.which", return_value="/usr/bin/gz"):
            runtime = RealGazeboRuntime(world_name="default", command_runner=_runner)

        sim = runtime.handle_simulate(
            {"duration_s": 0.1, "dt_s": 0.01, "output_interval": 0.1, "sdf_path": self.model_path}
        )
        self.assertIn("gear_a", sim["time_series"][-1]["parts"])
        self.assertAlmostEqual(sim["summary"]["steady_state_speeds"]["gear_a"], 75.0, places=3)

    def test_diagnose_reports_worlds(self) -> None:
        with patch("gazebo_bridge.runtime_gazebo.shutil.which", return_value="/usr/bin/gz"):
            runtime = RealGazeboRuntime(world_name="default", command_runner=self.runner)
        diag = runtime.handle_diagnose({})
        self.assertTrue(diag["connected"])
        self.assertIn("default", diag["worlds"])


class TestPoseReadback(unittest.TestCase):
    """Parsing ``gz topic -e`` output — protobuf text with zeros omitted."""

    def test_omitted_components_default_to_zero_and_identity(self) -> None:
        text = (
            "header {\n  stamp {\n    sec: 3\n  }\n}\n"
            'pose {\n  name: "base"\n  id: 11\n  position {\n    z: 0.5\n  }\n'
            "  orientation {\n    w: 1\n  }\n}\n"
        )
        poses = _parse_pose_message(text)
        self.assertEqual(poses["base"]["pos_m"], [0.0, 0.0, 0.5])
        self.assertEqual(poses["base"]["quat_wxyz"], [1.0, 0.0, 0.0, 0.0])

    def test_every_moving_link_is_returned(self) -> None:
        text = "".join(
            f'pose {{\n  name: "{name}"\n  position {{\n    x: {x}\n  }}\n}}\n'
            for name, x in (("a", 1.0), ("b", 2.0))
        )
        self.assertEqual(sorted(_parse_pose_message(text)), ["a", "b"])

    def test_a_still_body_reports_zero_speed(self) -> None:
        pose = {"link": {"pos_m": [0.0, 0.0, 0.0], "quat_wxyz": [1.0, 0.0, 0.0, 0.0]}}
        self.assertEqual(_angular_speeds_rpm([(0.0, pose), (0.1, pose)], 0.1), {"link": 0.0})

    def test_a_single_sample_yields_no_speeds(self) -> None:
        self.assertEqual(_angular_speeds_rpm([(0.0, {})], 0.1), {})
