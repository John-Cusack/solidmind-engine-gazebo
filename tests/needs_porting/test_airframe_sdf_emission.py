"""Airframe spec → package → Gazebo SDF.

Moved out of core with the split: the assertions are about *Gazebo's* dialect
(motor plugin tags, joint pose omission), so they belong to the engine that
emits it.  Needs porting: it still builds the package with core's
``server.airframes`` + ``server.sim_package_manifest``.  Replace those with a
fixture package under ``tests/fixtures/`` and this runs standalone.
"""

class TestSDFEmission(unittest.TestCase):
    """End-to-end: spec → SimModel → SDF, assert the bug-fixed tags."""

    def _compile_sdf(self, af: MulticopterAirframe) -> str:
        """Run the real pipeline: spec → SimModel → package → engine-side SDF.

        Core writes only the manifest; the Gazebo bridge compiles the SDF from
        it at load time, so that is what this asserts against.
        """
        import tempfile as _tempfile

        from gazebo_bridge.package_to_sdf import compile_package_to_sdf
        from server.sim_package_manifest import build_manifest, write_manifest

        pkg_dir = _tempfile.mkdtemp()
        manifest = build_manifest(
            name=af.name,
            output_dir=pkg_dir,
            sim_model=af.to_sim_model(),
            drone_config=af.to_drone_config(),
        )
        write_manifest(manifest, pkg_dir)
        return compile_package_to_sdf(pkg_dir)

    def test_no_joint_pose_in_sdf(self) -> None:
        """Bug 5 regression: joint <pose> would double-offset rotation axis."""
        path = self._compile_sdf(x500_like())
        tree = ET.parse(path)
        for joint_el in tree.iter("joint"):
            self.assertIsNone(
                joint_el.find("pose"),
                f"Joint {joint_el.attrib.get('name')} must omit <pose> "
                "(SDF 1.10 default = child link frame, which is the joint origin)",
            )

    def test_motor_plugin_uses_motor_number_and_velocity(self) -> None:
        """Bugs C-7, C-8, C-9 regression: gz-sim motor plugin tag set."""
        path = self._compile_sdf(x500_like())
        tree = ET.parse(path)
        plugins = [
            p
            for p in tree.iter("plugin")
            if "MulticopterMotorModel" in (p.attrib.get("name") or "")
        ]
        self.assertEqual(len(plugins), 4)
        for p in plugins:
            self.assertIsNotNone(
                p.find("motorNumber"), "must use <motorNumber>, not <actuator_number>"
            )
            self.assertIsNone(
                p.find("actuator_number"), "<actuator_number> is the wrong tag for gz-sim"
            )
            self.assertIsNone(p.find("robotNamespace"), "<robotNamespace> breaks PX4 topic match")
            mt = p.find("motorType")
            self.assertIsNotNone(mt, "missing <motorType>velocity</motorType>")
            self.assertEqual(mt.text, "velocity")


