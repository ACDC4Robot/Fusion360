"""Exercise names at the XML writer boundary, including legacy joint names."""
import importlib
from pathlib import Path
import sys
from tempfile import TemporaryDirectory
from types import ModuleType, SimpleNamespace as NS
import unittest
from xml.etree import ElementTree as ET

import test_mjcf_export as fusion_stubs

adsk = fusion_stubs.adsk
if not hasattr(adsk.fusion, "BRepBody"):
    adsk.fusion.BRepBody = type("FusionBRepBody", (), {})
if not hasattr(adsk.fusion, "Design"):
    adsk.fusion.Design = type("FusionDesign", (), {})
if not hasattr(adsk.fusion, "CalculationAccuracy"):
    adsk.fusion.CalculationAccuracy = NS(VeryHighCalculationAccuracy=1)

# Load the real exporters under a private package so importing constants never
# starts the Fusion command UI or changes another test's package dependencies.
PACKAGE = "acdc_export_name_test"
ADDIN_ROOT = Path(__file__).resolve().parents[1] / "Add-IN" / "ACDC4Robot"
for name, module_path in ((PACKAGE, [str(ADDIN_ROOT)]),
                          (f"{PACKAGE}.commands", []),
                          (f"{PACKAGE}.commands.ACDC4Robot", [])):
    module = ModuleType(name)
    module.__path__ = module_path
    sys.modules[name] = module
constants = ModuleType(f"{PACKAGE}.commands.ACDC4Robot.constants")
constants.get_text_palette = lambda: None
sys.modules[constants.__name__] = constants
urdf = importlib.import_module(f"{PACKAGE}.core.urdf")
write = importlib.import_module(f"{PACKAGE}.core.write")
Joint = importlib.import_module(f"{PACKAGE}.core.joint").Joint
validate_export_names = importlib.import_module(f"{PACKAGE}.core.serialization").validate_export_names
URDF_PLUS = importlib.import_module(f"{PACKAGE}.core.urdf_plus").URDF_PLUS


class ExportNameCollisionTests(unittest.TestCase):
    def setUp(self):
        self.occurrences = [fusion_stubs.Occurrence(path) for path in
                            ("base:1", "arm:1", "tool:1")]
        for occurrence in self.occurrences:
            occurrence.getPhysicalProperties = lambda accuracy: NS()
        self.joints = []
        for name, parent, child in zip(("Joint A", "Joint_A"),
                                       self.occurrences, self.occurrences[1:]):
            joint = Joint(fusion_stubs.Joint(name, parent, child, 0))
            joint.has_origin = lambda: False
            joint.get_sdf_origin = lambda: [0.0] * 6
            self.joints.append(joint)

    def test_legacy_urdf_rejects_sanitized_joint_collision_before_write(self):
        # Link identities remain unique, so occurrence validation alone cannot
        # catch the new collision introduced by replacing spaces with '_'.
        validate_export_names(self.occurrences)
        self.assertEqual([joint.get_name() for joint in self.joints],
                         ["Joint_A", "Joint_A"])
        with TemporaryDirectory() as folder:
            path = Path(folder) / "robot.urdf"
            path.write_text("existing export", encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "duplicate joint export name.*Joint_A"):
                write.write_urdf([], self.joints, folder, "robot")
            self.assertEqual(path.read_text(encoding="utf-8"), "existing export")

    def test_sdf_rejects_sanitized_joint_collision_before_write(self):
        with TemporaryDirectory() as folder:
            with self.assertRaisesRegex(ValueError, "duplicate joint export name.*Joint_A"):
                write.write_sdf([], self.joints, folder, "robot")
            self.assertFalse((Path(folder) / "robot.sdf").exists())

    def class_exporter(self, exporter_type=urdf.URDF):
        exporter = exporter_type.__new__(exporter_type)
        exporter.links = []
        exporter.tree_joints = self.joints
        exporter.loop_joints = []
        exporter.robot = NS(get_robot_name=lambda: "robot")
        return exporter

    def test_class_urdf_keeps_distinct_child_prefixed_joint_names(self):
        with TemporaryDirectory() as folder:
            path = Path(folder) / "robot.urdf"
            self.class_exporter().write_file(path)
            names = [joint.get("name") for joint in ET.parse(path).getroot().findall("joint")]
            self.assertEqual(names, ["arm_1_Joint_A", "tool_1_Joint_A"])

    def test_class_writers_reject_actual_emitted_name_collisions(self):
        self.joints[1].child = self.joints[0].child
        for exporter_type in (urdf.URDF, URDF_PLUS):
            with self.subTest(exporter=exporter_type.__name__), TemporaryDirectory() as folder:
                path = Path(folder) / "robot.urdf"
                with self.assertRaisesRegex(ValueError, "duplicate joint export name.*arm_1_Joint_A"):
                    self.class_exporter(exporter_type).write_file(path)
                self.assertFalse(path.exists())


if __name__ == "__main__":
    unittest.main()
