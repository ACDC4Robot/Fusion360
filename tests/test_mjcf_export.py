import json
import sys
import types
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
ADDIN_ROOT = REPOSITORY_ROOT / "Add-IN"
sys.path.insert(0, str(ADDIN_ROOT))

adsk = types.ModuleType("adsk")
adsk.core = types.ModuleType("adsk.core")
adsk.fusion = types.ModuleType("adsk.fusion")
adsk.fusion.Component = type("FusionComponent", (), {})
sys.modules.setdefault("adsk", adsk)
sys.modules.setdefault("adsk.core", adsk.core)
sys.modules.setdefault("adsk.fusion", adsk.fusion)

from ACDC4Robot.core import preflight, utils  # noqa: E402


class Collection:
    def __init__(self, values):
        self.values = list(values)
        self.count = len(self.values)

    def item(self, index):
        return self.values[index]

    def __iter__(self):
        return iter(self.values)


class IterableOnly:
    """Fusion exposes allJoints as this vector-like shape on current macOS builds."""

    def __init__(self, values):
        self.values = list(values)

    def __iter__(self):
        return iter(self.values)


class Matrix:
    def getCell(self, row, column):
        return 1.0 if row == column else 0.0


class Body:
    isLightBulbOn = True


class Component:
    def __init__(self, name):
        self.name = name
        self.bRepBodies = Collection([Body()])
        self.meshBodies = Collection([])
        self.isBodiesFolderLightBulbOn = True


class Occurrence:
    def __init__(self, path, referenced=True):
        self.fullPathName = path
        self.component = Component(path.split(":")[0])
        self.isLightBulbOn = True
        self.childOccurrences = Collection([])
        self.transform2 = Matrix()
        self.isReferencedComponent = referenced


class Motion:
    def __init__(self, joint_type):
        self.jointType = joint_type


class Joint:
    def __init__(self, name, parent, child, joint_type):
        self.name = name
        self.occurrenceTwo = parent
        self.occurrenceOne = child
        self.jointMotion = Motion(joint_type)
        self.objectType = "adsk::fusion::Joint"


class Root:
    def __init__(self, occurrences, joints):
        self.name = "MJCF-Test-Robot"
        self.allOccurrences = Collection(occurrences)
        self.allJoints = IterableOnly(joints)
        self.allAsBuiltJoints = IterableOnly([])


class Units:
    defaultLengthUnits = "mm"


class Design:
    def __init__(self, occurrences, joints):
        self.rootComponent = Root(occurrences, joints)
        self.unitsManager = Units()


class MJCFExportPatchTests(unittest.TestCase):
    def setUp(self):
        self.servo = Occurrence("XC330-M288-T:1")
        self.link = Occurrence("Link:1")
        self.pin1 = Occurrence("Pin:1")
        self.pin2 = Occurrence("Pin:2")
        self.payload1 = Occurrence("Payload:1")
        self.payload2 = Occurrence("Payload:2")
        self.occurrences = [
            self.servo,
            self.link,
            self.pin1,
            self.pin2,
            self.payload1,
            self.payload2,
        ]

    def test_repeated_occurrences_have_unique_export_names(self):
        self.assertEqual(utils.get_valid_filename("Payload:1"), "Payload_1")
        self.assertEqual(utils.get_valid_filename("Payload:2"), "Payload_2")
        self.assertNotEqual(
            utils.get_valid_filename("Root:1+Payload:1"),
            utils.get_valid_filename("Root:1+Payload:2"),
        )

    def test_iterable_only_joint_vector_is_supported(self):
        self.assertEqual(preflight._collection_items(IterableOnly([1, 2])), [1, 2])

    def test_report_contains_release_identity(self):
        report = preflight.inspect_mjcf_design(Design([self.servo], []))
        self.assertTrue(report["passed"], report["errors"])
        self.assertEqual(report["plugin_version"], "1.1.0")
        self.assertEqual(report["active_dof_count"], 0)
        self.assertIn("exported model is rigid", " ".join(report["warnings"]))

    def test_fusion_manifest_matches_python_release_identity(self):
        manifest_path = REPOSITORY_ROOT / "Add-IN" / "ACDC4Robot" / "ACDC4Robot.manifest"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        self.assertEqual(manifest["version"], "1.1.0")
        self.assertEqual(manifest["type"], "addin")
        self.assertEqual(manifest["supportedOS"], "windows|mac")

    def test_disconnected_screenshot_structure_is_rejected(self):
        report = preflight.inspect_mjcf_design(Design(self.occurrences, []))
        self.assertFalse(report["passed"])
        self.assertEqual(report["visible_link_count"], 6)
        self.assertEqual(report["active_dof_count"], 0)
        self.assertEqual(len(report["root_links"]), 6)

    def test_one_dof_connected_tree_passes(self):
        joints = [
            Joint("servo_hinge", self.servo, self.link, 1),
            Joint("link_pin_left", self.link, self.pin1, 0),
            Joint("link_pin_right", self.link, self.pin2, 0),
            Joint("pin_payload_left", self.pin1, self.payload1, 0),
            Joint("pin_payload_right", self.pin2, self.payload2, 0),
        ]
        report = preflight.inspect_mjcf_design(Design(self.occurrences, joints))
        self.assertTrue(report["passed"], report["errors"])
        self.assertEqual(report["active_dof_count"], 1)
        self.assertEqual(report["root_links"], ["XC330-M288-T:1"])

    def test_duplicate_joint_names_are_rejected(self):
        joints = [
            Joint("hinge", self.servo, self.link, 1),
            Joint("hinge", self.link, self.pin1, 0),
            Joint("pin_payload_left", self.pin1, self.payload1, 0),
        ]
        design = Design([self.servo, self.link, self.pin1, self.payload1], joints)
        report = preflight.inspect_mjcf_design(design)
        self.assertFalse(report["passed"])
        self.assertIn("Duplicate joint export name hinge", " ".join(report["errors"]))


if __name__ == "__main__":
    unittest.main()
