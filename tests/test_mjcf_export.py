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
adsk.fusion.Joint = type("FusionJoint", (), {})
adsk.fusion.AsBuiltJoint = type("FusionAsBuiltJoint", (), {})
adsk.fusion.Occurrence = type("FusionOccurrence", (), {})
adsk.fusion.JointList = type("FusionJointList", (), {})


class FusionJointOrigin:
    def __init__(self, geometry):
        self.geometry = geometry


adsk.fusion.JointOrigin = FusionJointOrigin


class Vector:
    def __init__(self, x=0.0, y=0.0, z=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.values = [x, y, z]

    def asArray(self):
        return list(self.values)


class Point(Vector):
    @classmethod
    def create(cls, x, y, z):
        return cls(x, y, z)


adsk.core.Vector3D = Vector
adsk.core.Point3D = Point


class FusionMatrix:
    def __init__(self, values=None):
        self.values = values or [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ]

    @classmethod
    def create(cls):
        return cls()

    @classmethod
    def translated(cls, x, y, z):
        matrix = cls()
        matrix.values[0][3] = x
        matrix.values[1][3] = y
        matrix.values[2][3] = z
        return matrix

    def copy(self):
        return FusionMatrix([row[:] for row in self.values])

    def invert(self):
        self.values[0][3] *= -1.0
        self.values[1][3] *= -1.0
        self.values[2][3] *= -1.0

    def getCell(self, row, column):
        return self.values[row][column]

    def setCell(self, row, column, value):
        self.values[row][column] = value

    def setWithCoordinateSystem(self, origin, x_axis, y_axis, z_axis):
        self.values[0][3], self.values[1][3], self.values[2][3] = origin.asArray()

    @property
    def translation(self):
        return Point(self.values[0][3], self.values[1][3], self.values[2][3])


adsk.core.Matrix3D = FusionMatrix
sys.modules.setdefault("adsk", adsk)
sys.modules.setdefault("adsk.core", adsk.core)
sys.modules.setdefault("adsk.fusion", adsk.fusion)

from ACDC4Robot.core import math_operation, preflight, utils  # noqa: E402
from ACDC4Robot.core.joint import Joint as ExportJoint  # noqa: E402


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


class Body:
    def __init__(self, is_solid=True):
        self.isLightBulbOn = True
        self.isSolid = is_solid


class Component:
    def __init__(self, name, is_solid=True, mesh_only=False):
        self.name = name
        self.bRepBodies = Collection([] if mesh_only else [Body(is_solid=is_solid)])
        self.meshBodies = Collection([Body()] if mesh_only else [])
        self.isBodiesFolderLightBulbOn = True


class Occurrence:
    def __init__(self, path, referenced=True, is_solid=True, mesh_only=False):
        self.fullPathName = path
        self.component = Component(path.split(":")[0], is_solid=is_solid, mesh_only=mesh_only)
        self.isLightBulbOn = True
        self.childOccurrences = Collection([])
        self.joints = Collection([])
        self.transform2 = FusionMatrix()
        self.isReferencedComponent = referenced


class Motion:
    def __init__(self, joint_type):
        self.jointType = joint_type
        self.rotationAxisVector = Vector(0.0, 0.0, 1.0)
        self.slideDirectionVector = Vector(1.0, 0.0, 0.0)
        self.primarySlideDirectionVector = Vector(1.0, 0.0, 0.0)
        self.secondarySlideDirectionVector = Vector(0.0, 1.0, 0.0)


class Geometry:
    def __init__(self):
        self.origin = Point(0.0, 0.0, 0.0)
        self.primaryAxisVector = Vector(0.0, 0.0, 1.0)
        self.secondaryAxisVector = Vector(1.0, 0.0, 0.0)
        self.thirdAxisVector = Vector(0.0, 1.0, 0.0)


class Joint:
    def __init__(self, name, parent, child, joint_type, geometry=None):
        self.name = name
        self.occurrenceTwo = parent
        self.occurrenceOne = child
        self.jointMotion = Motion(joint_type)
        self.objectType = "adsk::fusion::Joint"
        self.geometryOrOriginTwo = Geometry() if geometry is None else geometry


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

    def test_coordinate_transform_does_not_mutate_source_frame(self):
        source = FusionMatrix.translated(2.0, 0.0, 0.0)
        target = FusionMatrix.translated(5.0, 0.0, 0.0)
        result = math_operation.coordinate_transform(source, target)
        self.assertEqual(source.getCell(0, 3), 2.0)
        self.assertEqual(result.getCell(0, 3), 3.0)

    def test_surface_brep_is_rejected_before_export(self):
        surface = Occurrence("SurfacePart:1", is_solid=False)
        report = preflight.inspect_mjcf_design(Design([surface], []))
        self.assertFalse(report["passed"])
        self.assertIn("surface BRep", " ".join(report["errors"]))

    def test_mesh_only_occurrence_is_rejected_before_export(self):
        mesh = Occurrence("MeshPart:1", mesh_only=True)
        report = preflight.inspect_mjcf_design(Design([mesh], []))
        self.assertFalse(report["passed"])
        self.assertIn("only Fusion mesh bodies", " ".join(report["errors"]))

    def test_report_contains_release_identity(self):
        report = preflight.inspect_mjcf_design(Design([self.servo], []))
        self.assertTrue(report["passed"], report["errors"])
        self.assertEqual(report["plugin_version"], "1.2.0")
        self.assertEqual(report["active_dof_count"], 0)
        self.assertIn("exported model is rigid", " ".join(report["warnings"]))

    def test_fusion_manifest_matches_python_release_identity(self):
        manifest_path = REPOSITORY_ROOT / "Add-IN" / "ACDC4Robot" / "ACDC4Robot.manifest"
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        self.assertEqual(manifest["version"], "1.2.0")
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

    def test_joint_origin_wrapper_is_unwrapped(self):
        geometry = Geometry()
        wrapped = Joint(
            "servo_hinge",
            self.servo,
            self.link,
            1,
            geometry=FusionJointOrigin(geometry),
        )
        exported = ExportJoint(wrapped)
        self.assertIs(exported._origin_geometry(), geometry)
        self.assertTrue(exported.has_origin())

    def test_grounded_joint_has_actionable_failure(self):
        exported = ExportJoint(Joint("grounded_hinge", None, self.link, 1))
        self.assertFalse(exported.is_valid())
        with self.assertRaisesRegex(ValueError, "two component occurrences"):
            exported.get_parent()

    def test_unsupported_joint_type_has_actionable_failure(self):
        exported = ExportJoint(Joint("cylindrical_joint", self.servo, self.link, 3))
        with self.assertRaisesRegex(ValueError, "Cylindrical"):
            exported.get_mjcf_joint_type()
        self.assertEqual(exported.get_axes(), ([0.0, 0.0, 1.0], None))

    def test_ball_joint_axis_accessor_always_returns_tuple(self):
        exported = ExportJoint(Joint("ball_joint", self.servo, self.link, 6))
        self.assertEqual(exported.get_axes(), (None, None))

    def test_rigid_as_built_joint_without_origin_uses_occurrence_frames(self):
        self.link.transform2 = FusionMatrix.translated(5.0, 0.0, 0.0)
        joint = Joint("rigid_fixture", self.servo, self.link, 0, geometry=False)
        joint.geometryOrOriginTwo = None
        pose = ExportJoint(joint).get_urdf_origin()
        self.assertEqual(pose[:3], [0.05, 0.0, 0.0])

    def test_moving_joint_without_origin_fails_preflight(self):
        joint = Joint("missing_origin", self.servo, self.link, 1, geometry=False)
        joint.geometryOrOriginTwo = None
        report = preflight.inspect_mjcf_design(Design([self.servo, self.link], [joint]))
        self.assertFalse(report["passed"])
        self.assertIn("no usable origin geometry", " ".join(report["errors"]))

    def test_chinese_preflight_display_does_not_change_english_report(self):
        report = preflight.inspect_mjcf_design(Design(self.occurrences, []))
        displayed = preflight.format_report(report, locale="zh_CN")
        self.assertIn("ACDC4Robot MJCF 导出前检查", displayed)
        self.assertIn("可见连杆: 6", displayed)
        self.assertIn("MJCF 要求一个连通的实体树", displayed)
        self.assertIn("MJCF requires one connected body tree", " ".join(report["errors"]))


if __name__ == "__main__":
    unittest.main()
