import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
CORE_ROOT = REPOSITORY_ROOT / "Add-IN" / "ACDC4Robot" / "core"
PACKAGE = "acdc_root_inertia_test"


class Point:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z

    @classmethod
    def create(cls, x=0.0, y=0.0, z=0.0):
        return cls(x, y, z)


class Vector(Point):
    pass


class Matrix:
    def __init__(self, values):
        self.values = values
        self.translation = Point(values[0][3], values[1][3], values[2][3])

    @classmethod
    def create(cls):
        return cls(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

    def getCell(self, row, column):
        return self.values[row][column]


def _module(name, **attributes):
    module = ModuleType(name)
    for key, value in attributes.items():
        setattr(module, key, value)
    sys.modules[name] = module
    return module


adsk = _module("adsk")
adsk.core = _module(
    "adsk.core",
    Application=type("Application", (), {}),
    Matrix3D=Matrix,
    Point3D=Point,
    Vector3D=Vector,
    Palette=type("Palette", (), {}),
)
adsk.fusion = _module(
    "adsk.fusion",
    Occurrence=type("Occurrence", (), {}),
    Joint=type("Joint", (), {}),
    AsBuiltJoint=type("AsBuiltJoint", (), {}),
    BRepBody=type("BRepBody", (), {}),
    Design=type("Design", (), {}),
    PhysicalProperties=type("PhysicalProperties", (), {}),
    CalculationAccuracy=NS(VeryHighCalculationAccuracy=3),
)

_module(PACKAGE).__path__ = []
_module(f"{PACKAGE}.core").__path__ = []
_module(f"{PACKAGE}.commands").__path__ = []
_module(f"{PACKAGE}.commands.ACDC4Robot").__path__ = []
_module(f"{PACKAGE}.commands.ACDC4Robot.constants", get_text_palette=lambda: None)
_module(f"{PACKAGE}.core.utils", get_valid_filename=str)


class StubJoint:
    def __init__(self, joint):
        self.joint = joint

    def has_origin(self):
        return False


_module(f"{PACKAGE}.core.joint", Joint=StubJoint)
_module(f"{PACKAGE}.core.robot", Robot=type("Robot", (), {}))


def _load(module_name, filename):
    spec = importlib.util.spec_from_file_location(module_name, CORE_ROOT / filename)
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


math_operation = _load(f"{PACKAGE}.core.math_operation", "math_operation.py")
link_module = _load(f"{PACKAGE}.core.link", "link.py")
urdf_module = _load(f"{PACKAGE}.core.urdf", "urdf.py")


class PhysicalProperties:
    mass = 2.0
    centerOfMass = Point(3.0, -4.0, 5.0)  # cm in Fusion world

    def getXYZMomentsOfInertia(self):
        # Desired CoM/world tensor in kg*m^2:
        # [[.2, .01, -.02], [.01, .3, .03], [-.02, .03, .4]]
        x, y, z = 0.03, -0.04, 0.05
        mass = self.mass
        return (
            True,
            (0.2 + mass * (y * y + z * z)) * 1.0e4,
            (0.3 + mass * (x * x + z * z)) * 1.0e4,
            (0.4 + mass * (x * x + y * y)) * 1.0e4,
            (0.01 - mass * x * y) * 1.0e4,
            (0.03 - mass * y * z) * 1.0e4,
            (-0.02 - mass * x * z) * 1.0e4,
        )


class RotatedRoot:
    phyPro = PhysicalProperties()
    # +90 degrees around X and a non-zero occurrence translation.  Neither is
    # part of the root URDF inertial frame after the fix.
    pose = Matrix(
        [
            [1.0, 0.0, 0.0, 10.0],
            [0.0, 0.0, -1.0, 20.0],
            [0.0, 1.0, 0.0, 30.0],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )

    def get_parent_joint(self):
        return None


class RootUrdfInertiaTests(unittest.TestCase):
    def setUp(self):
        self.link = RotatedRoot()
        self.expected_origin = [0.03, -0.04, 0.05, 0.0, 0.0, 0.0]
        self.expected_inertia = [0.2, 0.3, 0.4, 0.01, 0.03, -0.02]

    def assertVectorAlmostEqual(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for left, right in zip(actual, expected):
            self.assertAlmostEqual(left, right, places=12)

    def test_class_and_module_urdf_origins_use_world_coordinates(self):
        writer = object.__new__(urdf_module.URDF)
        self.assertVectorAlmostEqual(
            writer.get_link_inertial_origin(self.link), self.expected_origin
        )
        self.assertVectorAlmostEqual(
            urdf_module.get_link_inertial_origin(self.link), self.expected_origin
        )

    def test_class_and_module_urdf_inertias_keep_world_axes(self):
        writer = object.__new__(urdf_module.URDF)
        self.assertVectorAlmostEqual(
            writer.get_link_inertia(self.link), self.expected_inertia
        )
        self.assertVectorAlmostEqual(
            urdf_module.get_link_inertia(self.link), self.expected_inertia
        )

    def test_link_urdf_helpers_match_world_frame_contract(self):
        self.assertVectorAlmostEqual(
            link_module.Link.get_CoM_urdf(self.link), self.expected_origin
        )
        self.assertVectorAlmostEqual(
            link_module.Link.get_initia_urdf(self.link), self.expected_inertia
        )


if __name__ == "__main__":
    unittest.main()
