"""Numerical URDF joint-frame regressions using rigid Fusion matrix doubles."""
import math
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import test_urdf_root_inertia as fixtures


class RigidMatrix(fixtures.Matrix):
    def copy(self):
        return type(self)([row[:] for row in self.values])

    def invert(self):
        old = self.values
        rotation = [[old[column][row] for column in range(3)] for row in range(3)]
        self.values = [
            rotation[row] + [-sum(rotation[row][column] * old[column][3]
                                  for column in range(3))]
            for row in range(3)
        ] + [[0.0, 0.0, 0.0, 1.0]]
        self._update_translation()

    def setCell(self, row, column, value):
        self.values[row][column] = value
        self._update_translation()

    def _update_translation(self):
        self.translation = fixtures.Point(*(self.values[row][3] for row in range(3)))


def frame(rotation, translation):
    return RigidMatrix([list(row) + [offset] for row, offset in
                        zip(rotation, translation)] + [[0.0, 0.0, 0.0, 1.0]])


IDENTITY = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
ROTATE_X_90 = [[1, 0, 0], [0, 0, -1], [0, 1, 0]]
ROTATE_Z_90 = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]


def link(pose, parent_joint=None):
    return NS(pose=pose, get_parent_joint=lambda: parent_joint)


def joint(parent, pose, has_origin=True):
    return NS(parent=parent, has_origin=lambda: has_origin,
              get_joint_frame=lambda: pose)


def pose_matrix_si(pose):
    """Independent URDF fixed-axis roll/pitch/yaw reconstruction, in metres."""
    x, y, z, roll, pitch, yaw = pose
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
        [-sp, cp * sr, cp * cr, z],
        [0.0, 0.0, 0.0, 1.0],
    ]


class UrdfJointFrameTests(unittest.TestCase):
    def setUp(self):
        self.urdf = fixtures.urdf_module
        self.writer = object.__new__(self.urdf.URDF)
        for target, name, replacement in (
            (fixtures.adsk.core, "Matrix3D", RigidMatrix),
            (self.urdf, "Link", lambda occurrence: occurrence),
            (self.urdf, "Joint", lambda api_joint: api_joint),
        ):
            patcher = patch.object(target, name, replacement)
            patcher.start()
            self.addCleanup(patcher.stop)

    def assertVectorAlmostEqual(self, actual, expected):
        self.assertEqual(len(actual), len(expected))
        for left, right in zip(actual, expected):
            self.assertAlmostEqual(left, right, places=12)

    def assertJointOrigins(self, item, expected):
        for origin in (self.writer.get_joint_origin, self.urdf.get_joint_origin):
            with self.subTest(builder=origin.__qualname__):
                self.assertVectorAlmostEqual(origin(item), expected)

    def test_rotated_translated_root_joint_uses_world_frame(self):
        root = link(frame(ROTATE_X_90, [10, 20, 30]))
        item = joint(root, frame(ROTATE_Z_90, [15, 25, 40]))
        self.assertJointOrigins(item, [0.15, 0.25, 0.4, 0.0, 0.0, math.pi / 2])
        self.assertVectorAlmostEqual(
            [root.pose.translation.x, root.pose.translation.y, root.pose.translation.z],
            [10, 20, 30],
        )

    def test_identity_root_joint_remains_unchanged(self):
        root = link(frame(IDENTITY, [0, 0, 0]))
        item = joint(root, frame(ROTATE_Z_90, [15, 25, 40]))
        self.assertJointOrigins(item, [0.15, 0.25, 0.4, 0.0, 0.0, math.pi / 2])

    def test_descendant_uses_geometric_parent_joint_frame(self):
        incoming = joint(None, frame(ROTATE_Z_90, [1, 2, 3]))
        parent = link(frame(ROTATE_X_90, [7, 8, 9]), incoming)
        item = joint(parent, frame(ROTATE_Z_90, [1, 12, 3]))
        self.assertJointOrigins(item, [0.1, 0.0, 0.0, 0.0, 0.0, 0.0])

    def test_descendant_without_geometric_parent_keeps_occurrence_fallback(self):
        incoming = joint(None, None, has_origin=False)
        parent = link(frame(ROTATE_Z_90, [1, 2, 3]), incoming)
        item = joint(parent, frame(ROTATE_Z_90, [1, 12, 3]))
        self.assertJointOrigins(item, [0.1, 0.0, 0.0, 0.0, 0.0, 0.0])

    def test_joint_and_child_mesh_compose_to_fusion_world_pose(self):
        root = link(frame(ROTATE_X_90, [10, 20, 30]))
        item = joint(root, frame(ROTATE_Z_90, [15, 25, 40]))
        child = link(frame(ROTATE_Z_90, [18, 27, 43]), item)
        expected = [[0, -1, 0, 0.18], [1, 0, 0, 0.27],
                    [0, 0, 1, 0.43], [0, 0, 0, 1]]
        for joint_origin, mesh_origin in (
            (self.writer.get_joint_origin, self.writer.get_mesh_origin),
            (self.urdf.get_joint_origin, self.urdf.get_mesh_origin),
        ):
            with self.subTest(builder=joint_origin.__qualname__):
                joint_world = pose_matrix_si(joint_origin(item))
                mesh_local = pose_matrix_si(mesh_origin(child))
                actual = [[sum(joint_world[row][k] * mesh_local[k][column]
                               for k in range(4)) for column in range(4)]
                          for row in range(4)]
                for actual_row, expected_row in zip(actual, expected):
                    self.assertVectorAlmostEqual(actual_row, expected_row)


if __name__ == "__main__":
    unittest.main()
