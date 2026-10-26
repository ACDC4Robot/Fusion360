"""Optional collision pipeline tests; skip cleanly without its extra dependencies."""
import importlib.util
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from xml.etree import ElementTree as ET

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "convexify_urdf.py"
spec = importlib.util.spec_from_file_location("convexify_urdf_test", SCRIPT)
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)
try:
    import coacd
    import numpy as np
    import trimesh
    HAVE_GEOMETRY = True
except ImportError:
    HAVE_GEOMETRY = False


def solid_box(path, extents=(100, 200, 300)):
    trimesh.creation.box(extents=extents).export(path)


def concave_l_prism():
    # Extrude a triangulated L polygon without optional shapely dependencies.
    polygon = np.array([[0, 0], [100, 0], [100, 50], [50, 50], [50, 100], [0, 100]])
    vertices = np.vstack([np.column_stack([polygon, np.zeros(6)]), np.column_stack([polygon, np.full(6, 50)])])
    caps = [[0, 1, 3], [1, 2, 3], [0, 3, 5], [3, 4, 5]]
    faces = [[c, b, a] for a, b, c in caps] + [[a + 6, b + 6, c + 6] for a, b, c in caps]
    for a in range(6):
        b = (a + 1) % 6
        faces.extend([[a, b, b + 6], [a, b + 6, a + 6]])
    return trimesh.Trimesh(vertices=vertices, faces=faces)


@unittest.skipUnless(HAVE_GEOMETRY, "optional collision requirements not installed")
class CollisionPipelineTests(unittest.TestCase):
    def setUp(self):
        self.temporary = TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.folder = Path(self.temporary.name)
        self.source_dir = self.folder / "source"
        self.source_dir.mkdir()
        self.source = self.source_dir / "robot.urdf"
        self.output = self.folder / "result"
        solid_box(self.source_dir / "part.stl")
        (self.source_dir / "texture.png").write_bytes(b"texture bytes are copied, not interpreted")
        self.source.write_text('''<robot name="test"><!-- keep this comment -->
          <material name="paint"><texture filename="texture.png"/></material>
          <link name="root"><inertial><mass value="2"/></inertial>
          <visual name="cad"><origin xyz="1 2 3" rpy="0.1 0.2 0.3"/>
          <geometry><mesh filename="part.stl" scale="0.001 0.002 0.003"/></geometry>
          <material name="paint"/></visual>
          <collision name="contact"><origin xyz="-1 2 -3" rpy="0.4 -0.2 1.5"/>
          <geometry><mesh filename="part.stl" scale="0.001 0.002 0.003"/></geometry>
          <contact_coefficients mu="0.7"/></collision>
          <collision name="primitive"><origin xyz="0 0 1"/><geometry><sphere radius="0.01"/></geometry></collision>
          </link><link name="child"/><joint name="fixed" type="fixed"><parent link="root"/><child link="child"/></joint>
          </robot>''', encoding="utf-8")
        self.original = self.source.read_bytes()
        self.asset_original = (self.source_dir / "part.stl").read_bytes()

    @staticmethod
    def split_as_two(vertices, faces, options):
        # The mocked backend isolates URDF scale/origin preservation from CoACD.
        return [(vertices, faces), (vertices, faces)]

    def test_scale_origins_visuals_materials_and_primitives_preserved(self):
        observed = []
        def backend(vertices, faces, options):
            observed.append((vertices.copy(), dict(options)))
            return self.split_as_two(vertices, faces, options)
        report = tool.convert(self.source, self.output, decompose=backend, threshold=0.07, seed=42)
        np.testing.assert_allclose(np.ptp(observed[0][0], axis=0), [0.1, 0.4, 0.9])
        self.assertEqual(observed[0][1], dict(tool.COACD_OPTIONS, threshold=0.07, seed=42))
        root = ET.parse(self.output / "robot.urdf").getroot()
        visual = root.find("link/visual")
        self.assertEqual(visual.find("origin").attrib, {"xyz": "1 2 3", "rpy": "0.1 0.2 0.3"})
        self.assertEqual(visual.find("material").get("name"), "paint")
        self.assertEqual(visual.find("geometry/mesh").get("scale"), "0.001 0.002 0.003")
        self.assertEqual((self.output / visual.find("geometry/mesh").get("filename")).read_bytes(), self.asset_original)
        texture = root.find("material/texture")
        self.assertEqual((self.output / texture.get("filename")).read_bytes(), (self.source_dir / "texture.png").read_bytes())
        collisions = root.findall("link/collision")
        self.assertEqual(len(collisions), 3)
        for collision in collisions[:2]:
            self.assertEqual(collision.find("origin").attrib, {"xyz": "-1 2 -3", "rpy": "0.4 -0.2 1.5"})
            self.assertEqual(collision.find("contact_coefficients").get("mu"), "0.7")
            mesh_element = collision.find("geometry/mesh")
            self.assertEqual(mesh_element.get("scale"), "1 1 1")
            hull = trimesh.load_mesh(self.output / mesh_element.get("filename"))
            np.testing.assert_allclose(np.ptp(hull.vertices, axis=0), [0.1, 0.4, 0.9])
        self.assertEqual(collisions[-1].find("geometry/sphere").get("radius"), "0.01")
        self.assertEqual(root.find("joint").get("name"), "fixed")
        self.assertEqual(root.find("link/inertial/mass").get("value"), "2")
        self.assertIn("keep this comment", (self.output / "robot.urdf").read_text())
        self.assertEqual(self.source.read_bytes(), self.original)
        self.assertEqual((self.source_dir / "part.stl").read_bytes(), self.asset_original)
        self.assertEqual(report["options"]["max_convex_hull"], -1)
        self.assertFalse(report["options"]["decimate"])
        self.assertEqual(len(report["decompositions"][0]["pieces"]), 2)

    def test_backend_failure_or_invalid_piece_publishes_nothing(self):
        def fail(*args):
            raise RuntimeError("backend stopped")
        for backend in (fail, lambda *args: [], lambda *args: [(np.zeros((4, 3)), np.array([[0, 1, 2]] * 4))]):
            with self.subTest(backend=backend), self.assertRaises((ValueError, RuntimeError)):
                tool.convert(self.source, self.output, decompose=backend)
            self.assertFalse(self.output.exists())
            self.assertEqual(list(self.folder.glob(".result.stage-*")), [])
            self.assertEqual(self.source.read_bytes(), self.original)

    def test_open_reversed_and_nonconvex_backend_pieces_are_rejected(self):
        box = trimesh.creation.box()
        open_box = trimesh.Trimesh(vertices=box.vertices, faces=box.faces[:-2], process=False)
        reversed_box = trimesh.Trimesh(vertices=box.vertices, faces=box.faces[:, ::-1], process=False)
        inconsistent_faces = box.faces.copy()
        inconsistent_faces[0] = inconsistent_faces[0, ::-1]
        inconsistent_box = trimesh.Trimesh(vertices=box.vertices, faces=inconsistent_faces, process=False)
        for piece in (open_box, reversed_box, inconsistent_box, concave_l_prism()):
            with self.subTest(piece=piece), self.assertRaisesRegex(ValueError, "closed, outward-wound convex solid"):
                tool.convert(self.source, self.output, decompose=lambda *args: [(piece.vertices, piece.faces)])
            self.assertFalse(self.output.exists())
            self.assertEqual(list(self.folder.glob(".result.stage-*")), [])
            self.assertEqual(self.source.read_bytes(), self.original)

    def test_coincident_duplicate_shells_are_rejected_without_repair(self):
        box = trimesh.creation.box()
        vertices = np.vstack([box.vertices, box.vertices])
        faces = np.vstack([box.faces, box.faces + len(box.vertices)])
        original_vertices, original_faces = vertices.copy(), faces.copy()
        with self.assertRaisesRegex(ValueError, "closed, outward-wound convex solid"):
            tool.convert(self.source, self.output, decompose=lambda *args: [(vertices, faces)])
        self.assertFalse(self.output.exists())
        self.assertEqual(list(self.folder.glob(".result.stage-*")), [])
        self.assertEqual(self.source.read_bytes(), self.original)
        np.testing.assert_array_equal(vertices, original_vertices)
        np.testing.assert_array_equal(faces, original_faces)

    def test_later_collision_failure_does_not_publish_earlier_hulls(self):
        root = ET.parse(self.source).getroot()
        link = root.find("link")
        extra = ET.SubElement(link, "collision", name="second_mesh")
        ET.SubElement(ET.SubElement(extra, "geometry"), "mesh", filename="part.stl", scale="0.001 0.001 0.001")
        ET.ElementTree(root).write(self.source, encoding="utf-8")
        calls = []
        def backend(vertices, faces, options):
            calls.append(1)
            if len(calls) == 2:
                raise RuntimeError("second mesh stopped")
            return [(vertices, faces)]
        with self.assertRaisesRegex(RuntimeError, "second mesh"):
            tool.convert(self.source, self.output, decompose=backend)
        self.assertEqual(len(calls), 2)
        self.assertFalse(self.output.exists())
        self.assertEqual(list(self.folder.glob(".result.stage-*")), [])

    def test_invalid_scale_settings_and_missing_asset_publish_nothing(self):
        for scale in ("0 1 1", "-1 1 1", "nan 1 1", "1 1", "bad 1 1"):
            self.source.write_text(self.original.decode().replace("0.001 0.002 0.003", scale))
            with self.subTest(scale=scale), self.assertRaisesRegex(ValueError, "Mesh scale"):
                tool.convert(self.source, self.output)
        self.source.write_bytes(self.original)
        for settings in ({"threshold": float("nan")}, {"threshold": 0}, {"seed": -1}):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                tool.convert(self.source, self.output, **settings)
        (self.source_dir / "part.stl").unlink()
        with self.assertRaises(FileNotFoundError):
            tool.convert(self.source, self.output)
        self.assertFalse(self.output.exists())

    def test_rejects_existing_nested_and_invalid_asset_paths(self):
        self.output.mkdir()
        (self.output / "keep").write_text("existing")
        with self.assertRaisesRegex(ValueError, "already exist"):
            tool.convert(self.source, self.output)
        self.assertEqual((self.output / "keep").read_text(), "existing")
        with self.assertRaisesRegex(ValueError, "non-nested"):
            tool.convert(self.source, self.source_dir / "nested")
        self.source.write_text(self.original.decode().replace('filename="part.stl"', 'filename="package://robot/part.stl"'))
        with self.assertRaisesRegex(ValueError, "Unsupported asset URI"):
            tool.convert(self.source, self.folder / "other")
        self.assertFalse((self.folder / "other").exists())

    def test_unsupported_mesh_and_open_mesh_fail_before_backend(self):
        (self.source_dir / "part.obj").write_text("v 0 0 0\n")
        self.source.write_text(self.original.decode().replace("part.stl", "part.obj"))
        with self.assertRaisesRegex(ValueError, "Unsupported mesh format"):
            tool.convert(self.source, self.output)
        self.source.write_bytes(self.original)
        trimesh.Trimesh(vertices=[[0, 0, 0], [1, 0, 0], [0, 1, 0]], faces=[[0, 1, 2]]).export(self.source_dir / "part.stl")
        with self.assertRaisesRegex(ValueError, "must be finite, closed"):
            tool.convert(self.source, self.output, decompose=lambda *args: self.fail("backend called"))
        self.assertFalse(self.output.exists())

    def test_settings_and_provenance_are_repeatable_for_same_backend(self):
        first = tool.convert(self.source, self.output, decompose=self.split_as_two)
        second_output = self.folder / "again"
        second = tool.convert(self.source, second_output, decompose=self.split_as_two)
        self.assertEqual(first, second)
        self.assertEqual((self.output / "collision-provenance.json").read_bytes(), (second_output / "collision-provenance.json").read_bytes())
        self.assertEqual(first["source_urdf_sha256"], tool.sha256(self.source))
        for piece in first["decompositions"][0]["pieces"]:
            self.assertEqual(piece["sha256"], tool.sha256(self.output / piece["path"]))

    @unittest.skipUnless(os.environ.get("ACDC_RUN_COACD") == "1", "set ACDC_RUN_COACD=1 for real native integration")
    def test_real_coacd_concave_l_prism(self):
        concave_l_prism().export(self.source_dir / "part.stl")
        report = tool.convert(self.source, self.output, threshold=0.03)
        pieces = report["decompositions"][0]["pieces"]
        self.assertGreaterEqual(len(pieces), 2)
        source_bounds = np.asarray(report["decompositions"][0]["source_bounds_m"])
        hull_bounds = np.asarray([piece["bounds_m"] for piece in pieces])
        combined = np.array([hull_bounds[:, 0].min(axis=0), hull_bounds[:, 1].max(axis=0)])
        # CoACD returns source-scale coordinates. Allow its normalized
        # approximation tolerance, while catching metre/mm unit mistakes.
        tolerance = float(np.ptp(source_bounds, axis=0).max()) * 0.03 + 1e-9
        np.testing.assert_allclose(combined, source_bounds, atol=tolerance, rtol=0)
        for piece in pieces:
            hull = trimesh.load_mesh(self.output / piece["path"])
            self.assertTrue(hull.is_watertight)
            # Each outward triangle supports the whole convex hull.
            for face in hull.faces:
                points = hull.vertices[face]
                normal = np.cross(points[1] - points[0], points[2] - points[0])
                self.assertLessEqual(float(np.max((hull.vertices - points[0]) @ normal)), 1e-10)
        root = ET.parse(self.output / "robot.urdf").getroot()
        self.assertEqual(len(root.findall("link/collision")), len(pieces) + 1)
