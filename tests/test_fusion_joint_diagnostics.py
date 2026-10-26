"""Exercise the standalone probe without claiming a live Fusion reproduction."""

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "fusion_joint_diagnostics_test",
    ROOT / "scripts/fusion_joint_diagnostics/fusion_joint_diagnostics.py",
)
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


def component(name):
    return NS(
        name=name, objectType="adsk::fusion::Component", isValid=True,
        occurrences=NS(count=1), bRepBodies=NS(count=2), meshBodies=[],
        allJoints=[], allAsBuiltJoints=[], joints=[], asBuiltJoints=[],
    )


class DiagnosticProbeTests(unittest.TestCase):
    def fixture(self):
        root, child = component("root"), component("arm")
        context = NS(fullPathName="arm:2")
        occurrence = NS(
            name="arm:2", fullPathName="arm:2", assemblyContext=None,
            component=child, isValid=True, isLightBulbOn=True,
            objectType="adsk::fusion::Occurrence",
        )
        joint = NS(
            name="hinge", objectType="adsk::fusion::Joint", isValid=True,
            occurrenceOne=occurrence, occurrenceTwo=None, assemblyContext=context,
            isSuppressed=False, jointMotion=NS(jointType=1),
        )
        root.allOccurrences = [occurrence]
        child.joints = [joint]
        return NS(version="test-fusion-build"), NS(rootComponent=root, allComponents=[root, child], designType=1)

    def test_metadata_preserves_proxy_context_and_marks_missing_constraint_api(self):
        app, design = self.fixture()
        report = probe.collect_report(app, design)
        self.assertEqual(report["fusion_version"]["value"], "test-fusion-build")
        self.assertEqual(report["components"]["count"], 2)
        self.assertEqual(report["occurrences"]["count"], 1)
        arm = report["components"]["items"][1]
        self.assertEqual(arm["collections"]["joints"]["items"][0]["assemblyContext"]["value"], "arm:2")
        self.assertEqual(arm["collections"]["assemblyConstraints"]["status"], "unavailable")
        self.assertEqual(arm["counts"]["bRepBodies"]["value"], 2)

    def test_property_failure_does_not_hide_successful_component_local_access(self):
        app, design = self.fixture()

        class FaultyRoot:
            @property
            def allJoints(self):
                raise RuntimeError("2 : InternalValidationError : jo")

            def __getattr__(self, name):
                return getattr(root, name)

        root = design.rootComponent
        design.rootComponent = FaultyRoot()
        report = probe.collect_report(app, design)
        query = report["root"]["collections"]["allJoints"]
        self.assertEqual((query["status"], query["stage"]), ("error", "property"))
        self.assertIn("InternalValidationError", query["error"])
        self.assertEqual(report["root"]["collections"]["allAsBuiltJoints"]["count"], 0)
        self.assertEqual(report["components"]["items"][1]["collections"]["joints"]["count"], 1)

    def test_iteration_failure_is_distinct_from_unsupported_api(self):
        class Broken:
            count = 0

            def __iter__(self):
                yield object()
                raise TypeError("iteration failed")

        query, objects = probe._collection(NS(allJoints=Broken()), "allJoints", probe._relationship)
        self.assertEqual(query["status"], "error")
        self.assertEqual(query["stage"], "iteration")
        self.assertEqual(query["error_type"], "TypeError")
        self.assertEqual(objects, [])

    def test_constraint_types_are_logged_without_geometry(self):
        app, design = self.fixture()
        constraint = NS(
            name="closure", objectType="adsk::fusion::AssemblyConstraint", isValid=True,
            isSuppressed=False, geometricRelationships=[NS(objectType="adsk::fusion::RigidRelationship")],
        )
        design.rootComponent.allAssemblyConstraints = [constraint]
        report = probe.collect_report(app, design)
        constraints = report["root"]["collections"]["allAssemblyConstraints"]
        self.assertEqual(constraints["count"], 1)
        self.assertFalse(constraints["items"][0]["isSuppressed"]["value"])
        relationships = constraints["items"][0]["geometricRelationships"]
        self.assertEqual(relationships["items"][0]["objectType"]["value"], "adsk::fusion::RigidRelationship")

    def test_json_is_local_and_existing_report_is_preserved(self):
        app, design = self.fixture()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "probe.json"
            probe.write_report(app, design, path)
            before = path.read_bytes()
            saved = json.loads(before)
            self.assertEqual(saved["schema"], "acdc_joint_diagnostics/v1")
            with self.assertRaises(FileExistsError):
                probe.write_report(app, design, path)
            self.assertEqual(path.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
