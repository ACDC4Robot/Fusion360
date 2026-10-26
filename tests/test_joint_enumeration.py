"""Fail-closed regressions; these do not reproduce Fusion's constraint bug."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch


ROOT = Path(__file__).resolve().parents[1]
ADDIN = ROOT / "Add-IN" / "ACDC4Robot"


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


discovery = load("joint_enumeration_discovery_test", ADDIN / "core/link_discovery.py")


class BrokenRoot:
    allOccurrences = []
    name = "fixture"

    def __init__(self, failing, failure):
        self.failing = failing
        self.failure = failure

    def __getattr__(self, name):
        if name == self.failing:
            raise self.failure
        if name in ("allJoints", "allAsBuiltJoints"):
            return []
        raise AttributeError(name)


class BrokenIterator:
    # An attractive fallback must not conceal an iterator failure.
    count = 0

    def __init__(self, failure):
        self.failure = failure

    def __iter__(self):
        yield NS(name="partial")
        raise self.failure

    def item(self, index):
        raise AssertionError("must not retry a broken iterator")


class EnumerationTests(unittest.TestCase):
    def assert_stopped(self, root, collection, cause):
        with self.assertRaises(discovery.JointEnumerationError) as raised:
            discovery.unique_joints(root)
        self.assertIs(raised.exception.__cause__, cause)
        self.assertIn("root." + collection, str(raised.exception))
        self.assertIn("joint list may be incomplete", str(raised.exception))

    def test_property_errors_identify_each_collection_and_preserve_cause(self):
        for name in ("allJoints", "allAsBuiltJoints"):
            with self.subTest(collection=name):
                cause = RuntimeError("2 : InternalValidationError : jo")
                self.assert_stopped(BrokenRoot(name, cause), name, cause)

    def test_partial_iteration_errors_never_return_incomplete_joint_lists(self):
        for name in ("allJoints", "allAsBuiltJoints"):
            for error in (RuntimeError, TypeError):
                with self.subTest(collection=name, error=error):
                    cause = error("iterator failed after one joint")
                    root = NS(allJoints=[], allAsBuiltJoints=[])
                    setattr(root, name, BrokenIterator(cause))
                    self.assert_stopped(root, name, cause)

    def test_count_item_api_failure_is_preserved(self):
        cause = RuntimeError("item access failed")

        class Collection:
            count = 1

            def item(self, index):
                raise cause

        self.assert_stopped(NS(allJoints=Collection(), allAsBuiltJoints=[]), "allJoints", cause)

    def test_healthy_count_item_and_duplicate_semantics_are_preserved(self):
        one, two = NS(name="same"), NS(name="same", other=True)

        class Collection:
            count = 2

            def item(self, index):
                return [one, two][index]

        result = discovery.unique_joints(NS(allJoints=Collection(), allAsBuiltJoints=[one]))
        self.assertEqual(len(result), 2)
        self.assertIs(result[0], one)
        self.assertIs(result[1], two)

    def test_diagnostic_localizes_without_losing_fusion_details(self):
        i18n = load("joint_diagnostic_i18n_test", ADDIN / "i18n.py")
        cause = RuntimeError("2 : InternalValidationError : jo")
        try:
            discovery.unique_joints(BrokenRoot("allJoints", cause))
        except discovery.JointEnumerationError as exc:
            source = str(exc)
        self.assertEqual(i18n.localize_error_message(source, "en"), source)
        localized = i18n.localize_error_message(source, "zh_CN")
        self.assertIn("root.allJoints", localized)
        self.assertIn(str(cause), localized)
        self.assertIn("关节列表可能不完整", localized)


class ExportBoundaryTests(unittest.TestCase):
    """Load the actual command/Link with isolated API dependencies."""

    def setUp(self):
        modules_patch = patch.dict(sys.modules)
        modules_patch.start()
        self.addCleanup(modules_patch.stop)
        self.package = "joint_boundary_test"

        def module(name, **attributes):
            result = ModuleType(name)
            result.__dict__.update(attributes)
            sys.modules[name] = result
            return result

        for suffix in ("", ".core", ".commands", ".commands.ACDC4Robot"):
            module(self.package + suffix).__path__ = []
        core = module("adsk.core", Application=NS(get=lambda: self.app))
        fusion = module("adsk.fusion", Design=NS(cast=lambda product: product), Occurrence=object, Joint=object, BRepBody=object)
        module("adsk", core=core, fusion=fusion)
        self.constants = module(
            self.package + ".commands.ACDC4Robot.constants",
            set_text_palette=Mock(), set_robot_name=Mock(),
            get_rdf=lambda: "URDF", get_sim_env=lambda: "Gazebo",
        )
        module(self.package + ".core.utils", get_valid_filename=str)
        module(self.package + ".core.math_operation")
        module(self.package + ".core.joint", Joint=object)
        module(self.package + ".core.robot", Robot=object)
        module(self.package + ".core.urdf_plus", URDF_PLUS=object)
        module(self.package + ".core.write")
        self.preflight = module(self.package + ".core.preflight", write_report=Mock())
        sys.modules[self.package + ".core.link_discovery"] = discovery
        load(self.package + ".i18n", ADDIN / "i18n.py")
        self.link = load(self.package + ".core.link", ADDIN / "core/link.py")
        self.command = load(self.package + ".commands.ACDC4Robot.acdc4robot", ADDIN / "commands/ACDC4Robot/acdc4robot.py")
        self.ui = NS(
            messageBox=Mock(), createFolderDialog=Mock(),
            palettes=NS(itemById=lambda name: NS(isVisible=True, writeText=Mock())),
        )
        self.app = NS(userInterface=self.ui)

    def test_parent_lookup_reads_complete_list_even_if_first_joint_matches(self):
        occurrence = object()
        exported = self.link.Link.__new__(self.link.Link)
        exported.link = occurrence
        cause = RuntimeError("as-built list is unavailable")
        root = BrokenRoot("allAsBuiltJoints", cause)
        root.allJoints = [NS(occurrenceOne=occurrence)]
        self.app.activeProduct = NS(rootComponent=root)
        with self.assertRaises(discovery.JointEnumerationError) as raised:
            exported.get_parent_joint()
        self.assertIs(raised.exception.__cause__, cause)

    def test_parent_lookup_accepts_healthy_as_built_joint(self):
        occurrence = object()
        joint = NS(occurrenceOne=occurrence)
        exported = self.link.Link.__new__(self.link.Link)
        exported.link = occurrence
        self.app.activeProduct = NS(rootComponent=NS(allJoints=[], allAsBuiltJoints=[joint]))
        self.assertIs(exported.get_parent_joint(), joint)

    def test_command_stops_before_folder_dialog_or_output_on_query_failure(self):
        for name in ("allJoints", "allAsBuiltJoints"):
            for stage in ("property", "iteration"):
                with self.subTest(collection=name, stage=stage):
                    cause = RuntimeError("2 : InternalValidationError : jo")
                    root = BrokenRoot(name, cause)
                    if stage == "iteration":
                        root = NS(name="fixture", allJoints=[], allAsBuiltJoints=[], allOccurrences=[])
                        setattr(root, name, BrokenIterator(cause))
                    self.app.activeProduct = NS(rootComponent=root)
                    with patch.object(self.command.os, "makedirs") as mkdir:
                        self.command.run()
                    mkdir.assert_not_called()
                    self.ui.createFolderDialog.assert_not_called()
                    self.preflight.write_report.assert_not_called()
                    self.assertIn("root." + name, self.ui.messageBox.call_args.args[0])


if __name__ == "__main__":
    unittest.main()
