import sys
import unittest
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT / "Add-IN"))

from ACDC4Robot import i18n  # noqa: E402


class LocalizationTests(unittest.TestCase):
    def test_catalogs_have_the_same_keys(self):
        self.assertEqual(
            set(i18n._TRANSLATIONS["en"]),
            set(i18n._TRANSLATIONS["zh_CN"]),
        )

    def test_unsupported_locale_falls_back_to_english(self):
        self.assertEqual(
            i18n.translate("robot_description_format", "fr_FR"),
            "Robot Description Format",
        )

    def test_simplified_chinese_ui_translation(self):
        self.assertEqual(i18n.translate("simulation_environment", "zh_CN"), "仿真环境")
        self.assertIn("已完成", i18n.translate("finished_mjcf", "zh_CN", warning_count=2))

    def test_option_labels_round_trip_to_stable_values(self):
        for value in ("None", "URDF", "SDFormat", "MJCF", "URDF+", "MuJoCo"):
            label = i18n.option_label(value, "zh_CN")
            self.assertEqual(i18n.option_value(label), value)
        self.assertEqual(i18n.option_label("MJCF", "zh_CN"), "MJCF")
        self.assertEqual(i18n.option_label("MuJoCo", "zh_CN"), "MuJoCo")

    def test_dynamic_preflight_message_preserves_identifiers(self):
        source = (
            "MJCF requires one connected body tree, but the visible assembly has "
            "2 roots: Base:1, Link:1."
        )
        localized = i18n.localize_error_message(source, "zh_CN")
        self.assertIn("2 个根", localized)
        self.assertIn("Base:1, Link:1", localized)

    def test_legacy_geometry_error_preserves_link_name(self):
        source = (
            "Please set two bodies, one for visual and one for collision. \n"
            "pendulum_link body for visual missing."
        )
        localized = i18n.localize_error_message(source, "zh_CN")
        self.assertIn("pendulum_link 缺少视觉实体", localized)

    def test_unknown_dynamic_message_is_not_changed(self):
        message = "A new diagnostic that has not been translated."
        self.assertEqual(i18n.localize_error_message(message, "zh_CN"), message)


if __name__ == "__main__":
    unittest.main()
