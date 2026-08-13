import importlib.util
import pathlib
import unittest


MODULE_PATH = pathlib.Path(__file__).parents[1] / "Add-IN" / "ACDC4Robot" / "i18n.py"
SPEC = importlib.util.spec_from_file_location("acdc4robot_i18n", MODULE_PATH)
I18N = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(I18N)


class I18nTests(unittest.TestCase):
    def test_translation_catalogs_have_matching_keys(self):
        english_keys = set(I18N._TRANSLATIONS["en"])
        chinese_keys = set(I18N._TRANSLATIONS["zh_CN"])
        self.assertEqual(chinese_keys, english_keys)

    def test_english_fallback(self):
        self.assertEqual(I18N.translate("simulation_environment", "en"), "Simulation Environment")

    def test_simplified_chinese_translation(self):
        self.assertEqual(I18N.translate("simulation_environment", "zh_CN"), "仿真环境")

    def test_localized_option_round_trip(self):
        label = I18N.option_label("None", "zh_CN")
        self.assertEqual(label, "未选择")
        self.assertEqual(I18N.option_value(label), "None")

    def test_technical_option_is_not_translated(self):
        self.assertEqual(I18N.option_label("MJCF", "zh_CN"), "MJCF")
        self.assertEqual(I18N.option_value("MJCF"), "MJCF")

    def test_dynamic_error_translation(self):
        source = "Please set two bodies, one for visual and one for collision. \npendulum_link body for visual missing."
        translated = I18N.translate_error_message(source, "zh_CN")
        self.assertIn("pendulum_link 缺少视觉实体", translated)


if __name__ == "__main__":
    unittest.main()
