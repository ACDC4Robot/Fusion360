"""Small, dependency-free localization helpers for the Fusion add-in UI."""

DEFAULT_LOCALE = "en"
SIMPLIFIED_CHINESE_LOCALE = "zh_CN"

_TRANSLATIONS = {
    "en": {
        "command_description": "Export Autodesk Fusion design models to robot description formats",
        "robot_description_format": "Robot Description Format",
        "simulation_environment": "Simulation Environment",
        "none": "None",
        "author_name": "Author Name",
        "description": "Description",
        "description_placeholder": "Description of the robot model",
        "message_title": "ACDC4Robot Message",
        "error_title": "ACDC4Robot Error",
        "choose_export_folder": "Choose a folder to export to",
        "canceled": "ACDC4Robot was canceled.",
        "start_export": "Starting ACDC4Robot export.",
        "select_description_format": "No robot description format is selected.\nPlease select one.",
        "select_simulation_environment": "No simulation environment is selected.\nPlease select one.",
        "finished_urdf": "Finished exporting URDF for {simulator}.",
        "finished_sdf_gazebo": "Finished exporting SDFormat for Gazebo.",
        "finished_sdf_pybullet": "Finished exporting SDFormat for PyBullet.",
        "mujoco_no_sdf": "MuJoCo does not support SDFormat.\nPlease select PyBullet or Gazebo.",
        "gazebo_no_mjcf": "Gazebo does not support MJCF.\nPlease select MuJoCo.",
        "pybullet_no_mjcf": "PyBullet does not support MJCF.\nPlease select MuJoCo.",
        "finished_mjcf": "Finished exporting MJCF for MuJoCo.",
        "finished_urdf_plus": "Finished exporting URDF+.",
        "failed": "Failed:\n{error}",
        "visual_body_missing": "Please provide two bodies: one visual and one collision body.\nThe visual body is missing.",
        "collision_body_missing": "Please provide two bodies: one visual and one collision body.\nThe collision body is missing.",
    },
    "zh_CN": {
        "command_description": "将 Autodesk Fusion 设计模型导出为机器人描述格式",
        "robot_description_format": "机器人描述格式",
        "simulation_environment": "仿真环境",
        "none": "未选择",
        "author_name": "作者名称",
        "description": "描述",
        "description_placeholder": "机器人模型说明",
        "message_title": "ACDC4Robot 提示",
        "error_title": "ACDC4Robot 错误",
        "choose_export_folder": "选择导出文件夹",
        "canceled": "已取消 ACDC4Robot 导出。",
        "start_export": "开始执行 ACDC4Robot 导出。",
        "select_description_format": "尚未选择机器人描述格式。\n请选择一种格式。",
        "select_simulation_environment": "尚未选择仿真环境。\n请选择一个仿真环境。",
        "finished_urdf": "已完成面向 {simulator} 的 URDF 导出。",
        "finished_sdf_gazebo": "已完成面向 Gazebo 的 SDFormat 导出。",
        "finished_sdf_pybullet": "已完成面向 PyBullet 的 SDFormat 导出。",
        "mujoco_no_sdf": "MuJoCo 不支持 SDFormat。\n请选择 PyBullet 或 Gazebo。",
        "gazebo_no_mjcf": "Gazebo 不支持 MJCF。\n请选择 MuJoCo。",
        "pybullet_no_mjcf": "PyBullet 不支持 MJCF。\n请选择 MuJoCo。",
        "finished_mjcf": "已完成面向 MuJoCo 的 MJCF 导出。",
        "finished_urdf_plus": "已完成 URDF+ 导出。",
        "failed": "执行失败：\n{error}",
        "visual_body_missing": "请提供两个实体：一个用于视觉，一个用于碰撞。\n缺少视觉实体。",
        "collision_body_missing": "请提供两个实体：一个用于视觉，一个用于碰撞。\n缺少碰撞实体。",
    },
}

_OPTION_KEYS = {
    "None": "none",
    "URDF": "URDF",
    "SDFormat": "SDFormat",
    "MJCF": "MJCF",
    "URDF+": "URDF+",
    "Gazebo": "Gazebo",
    "PyBullet": "PyBullet",
    "MuJoCo": "MuJoCo",
}


def get_locale(user_language=None):
    """Return a supported locale, following Fusion's current UI language."""
    try:
        import adsk.core

        if user_language is None:
            app = adsk.core.Application.get()
            user_language = app.preferences.generalPreferences.userLanguage
        if user_language == adsk.core.UserLanguages.ChinesePRCLanguage:
            return SIMPLIFIED_CHINESE_LOCALE
    except Exception:
        pass
    return DEFAULT_LOCALE


def translate(key, locale=None, **values):
    """Translate a message key, falling back to English and then the key."""
    selected_locale = locale or get_locale()
    catalog = _TRANSLATIONS.get(selected_locale, _TRANSLATIONS[DEFAULT_LOCALE])
    text = catalog.get(key, _TRANSLATIONS[DEFAULT_LOCALE].get(key, key))
    return text.format(**values) if values else text


def option_label(value, locale=None):
    """Return a localized label without changing the stable option value."""
    key = _OPTION_KEYS[value]
    return translate(key, locale) if key in _TRANSLATIONS[DEFAULT_LOCALE] else key


def option_value(label):
    """Convert a localized drop-down label back to its stable option value."""
    for value in _OPTION_KEYS:
        if label == option_label(value, DEFAULT_LOCALE) or label == option_label(value, SIMPLIFIED_CHINESE_LOCALE):
            return value
    return label


def translate_error_message(message, locale=None):
    """Translate common exporter errors that include dynamic link names."""
    if (locale or get_locale()) != SIMPLIFIED_CHINESE_LOCALE:
        return message

    replacements = (
        ("Please set two bodies, one for visual and one for collision. \n", "请提供两个实体：一个用于视觉，一个用于碰撞。\n"),
        ("Body for visual missing.", "缺少视觉实体。"),
        ("Body for collision missing.", "缺少碰撞实体。"),
        (" body for visual missing.", " 缺少视觉实体。"),
        (" body for collision missing.", " 缺少碰撞实体。"),
        ("mjcf will automatically generate geometry for collision. \n", "MJCF 会自动生成碰撞几何体。\n"),
        (" does not need to set geometry for visual and collision seperately.", " 无需分别设置视觉和碰撞几何体。"),
    )
    translated = message
    for source, target in replacements:
        translated = translated.replace(source, target)
    return translated
