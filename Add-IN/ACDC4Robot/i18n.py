"""Dependency-free localization helpers for the Fusion Add-In UI.

Technical identifiers and the English provenance JSON remain stable across
locales. Only user-facing labels, dialogs, and the rendered preflight summary
are localized.
"""

from __future__ import annotations

import re


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
        "canceled": "ACDC4Robot export was canceled.",
        "start_export": "Starting ACDC4Robot export.",
        "active_document_not_design": "The active document is not a Fusion design.",
        "select_description_format": "No robot description format is selected.\nPlease select one.",
        "select_simulation_environment": "No simulation environment is selected.\nPlease select one.",
        "finished_urdf": "Finished exporting URDF for {simulator}.",
        "finished_sdf_gazebo": "Finished exporting SDFormat for Gazebo.",
        "finished_sdf_pybullet": "Finished exporting SDFormat for PyBullet.",
        "mujoco_no_sdf": "MuJoCo does not support SDFormat.\nPlease select PyBullet or Gazebo.",
        "gazebo_no_mjcf": "Gazebo does not support MJCF.\nPlease select MuJoCo.",
        "pybullet_no_mjcf": "PyBullet does not support MJCF.\nPlease select MuJoCo.",
        "finished_mjcf": (
            "Finished exporting MJCF for MuJoCo.\n"
            "Preflight warnings: {warning_count}\n"
            "See acdc4robot-export-report.json in the export directory."
        ),
        "finished_urdf_plus": "Finished exporting URDF+.",
        "visual_body_missing": (
            "Please provide two bodies: one visual and one collision body.\n"
            "The visual body is missing."
        ),
        "collision_body_missing": (
            "Please provide two bodies: one visual and one collision body.\n"
            "The collision body is missing."
        ),
        "preflight_title": "ACDC4Robot MJCF preflight",
        "preflight_design": "Design",
        "preflight_visible_links": "Visible links",
        "preflight_joints": "Joints",
        "preflight_active_dof": "Active DOF",
        "preflight_roots": "Roots",
        "preflight_errors": "Errors",
        "preflight_warnings": "Warnings",
        "preflight_none": "(none)",
        "preflight_no_files": (
            "No files were exported. Correct the Fusion assembly and try again."
        ),
        "export_stopped": "Export stopped:\n\n{error}",
        "export_failed": (
            "Export failed with an unexpected error.\n\n"
            "The full traceback was written to the Text Commands palette.\n\n"
            "{error}"
        ),
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
        "active_document_not_design": "当前文档不是 Fusion 设计。",
        "select_description_format": "尚未选择机器人描述格式。\n请选择一种格式。",
        "select_simulation_environment": "尚未选择仿真环境。\n请选择一个仿真环境。",
        "finished_urdf": "已完成面向 {simulator} 的 URDF 导出。",
        "finished_sdf_gazebo": "已完成面向 Gazebo 的 SDFormat 导出。",
        "finished_sdf_pybullet": "已完成面向 PyBullet 的 SDFormat 导出。",
        "mujoco_no_sdf": "MuJoCo 不支持 SDFormat。\n请选择 PyBullet 或 Gazebo。",
        "gazebo_no_mjcf": "Gazebo 不支持 MJCF。\n请选择 MuJoCo。",
        "pybullet_no_mjcf": "PyBullet 不支持 MJCF。\n请选择 MuJoCo。",
        "finished_mjcf": (
            "已完成面向 MuJoCo 的 MJCF 导出。\n"
            "导出前检查警告：{warning_count}\n"
            "请查看导出目录中的 acdc4robot-export-report.json。"
        ),
        "finished_urdf_plus": "已完成 URDF+ 导出。",
        "visual_body_missing": "请提供两个实体：一个用于视觉，一个用于碰撞。\n缺少视觉实体。",
        "collision_body_missing": "请提供两个实体：一个用于视觉，一个用于碰撞。\n缺少碰撞实体。",
        "preflight_title": "ACDC4Robot MJCF 导出前检查",
        "preflight_design": "设计",
        "preflight_visible_links": "可见连杆",
        "preflight_joints": "关节",
        "preflight_active_dof": "主动自由度",
        "preflight_roots": "根连杆",
        "preflight_errors": "错误",
        "preflight_warnings": "警告",
        "preflight_none": "（无）",
        "preflight_no_files": "未导出任何文件。请修正 Fusion 装配体后重试。",
        "export_stopped": "导出已停止：\n\n{error}",
        "export_failed": (
            "导出因意外错误而失败。\n\n"
            "完整回溯已写入“文本命令”面板。\n\n"
            "{error}"
        ),
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

_EXACT_PREFLIGHT_ZH = {
    "No visible component occurrences containing bodies were found.": (
        "未找到包含实体的可见组件实例。"
    ),
    "No Fusion joints or as-built joints were found. The export would contain disconnected bodies.": (
        "未找到 Fusion 关节或装配位置关节；导出结果将包含断开的实体。"
    ),
    "No revolute or slider joint was found; the exported model is rigid.": (
        "未找到旋转或滑动关节；导出的模型为刚性模型。"
    ),
}

_LEGACY_EXPORT_REPLACEMENTS_ZH = (
    (
        "Please set two bodies, one for visual and one for collision. \n",
        "请提供两个实体：一个用于视觉，一个用于碰撞。\n",
    ),
    (" body for visual missing.", " 缺少视觉实体。"),
    (" body for collision missing.", " 缺少碰撞实体。"),
    (
        "mjcf will automatically generate geometry for collision. \n",
        "MJCF 会自动生成碰撞几何体。\n",
    ),
    (
        " does not need to set geometry for visual and collision seperately.",
        " 无需分别设置视觉和碰撞几何体。",
    ),
)

_PREFLIGHT_PATTERNS_ZH = (
    (
        r"(?P<name>.+) contains nested occurrences; flatten the robot assembly before MJCF export\.",
        "{name} 包含嵌套实例；请在导出 MJCF 前将机器人装配体扁平化。",
    ),
    (
        r"(?P<name>.+) contains (?P<count>\d+) visible surface BRep body/bodies\. "
        r"Convert them to solid bodies before STL/MJCF export\.",
        "{name} 包含 {count} 个可见曲面 BRep 实体；请在导出 STL/MJCF 前将其转换为实体。",
    ),
    (
        r"(?P<name>.+) contains only Fusion mesh bodies\. Convert them to solid BRep bodies "
        r"before STL/MJCF export\.",
        "{name} 仅包含 Fusion 网格实体；请在导出 STL/MJCF 前将其转换为实体 BRep。",
    ),
    (
        r"Duplicate export name (?P<export_name>.+): (?P<paths>.+)",
        "导出名称重复 {export_name}：{paths}",
    ),
    (
        r"Joint (?P<joint>.+) endpoints cannot be read: (?P<details>.+)",
        "无法读取关节 {joint} 的端点：{details}",
    ),
    (
        r"Joint (?P<joint>.+) motion cannot be read: (?P<details>.+)\. "
        r"The Fusion joint may be broken; recreate it before export\.",
        "无法读取关节 {joint} 的运动：{details}。该 Fusion 关节可能已损坏；请重新创建后导出。",
    ),
    (
        r"Joint (?P<joint>.+) is not between two component occurrences\. "
        r"Move grounded/root bodies into a component such as 'base_link' "
        r"and create the joint between component occurrences\.",
        "关节 {joint} 并非连接两个组件实例。请将固定/根实体移入 base_link 等组件，并在组件实例之间创建关节。",
    ),
    (
        r"Joint (?P<joint>.+) uses unsupported type (?P<joint_type>.+)\. "
        r"Only rigid, revolute, and slider joints can be exported\.",
        "关节 {joint} 使用不支持的类型 {joint_type}。仅支持导出刚性、旋转和滑动关节。",
    ),
    (
        r"Joint (?P<joint>.+) has no usable origin geometry\. Recreate the "
        r"moving joint or define a valid joint origin before export\.",
        "关节 {joint} 没有可用的原点几何体。请重新创建运动关节或在导出前定义有效的关节原点。",
    ),
    (
        r"Joint (?P<joint>.+) (?P<role>parent|child) (?P<link>.+) is not an exported visible link\.",
        "关节 {joint} 的{role}连杆 {link} 不是将被导出的可见连杆。",
    ),
    (
        r"(?P<child>.+) has more than one parent: (?P<parents>.+)\.",
        "{child} 有多个父连杆：{parents}。",
    ),
    (
        r"Duplicate joint export name (?P<export_name>.+): (?P<names>.+)\. "
        r"Give every Fusion joint a unique name\.",
        "关节导出名称重复 {export_name}：{names}。请为每个 Fusion 关节指定唯一名称。",
    ),
    (
        r"MJCF requires one connected body tree, but the visible assembly has "
        r"(?P<count>\d+) roots: (?P<roots>.+)\.",
        "MJCF 要求一个连通的实体树，但可见装配体有 {count} 个根：{roots}。",
    ),
    (
        r"Cycle detected at (?P<name>.+); MJCF requires a tree\.",
        "在 {name} 检测到环路；MJCF 要求树形结构。",
    ),
    (
        r"Disconnected visible links: (?P<links>.+)",
        "断开的可见连杆：{links}",
    ),
    (
        r"Referenced components will be exported by occurrence\. Keep the source revisions frozen "
        r"and verify every mesh and inertia value: (?P<paths>.+)",
        "引用组件将按实例导出。请冻结源版本并验证每个网格和惯量值：{paths}",
    ),
    (
        r"Joint '(?P<joint>.+)' is not between two component occurrences\. "
        r"Move grounded/root bodies into a component such as 'base_link' "
        r"and create the joint between component occurrences\.",
        "关节“{joint}”并非连接两个组件实例。请将固定/根实体移入 base_link 等组件，并在组件实例之间创建关节。",
    ),
    (
        r"Joint '(?P<joint>.+)' has no usable origin geometry\. Recreate "
        r"the joint or define a valid joint origin before export\.",
        "关节“{joint}”没有可用的原点几何体。请重新创建关节或在导出前定义有效的关节原点。",
    ),
)


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
        # Locale detection must never prevent the exporter from loading in a
        # Fusion build whose preferences API differs from the current one.
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
    """Convert an English or Simplified Chinese label to its stable value."""
    for value in _OPTION_KEYS:
        labels = {
            option_label(value, DEFAULT_LOCALE),
            option_label(value, SIMPLIFIED_CHINESE_LOCALE),
        }
        if label in labels:
            return value
    return label


def localize_error_message(message, locale=None):
    """Localize a known dynamic exporter/preflight message for display."""
    selected_locale = locale or get_locale()
    if selected_locale != SIMPLIFIED_CHINESE_LOCALE:
        return message
    if message in _EXACT_PREFLIGHT_ZH:
        return _EXACT_PREFLIGHT_ZH[message]
    translated = message
    for source, target in _LEGACY_EXPORT_REPLACEMENTS_ZH:
        translated = translated.replace(source, target)
    if translated != message:
        return translated
    for pattern, template in _PREFLIGHT_PATTERNS_ZH:
        match = re.fullmatch(pattern, message, flags=re.DOTALL)
        if match:
            values = match.groupdict()
            if values.get("role") == "parent":
                values["role"] = "父"
            elif values.get("role") == "child":
                values["role"] = "子"
            return template.format(**values)
    return message
