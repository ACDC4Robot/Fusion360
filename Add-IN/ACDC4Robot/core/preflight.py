"""Preflight checks and provenance reporting for MJCF export.

The Fusion document is the assembly authority. These checks prevent the
exporter from silently turning an under-constrained assembly into disconnected
MuJoCo bodies.
"""

import json
import os

from . import utils
from ..version import __version__


_JOINT_TYPES = {
    0: "rigid",
    1: "revolute",
    2: "slider",
    3: "cylindrical",
    4: "pin-slot",
    5: "planar",
    6: "ball",
}
_SUPPORTED_JOINT_TYPES = {0, 1, 2}


def _collection_items(collection):
    """Return values from both Fusion collection and vector API shapes."""
    if collection is None:
        return []
    try:
        return list(collection)
    except TypeError:
        pass

    count = getattr(collection, "count", None)
    if callable(count):
        count = count()
    if count is not None:
        item = getattr(collection, "item", None)
        if callable(item):
            return [item(index) for index in range(count)]

    try:
        return [collection[index] for index in range(len(collection))]
    except (TypeError, AttributeError) as exc:
        raise TypeError(f"Unsupported Fusion collection type: {type(collection)!r}") from exc


def _has_visible_bodies(occurrence):
    return occurrence.isLightBulbOn and utils.component_has_bodies(occurrence.component)


def _matrix_values(matrix):
    return [matrix.getCell(row, column) for row in range(4) for column in range(4)]


def _is_referenced(occurrence):
    # The property name has varied across Fusion API releases. Treat an
    # unavailable property as unknown rather than failing the export.
    for owner, attribute in (
        (occurrence, "isReferencedComponent"),
        (occurrence.component, "isReferencedComponent"),
    ):
        try:
            return bool(getattr(owner, attribute))
        except (AttributeError, RuntimeError):
            continue
    return None


def inspect_mjcf_design(design):
    root = design.rootComponent
    occurrences = [
        occurrence
        for occurrence in _collection_items(root.allOccurrences)
        if _has_visible_bodies(occurrence)
    ]
    joints = _collection_items(root.allJoints) + _collection_items(root.allAsBuiltJoints)

    errors = []
    warnings = []
    occurrence_records = []
    occurrence_by_path = {occ.fullPathName: occ for occ in occurrences}
    export_names = {}

    for occurrence in occurrences:
        path = occurrence.fullPathName
        export_name = utils.get_valid_filename(path)
        export_names.setdefault(export_name, []).append(path)
        referenced = _is_referenced(occurrence)
        occurrence_records.append(
            {
                "full_path_name": path,
                "export_name": export_name,
                "component_name": occurrence.component.name,
                "body_count": occurrence.component.bRepBodies.count,
                "child_occurrence_count": occurrence.childOccurrences.count,
                "referenced_component": referenced,
                "transform_fusion_internal_cm": _matrix_values(occurrence.transform2),
            }
        )
        if occurrence.childOccurrences.count:
            errors.append(
                f"{path} contains nested occurrences; flatten the robot assembly before MJCF export."
            )

    for export_name, paths in export_names.items():
        if len(paths) > 1:
            errors.append(f"Duplicate export name {export_name}: {', '.join(paths)}")

    if not occurrences:
        errors.append("No visible component occurrences containing bodies were found.")

    joint_records = []
    joint_export_names = {}
    child_to_parent = {}
    adjacency = {}
    active_dof_count = 0

    for joint in joints:
        parent = getattr(joint, "occurrenceTwo", None)
        child = getattr(joint, "occurrenceOne", None)
        joint_type = joint.jointMotion.jointType
        joint_type_name = _JOINT_TYPES.get(joint_type, f"unknown-{joint_type}")
        joint_export_name = utils.get_valid_filename(joint.name)
        joint_export_names.setdefault(joint_export_name, []).append(joint.name)
        parent_path = parent.fullPathName if parent else None
        child_path = child.fullPathName if child else None
        joint_records.append(
            {
                "name": joint.name,
                "export_name": joint_export_name,
                "object_type": joint.objectType,
                "type": joint_type_name,
                "parent": parent_path,
                "child": child_path,
            }
        )

        if parent is None or child is None:
            errors.append(
                f"Joint {joint.name} is not between two component occurrences; "
                "connect the moving component to the servo/base occurrence."
            )
            continue
        if joint_type not in _SUPPORTED_JOINT_TYPES:
            errors.append(f"Joint {joint.name} uses unsupported type {joint_type_name}.")
        if parent_path not in occurrence_by_path:
            errors.append(f"Joint {joint.name} parent {parent_path} is not an exported visible link.")
        if child_path not in occurrence_by_path:
            errors.append(f"Joint {joint.name} child {child_path} is not an exported visible link.")
        if child_path in child_to_parent and child_to_parent[child_path] != parent_path:
            errors.append(
                f"{child_path} has more than one parent: "
                f"{child_to_parent[child_path]} and {parent_path}."
            )
        child_to_parent[child_path] = parent_path
        adjacency.setdefault(parent_path, []).append(child_path)
        if joint_type in (1, 2):
            active_dof_count += 1

    for export_name, names in joint_export_names.items():
        if len(names) > 1:
            errors.append(
                f"Duplicate joint export name {export_name}: {', '.join(names)}. "
                "Give every Fusion joint a unique name."
            )

    occurrence_paths = set(occurrence_by_path)
    child_paths = set(child_to_parent)
    root_paths = sorted(occurrence_paths - child_paths)

    if not joints and len(occurrences) > 1:
        errors.append(
            "No Fusion joints or as-built joints were found. The export would contain disconnected bodies."
        )
    if len(root_paths) != 1:
        errors.append(
            "MJCF requires one connected body tree, but the visible assembly has "
            f"{len(root_paths)} roots: {', '.join(root_paths) if root_paths else '(none)'}."
        )
    if active_dof_count == 0:
        warnings.append("No revolute or slider joint was found; the exported model is rigid.")

    visited = set()
    visiting = set()

    def visit(path):
        if path in visiting:
            errors.append(f"Cycle detected at {path}; MJCF requires a tree.")
            return
        if path in visited:
            return
        visiting.add(path)
        for child_path in adjacency.get(path, []):
            visit(child_path)
        visiting.remove(path)
        visited.add(path)

    if len(root_paths) == 1:
        visit(root_paths[0])
        disconnected = sorted(occurrence_paths - visited)
        if disconnected:
            errors.append(f"Disconnected visible links: {', '.join(disconnected)}")

    referenced_paths = [
        record["full_path_name"]
        for record in occurrence_records
        if record["referenced_component"] is True
    ]
    if referenced_paths:
        warnings.append(
            "Referenced components will be exported by occurrence. Keep the source revisions frozen "
            "and verify every mesh and inertia value: " + ", ".join(referenced_paths)
        )
    return {
        "schema_version": 1,
        "plugin_version": __version__,
        "design_name": root.name,
        "default_length_units": design.unitsManager.defaultLengthUnits,
        "visible_link_count": len(occurrences),
        "joint_count": len(joints),
        "active_dof_count": active_dof_count,
        "root_links": root_paths,
        "occurrences": occurrence_records,
        "joints": joint_records,
        "errors": errors,
        "warnings": warnings,
        "passed": not errors,
    }


def format_report(report):
    lines = [
        "ACDC4Robot MJCF preflight",
        f"Design: {report['design_name']}",
        f"Visible links: {report['visible_link_count']}",
        f"Joints: {report['joint_count']}",
        f"Active DOF: {report['active_dof_count']}",
        f"Roots: {', '.join(report['root_links']) if report['root_links'] else '(none)'}",
    ]
    if report["errors"]:
        lines.append("Errors:")
        lines.extend(f"- {item}" for item in report["errors"])
    if report["warnings"]:
        lines.append("Warnings:")
        lines.extend(f"- {item}" for item in report["warnings"])
    return "\n".join(lines)


def write_report(save_directory, report):
    path = os.path.join(save_directory, "acdc4robot-export-report.json")
    with open(path, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write("\n")
    return path
