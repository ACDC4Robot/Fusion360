# -*- coding: utf-8 -*-
"""
Pre-export validation of the Fusion design.

Instead of crashing halfway through an export with a raw traceback, we check
the design first and report every problem at once, with the name of the
offending joint/link and a hint how to fix it in Fusion.
"""
from typing import List, Tuple
import adsk, adsk.fusion, adsk.core
from .link import Link
from .joint import Joint


SUPPORTED_JOINT_TYPES = (0, 1, 2)  # Rigid, Revolute, Slider


def validate_design(link_list: List[Link], joint_list: List[Joint],
                    rdf: str) -> Tuple[List[str], List[str]]:
    """
    Check links and joints for problems that would break the export or
    produce a robot description that simulators refuse to load.

    Parameters
    ---------
    link_list: links that will be exported
    joint_list: joints that will be exported
    rdf: target format ("URDF", "SDFormat", "MJCF", "URDF+")

    Return
    ---------
    errors: List[str]
        problems that make the export impossible or the output unusable
    warnings: List[str]
        problems the user should know about, export can continue
    """
    errors: List[str] = []
    warnings: List[str] = []

    link_names = set()
    for link in link_list:
        try:
            link_names.add(link.get_name())
        except Exception as e:
            errors.append("Link '{}': cannot resolve name ({})".format(
                _safe_full_path(link), e))

    if not link_list:
        errors.append(
            "No exportable links found. A link is a component that contains "
            "bodies, has no child components and no internal joints, and has "
            "its light bulb switched on.")

    # --- joint checks -----------------------------------------------------
    for joint in joint_list:
        name = joint.name

        # joints against the root component / ground
        if not joint.is_valid():
            errors.append(
                "Joint '{}' is connected to the root component (ground). "
                "Move the grounded bodies into a component named 'base_link' "
                "and re-create the joint against that component.".format(name))
            continue

        # unsupported motion types
        try:
            joint_type = joint.joint.jointMotion.jointType
        except Exception as e:
            errors.append("Joint '{}': cannot read joint motion ({}). "
                          "The joint may be broken; try re-creating it.".format(name, e))
            continue
        if joint_type not in SUPPORTED_JOINT_TYPES:
            errors.append(
                "Joint '{}' has unsupported type '{}'. Only Rigid, Revolute "
                "and Slider joints can be exported.".format(
                    name, joint.get_joint_type_name()))

        # suppressed / light-bulb-off sides produce dangling references
        for side, occ in (("parent", joint.parent), ("child", joint.child)):
            try:
                occ_name = Link(occ).get_name()
            except Exception as e:
                errors.append("Joint '{}': cannot resolve {} occurrence ({})".format(
                    name, side, e))
                continue
            if occ_name not in link_names:
                errors.append(
                    "Joint '{}' references {} '{}', which is not exported as a "
                    "link (hidden, empty, or a nested assembly). Every jointed "
                    "component must be a leaf component with visible bodies. "
                    "For nested assemblies, joint the innermost components "
                    "directly.".format(name, side, occ_name))

        # missing joint origin geometry (typical for as-built rigid joints is
        # fine, but for moving joints the origin defines the axis position)
        try:
            if joint_type in (1, 2) and not joint.has_origin():
                warnings.append(
                    "Joint '{}' ({}) has no origin geometry; its frame will "
                    "fall back to the parent frame, which may misplace the "
                    "axis.".format(name, joint.get_joint_type_name()))
        except Exception:
            pass

    # --- duplicate joint names -------------------------------------------
    seen = {}
    for joint in joint_list:
        seen.setdefault(joint.get_name(), []).append(joint)
    for jname, joints in seen.items():
        if len(joints) > 1:
            warnings.append(
                "{} joints share the name '{}'; exported joint names will be "
                "disambiguated automatically.".format(len(joints), jname))

    # --- per-link physics checks -------------------------------------------
    for link in link_list:
        try:
            lname = link.get_name()
            mass = link.get_mass()
        except Exception as e:
            errors.append("Link '{}': cannot read physical properties ({}). "
                          "Check that the component has solid bodies.".format(
                              _safe_full_path(link), e))
            continue
        if mass is None or mass <= 1e-9:
            warnings.append(
                "Link '{}' has (near-)zero mass. Assign a physical material "
                "in Fusion, otherwise simulators like MuJoCo will reject the "
                "model or behave badly.".format(lname))

    # --- tree structure (URDF must be a single tree) -----------------------
    if rdf in ("URDF", "SDFormat", "URDF+"):
        child_names = set()
        for joint in joint_list:
            if not joint.is_valid():
                continue
            try:
                child_names.add(joint.get_child())
            except Exception:
                continue
        roots = [n for n in sorted(link_names) if n not in child_names]
        if len(roots) > 1 and rdf != "URDF+":
            warnings.append(
                "Found {} root links (no parent joint): {}. {} requires a "
                "single connected tree; unconnected components will float "
                "freely or fail to load. Joint every component (use Rigid "
                "joints for fixed parts).".format(
                    len(roots), ", ".join(roots), rdf))
        # a link used as child by more than one joint breaks the tree
        child_count = {}
        for joint in joint_list:
            if not joint.is_valid():
                continue
            try:
                child_count.setdefault(joint.get_child(), []).append(joint.get_name())
            except Exception:
                continue
        for lname, jnames in child_count.items():
            if len(jnames) > 1:
                msg = ("Link '{}' is the child of {} joints ({}). In URDF a link "
                       "can only have one parent joint. Flip the parent/child "
                       "order of one joint, or mark loop-closing joints with "
                       "the name prefix 'loop' and export URDF+."
                       .format(lname, len(jnames), ", ".join(jnames)))
                if rdf == "URDF+":
                    warnings.append(msg)
                else:
                    errors.append(msg)

    return errors, warnings


def _safe_full_path(link: Link) -> str:
    try:
        return link.link.fullPathName
    except Exception:
        return "<unknown>"


def format_report(errors: List[str], warnings: List[str]) -> str:
    """
    Human readable report for a message box.
    """
    lines = []
    if errors:
        lines.append("ERRORS ({}) - export stopped:".format(len(errors)))
        for i, e in enumerate(errors, 1):
            lines.append("  {}. {}".format(i, e))
    if warnings:
        if lines:
            lines.append("")
        lines.append("Warnings ({}):".format(len(warnings)))
        for i, w in enumerate(warnings, 1):
            lines.append("  {}. {}".format(i, w))
    return "\n".join(lines)
