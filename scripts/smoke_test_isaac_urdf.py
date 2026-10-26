#!/usr/bin/env python3
"""Import local URDF assets with the Isaac Sim 6.x API and check USD output.

Works with isaacsim-asset-isolated or, with --kit, a full Isaac Sim environment.
This checks asset conversion; it does not step a physics engine.
"""
from __future__ import annotations

import argparse
import hashlib
from importlib.metadata import PackageNotFoundError, version
import json
import math
from pathlib import Path
import sys
from xml.etree import ElementTree as ET


def source_data(path, require_decimal=False):
    root = ET.parse(path).getroot()
    if root.tag != "robot":
        raise ValueError("Input must be a URDF robot document")
    links = {}
    for link in root.findall("link"):
        name = link.get("name")
        if not name or name in links:
            raise ValueError(f"Missing or duplicate link name: {name!r}")
        inertial = link.find("inertial")
        if inertial is None:
            raise ValueError(f"Link {name!r} requires explicit inertial data for this check")
        mass = float(inertial.find("mass").get("value"))
        tensor = inertial.find("inertia")
        inertia = {key: float(tensor.get(key)) for key in
                   ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
        origin = inertial.find("origin")
        com = [float(v) for v in origin.get("xyz", "0 0 0").split()] if origin is not None else [0.0] * 3
        if mass <= 0 or len(com) != 3 or not all(math.isfinite(v) for v in [mass, *com, *inertia.values()]):
            raise ValueError(f"Invalid inertial data on {name!r}")
        links[name] = {"mass": mass, "inertia": inertia, "com": com}
    if not links:
        raise ValueError("Input contains no links")
    joints = [joint.get("name") for joint in root.findall("joint")]
    if any(not name for name in joints) or len(set(joints)) != len(joints):
        raise ValueError("Input contains missing or duplicate joint names")
    numeric = {"origin": ("xyz", "rpy"), "mass": ("value",),
               "inertia": ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"),
               "axis": ("xyz",), "limit": ("lower", "upper", "effort", "velocity"),
               "mesh": ("scale",)}
    for node in root.iter():
        for attribute in numeric.get(node.tag, ()):
            for token in node.get(attribute, "").split():
                if not math.isfinite(float(token)):
                    raise ValueError(f"Non-finite {node.tag}.{attribute}")
                if require_decimal and "e" in token.lower():
                    raise ValueError(f"Exponent notation in {node.tag}.{attribute}")
    assets = {}
    for node in list(root.iter("mesh")) + list(root.iter("texture")):
        filename = node.get("filename", "")
        if not filename or "://" in filename:
            raise ValueError("This smoke test requires local relative or absolute mesh/texture paths")
        asset = (path.parent / filename).resolve(strict=True)
        if not asset.is_file():
            raise ValueError(f"Asset is not a file: {filename!r}")
        assets[filename] = hashlib.sha256(asset.read_bytes()).hexdigest()
    return links, joints, assets, [node.get("name") for node in root.findall("./link/collision")]


def package_versions():
    result = {}
    for name in ("isaacsim", "isaacsim-asset-isolated", "isaacsim-robot-schema",
                 "urdf-usd-converter", "usd-exchange", "usd-optimize"):
        try:
            result[name] = version(name)
        except PackageNotFoundError:
            pass
    return result


def check_stage(output_path, links, joints, expected_collisions):
    import numpy as np
    from pxr import Usd, UsdGeom, UsdPhysics, UsdUtils

    stage = Usd.Stage.Open(str(output_path))
    if not stage or not stage.GetDefaultPrim():
        raise ValueError("Importer did not produce an openable USD with a default prim")
    # The 6.2 asset transformer separates rendering and physics into variants.
    # Select physics in the session layer, leaving the saved artifact untouched.
    physics = stage.GetDefaultPrim().GetVariantSet("Physics")
    selected = None
    if physics and "physics" in physics.GetVariantNames():
        stage.SetEditTarget(stage.GetSessionLayer())
        physics.SetVariantSelection("physics")
        selected = "physics"
        stage.Load()
    unresolved = UsdUtils.ComputeAllDependencies(str(output_path))[2]
    if unresolved:
        raise ValueError(f"USD contains unresolved dependencies: {unresolved}")
    if not math.isclose(UsdGeom.GetStageMetersPerUnit(stage), 1.0):
        raise ValueError("This check expects imported SI metre units")
    if not math.isclose(UsdPhysics.GetStageKilogramsPerUnit(stage), 1.0):
        raise ValueError("This check expects imported SI kilogram units")
    prims = list(stage.Traverse(Usd.TraverseInstanceProxies()))
    meshes = [prim for prim in prims if prim.IsA(UsdGeom.Mesh)]
    if not meshes:
        raise ValueError("Imported USD contains no mesh geometry")
    for prim in meshes:
        mesh = UsdGeom.Mesh(prim)
        if not mesh.GetPointsAttr().Get() or not mesh.GetFaceVertexCountsAttr().Get():
            raise ValueError(f"Imported mesh is empty: {prim.GetPath()}")
    imported_joints = {prim.GetName() for prim in prims if prim.IsA(UsdPhysics.Joint)}
    if not set(joints).issubset(imported_joints):
        raise ValueError(f"Missing imported joints: {sorted(set(joints) - imported_joints)}")
    collision_paths = [str(prim.GetPath()) for prim in prims if prim.HasAPI(UsdPhysics.CollisionAPI)]
    if len(collision_paths) < len(expected_collisions):
        raise ValueError(f"Missing collision geometry: expected at least {len(expected_collisions)}, got {len(collision_paths)}")
    for name in expected_collisions:
        if name and not any(name in path.split("/") for path in collision_paths):
            raise ValueError(f"Named collision geometry is missing or renamed: {name}")
    masses = {}
    for prim in prims:
        if prim.HasAPI(UsdPhysics.MassAPI) and prim.GetName() in links:
            if prim.GetName() in masses:
                raise ValueError(f"Ambiguous imported rigid-body name: {prim.GetName()}")
            mass_api = UsdPhysics.MassAPI(prim)
            source = links[prim.GetName()]
            mass = mass_api.GetMassAttr().Get()
            com = list(mass_api.GetCenterOfMassAttr().Get())
            diagonal = list(mass_api.GetDiagonalInertiaAttr().Get())
            inertia = source["inertia"]
            eigenvalues = np.linalg.eigvalsh([
                [inertia["ixx"], inertia["ixy"], inertia["ixz"]],
                [inertia["ixy"], inertia["iyy"], inertia["iyz"]],
                [inertia["ixz"], inertia["iyz"], inertia["izz"]],
            ])
            if not math.isclose(mass, source["mass"], rel_tol=1e-5, abs_tol=0):
                raise ValueError(f"Mass changed for {prim.GetName()}: {mass}")
            if not np.allclose(com, source["com"], rtol=1e-5, atol=1e-8):
                raise ValueError(f"CoM changed for {prim.GetName()}: {com}")
            if any(value <= 0 or not math.isfinite(value) for value in diagonal):
                raise ValueError(f"Invalid imported principal inertia for {prim.GetName()}")
            if not np.allclose(sorted(diagonal), eigenvalues, rtol=1e-5, atol=0):
                raise ValueError(f"Inertia magnitudes changed for {prim.GetName()}: {diagonal}")
            masses[prim.GetName()] = {"mass_kg": mass, "com_m": com,
                                      "principal_inertia_kg_m2": diagonal}
    if set(masses) != set(links):
        raise ValueError(f"Missing imported inertials: {sorted(set(links) - set(masses))}")
    return {"mesh_prims": len(meshes), "collision_prims": len(collision_paths),
            "collision_paths": collision_paths,
            "joint_names": sorted(imported_joints),
            "inertials": masses, "physics_variant": selected,
            "unresolved_dependencies": list(unresolved)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", type=Path)
    parser.add_argument("output", type=Path, help="new output directory")
    parser.add_argument("--require-decimal", action="store_true")
    parser.add_argument("--kit", action="store_true", help="start full headless SimulationApp before importing")
    args = parser.parse_args()
    path = args.urdf.resolve(strict=True)
    links, joints, assets, collision_names = source_data(path, args.require_decimal)
    args.output.mkdir(parents=False, exist_ok=False)
    report = {"schema": "acdc4robot-isaac-import/v1", "passed": False,
              "mode": "kit-headless" if args.kit else "isolated-asset-conversion",
              "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
              "asset_sha256": assets, "packages": package_versions(),
              "input_links": len(links), "input_joints": len(joints),
              "physics_simulation_run": False}
    app = None
    try:
        if args.kit:
            from isaacsim import SimulationApp
            app = SimulationApp({"headless": True})
            import omni.kit.app
            manager = omni.kit.app.get_app().get_extension_manager()
            for extension in ("omni.scene.optimizer.core", "isaacsim.robot.schema",
                              "isaacsim.asset.importer.urdf"):
                manager.set_extension_enabled_immediate(extension, True)
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
        config = URDFImporterConfig(urdf_path=str(path), usd_path=str(args.output.resolve()),
                                    fix_base=None, merge_fixed_joints=False, merge_mesh=False)
        output_path = URDFImporter(config).import_urdf()
        report["usd_path"] = str(output_path)
        report.update(check_stage(output_path, links, joints, collision_names))
        report["passed"] = True
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if app is not None:
            app.close()
        (args.output / "smoke-test-report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
