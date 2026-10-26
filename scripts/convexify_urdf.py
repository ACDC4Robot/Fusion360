#!/usr/bin/env python3
"""Build a separate URDF bundle with optional CoACD collision meshes.

No Fusion imports or optional geometry dependencies are loaded until conversion.
See docs/COLLISION_MESHES.md for the supported asset and frame contract.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import hashlib
from importlib.metadata import version
import json
import math
import os
from pathlib import Path
import shutil
import sys
import tempfile
from xml.etree import ElementTree as ET


COACD_OPTIONS = {
    "threshold": 0.05, "max_convex_hull": -1, "preprocess_mode": "off",
    "preprocess_resolution": 50, "resolution": 2000, "mcts_nodes": 20,
    "mcts_iterations": 150, "mcts_max_depth": 3, "pca": False,
    "merge": True, "decimate": False, "max_ch_vertex": 256,
    "extrude": False, "extrude_margin": 0.01, "apx_mode": "ch", "seed": 0,
}


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def local_asset(filename, source_dir):
    if not filename or "\\" in filename or (":" in filename and not Path(filename).is_absolute()):
        raise ValueError(f"Unsupported asset URI {filename!r}; use local relative or absolute paths, not package/file/remote URIs.")
    path = Path(filename)
    if not path.is_absolute():
        path = source_dir / path
    path = path.resolve(strict=True)
    if not path.is_file():
        raise ValueError(f"Asset is not a regular file: {filename}")
    return path


def mesh_scale(element):
    try:
        values = [float(value) for value in element.get("scale", "1 1 1").split()]
    except ValueError as exc:
        raise ValueError("Mesh scale must contain three finite positive numbers") from exc
    if len(values) != 3 or any(not math.isfinite(value) or value <= 0 for value in values):
        raise ValueError("Mesh scale must contain three finite positive numbers")
    return values


def dependencies():
    try:
        import coacd
        import numpy as np
        import trimesh
    except ImportError as exc:
        raise ValueError("Install optional dependencies with: python -m pip install -r scripts/requirements-collision.txt") from exc
    return coacd, np, trimesh


def load_solid(path, scale, np, trimesh):
    if path.suffix.lower() != ".stl":
        raise ValueError(f"Unsupported mesh format {path.suffix!r}; this tool supports Fusion-exported STL only.")
    mesh = trimesh.load_mesh(path, process=False)
    if not isinstance(mesh, trimesh.Trimesh):
        raise ValueError(f"Expected one triangle mesh: {path.name}")
    mesh.vertices = np.asarray(mesh.vertices, dtype=float) * np.asarray(scale)
    if not np.isfinite(mesh.vertices).all():
        raise ValueError(f"Collision mesh {path.name} contains non-finite vertices")
    # STL repeats triangle vertices. Weld coincident vertices without removing
    # faces or repairing geometry so topology checks reflect the actual solid.
    mesh.merge_vertices()
    if (len(mesh.vertices) < 4 or len(mesh.faces) < 4 or
            not np.isfinite(mesh.vertices).all() or
            not mesh.is_watertight or not mesh.is_winding_consistent or
            not math.isfinite(mesh.volume) or mesh.volume <= 0):
        raise ValueError(f"Collision mesh {path.name} must be finite, closed, consistently outward wound, and have positive volume. Repair it explicitly before decomposition.")
    return mesh


def write_hull(path, vertices, faces, np, trimesh):
    vertices = np.asarray(vertices, dtype=float)
    faces = np.asarray(faces)
    if (vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < 4 or
            not np.isfinite(vertices).all() or faces.ndim != 2 or
            faces.shape[1] != 3 or len(faces) < 4 or
            not np.issubdtype(faces.dtype, np.integer) or
            faces.min() < 0 or faces.max() >= len(vertices)):
        raise ValueError("CoACD returned an invalid convex piece; no output bundle was published.")
    hull = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
    if (not hull.is_watertight or not hull.is_winding_consistent or
            not math.isfinite(hull.volume) or hull.volume <= 0):
        raise ValueError("CoACD returned a piece that is not a closed, outward-wound convex solid; no output bundle was published.")
    triangles = vertices[faces]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    if not np.isfinite(lengths).all() or (lengths <= 0).any():
        raise ValueError("CoACD returned a degenerate triangle")
    normals /= lengths[:, None]
    # Every outward face must support the entire hull. Using unit normals
    # makes the tolerance a distance; scale it for mesh size and cancellation
    # at large source-coordinate offsets. No optional SciPy dependency needed.
    scale = float(np.linalg.norm(np.ptp(vertices, axis=0)))
    tolerance = max(scale * 1e-8, float(np.abs(vertices).max()) * np.finfo(float).eps * 32)
    for triangle, normal in zip(triangles, normals):
        if float(np.max((vertices - triangle[0]) @ normal)) > tolerance:
            raise ValueError("CoACD returned a piece that is not a closed, outward-wound convex solid; no output bundle was published.")
    # ASCII STL retains binary64 coordinates rather than introducing binary
    # STL's float32 rounding. Triangle winding is retained from CoACD.
    with path.open("w", encoding="ascii", newline="\n") as stream:
        stream.write("solid coacd\n")
        for points, normal in zip(triangles, normals):
            stream.write("  facet normal " + " ".join(format(float(v), ".17g") for v in normal) + "\n    outer loop\n")
            for point in points:
                stream.write("      vertex " + " ".join(format(float(v), ".17g") for v in point) + "\n")
            stream.write("    endloop\n  endfacet\n")
        stream.write("endsolid coacd\n")
    return {"vertices": len(vertices), "triangles": len(faces),
            "bounds_m": [vertices.min(axis=0).tolist(), vertices.max(axis=0).tolist()]}


def convert(source, output, *, threshold=0.05, seed=0, decompose=None):
    """Convert atomically; ``decompose`` is an optional callable for tests."""
    source = Path(source).resolve(strict=True)
    if not source.is_file() or source.suffix.lower() != ".urdf":
        raise ValueError("Input must be a .urdf file")
    output = Path(output).absolute()
    if output.exists() or output.is_symlink():
        raise ValueError("Output directory must not already exist")
    destination = output.resolve()
    if destination.is_relative_to(source.parent) or source.parent.is_relative_to(destination):
        raise ValueError("Input and output must be in separate, non-nested directories")
    if not destination.parent.is_dir():
        raise ValueError("Output parent directory must already exist")
    if not math.isfinite(threshold) or not 0.01 <= threshold <= 1:
        raise ValueError("CoACD threshold must be finite and between 0.01 and 1 (normalized units)")
    if not isinstance(seed, int) or not 0 <= seed <= 2**32 - 1:
        raise ValueError("Seed must be an integer between 0 and 4294967295")
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    tree = ET.parse(source, parser=parser)
    root = tree.getroot()
    if root.tag != "robot":
        raise ValueError("Input must be a URDF <robot> document")
    coacd, np, trimesh = dependencies()
    options = dict(COACD_OPTIONS, threshold=threshold, seed=seed)
    coacd.set_log_level("off")
    if decompose is None:
        decompose = lambda vertices, faces, settings: coacd.run_coacd(coacd.Mesh(vertices, faces), **settings)
    report = {
        "schema": "acdc4robot-collision/v1", "tool_sha256": sha256(__file__),
        "python": sys.version.split()[0], "source_urdf_sha256": sha256(source),
        "source_urdf_name": source.name,
        "versions": {name: version(name) for name in ("coacd", "trimesh", "numpy")},
        "options": options, "threshold_units": "CoACD normalized units",
        "geometry_units": "metres (source URDF mesh scale baked into hull vertices)",
        "repair": "none; closed outward-wound source meshes required",
        "assets": [], "decompositions": [],
    }
    # Resolve every referenced mesh/texture before creating the staging area.
    assets = {}
    for element in list(root.iter("mesh")) + list(root.iter("texture")):
        filename = element.get("filename")
        asset = local_asset(filename, source.parent)
        if element.tag == "mesh" and asset.suffix.lower() != ".stl":
            raise ValueError(f"Unsupported mesh format {asset.suffix!r}; supports Fusion-exported STL only")
        digest = sha256(asset)
        relative = f"assets/{digest}/{asset.name}"
        assets[id(element)] = (asset, relative, filename, digest)
    jobs = []
    for link_index, link in enumerate(root.findall("link")):
        for collision_index, collision in enumerate(link.findall("collision")):
            geometry = collision.find("geometry")
            if geometry is None or len(geometry) != 1:
                raise ValueError(f"Link {link.get('name')!r} has an invalid collision geometry")
            mesh_element = geometry.find("mesh")
            if mesh_element is not None:
                asset, _, filename, digest = assets[id(mesh_element)]
                scale = mesh_scale(mesh_element)
                mesh = load_solid(asset, scale, np, trimesh)
                jobs.append((link_index, collision_index, link, collision, mesh_element, mesh, filename, digest, scale))
    if not jobs:
        raise ValueError("URDF contains no mesh collision geometry to decompose")
    with tempfile.TemporaryDirectory(prefix=f".{destination.name}.stage-", dir=destination.parent) as temporary:
        staging = Path(temporary) / "bundle"
        staging.mkdir()
        for element in list(root.iter("mesh")) + list(root.iter("texture")):
            asset, relative, filename, digest = assets[id(element)]
            target = staging / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            if not target.exists():
                shutil.copyfile(asset, target)
                report["assets"].append({"source_reference": filename, "path": relative, "sha256": digest})
            element.set("filename", relative)
        (staging / "collision").mkdir()
        for li, ci, link, collision, mesh_element, mesh, filename, digest, scale in jobs:
            parts = decompose(mesh.vertices.copy(), mesh.faces.copy(), dict(options))
            if not parts:
                raise ValueError(f"CoACD returned no pieces for link {link.get('name')!r}")
            record = {"link": link.get("name"), "collision_index": ci,
                      "source_reference": filename, "source_sha256": digest,
                      "source_scale": scale, "source_bounds_m": mesh.bounds.tolist(),
                      "source_volume_m3": float(mesh.volume), "pieces": []}
            insertion = list(link).index(collision)
            used_names = {node.get("name") for node in link.findall("collision")}
            for pi, (vertices, faces) in enumerate(parts):
                relative = f"collision/link_{li:04d}_collision_{ci:04d}_part_{pi:04d}.stl"
                info = write_hull(staging / relative, vertices, faces, np, trimesh)
                info.update(path=relative, sha256=sha256(staging / relative))
                record["pieces"].append(info)
                piece = deepcopy(collision)
                name = f"acdc_collision_{li}_{ci}_{pi}"
                while name in used_names:
                    name += "_"
                used_names.add(name)
                piece.set("name", name)
                element = piece.find("geometry/mesh")
                element.set("filename", relative)
                element.set("scale", "1 1 1")
                link.insert(insertion + pi, piece)
            link.remove(collision)
            report["decompositions"].append(record)
        urdf_path = staging / source.name
        tree.write(urdf_path, encoding="utf-8", xml_declaration=True)
        report["output_urdf_sha256"] = sha256(urdf_path)
        (staging / "collision-provenance.json").write_text(json.dumps(report, sort_keys=True, indent=2) + "\n", encoding="utf-8")
        if destination.exists() or destination.is_symlink():
            raise ValueError("Output directory appeared during conversion; refusing to replace it")
        os.rename(staging, destination)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", type=Path)
    parser.add_argument("output", type=Path, help="new bundle directory, separate from the source export")
    parser.add_argument("--threshold", type=float, default=0.05, help="normalized CoACD concavity threshold, 0.01–1")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args(argv)
    try:
        report = convert(args.urdf, args.output, threshold=args.threshold, seed=args.seed)
    except (ValueError, OSError, RuntimeError, ET.ParseError) as exc:
        parser.exit(1, f"Collision conversion stopped: {exc}\n")
    count = sum(len(item["pieces"]) for item in report["decompositions"])
    print(f"Wrote {count} convex collision meshes to {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
