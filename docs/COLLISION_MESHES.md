# Optional URDF convex collision meshes

The Fusion Add-In keeps its existing defaults. For contact applications, the
standalone `scripts/convexify_urdf.py` tool makes a **new URDF bundle** with one
mesh and one `<collision>` element per CoACD convex piece. It runs after export,
outside Fusion's Python, and never installs native packages into Fusion.

## Install and run

Use Python 3.12 or later (required by the pinned NumPy version) in a separate virtual environment:

```bash
python3 -m venv .venv-collision
.venv-collision/bin/python -m pip install -r scripts/requirements-collision.txt
.venv-collision/bin/python scripts/convexify_urdf.py \
  /path/to/export/robot.urdf /path/to/robot-convex \
  --threshold 0.05 --seed 0
```

On Windows, use `.venv-collision\Scripts\python.exe` for the last two commands.
The output parent directory must exist. The output directory must be new and
must not contain, or be contained by, the input URDF's directory. Use sibling
export and processed directories. An existing destination is rejected.

Load `robot-convex/robot.urdf` in the intended target simulator and review the
collision geometry and contact behavior before relying on it. This tool does
not alter the Fusion design or its original export.

## Geometry and frame contract

- Input is ordinary URDF with local STL mesh references, as written by
  ACDC4Robot. Relative paths resolve from the input URDF's directory. Absolute
  local paths are also accepted. `package://`, `file://`, remote URLs, Windows
  backslash paths, and OBJ/DAE meshes are rejected with an explicit error.
  Convert references to local paths with forward slashes first. OBJ materials
  and scene hierarchies need a separate supported workflow.
- Only existing **mesh collision** elements are decomposed. Primitive
  collisions remain intact; the tool does not invent collision geometry from
  visual-only links. Multiple mesh collisions per link are supported.
- The URDF mesh `scale` is authoritative. Source STL coordinates are multiplied
  by the three scale values before decomposition. Thus Fusion's usual
  millimetre STL plus `scale="0.001 0.001 0.001"` enters CoACD in metres.
  Nonuniform positive scales are supported; missing scale means `1 1 1`.
  Zero, negative, malformed, and non-finite scales are rejected.
- Every resulting collision retains the original collision origin and other
  child metadata. Hull vertices use metres and the resulting mesh has
  `scale="1 1 1"`. The collision origin is **not baked into the mesh**, so its
  original translation and rotation still apply exactly once.
- Visual meshes and textures are copied byte for byte into hash-addressed
  `assets/` paths. Their references are relocated into the new bundle; visual
  scale, origin, material definitions, link inertias, joints, and primitive
  collisions are retained. XML formatting can change; XML comments survive.
- Source meshes must be finite, closed, consistently outward wound solids with
  positive volume. Coincident STL vertices are welded for topology checking.
  No mesh repair, remeshing, decimation, hull-count cap, extrusion, or hull
  dropping happens silently. Invalid source solids or empty/invalid backend
  results stop conversion. Each returned hull is checked for closed topology,
  consistent outward winding, positive volume, and convexity before writing. Repair source geometry explicitly if needed.
- Hulls are individual ASCII STL files with 17-significant-digit coordinates.
  They are separate assets because engines that convexify each mesh as a whole
  can erase concavities if all pieces are combined into one file.

## Settings and provenance

`--threshold` is CoACD 1.0.7's **dimensionless normalized concavity threshold**
from 0.01 to 1. It is not a distance in metres. Lower values generally request
closer approximation and more parts. `--seed` defaults to zero. All other
backend arguments are fixed explicitly and recorded in
`collision-provenance.json`, including unlimited hull count, preprocessing
`off`, and decimation `false`.

The report records the tool hash, Python and dependency versions, source URDF
and mesh SHA-256 hashes, input scales, source bounds/volume, every output part's
hash/bounds/vertex and triangle counts, copied assets, and output URDF hash.
Keep the input export and report for reruns. A fixed seed and pinned dependencies
make configuration reproducible; byte-identical CoACD results across platforms
or native parallel backends are not promised.

The new bundle is built in a temporary sibling directory and published only
after all assets, hulls, XML, and provenance succeed. A normal conversion error
removes staging and leaves the destination absent. No existing output or input
file is replaced. No simulator-specific vertex/hull limits are guessed: if a
target rejects a result, review settings and explicitly validate a revised
model rather than dropping geometry.

## Validation and limits

The optional test suite checks nonuniform scaling, translated/rotated collision
origins, preserved visual bytes/materials/textures, unchanged primitives and
joints, distinct hull assets, source integrity, invalid inputs, backend failure
cleanup, rejection of open/reversed/nonconvex backend pieces, and repeatable provenance with a deterministic test backend.

Run the real native CoACD integration on a synthetic concave L prism with:

```bash
ACDC_RUN_COACD=1 OMP_NUM_THREADS=1 .venv-collision/bin/python \
  -m unittest discover -s tests -p test_collision_pipeline.py -v
```

This checks that real CoACD produces multiple closed convex pieces and that
URDF references match them. The ordinary suite skips the optional tests when
these dependencies are absent. No live Gazebo, Isaac Sim, PyBullet, or MuJoCo
acceptance is implied; collision approximation and contact performance must be
checked in the target application. This first workflow supports URDF/STL only,
not SDF/MJCF rewriting or an in-Fusion decomposition UI.

## References

- [CoACD upstream documentation](https://github.com/SarahWeiii/CoACD) and
  [CoACD 1.0.7 package](https://pypi.org/project/coacd/1.0.7/)
  describe normalized thresholds, seed, preprocessing, and part output.
- [Issue #6](https://github.com/ACDC4Robot/Fusion360/issues/6) proposed optional
  convex collision meshes. The contributor's
  [post-export workflow](https://github.com/NmDongQ/acdc4robot-fix/blob/main/fix_acdc_export.py)
  demonstrates separate collision assets. This implementation preserves the
  existing collision source geometry and adds explicit unit handling and
  provenance without adopting its simulator-specific hull caps.
