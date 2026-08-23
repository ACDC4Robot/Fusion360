# MJCF export guide

ACDC4Robot 1.1.1 exports a Fusion assembly as a structural MuJoCo model. The
Fusion document remains the authority for component occurrence transforms,
physical properties, and supported joint kinematics.

## Assembly requirements

- Use a flat occurrence structure. Nested occurrences fail preflight.
- Every visible link must contain at least one body.
- Exportable geometry must be a solid BRep body. Surface BRep and Fusion mesh
  bodies are rejected before STL export.
- Give every occurrence and joint a unique name.
- Connect visible links as one tree using rigid, revolute, or slider joints.
- Create joints between two component occurrences. A Fusion joint whose parent
  or child is the root/ground is rejected with a corrective message; put the
  grounded geometry in a component such as `base_link` instead.
- Give every revolute or slider joint a valid origin. Missing or broken origin
  geometry is rejected because MuJoCo needs the axis location.
- In a Fusion joint, occurrence one becomes the child and occurrence two the
  parent in the exported body tree.
- Freeze revisions of externally referenced components before publishing a
  reproducible model.

The preflight runs before an export directory is created. A rejected assembly
therefore cannot leave a partial model that looks successful. Every successful
export includes `acdc4robot-export-report.json`.

## Student-facing failure contract

The exporter stops before writing a partial model when it encounters:

- a joint connected directly to Fusion root/ground;
- a moving joint with missing or broken origin geometry;
- a cylindrical, pin-slot, planar, ball, or unknown joint type;
- a hidden/dangling endpoint, duplicate name, disconnected link, multiple
  parent, cycle, or multiple-root assembly;
- unsupported surface or mesh-only source geometry.

These failures name the offending object and state the corrective action. This
contract is suitable for course instructions and automated evidence screening:
students should fix the Fusion assembly rather than editing an incomplete MJCF
until it happens to load.

## Deliberate model boundary

The generated MJCF contains:

- one body per exported occurrence;
- Fusion-derived transforms, mass, center of mass, and full inertia tensor;
- rigid, hinge, and slide relationships;
- uniquely named STL visual meshes;
- Fusion joint limits when both limits are enabled.

The generated MJCF does not invent:

- `<actuator>` or `<sensor>` elements;
- joint `armature`, `damping`, or `frictionloss`;
- motor gains, delay, force limits, or velocity limits;
- collision geometry or a floor datum.

Add those properties in a reviewed, application-specific MJCF layer. This
separation prevents CAD export from silently asserting unmeasured actuator or
contact behavior.

## Verification performed for 1.1.1

The v1.1.0 MJCF path was exercised in Fusion on macOS with a linked six-occurrence,
one-revolute-joint assembly containing repeated pin and payload components. The
result passed preflight, exported six uniquely named STLs, and compiled in
MuJoCo 3.3.7 as one connected one-DOF model with no startup contacts.

Version 1.1.1 retains that exporter/model path and adds Fusion API stub
regressions for wrapped `JointOrigin` geometry, grounded endpoints, missing
moving-joint origins, unsupported joint types, stable ball/planar axis access,
valid connected trees, and release identity. The deterministic installable ZIP
is also rebuilt twice and compared byte-for-byte. These tests do not replace a
live Fusion regression for every assembly topology.
