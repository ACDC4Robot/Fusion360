# MJCF export guide

ACDC4Robot 1.1.0 exports a Fusion assembly as a structural MuJoCo model. The
Fusion document remains the authority for component occurrence transforms,
physical properties, and supported joint kinematics.

## Assembly requirements

- Use a flat occurrence structure. Nested occurrences fail preflight.
- Every visible link must contain at least one body.
- Exportable geometry must be a solid BRep body. Surface BRep and Fusion mesh
  bodies are rejected before STL export.
- Give every occurrence and joint a unique name.
- Connect visible links as one tree using rigid, revolute, or slider joints.
- In a Fusion joint, occurrence one becomes the child and occurrence two the
  parent in the exported body tree.
- Freeze revisions of externally referenced components before publishing a
  reproducible model.

The preflight runs before an export directory is created. A rejected assembly
therefore cannot leave a partial model that looks successful. Every successful
export includes `acdc4robot-export-report.json`.

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

## Verification performed for 1.1.0

The MJCF path was exercised in Fusion on macOS with a linked six-occurrence,
one-revolute-joint assembly containing repeated pin and payload components. The
result passed preflight, exported six uniquely named STLs, and compiled in
MuJoCo 3.3.7 as one connected one-DOF model with no startup contacts.

The automated tests use Fusion API stubs and cover name preservation, current
iterable vector compatibility, disconnected assemblies, valid connected trees,
duplicate joint names, and release identity. They do not replace live Fusion
regression testing.
