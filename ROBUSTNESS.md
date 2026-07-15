# Robustness improvements

This fork hardens the ACDC4Robot exporter against the most common conversion
failures. Summary of what changed and why.

## Crash fixes

| Problem | Symptom before | Fix |
|---|---|---|
| Joint connected to the root component (ground) | `AttributeError: 'NoneType' object has no attribute 'component'` mid-export | Detected up front by the new pre-flight validation with a fix-it hint (put grounded bodies in a `base_link` component) |
| Ball / Planar / Cylindrical / Pin-Slot joints | `UnboundLocalError` deep inside the export | Clear error naming the joint and its type; validation catches it before the export starts |
| Planar joint axis query fell off the end of `get_axes()` | `TypeError: cannot unpack non-iterable NoneType` | `get_axes()` always returns a tuple |
| `geometryOrOriginTwo == adsk.fusion.JointOrigin` compared an object to a class (always False) | Joints defined via a Joint Origin took the wrong code path; broken geometry references raised `RuntimeError` | New `Joint._origin_geometry()` helper with `isinstance` check and `RuntimeError` guard, used everywhere |
| Empty or symbol-only document names | `IndexError` / invalid folder names | Robot name is sanitized with a `robot` fallback |
| Text palette unavailable | `AttributeError` before the export even started | All logging goes through a None-safe `utils.log()` |

## Correctness fixes

- **`coordinate_transform()` no longer mutates its input.** It called
  `invert()` in place on the caller's matrix, silently corrupting frames
  (e.g. a `Link.pose`) that were reused later in the same export. It now
  inverts a copy.
- **Unique link names.** Sanitizing full path names strips the occurrence
  counters (`Part:1`, `Part:2` → both `Part`), which produced duplicate link
  names and an invalid URDF. A per-export name registry now assigns stable,
  unique names (`Part`, `Part_2`) used consistently by links and joints.
- **Joints with no origin geometry** (typical for as-built rigid joints) no
  longer crash the axis computation; the axis falls back to the world frame,
  consistent with the existing pose fallback.

## New: pre-flight validation

Before anything is written, the design is checked and **all** problems are
reported at once (message box + Text Commands palette), instead of dying on
the first one with a raw traceback:

- joints connected to the ground / root component
- unsupported joint types (only Rigid, Revolute, Slider are exportable)
- joints referencing components that are not exported as links
  (hidden, empty, or nested assemblies) — the classic "URDF link not found"
- links with (near-)zero mass (missing physical material) — warning
- multiple root links (URDF must be a single tree) — warning
- a link with more than one parent joint (kinematic loop) — error for URDF,
  warning for URDF+ with a hint about the `loop` name prefix
- moving joints without origin geometry — warning

Errors stop the export; warnings ask whether to continue.

## Quality-of-life

- Export failures now name the exact link/joint that caused them.
- `os.makedirs(..., exist_ok=True)` instead of bare `try/except: pass`
  (real filesystem errors are reported, not swallowed).
- Format/simulator selection is validated *before* the folder dialog.
- Progress is logged to the Text Commands palette.

## Tests

`scratchpad`-style mock tests for the pure-Python logic (name registry,
matrix math, pose conversion) live outside the add-in; the Fusion API is
mocked so they run with plain Python 3.
