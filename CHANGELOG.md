# Changelog

## 1.1.0 - 2026-08-23

### Fixed

- Preserve Fusion occurrence instance and hierarchy suffixes in exported link,
  mesh, and body names. Repeated components no longer overwrite one another's
  STL files.
- Support both iterable Fusion vectors and legacy `count`/`item` collections in
  MJCF preflight checks.
- Include both standard joints and as-built joints in the MJCF assembly graph.
- Use `time.sleep` in the MJCF completion path; the App Store 1.0.0 build used
  the invalid `time.stop` call.
- Stop generating a floor at the Fusion world origin, which could intersect an
  exported assembly.
- Avoid mutating Fusion occurrence transforms while calculating relative body
  frames, adopting the defect analysis from pull request #13.
- Reject surface BRep and mesh-only occurrences during MJCF preflight instead
  of failing later with Fusion's `invalid geometry` STL error.
- Remove a committed developer-specific Fusion Python path.

### Added

- A fail-closed MJCF preflight for duplicate names, nested occurrences,
  unsupported joints, missing endpoints, multiple parents, cycles,
  disconnected links, and multiple roots.
- `acdc4robot-export-report.json`, recording Fusion units, occurrence paths,
  transforms, joints, referenced-component status, warnings, and plugin version.
- A versioned Fusion add-in manifest and regression tests for the MJCF fixes.

### Changed

- Exported CAD meshes are visual-only (`contype=0`, `conaffinity=0`) by default.
  Collision geometry must be deliberately added downstream.
- MJCF explicitly preserves CAD inertials (`inertiafromgeom=false`) and uses a
  2 ms `implicitfast` integration default.

### Scope

This release exports structure, transforms, meshes, inertials, and supported
joint kinematics. It intentionally does not infer actuators, sensors, control
gains, `armature`, `damping`, `frictionloss`, effort limits, or task-specific
collision geometry from CAD.
