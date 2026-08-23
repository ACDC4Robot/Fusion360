# Changelog

## 1.2.0 - Unreleased

### Added

- Simplified Chinese localization for the command panel, selection prompts,
  completion dialogs, actionable export errors, and the MJCF preflight summary.
- Locale-aware option labels that always map back to stable technical values
  such as `MJCF` and `MuJoCo` before export logic runs.
- English fallback for unsupported Fusion UI languages and untranslated future
  diagnostics.
- Regression coverage for catalog parity, fallback behavior, option round trips,
  localized dynamic diagnostics, and the localized MJCF preflight summary.

### Compatibility and attribution

This implementation ports the dependency-free localization design and tests
contributed by MermaidFAR in pull request #14 onto the current exporter. It
retains the version 1.1.1 fail-closed MJCF preflight, occurrence-preserving
naming, actionable joint validation, and English machine-readable provenance
report. Version 1.2.0 remains a development candidate until live Fusion testing
is complete; version 1.1.1 remains the current supported GitHub release.

## 1.1.1 - 2026-08-23

### Fixed

- Correctly distinguish and unwrap Fusion `JointOrigin` objects when resolving
  a joint frame. The previous instance-to-class comparison could never be true.
- Reject moving joints with missing or broken origin geometry before export.
- Replace undefined-variable crashes for cylindrical, pin-slot, planar, ball,
  and unknown joint types with errors that name the joint and supported types.
- Replace grounded/root-joint attribute failures with an explanation of how to
  create a component-to-component joint, such as against `base_link`.
- Guarantee that joint-axis accessors return a stable two-value tuple.
- Preserve the failing link, joint, or assembly name when lower-level writers
  raise an unexpected exception.

### Changed

- Sanitize the Fusion document name for the export directory/model name and use
  `robot` if sanitization produces an empty name.
- Show expected validation failures as concise Fusion dialogs while retaining
  full tracebacks in Text Commands for diagnosis.

### Attribution and scope

This release reconciles the joint-origin, grounded-joint, unsupported-type,
axis-return, and error-context findings contributed in pull request #13. It
does not merge that pull request's conflicting occurrence-name registry or its
broader URDF/SDF validation redesign; v1.1.0 occurrence-preserving names and
the MJCF-specific fail-closed preflight remain authoritative.

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
