# PR #11 compatibility revision — 2026-10-04

The contributor reported Omniverse scientific-notation import warnings and proposed six decimal places plus underscore filenames. The revision emits finite numeric URDF attributes in fixed decimal notation without truncating significant digits: tiny positive mass/inertia values stay positive, and joint axes/limits retain precision. Non-finite values are rejected. This affects both class-based and legacy URDF element builders, without rewriting names or mesh URIs as numbers.

Shared filename normalization uses underscores for spaces and retains occurrence instance suffixes and hierarchy. Consequently all exporters and mesh writers agree on filenames; names containing spaces change relative to prior releases. Sanitized-name collisions are rejected before exporter output instead of overwriting meshes or creating ambiguous links.

38 unittest tests pass: six new serialization tests cover tiny/large decimals, exact decimal roundtrip, zero handling, non-finite rejection, numeric-only XML rewriting, occurrence paths and filename collisions. Existing MJCF, release, localization and discovery tests pass.

No Omniverse live-import compatibility claim is made. Test a generated URDF with meshes in the relevant Isaac Sim/Omniverse release, including small inertias and repeated occurrences, before declaring that target supported. Nested MJCF export remains subject to the separate PR #17 validation gate. The six-decimal global setting is intentionally not adopted because it can silently erase physically meaningful values.
