# PR #17 revision — 2026-10-04

Prepared against main 1acbfc138bfcbafb1f311c0e261a6fac68fa4806, following review of contributor revision 61204f3c9556ede381fe67f65ddfba1bbf5cfb2b.

The exporter now discovers links from joint endpoint occurrences and shares that implementation with Robot. A selected endpoint owns its rigid subtree; endpoints must not overlap through ancestor/descendant paths. Distinct joints with identical names are preserved so format validation can report name collisions rather than silently discarding mechanical relationships. Repeated references to the same API joint are removed by object identity/equality, not display names or raw entityToken string comparison.

Discovery fails before mesh export on root/ground endpoints, hidden endpoints, overlapping link subtrees, or visible body-containing occurrences outside the link subtrees. A jointless model retains the prior visible leaf-occurrence behavior. MJCF preflight also uses shared joint deduplication.

Validation: 32 unittest tests pass, including seven isolated discovery regressions. These use Fusion API mocks, not live exported CAD. No numerical subtree mass or STL aggregation validation is claimed.

Nested MJCF remains deliberately rejected by the existing preflight until live tests establish mesh/inertia accounting. URDF/SDF nested discovery is improved but needs live fixtures too, particularly named visual/collision body overrides, nested joints/proxies, repeated component instances, hidden subparts and physical-property visibility rules. Do not advertise general nested-assembly support yet.

Before merging: open a flat repeated-occurrence robot and a non-overlapping nested rigid-subpart robot in Fusion. Verify every body is represented once, total mass and inertia against CAD, and joint frame/axis transforms. Compile generated MJCF for the currently supported flat fixture. Confirm unsupported nested MJCF fails before writing files. Keep PR #11 precision changes separate.
