# Joint and nested-assembly investigation (#7 and #10)

The exporter stops with a specific error if Fusion cannot supply either
`root.allJoints` or `root.allAsBuiltJoints`, including a failure while iterating
the collection. It preserves the original exception in Text Commands and does
not assume the missing joints can be ignored. Initial discovery happens before
the output folder or report is created. Parent-joint lookup and the MJCF writer
use the same checked enumeration. A later API failure also stops export;
already-written files from an earlier stage or earlier export are not removed.

This improves diagnosis of #10. It does **not** fix Fusion's
`InternalValidationError`, add a component-local fallback, or establish support
for closed-loop URDF. Keep #10 open until a live reproducer and fix are verified.
The regression tests inject failures; they are not live Fusion validation.

## Collect a local report in current Fusion

1. Install the latest Fusion available to your account and record its full
   version (the report also reads `Application.version`). Record the exporter
   commit, OS, selected format, target simulator, and its version separately.
2. Open a **copy** of the affected assembly in the Design workspace. Keep its
   existing joints and constraints intact. The probe does not change them.
3. In Utilities → Scripts and Add-Ins → Scripts, add the folder
   `scripts/fusion_joint_diagnostics` using the add/link control, then run
   `fusion_joint_diagnostics`. Keep the `.py` and `.manifest` files together in
   that folder. [Autodesk's installation guide](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/UsingSamplesFromGitHub_UM.htm)
   describes linking a script outside Fusion's default Scripts directory.
4. Choose a new local `.json` filename. Alternatively set the script's
   `OUTPUT_PATH` to an absolute new filename before running. Existing reports
   are never overwritten. Canceling the save dialog writes nothing.
5. Read the JSON before sharing it. It contains component/joint/constraint
   names, occurrence paths, API error text, Fusion/Python version and platform.
   It does not request geometry, entity tokens or cloud file identifiers, and
   performs no upload. Nothing is sent by the script. API error text can itself
   include model-specific data; redact it if necessary.

The report distinguishes a healthy empty collection (`status: ok, count: 0`),
an unavailable API (`status: unavailable`), and a property/iteration failure
(`status: error`, with its stage and exception). Constraint APIs are optional
guarded probes because availability varies across Fusion builds. Missing
constraint APIs do not prevent the remaining joint metadata from being saved.
Per-component standard/as-built lists and their returned proxy contexts are
recorded independently of the root-wide queries. These native lists are
diagnostic evidence, not a proven replacement for root-context joints.

## #10: minimal live reproduction and A/B test

Use a reproducing CAD copy if one is available. If no CAD is available, a new
synthetic model can investigate the problem, but successful exports of that
model do not establish that the historical error is fixed.

1. Reduce a reproducing copy to the smallest set of solid component
   occurrences, ordinary/as-built joints, and a single closure constraint that
   still triggers the error. Keep a separate untouched reproducing copy. Note
   the exact UI constraint type and which occurrences it connects. Do not
   replace the intended mechanism merely to make export pass.
2. Write down the expected exported links and tree joints. Identify which
   relationship closes the loop and how that relationship should be represented
   by the intended simulator; ignoring a Fusion constraint does not reproduce
   its physical effect in a URDF tree.
3. With the constraint **enabled**, run the JSON probe and attempt export to a
   new destination. Save the report, complete Text Commands traceback and the
   exact exporter revision. If the query fails, confirm that the initial
   failure did not create the chosen output folder or alter an existing model.
4. On the test copy only, manually suppress that one constraint using Fusion's
   UI. Do not delete it, move components, or change any joint. Run the same
   probe/export to different filenames. Record any pose/solver changes.
5. Re-enable the constraint, recompute if Fusion requires it, and repeat the
   enabled run. Verify the failure is repeatable and not a transient stale
   document state. The script performs none of these UI changes automatically.
6. Compare `root.collections.allJoints` and `allAsBuiltJoints` with each
   component's `joints`/`asBuiltJoints`. Include whether the property itself or
   iteration fails, endpoint occurrence paths, and assembly contexts.

For exploratory synthetic work, start with three simple solid occurrences, a
two-joint tree, and a manually chosen closure constraint. Try the reporter's
constraint type when known; record each type tested. Also test standard versus
as-built joints, nested components and two instances of one component. If none
triggers the real error, report "not reproduced on Fusion <version>" rather
than "fixed." Do not use a mock raising the same error string as proof.

A future fallback must preserve every intended occurrence-specific joint,
including repeated/nested assembly contexts, and fail if completeness cannot
be established. Verify exported body coverage, mass/inertia and joint
transforms, then load the output in the named target simulator. Only after
reproducing and resolving the live error should #10 be marked fixed.

## #7: retest the pictured failures and nested assemblies

The original screenshot fails in `Link.get_parent_joint` on
`Occurrence.joints` ("object does not belong to the occurrence's component").
PR #9 replaced that particular access with root-wide lookup. The later
commenter's `NoneType.transform2` traceback is a distinct root/ground endpoint
case; current discovery stops with explicit `base_link` guidance. These
changes justify a retest of the original CAD, not a general nested-support
claim. Other occurrence/joint APIs still need runtime validation.

On the current exporter and latest available Fusion:

1. Run the probe and export the original assembly, or the smallest CAD that
   reproduces the reported exception. Include full version/revision information.
2. For the grounded-endpoint case, keep a failing fixture to check the explicit
   diagnosis; in a separate valid fixture represent fixed geometry as a
   `base_link` component and connect component occurrences with supported joints.
3. Exercise a flat two-link baseline, nested rigid subparts under one jointed
   link, and repeated instances. Include a nested joint endpoint and a hidden
   subpart where the intended ownership is explicit. Overlapping link subtrees
   and unowned visible bodies should fail explicitly, not produce missing or
   duplicated geometry. MJCF continues to reject nested assemblies in preflight.
4. Check every intended body is represented once, mesh references exist, mass
   matches CAD, CoM/inertia use the correct frames, and joint origins/axes/limits
   agree with CAD. Load the URDF in the intended Gazebo version and exercise the
   joints. Record visibility and named visual/collision body settings.

Synthetic fixtures can establish those contracts and regression coverage. If
the original CAD is unavailable, keep #7 awaiting reporter retest unless a
reduced fixture reproduces and verifies the same original failure. A successful
synthetic model alone cannot establish the result for the original assembly.

API contracts: Autodesk's [root joint enumeration](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Component_allJoints.htm)
returns joints in the selected component's context; [component-local joints](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Component_joints.htm)
and [assembly proxies](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Joint_createForAssemblyContext.htm)
must be distinguished when investigating a fallback.
