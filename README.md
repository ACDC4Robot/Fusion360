# ACDC4Robot: Automated Conversion of Description Conventions for Robots from Design to Learning
<!-- Alternative name# ACDC4Robot: Automated Conversion of Description Conventions for Robots from Design to Learning -->
<!-- <div style="text-align: center;">
    <img src="./pictures/ACDC4Robot-Logo-Text.png" alt="ACDC4Robot Logo" width="400"/>
</div>

## Survey
We are currently doing a survey on robot description formats and the user experience of ACDC4Robot. It would be very appreciate for you to fill in the questionnaire.

[👉**Survey Link**👈](https://forms.gle/v3gUau9YgiAZG1XB8)

<div style="text-align: center;">
    <img src="./pictures/Github-Survey.png" alt="Survey QR Code" width="400"/>
</div> -->

## Introduction
Robot description formats encode the geometry, kinematics, and physical
properties required for simulation, visualization, and planning. ACDC4Robot is
a Fusion 360 Add-In that exports these descriptions directly from a robot CAD
assembly.

The Add-In exports URDF, SDFormat (SDF), and MJCF, and also exposes an
experimental URDF+ option. Version 1.1.1 has its strongest current validation
on the Fusion→MJCF→MuJoCo path. URDF, SDF, URDF+, and target-simulator behavior
should be independently checked for the intended application.

The companion [ACDC4Robot RobotLibrary](https://github.com/ACDC4Robot/RobotLibrary)
provides reusable Fusion 360 robot models for design, simulation, and learning.

## Project update and roadmap

### Current update

- **ACDC4Robot v1.1.1 is the current supported GitHub release.** Download the
  installable plugin and checksum from the
  [v1.1.1 release page](https://github.com/ACDC4Robot/Fusion360/releases/tag/v1.1.1).
- **The `main` branch now contains the v1.2.0 development candidate**, adding
  Simplified Chinese UI localization contributed through pull request #14.
  It is available for live Fusion testing before its GitHub Release is cut.
- The Fusion→MJCF→MuJoCo path now preserves repeated occurrence identities,
  runs a fail-closed assembly preflight, records an export provenance report,
  and gives actionable errors for grounded endpoints, missing joint origins,
  unsupported joint types, disconnected assemblies, and unsupported geometry.
- GitHub Releases are the canonical update channel. We do not currently plan
  an Autodesk App Store update, so the App Store build may be older.
- Reusable robot designs are maintained separately in the
  [ACDC4Robot RobotLibrary](https://github.com/ACDC4Robot/RobotLibrary).

### Future directions

- expand live Fusion regression fixtures and automated export checks;
- improve format-specific URDF, SDF, and simulator compatibility without
  weakening the current MJCF validation contract;
- investigate broader nested-assembly support after reproducible fixtures are
  available;
- add an optional, reproducible collision-mesh workflow for contact-rich
  applications while keeping visual and collision assets distinct;
- continue improving documentation, examples, and community-contributed robot
  models through this repository and the RobotLibrary.

These are roadmap directions, not committed release dates. Issues and pull
requests with reproducible Fusion fixtures are especially helpful.

## Key Features
Export robot description files directly from a Fusion 360 design using a GUI
panel.

<!-- - Support 3 Fusion360 joint motion types:
    - Fixed joint type
    - Revolute joint type with limitation
    - Slider joint type with limitation -->
- Supported robot description formats:
    - [URDF](http://wiki.ros.org/urdf/XML) (Unified Robotics Description Format)
    - [SDFormat](http://sdformat.org/spec) (Simulation Description Format) or SDF
    - [MJCF](https://mujoco.readthedocs.io/en/latest/XMLreference.html) (MuJoCo Format)
    - URDF+ (experimental; not part of the v1.1.1 regression scope)
- A companion [robot model library](https://github.com/ACDC4Robot/RobotLibrary)
  containing various robot types:
  - Robot Arm
  - Gripper
  - Mobile Robot
  - Quadruped Robot
  - Humanoid

## Installation
Install the Add-In from GitHub Releases (recommended), from the legacy Autodesk
App Store package, or manually from source.

### GitHub release (recommended)

Download the current installable ZIP and checksum from
[GitHub Releases](https://github.com/ACDC4Robot/Fusion360/releases). GitHub is
the supported update channel and contains fixes that may not be present in the
older Autodesk App Store package. For v1.1.1, download
`ACDC4Robot-1.1.1.zip` and its `.sha256` file, verify the checksum, unzip it,
and copy the resulting top-level `ACDC4Robot` folder into Fusion's Add-Ins
directory. Do not copy the ZIP itself or create a doubly nested
`ACDC4Robot/ACDC4Robot` directory.

Verify the downloaded ZIP on macOS/Linux with:

```bash
shasum -a 256 ACDC4Robot-1.1.1.zip
```

On Windows PowerShell, use:

```powershell
Get-FileHash .\ACDC4Robot-1.1.1.zip -Algorithm SHA256
```

### Autodesk App Store (legacy distribution)

An earlier Windows/macOS package remains available from the
[Autodesk Fusion App Store](https://apps.autodesk.com/FUSION/en/Detail/Index?id=5028052292896011577),
but no immediate App Store update is planned. Prefer the GitHub release for
current behavior and reproducible checksums.

### Manual installation

Clone or download this repository and copy the `Add-IN/ACDC4Robot` folder into
Fusion 360's Add-Ins directory, which is shown under
`Preferences -> General -> API -> Default Path for Scripts and Add-Ins`.

Release maintainers can create a deterministic, directly installable archive
with `python3 scripts/build_addin_release.py`. Unzip it and copy the resulting
`ACDC4Robot` folder into the same add-in directory.

The default locations are:

Windows:
```
%appdata%\Autodesk\Autodesk Fusion 360\API\AddIns
```

Mac:
```
$HOME/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns
```
If Fusion uses a customized location, use the path reported under
`Preferences -> General -> API -> Default Path for Scripts and Add-Ins`.

### First Run
Restart Fusion after installing or replacing the Add-In. The v1.1.1 manifest
enables `Run on Startup`; if ACDC4Robot is not running, press `Shift+S` or open
`UTILITIES -> Add-Ins -> Scripts and Add-Ins`, find `ACDC4Robot` under
`Add-Ins -> My Add-Ins`, and click `Run`.

The promoted `ACDC4Robot` command appears in the Design workspace's
`UTILITIES -> Add-Ins` panel beside `Scripts and Add-Ins`.
![Run the Add-In](./pictures/RunAdd-In.gif)
Click the command to begin exporting the active design.


## Usage

### MJCF export in version 1.1.1

The MJCF path runs a fail-closed assembly preflight before writing files,
preserves repeated occurrence names, supports standard and as-built joints, and
writes an `acdc4robot-export-report.json` provenance report. Version 1.1.1 also
reports grounded endpoints, missing joint origins, and unsupported joint types
with corrective messages instead of secondary Python failures. See the
[MJCF export guide](docs/MJCF_EXPORT.md) for the supported assembly structure,
verification status, and the deliberate boundary between CAD export and
application-specific actuator/contact modeling.

### User-interface language

The v1.2.0 development candidate follows Fusion's user-interface language and
supports English and Simplified Chinese. Export-format and simulator choices
remain stable technical values (`URDF`, `SDFormat`, `MJCF`, `URDF+`, `Gazebo`,
`PyBullet`, and `MuJoCo`) regardless of the displayed language. Unsupported
languages fall back to English. The machine-readable
`acdc4robot-export-report.json` remains English so reports are reproducible and
can be compared across computers; only user-facing labels and messages are
localized.

### Format and target choices

| Export format | Target selection used by the Add-In | v1.1.1 status |
| --- | --- | --- |
| URDF | Gazebo, PyBullet, or MuJoCo | Available; not revalidated in the v1.1.1 regression suite |
| SDFormat | Gazebo or PyBullet | Available; not revalidated in the v1.1.1 regression suite |
| MJCF | MuJoCo | Current regression-tested path |
| URDF+ | Target selection is not used | Experimental |

An incompatible format/target selection is rejected with a Fusion message.

### Prepare a design for MJCF 1.1.1

The MJCF exporter reads Fusion's internal centimetre-based API values and
converts model quantities to SI units; the document's display unit does not
need to be metres. Parametric and direct-modeling documents are both accepted,
so disabling design history is not a prerequisite. Running the exporter on a
copy remains good practice when preparing or restructuring an assembly.

- Use a flat top-level occurrence structure. Each visible occurrence must
  contain solid BRep geometry; nested occurrences, surface-only components,
  and mesh-only components fail the MJCF preflight.
- Connect the visible occurrences as one tree with rigid, revolute, or slider
  joints. A joint cannot connect directly to Fusion root/ground; represent the
  fixed structure as a component such as `base_link`.
- Give occurrences and joints unique names. Moving joints require a valid
  Fusion joint origin.
- For a self-contained design, use `Break Link` to make an *external component*
  internal. MJCF 1.1.1 can export referenced occurrences, but records a warning
  because their source revisions must be frozen for reproducibility.
![Break Link](./pictures/BreakLink.gif)
- Repeated occurrences are supported by MJCF 1.1.1 and receive distinct export
  names. Other export formats have not been revalidated for this behavior; use
  `Make Independent` if you encounter a format-specific problem.
![Make Independent](./pictures/MakeIndependent.gif)
- Make sure all components are named with alphanumeric characters, underscores
  `_`, or hyphens `-`. Other characters may cause compatibility problems.
- See the [Fusion 360 assembly instructions](./InstructionsForAssembly.md) and
  the stricter [MJCF export guide](docs/MJCF_EXPORT.md) before exporting.

### Export the active design

1. Open the prepared design in Fusion's Design workspace and click
   `ACDC4Robot`.
2. Select a description format and target environment. For the validated
   v1.1.1 path, select `MJCF` and `MuJoCo`.
3. Choose a dedicated parent output folder. ACDC4Robot creates or reuses a
   sanitized document-name subfolder, so use a clean destination to avoid
   mixing files from an earlier export.
4. For MJCF, correct every preflight error and export again. A successful
   export contains `<model-name>.xml`, STL visual meshes, and
   `acdc4robot-export-report.json`.
5. Load the exported model in MuJoCo and review the report, geometry, inertias,
   joint axes, and limits. Add actuators, sensors, joint dynamics, and explicit
   collision geometry in a reviewed downstream MJCF layer; ACDC4Robot does not
   infer them from CAD.

![Execute Fusion 360 Add-In](./pictures/ExcuteAdd-In.gif)

## Tested Examples

The examples below document earlier URDF/SDF workflows. They have not been
revalidated as part of the v1.1.1 MJCF regression suite and should be tested in
the current target simulator before use. The SDF closed-chain example does not
imply MJCF closed-chain support; the v1.1.1 MJCF preflight requires one
connected tree.

### Closed Chain Linkages
- Historical Gazebo test showing an SDFormat closed-chain mechanism
![Test Four Bar Linkages](./pictures/Four-Bar-Linkages-Test.png)

### Robot Manipulator: UR5e
- Historical UR5e test in Gazebo
![Test UR5e manipulator](./pictures/UR5e-Test.png)

### Robot Gripper: Robotiq-2F85-Gripper
- Historical Robotiq 2F-85 gripper test in Gazebo
![Test Robotiq-2F85 Gripper](./pictures/Robotiq-Gripper-Test.png)

## Robot Library
[🤖 ACDC4Robot RobotLibrary](https://github.com/ACDC4Robot/RobotLibrary)

The companion repository contains reusable Fusion 360 robot models intended to
reduce repeated assembly work and support exporter examples. Contributions of
well-documented models, source revisions, and validated exports are welcome.
The [legacy in-repository index](RobotLibrary.md) is retained for historical
context.

## Citation

If ACDC4Robot or the RobotLibrary supports your research, teaching, or robot
development workflow, please cite the peer-reviewed paper:

- [IEEE Xplore](https://ieeexplore.ieee.org/document/10715835/)
- [DOI: 10.1109/ICARM62033.2024.10715835](https://doi.org/10.1109/ICARM62033.2024.10715835)
- [arXiv preprint: 2312.12295](https://arxiv.org/abs/2312.12295)

### BibTeX

```
@inproceedings{qiu2024describing,
  author    = {Qiu, Nuofan and Song, Chaoyang and Wan, Fang},
  title     = {Describing Robots from Design to Learning: Towards an
               Interactive Lifecycle Representation of Robots},
  booktitle = {2024 International Conference on Advanced Robotics and
               Mechatronics (ICARM)},
  year      = {2024},
  pages     = {1081--1086},
  doi       = {10.1109/ICARM62033.2024.10715835},
  url       = {https://doi.org/10.1109/ICARM62033.2024.10715835}
}
```

### Plain text

```
N. Qiu, C. Song, and F. Wan, “Describing Robots from Design to Learning:
Towards an Interactive Lifecycle Representation of Robots,” in 2024
International Conference on Advanced Robotics and Mechatronics (ICARM),
Tokyo, Japan, 2024, pp. 1081–1086,
doi: 10.1109/ICARM62033.2024.10715835.
```

## Reference
- [Fusion2PyBullet](https://github.com/yanshil/Fusion2PyBullet)
- [fusion2urdf](https://github.com/syuntoku14/fusion2urdf)
