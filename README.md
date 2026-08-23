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
Robot description format (RDF) contains information about robot model which is required by simulation, visualization, planning etc. In this project, we provide a Fusion360 Add-In for generating robot description files automatically from robot design. 

Currently, this Add-In supports exporting URDF, SDFormat, and MJCF. 
URDF (Unified Robotics Description Format) has been the most widely used robot description format, but has several limitations and lack of update. 
SDFormat (Simulation Description Format) has more features than URDF, such as supporting closed loop chain mechanism. SDFormat has been a seperated project from Gazebo aims to be a simulator indenpendt format but still not as popular as URDF. 
MJCF is a robot description format used in simulator MuJoCo and has been support by more simulators such as Nvidia Isaac Sim. It also has more features then URDF to provide more robotic system information. 
Other robot description formats might be supported in the future.

The companion [ACDC4Robot RobotLibrary](https://github.com/ACDC4Robot/RobotLibrary)
provides reusable Fusion 360 robot models for design, simulation, and learning.

## Project update and roadmap

### Current update

- **ACDC4Robot v1.1.1 is the current supported GitHub release.** Download the
  installable plugin and checksum from the
  [v1.1.1 release page](https://github.com/ACDC4Robot/Fusion360/releases/tag/v1.1.1).
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
Export robot description files from Fusion360 design file directly with GUI panel.

<!-- - Support 3 Fusion360 joint motion types:
    - Fixed joint type
    - Revolute joint type with limitation
    - Slider joint type with limitation -->
- Supported robot description formats:
    - [URDF](http://wiki.ros.org/urdf/XML) (Unified Robotics Description Format)
    - [SDFormat](http://sdformat.org/spec) (Simulation Description Format) or SDF
    - [MJCF](https://mujoco.readthedocs.io/en/latest/XMLreference.html) (MuJoCo Format)
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
older Autodesk App Store package.

### Autodesk App Store (legacy distribution)

An earlier Windows/macOS package remains available from the
[Autodesk Fusion App Store](https://apps.autodesk.com/FUSION/en/Detail/Index?id=5028052292896011577),
but no immediate App Store update is planned. Prefer the GitHub release for
current behavior and reproducible checksums.

### Manual installation

Download and copy the `/Add-IN/ACDC4Robot` folder into Fusion 360's Add-In
directory, which is shown under
`Preferences -> General -> API -> Default Path for Scripts and Add-Ins`.

Release maintainers can create a deterministic, directly installable archive
with `python3 scripts/build_addin_release.py`. Unzip it and copy the resulting
`ACDC4Robot` folder into the same add-in directory.

In default it should be at:

Windows:
```
%appdata%\Autodesk\Autodesk Fusion 360\API\AddIns
```

Mac:
```
$HOME/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns
```
or it can be found at `Preferences -> General -> API -> Default Path for Scripts and Add-Ins`.

<!-- ### Installation Using Shell Command
Windows (PowerShell):
```PowerShell
cd <path to /Add-In/Fusion2Robot>
Copy-Item ".\Fusion2Robot\" -Destination "${env:APPDATA}\Autodesk\Autodesk Fusion 360\API\AddIns\" -Recurse
```

macOS (Terminal):
```bash
cd <path to /Add-In/Fusion2Robot>
cp -r ./Fusion2Robot "$HOME/Library/Application Support/Autodesk/Autodesk Fusion 360/API/AddIns/"
``` -->

### First Run
After installation for the first time, use `Shift+S` or click `UTILITIES -> Add-Ins -> Scripts and Add-Ins` to open `Scripts and Add-Ins` window.

Find `ACDC4Robot` at `Add-Ins -> My Add-Ins`, select `ACDC4Robot` and click `Run` (for normal use, select `Run on Startup`). Then the icon will appear beside `UTILITIES -> Add-Ins icon`.
![Run the Add-In](./pictures/RunAdd-In.gif)
Click the icon to start exporting process from the current design.


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

### Before Using This Add-In
Before exporting robot description files, please follow the following instructions to make sure the design file is suitable to execute this add-in. 

To prevent unexpected modification of the original design, it is better to <mark>run this add-in in a copy of the design file</mark>.

- Exit parametric mode: right click the root component of the design, choose `Do not capture Design History`
![Do not capture Design History](./pictures/DoNotCaptureDesignHistory.PNG)
- Set the default unit of the design document to `m`
![Change Units](./pictures/ChangeUnits.png)
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
- Components must be joined in a **flat** assembly structure for reliable export.
  See the [Fusion 360 assembly instructions](./InstructionsForAssembly.md).

### After Setting Up Design File
Click the add-in icon, then chose the robot description format and targeted simulation platform to export.
![Execute Fusion 360 Add-In](./pictures/ExcuteAdd-In.gif)

## Tested Examples
### Closed Chain Linkages
- Test a closed loop linkages in Gazebo to show the ability of SDFormat to describe a closed-chain mechanism
![Test Four Bar Linkages](./pictures/Four-Bar-Linkages-Test.png)

### Robot Manipulator: UR5e
- Test a UR5e manipulator in Gazebo
![Test UR5e manipulator](./pictures/UR5e-Test.png)

### Robot Gripper: Robotiq-2F85-Gripper
- Test Robotiq-2F85 Gripper in Gazebo
![Test Robotiq-2F85 Gripper](./pictures//Robotiq-Gripper-Test.png)

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
