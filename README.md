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

Also, we provide a Fusion 360 robot model library that can be used out-of-box for design, simulation, and learning. 
👉[🤖Fusion 360 Robot Library](RobotLibrary.md)

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
- An [robot model library](RobotLibrary.md) including various robot types:
  - Robot Arm
  - Gripper
  - Mobile Robot
  - Quadruped Robot
  - Humanoid

## Installation
You can install this Add-In form Autodesk Fusion app store or manually.

### Installation form Autodesk App Store
You can download [ACDC4Robot](https://apps.autodesk.com/FUSION/en/Detail/Index?id=5028052292896011577) from Autodesk Fusion App Store, Windows and Mac version are provided.

Due to the review process of app store, ACDC4Robot at app store might not update in the same frequence as this repository. So updates such as new features and bug fix might be late for ACDC4Robot app store version.

### Manually Installation
Download and copy `/Add-IN/ACDC4Robot` folder into Fusion360's Add-in directory which can be found at `Preferences -> General -> API -> Default Path for Scripts and Add-Ins`.

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

Find `Fusion2Robot` at `Add-Ins -> My Add-Ins`, select `Fusion2Robot` and click `Run`(for normally use, please select `Run on Startup`). Then the icon will appear beside `UTILITIES -> Add-Ins icon`.

### Localization

The add-in follows Fusion's current UI language. Simplified Chinese (`ChinesePRCLanguage`) is supported, and all other Fusion languages fall back to English. Robot-description and simulator names such as URDF, MJCF, Gazebo, and MuJoCo remain unchanged so exported behavior and file formats are language-independent.
![Run the Add-In](./pictures/RunAdd-In.gif)
Click the icon to start exporting process from the current design.


## Usage
### Before Using This Add-In
Before exporting robot description files, please follow the following instructions to make sure the design file is suitable to execute this add-in. 

To prevent unexpected modification of the original design, it is better to <mark>run this add-in in a copy of the design file</mark>.

- Exit parametric mode: right click the root component of the design, choose `Do not capture Design History`
![Do not capture Design History](./pictures/DoNotCaptureDesignHistory.PNG)
- Set the default unit of the design document to `m`
![Change Units](./pictures/ChangeUnits.png)
- Using `Break Link` to make *external component* into *internal component*
![Break Link](./pictures/BreakLink.gif)
- Occurrences refer to the same component are not supported yet, use `Make Independent` to independ those occurrences.
![Make Independent](./pictures/MakeIndependent.gif)
- - Make sure all the components are named with alphanumeric characters, underscore character `_`, and hyphe character `-`. Other languages and characters might cause problems.
- - Components need to be joint together with a **flat** structure in order to export the robot description files properly. Please check the [instructions for Fusion360 Assembly](./InstructionsForAssembly.md).

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
[🤖Fusion 360 Robot Library](RobotLibrary.md)

We also provides some robot models that assembled manually, and tested the Add-In with these models. 
This robot library is continuing update, it is welcome to contribute this library.

## Citation
For BibTex:
```
@INPROCEEDINGS{10715835,
  author={Qiu, Nuofan and Song, Chaoyang and Wan, Fang},
  booktitle={2024 International Conference on Advanced Robotics and Mechatronics (ICARM)}, 
  title={Describing Robots from Design to Learning: Towards an Interactive Lifecycle Representation of Robots}, 
  year={2024},
  volume={},
  number={},
  pages={1081-1086},
  keywords={Solid modeling;Mechatronics;Codes;Automation;Pipelines;Morphology;Transforms;Software;Libraries;Robots},
  doi={10.1109/ICARM62033.2024.10715835}}
```

Plain Text:
```
N. Qiu, C. Song and F. Wan, "Describing Robots from Design to Learning: Towards an Interactive Lifecycle Representation of Robots," 2024 International Conference on Advanced Robotics and Mechatronics (ICARM), Tokyo, Japan, 2024, pp. 1081-1086, doi: 10.1109/ICARM62033.2024.10715835. keywords: {Solid modeling;Mechatronics;Codes;Automation;Pipelines;Morphology;Transforms;Software;Libraries;Robots},
```

## Reference
- [Fusion2PyBullet](https://github.com/yanshil/Fusion2PyBullet)
- [fusion2urdf](https://github.com/syuntoku14/fusion2urdf)
