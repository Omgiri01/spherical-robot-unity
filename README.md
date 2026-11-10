# 3-DOF Spherical Robot — Real-Time Unity 3D Simulator

[![Unity](https://img.shields.io/badge/Unity-2022.3%2B-black.svg?logo=unity)](https://unity.com/)
[![Render Pipeline](https://img.shields.io/badge/URP-Universal%20Render%20Pipeline-blue.svg)](https://unity.com/srp/universal-render-pipeline)
[![Platform](https://img.shields.io/badge/Platforms-Windows%20%7C%20Android-green.svg)](https://github.com/Omgiri01/spherical-robot-unity/releases)
[![C#](https://img.shields.io/badge/C%23-10-purple.svg?logo=csharp)](https://docs.microsoft.com/en-us/dotnet/csharp/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A high-performance, real-time 3D simulation and kinematics controller for a 3-degree-of-freedom Spherical Manipulator (Revolute-Revolute-Prismatic: R-R-P) built using Unity and the Universal Render Pipeline (URP). Features procedural geometry generation, real-time forward kinematics execution, dynamic 3D workspace trajectory trail rendering, and comprehensive dual-input support (Desktop keyboard/mouse and Mobile multi-touch virtual joysticks).

---

## 🤖 Manipulator Specifications

The simulation models an industrial R-R-P spherical robot arm conforming to Standard Denavit–Hartenberg conventions:

- **Base Offset ($d_1$):** $0.35$ m vertical column height.
- **Joint 1 ($\theta_1$):** Azimuth turntable rotation: $-180^\circ \le \theta_1 \le +180^\circ$ ($360^\circ$ continuous).
- **Joint 2 ($\theta_2$):** Shoulder pitch / elevation: $-90^\circ \le \theta_2 \le +90^\circ$ (horizontal to vertical upper-hemisphere sweep).
- **Joint 3 ($d_3$):** Prismatic extension stroke: $0.20\text{ m} \le d_3 \le 0.90\text{ m}$ (total radial stroke $= 0.70$ m).

---

## 🎮 Interactive Controls & Input Schemes

The controller automatically detects whether it is running on desktop or touch-enabled devices and presents optimized interaction modes:

### Desktop (Keyboard & Mouse)
- `A` / `D` or `Left` / `Right Arrow`: Rotate Joint 1 ($\theta_1$) base azimuth.
- `W` / `S` or `Up` / `Down Arrow`: Pitch Joint 2 ($\theta_2$) elevation.
- `E` / `Q`: Extend / Retract Joint 3 ($d_3$) prismatic linear axis.
- `Space`: Pause or Resume automatic trajectory playback.
- `Right-Click + Drag`: Orbit 3D perspective camera around the manipulator.
- `Mouse Scroll Wheel`: Zoom camera in and out.

### Mobile & Touchscreen (Android / Tablets)
- **Virtual On-Screen Joystick:** Dynamic floating thumb joystick on the left screen half for dual-axis simultaneous rotation ($\theta_1$) and pitch ($\theta_2$).
- **Prismatic Action Buttons:** High-contrast `+` (Extend) and `-` (Retract) on-screen buttons for smooth prismatic telescoping.
- **Single-Finger Touch Drag:** Pan across the environment to freely orbit the 3D inspection camera.

---

## ⚡ Simulation Modes

1. **Manual Jogging Mode (`Manual`):**
   Full manual control with live on-screen HUD telemetry displaying current joint variables ($\theta_1, \theta_2, d_3$), Cartesian tool coordinates $(X, Y, Z)$, and motion velocity.
2. **Automated Workspace Shell Sweep (`AutoShell`):**
   Drives the arm across its maximum radial boundaries ($d_3 = 0.90$ m) to dynamically paint the outer boundary spherical shell.
3. **Full 3-DOF Volumetric Reach (`Auto3DOF`):**
   Executes automated multi-frequency joint oscillations sweeping the full 3D reachable volume.
4. **Isolated Single Joint Verification (`AutoSingle`):**
   Cycles through each individual joint sequentially between its mechanical limits to verify kinematic continuity.

---

## 🛠 Technical Architecture

- **Procedural Mesh Generation:** Dynamic construction of turntable, shoulder dome, outer guide sleeve, inner telescoping rectangular prism, and gripper jaws in `SphericalRobotController.cs`.
- **GPU-Accelerated Trail Tracing:** Low-overhead `LineRenderer` with minimum spatial delta thresholding ($2.5$ mm) and frame-skipping logic to prevent memory allocation and maintain 60+ FPS on mobile hardware.
- **Universal Render Pipeline (URP):** Uses custom URP Lit material (`RobotLit.mat`) with directional rim lighting and ambient occlusion for industrial realism.
- **Responsive Immediate Mode GUI:** DPI-scaled HUD styling (`Sc = Screen.dpi / 160`) adapting seamlessly from mobile phones to high-resolution desktop monitors.

---

## 📂 Repository Structure

```text
spherical-robot-unity/
├── Assets/
│   ├── Scenes/
│   │   └── SampleScene.unity             # Primary interactive simulation environment
│   ├── Scripts/
│   │   └── SphericalRobotController.cs   # Complete kinematics, geometry & input engine
│   ├── Settings/                         # Universal Render Pipeline configuration assets
│   ├── RobotLit.mat                      # Industrial robot surface shader material
│   └── InputSystem_Actions.inputactions  # Unity Input System bindings
├── Packages/
│   ├── manifest.json                     # URP and core package manifests
│   └── packages-lock.json
├── ProjectSettings/                      # Project, quality, and platform build settings
├── Builds/
│   ├── StandaloneWindows64/              # Ready-to-run Windows x64 executable and runtime
│   │   ├── SphericalRobotUnity.exe
│   │   └── UnityPlayer.dll
│   └── Android/
│       └── SphericalRobot.apk            # Pre-compiled Android installation APK
├── .gitignore                            # Comprehensive Unity Git ignore rules
├── LICENSE                               # MIT License
└── README.md                             # Technical documentation
```

---

## 🚀 How to Run

### Option 1: Run the Pre-Built Binaries (No Unity Required)

- **Windows:**
  1. Open `Builds/StandaloneWindows64/`.
  2. Double-click `SphericalRobotUnity.exe` to run.
- **Android:**
  1. Install `Builds/Android/SphericalRobot.apk` onto any Android phone or tablet.
  2. Launch **SphericalRobot** to experience full multi-touch joystick control.

### Option 2: Open and Build from Source in Unity Editor

1. Open **Unity Hub**.
2. Click **Add project from disk** and select this directory (`spherical-robot-unity`).
3. Compatible with **Unity 2022.3 LTS** or newer.
4. In the Project pane, open `Assets/Scenes/SampleScene.unity`.
5. Press the **Play** button at the top of the editor.

---

## 🔗 Theoretical & Mathematical Companion

For the mathematical Denavit–Hartenberg derivation, analytical matrices, and Python trajectory analysis script, refer to the companion project:
👉 **[spherical-robot-workspace](https://github.com/Omgiri01/spherical-robot-workspace)**

---

## 📄 License

This project is licensed under the MIT License — see the [LICENSE](LICENSE) file for details.
