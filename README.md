# Spherical Robot Unity Simulation

A real-time 3D simulation and interactive kinematics controller for a 3-DOF Spherical Manipulator (Revolute-Revolute-Prismatic: R-R-P) built with Unity and Universal Render Pipeline (URP). Supports multi-touch virtual joysticks for mobile (Android/iOS) and keyboard/mouse controls for Desktop (Windows).

---

## Overview

The simulator models an industrial spherical robot arm with forward kinematics based on the Standard Denavit–Hartenberg (DH) convention. It includes dynamic procedural robot link rendering, end-effector 3D trail tracing, and automated workspace boundary visualization.

### Robot Joint Specifications

- **Joint 1 (Base Rotation):** $\theta_1 \in [-180^\circ, +180^\circ]$
- **Joint 2 (Elevation / Shoulder Pitch):** $\theta_2 \in [-90^\circ, +90^\circ]$
- **Joint 3 (Prismatic Extension):** $d_3 \in [0.20, 0.90]$ meters
- **Base Offset:** $d_1 = 0.35$ meters

---

## Features

- **Kinematics Engine:** Evaluates real-time forward kinematics and updates joint poses every frame.
- **Dynamic 3D Geometry:** Procedural procedural rendering of base plate, turntable, shoulder dome, arm housing, telescoping slider, and gripper.
- **Cross-Platform Controls:**
  - **Desktop (Keyboard / Mouse):**
    - `A` / `D` or Left/Right Arrow: Rotate Joint 1 ($\theta_1$)
    - `W` / `S` or Up/Down Arrow: Pitch Joint 2 ($\theta_2$)
    - `E` / `Q`: Extend / Retract Joint 3 ($d_3$)
    - Right-click + Drag: Orbit camera
    - Scroll Wheel: Zoom in / out
  - **Mobile (Touch):**
    - Floating on-screen virtual joystick for joint rotation and elevation.
    - Touch extension and retraction buttons for radial prismatic travel.
    - Single finger pan to rotate camera perspective.
- **Workspace Modes:**
  - **Manual Control:** Free jog mode with live telemetry readouts.
  - **Auto Shell:** Sweeps the outer spherical workspace boundary.
  - **Auto 3DOF:** Explores full combined 3D reach volume.
  - **Auto Single Joint:** Cycles individual joints for isolated verification.
- **Trajectory Trail:** Dynamic GPU-friendly LineRenderer tracing end-effector motion in 3D space with auto-throttling.
- **Pre-Built Binaries:** Includes standalone Windows x64 build and Android `.apk`.

---

## Repository Structure

```text
spherical-robot-unity/
├── Assets/
│   ├── Scenes/
│   │   └── SampleScene.unity             # Main interactive simulation scene
│   ├── Scripts/
│   │   └── SphericalRobotController.cs   # Core kinematics controller & input handler
│   ├── Settings/                         # Universal Render Pipeline (URP) profiles
│   └── RobotLit.mat                      # Robot material configuration
├── Packages/
│   └── manifest.json                     # Unity package manifest
├── ProjectSettings/                      # Unity engine project settings
├── Builds/
│   ├── StandaloneWindows64/              # Pre-compiled Windows standalone executable
│   └── Android/
│       └── SphericalRobot.apk            # Pre-compiled Android installation package
├── .gitignore
└── README.md
```

---

## Getting Started

### Option 1: Run Pre-Built Application

#### Windows
1. Navigate to `Builds/StandaloneWindows64/`.
2. Launch `SphericalRobotUnity.exe`.

#### Android
1. Transfer `Builds/Android/SphericalRobot.apk` to an Android device.
2. Enable installation from unknown sources and install the APK.

### Option 2: Open in Unity Editor

1. Open **Unity Hub**.
2. Click **Add** and select this repository directory (`spherical-robot-unity`).
3. Recommended Unity version: **Unity 2022.3 LTS** or newer (with Universal Render Pipeline).
4. Open the project and load `Assets/Scenes/SampleScene.unity`.
5. Press **Play** to start the interactive simulation.

---

## Kinematics Reference

This simulation implements the Denavit–Hartenberg formulation developed in the companion Python workspace project:
[spherical-robot-workspace](https://github.com/Omgiri01/spherical-robot-workspace).
