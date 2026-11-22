# MuJoCo Sim (Task C)

This folder replaces the old ROS2/RViz2 plan with a MuJoCo replay demo.

## Run
From `axion/` root:

```powershell
python sim/replay.py
```

## What it does
- Loads `sim/assets/scene.xml` (includes vendored `axion_arm.xml`).
- Reads `data/commands.json`.
- Replays 200 commands at 0.5s intervals.
- Applies `joint_angles.elbow` and `joint_angles.wrist` (degrees in JSON, radians in MuJoCo).
- Shows on-screen HUD: status, intent, confidence, EEG/EMG weights, commanded vs applied angles.

## Safety behavior
- `ACTIVE`: normal movement, green.
- `DEGRADED`: movement continues, amber warning + source weights.
- `PAUSED`: arm freezes; `intent` and `joint_angles` are ignored until status becomes `ACTIVE` or `DEGRADED`.

## Files
- `assets/axion_arm.xml` — 2-DOF arm model (elbow, wrist), actuators, sensors, keyframes.
- `assets/scene.xml` — lighting, cameras, workspace visuals.
- `replay.py` — parser, safety state machine, real-time replay, MuJoCo viewer overlay.

## Headless check
```powershell
python sim/replay.py --smoke
```
Expected: `SMOKE_OK steps=200`.
