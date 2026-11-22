# AXION-1 — Adaptive Neuro-Mirroring Interface (MuJoCo Demo)

AXION-1 is a post-stroke assistive BCI rehab concept: **EEG + EMG -> intent/safety decision -> robot arm movement**.

This repo is structured for a national-hackathon demo. **MuJoCo does only Task C (arm replay).** It does **not** process raw EEG/EMG.

## What’s Already Done (Phase A/B)
- **Task A (signal processing, upstream/MATLAB)**: produced `phase2/output/axion1_features.csv` (135 windows, CSP EEG + 4ch EMG features, quality scores).
- **Task B (Python ML/adaptive engine)**: `phase2/train_axion_ml.py` trains EEG/EMG/fusion classifiers; `phase2/axion_adaptive_engine.py` applies adaptive weighting + spasm safety logic.

## Task C — MuJoCo Arm Replay (this build)
Input: `data/commands.json` (200 commands, 0.5s cadence, 0–99.5s).  
Each command already contains the decision:

```json
{
  "timestamp": 0.5,
  "intent": "flex",
  "confidence": 0.84,
  "source_weights": {"eeg": 0.58, "emg": 0.42},
  "safety_status": "ACTIVE",
  "joint_angles": {"elbow": 45.0, "wrist": 10.0}
}
```

### Safety Contract (implemented exactly)
- `ACTIVE` — move arm to commanded angles, green status.
- `DEGRADED` — move arm, amber warning + show EEG/EMG weights.
- `PAUSED` — **freeze arm**, ignore `intent` and `joint_angles` until status returns to `ACTIVE`/`DEGRADED`.

## Project Structure
```text
axion/
  README.md
  requirements.txt
  .gitignore
  data/
    commands.json                 # Canonical MuJoCo replay command stream
  docs/
    Axion_v1_Brief.pdf
    figures/fig4_degradation_demo.png
  phase2/                         # Extracted Phase 2 bundle (Task A/B)
    README.md
    train_axion_ml.py
    axion_adaptive_engine.py
    run_task_b.py
    models/axion_models.joblib
    output/
  sim/
    assets/
      axion_arm.xml               # Vendored custom 2-DOF assistive arm
      scene.xml                   # Camera, lighting, floor/workspace
    replay.py                     # MuJoCo passive viewer replay + HUD
    README.md
  archives/                       # Original zips preserved
```

## Quick Start (Windows / PowerShell)
```powershell
cd C:\Users\User\Desktop\axion
python -m pip install -r requirements.txt
python sim/replay.py
```

### Useful options
```powershell
python sim/replay.py --speed 1.0          # real-time 0.5s/command
python sim/replay.py --speed 2.0          # faster demo
python sim/replay.py --smoke              # headless validation, no GUI
```

## Arm Design
Custom **2-DOF assistive manipulator** (vendored MJCF, no external model dependency):
- Joints: `elbow`, `wrist` (hinge, radians internally; commands are degrees).
- Actuators: position actuators `elbow_act`, `wrist_act`.
- Visual: base/pedestal, shoulder/elbow/wrist housings, upper arm, forearm, end-effector, cyan accent, studio lighting, fixed cameras.
- Direct 1:1 mapping to command contract: `joint_angles.elbow`, `joint_angles.wrist`.

## Validation Performed
- `commands.json` schema validated (200 commands, required keys, valid statuses/intents).
- Headless smoke replay: `python sim/replay.py --smoke` -> `SMOKE_OK steps=200`.
- Safety state machine verified: PAUSED freezes applied control; transitions match expected stream (`ACTIVE -> DEGRADED @40.5s -> PAUSED bursts 57.5-74s -> DEGRADED -> ACTIVE @86.0s`).

## Notes
- `data/commands.json` is the canonical sim replay input.  
- `phase2/output/axion_live_commands.json` is a different, earlier bundle schema (135 windows, shoulder/elbow/wrist, FLEX/EXTEND/HOLD) kept for traceability.
- Original zips are preserved in `archives/`.
