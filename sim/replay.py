"""AXION MuJoCo replay: 2-DOF assistive arm driven by command stream.

Safety contract:
- ACTIVE: apply joint_angles, green status.
- DEGRADED: apply joint_angles, amber warning + show source_weights.
- PAUSED: freeze arm, ignore intent/joint_angles until ACTIVE/DEGRADED.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import mujoco
import numpy as np
from mujoco import viewer

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = ROOT / "sim" / "assets" / "scene.xml"
DEFAULT_COMMANDS = ROOT / "data" / "commands.json"
JOINT_NAMES = ["elbow", "wrist"]
ACTUATOR_NAMES = ["elbow_act", "wrist_act"]
DEG2RAD = math.pi / 180.0


class ReplayError(RuntimeError):
    pass


def load_commands(path: Path) -> list[dict]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, list) or not data:
        raise ReplayError(f"Command file must be a non-empty list: {path}")

    required = {"timestamp", "intent", "confidence", "source_weights", "safety_status", "joint_angles"}
    statuses = {"ACTIVE", "DEGRADED", "PAUSED"}
    intents = {"rest", "flex", "extend"}
    cleaned: list[dict] = []
    for i, cmd in enumerate(data):
        if not isinstance(cmd, dict):
            raise ReplayError(f"Command {i} is not an object")
        missing = required - set(cmd.keys())
        if missing:
            raise ReplayError(f"Command {i} missing keys: {sorted(missing)}")
        status = str(cmd["safety_status"]).upper()
        intent = str(cmd["intent"]).lower()
        if status not in statuses:
            raise ReplayError(f"Command {i} has invalid safety_status: {cmd['safety_status']!r}")
        if intent not in intents:
            raise ReplayError(f"Command {i} has invalid intent: {cmd['intent']!r}")
        angles = cmd["joint_angles"]
        if not isinstance(angles, dict) or "elbow" not in angles or "wrist" not in angles:
            raise ReplayError(f"Command {i} joint_angles must include elbow and wrist")
        weights = cmd["source_weights"]
        if not isinstance(weights, dict) or "eeg" not in weights or "emg" not in weights:
            raise ReplayError(f"Command {i} source_weights must include eeg and emg")
        cleaned.append(
            {
                "timestamp": float(cmd["timestamp"]),
                "intent": intent,
                "confidence": float(cmd["confidence"]),
                "source_weights": {"eeg": float(weights["eeg"]), "emg": float(weights["emg"])},
                "safety_status": status,
                "joint_angles": {"elbow": float(angles["elbow"]), "wrist": float(angles["wrist"])},
            }
        )
    for i in range(1, len(cleaned)):
        if cleaned[i]["timestamp"] < cleaned[i - 1]["timestamp"]:
            raise ReplayError(f"Command timestamps must be non-decreasing at index {i}")
    return cleaned


def resolve_actuators(model: mujoco.MjModel) -> np.ndarray:
    ids = []
    for name in ACTUATOR_NAMES:
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        if aid < 0:
            raise ReplayError(f"Actuator not found in model: {name}")
        ids.append(aid)
    return np.array(ids, dtype=int)


def cmd_to_ctrl(model: mujoco.MjModel, act_ids: np.ndarray, cmd: dict) -> np.ndarray:
    ctrl = np.zeros(len(act_ids), dtype=float)
    for i, aid in enumerate(act_ids):
        jname = JOINT_NAMES[i]
        deg = cmd["joint_angles"][jname]
        rad = deg * DEG2RAD
        lo, hi = model.actuator_ctrlrange[aid]
        ctrl[i] = float(np.clip(rad, lo, hi))
    return ctrl


def status_color(status: str) -> tuple[str, str]:
    if status == "ACTIVE":
        return "GREEN", "NORMAL"
    if status == "DEGRADED":
        return "AMBER", "DEGRADED - MUSCLE FATIGUE / ADAPTIVE SHIFT"
    return "RED", "PAUSED - SAFETY FREEZE"


def format_overlay(cmd: dict, idx: int, total: int, applied_ctrl_deg: dict[str, float], frozen: bool) -> list[tuple]:
    font = mujoco.mjtFontScale.mjFONTSCALE_150
    tl = mujoco.mjtGridPos.mjGRID_TOPLEFT
    tr = mujoco.mjtGridPos.mjGRID_TOPRIGHT
    br = mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT
    color_name, status_detail = status_color(cmd["safety_status"])
    eeg = cmd["source_weights"]["eeg"]
    emg = cmd["source_weights"]["emg"]
    freeze_note = "YES (holding last safe command)" if frozen else "NO"
    left = "\n".join(
        [
            "AXION-1  |  Task C MuJoCo Replay",
            f"Status: {cmd['safety_status']}  [{color_name}]",
            f"Detail: {status_detail}",
            f"Intent: {cmd['intent'].upper()}    Confidence: {cmd['confidence']:.2f}",
            f"EEG weight: {eeg:.2f}    EMG weight: {emg:.2f}",
        ]
    )
    right = "\n".join(
        [
            f"Command: {idx + 1}/{total}",
            f"Timestamp: {cmd['timestamp']:.1f}s",
            f"Cmd elbow/wrist (deg): {cmd['joint_angles']['elbow']:.1f} / {cmd['joint_angles']['wrist']:.1f}",
            f"Applied elbow/wrist (deg): {applied_ctrl_deg['elbow']:.1f} / {applied_ctrl_deg['wrist']:.1f}",
            f"Frozen: {freeze_note}",
        ]
    )
    rules = "\n".join(
        [
            "PAUSED => freeze arm, ignore intent/joint_angles",
            "DEGRADED => move, show warning + weights",
            "ACTIVE => normal movement",
        ]
    )
    return [(font, tl, left, ""), (font, tr, right, ""), (font, br, rules, "")]


def apply_command(model: mujoco.MjModel, data: mujoco.MjData, act_ids: np.ndarray, cmd: dict, last_safe_ctrl: np.ndarray | None):
    status = cmd["safety_status"]
    frozen = status == "PAUSED"
    if frozen:
        if last_safe_ctrl is None:
            target = np.zeros(len(act_ids), dtype=float)
        else:
            target = last_safe_ctrl.copy()
    else:
        target = cmd_to_ctrl(model, act_ids, cmd)
        last_safe_ctrl = target.copy()
    data.ctrl[act_ids] = target
    applied_deg = {
        "elbow": float(target[0] / DEG2RAD),
        "wrist": float(target[1] / DEG2RAD),
    }
    return last_safe_ctrl, applied_deg, frozen


def run_smoke(commands: list[dict], scene_path: Path, max_steps: int | None = None) -> int:
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)
    act_ids = resolve_actuators(model)
    last_safe = None
    steps = len(commands) if max_steps is None else min(len(commands), max_steps)
    for i in range(steps):
        cmd = commands[i]
        last_safe, applied, frozen = apply_command(model, data, act_ids, cmd, last_safe)
        if cmd["safety_status"] == "PAUSED":
            # frozen means applied should equal last_safe or zeros
            if last_safe is not None and not np.allclose(data.ctrl[act_ids], last_safe):
                raise ReplayError(f"PAUSED command {i} did not freeze")
        for _ in range(int(0.5 / model.opt.timestep)):
            mujoco.mj_step(model, data)
    return steps


def main() -> int:
    parser = argparse.ArgumentParser(description="AXION-1 MuJoCo command replay")
    parser.add_argument("--commands", type=Path, default=DEFAULT_COMMANDS)
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier")
    parser.add_argument("--smooth", type=float, default=0.0, help="Optional ctrl smoothing tau seconds (0=off, faithful). Try 0.08")
    parser.add_argument("--smoke", action="store_true", help="Run headless smoke test and exit")
    parser.add_argument("--smoke-steps", type=int, default=None)
    args = parser.parse_args()

    commands = load_commands(args.commands)
    if args.speed <= 0:
        raise ReplayError("--speed must be > 0")

    if args.smoke:
        n = run_smoke(commands, args.scene, args.smoke_steps)
        print(f"SMOKE_OK steps={n}")
        return 0

    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    act_ids = resolve_actuators(model)

    # Reset to home keyframe if present.
    home_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if home_id >= 0:
        mujoco.mj_resetDataKeyframe(model, data, home_id)
    mujoco.mj_forward(model, data)

    def reset_replay_state():
        nonlocal last_safe_ctrl, wall_start, cmd_idx, current_cmd, applied_deg, frozen, smooth_ctrl
        home_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
        if home_id >= 0:
            mujoco.mj_resetDataKeyframe(model, data, home_id)
        else:
            mujoco.mj_resetData(model, data)
        mujoco.mj_forward(model, data)
        last_safe_ctrl = np.array(data.ctrl[act_ids], dtype=float)
        wall_start = time.perf_counter()
        cmd_idx = 0
        current_cmd = commands[0]
        last_safe_ctrl, applied_deg, frozen = apply_command(model, data, act_ids, current_cmd, last_safe_ctrl)
        smooth_ctrl = np.array(data.ctrl[act_ids], dtype=float)

    last_safe_ctrl: np.ndarray | None = np.array(data.ctrl[act_ids], dtype=float)
    dt_cmd = 0.5 / args.speed
    wall_start = time.perf_counter()
    cmd_idx = 0
    current_cmd = commands[0]
    applied_deg = {"elbow": 0.0, "wrist": 0.0}
    frozen = False
    smooth_ctrl = np.array(data.ctrl[act_ids], dtype=float)
    reset_replay_state()

    restart_requested = False

    def on_key(keycode: int) -> None:
        nonlocal restart_requested
        if keycode in (ord("r"), ord("R")):
            restart_requested = True

    print(f"Loaded {len(commands)} commands from {args.commands}")
    print(f"Scene: {args.scene}")
    print("Safety: PAUSED freezes arm and ignores intent/joint_angles")
    print("Controls: R=restart replay, mouse wheel=zoom, left-drag=rotate, right-drag=pan")

    with viewer.launch_passive(model, data, show_left_ui=False, show_right_ui=True, key_callback=on_key) as handle:
        try:
            handle.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
            cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "main")
            if cam_id >= 0:
                handle.cam.fixedcamid = cam_id
        except Exception:
            pass

        while handle.is_running():
            if restart_requested:
                restart_requested = False
                reset_replay_state()
                print("Replay restarted.")

            now = time.perf_counter() - wall_start
            next_idx = min(int(now / dt_cmd), len(commands) - 1)
            if next_idx != cmd_idx:
                for i in range(cmd_idx + 1, next_idx + 1):
                    current_cmd = commands[i]
                    last_safe_ctrl, applied_deg, frozen = apply_command(
                        model, data, act_ids, current_cmd, last_safe_ctrl
                    )
                    print(
                        f"[{current_cmd['timestamp']:6.1f}s] {current_cmd['safety_status']:<8} "
                        f"intent={current_cmd['intent']:<6} conf={current_cmd['confidence']:.2f} "
                        f"eeg={current_cmd['source_weights']['eeg']:.2f} emg={current_cmd['source_weights']['emg']:.2f} "
                        f"applied=({applied_deg['elbow']:.1f},{applied_deg['wrist']:.1f}) frozen={frozen}"
                    )
                cmd_idx = next_idx
            else:
                # keep applying current command (important for PAUSED freeze/hold)
                last_safe_ctrl, applied_deg, frozen = apply_command(model, data, act_ids, current_cmd, last_safe_ctrl)

            step_start = time.perf_counter()
            # Optional smoothing for display only; target/PAUSED semantics unchanged.
            target_ctrl = np.array(data.ctrl[act_ids], dtype=float)
            if args.smooth > 0:
                alpha = 1.0 - math.exp(-model.opt.timestep / max(args.smooth, 1e-6))
                smooth_ctrl += alpha * (target_ctrl - smooth_ctrl)
                data.ctrl[act_ids] = smooth_ctrl
            else:
                smooth_ctrl[:] = target_ctrl
            mujoco.mj_step(model, data)
            display_deg = {"elbow": float(smooth_ctrl[0] / DEG2RAD), "wrist": float(smooth_ctrl[1] / DEG2RAD)}
            handle.set_texts(format_overlay(current_cmd, cmd_idx, len(commands), display_deg, frozen))
            handle.sync()

            # Real-time pacing for simulation steps.
            elapsed = time.perf_counter() - step_start
            remaining = model.opt.timestep - elapsed
            if remaining > 0:
                time.sleep(remaining)

            if cmd_idx == len(commands) - 1 and now >= (len(commands) - 1) * dt_cmd + 2.0:
                # hold final state briefly, then exit.
                print("Replay complete.")
                break

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
