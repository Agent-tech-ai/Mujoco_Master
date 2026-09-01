"""Command-line and interactive entry point for the local simulator."""

from __future__ import annotations

import argparse
import math
import time
from collections.abc import Sequence

import mujoco
import numpy as np

from .controller import JointPositionController, POSES, SimulationStabilityController
from .model import (
    EXPECTED_LIMITS_DEG,
    load_model,
    object_name,
    validate_model,
    validation_summary,
)


KEY_HELP = """Viewer keys:
  1  home pose
  2  crouch pose
  3  T-pose
  4  wave animation
  Space  pause/resume physics
  R  reset the selected pose
  H  print this help
"""


def _parse_joint_override(text: str) -> tuple[str, float]:
    if "=" not in text:
        raise argparse.ArgumentTypeError("Use JOINT=DEGREES, for example head_yaw_joint=15")
    name, raw_value = text.split("=", 1)
    name = name.strip()
    if not name:
        raise argparse.ArgumentTypeError("Joint name cannot be empty")
    try:
        value = float(raw_value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"Invalid angle: {raw_value!r}") from exc
    return name, value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Local MuJoCo simulator for the FF Master / AgiBot X2 Ultra"
    )
    parser.add_argument(
        "--free-base",
        action="store_true",
        help="simulate balance/contact instead of welding the pelvis to the world",
    )
    parser.add_argument(
        "--legacy-controller",
        action="store_true",
        help="use the pre-cleanup joint-only controller (diagnostic; free base falls)",
    )
    parser.add_argument(
        "--pose",
        choices=sorted(POSES),
        default="home",
        help="initial pose (default: home)",
    )
    parser.add_argument(
        "--set",
        dest="joint_overrides",
        action="append",
        type=_parse_joint_override,
        default=[],
        metavar="JOINT=DEGREES",
        help="override a target angle; repeat for multiple joints",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="run without opening the MuJoCo viewer",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        metavar="SECONDS",
        help="stop after simulated seconds (headless default: 2)",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="compile and validate the model, then exit",
    )
    parser.add_argument(
        "--list-joints",
        action="store_true",
        help="print controllable joints and MJCF-coordinate limits, then exit",
    )
    return parser


def _configure(
    model: mujoco.MjModel,
    pose: str,
    overrides: Sequence[tuple[str, float]],
    *,
    stability_cleanup: bool = False,
) -> tuple[mujoco.MjData, JointPositionController]:
    data = mujoco.MjData(model)
    controller_type = SimulationStabilityController if stability_cleanup else JointPositionController
    controller = controller_type(model)
    controller.set_pose(pose)
    if overrides:
        controller.set_targets_degrees(dict(overrides))
    controller.initialize_data(data)
    return data, controller


def _print_joints(model: mujoco.MjModel) -> None:
    print(f"{'joint':36} {'minimum':>10} {'maximum':>10}")
    print("-" * 58)
    for actuator_id in range(model.nu):
        joint_id = int(model.actuator_trnid[actuator_id, 0])
        name = object_name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        lower, upper = np.rad2deg(model.jnt_range[joint_id])
        print(f"{name:36} {lower:9.1f}° {upper:9.1f}°")
    print(f"{'head_pitch_joint (fixed link)':36} {0:9.1f}° {0:9.1f}°")


def run_headless(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    controller: JointPositionController,
    duration: float,
) -> None:
    if duration <= 0:
        raise ValueError("--duration must be positive")
    end_time = data.time + duration
    while data.time < end_time:
        controller.apply(data)
        mujoco.mj_step(model, data)
        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            raise RuntimeError(f"Non-finite state at t={data.time:.6f}s")
    print(
        f"HEADLESS OK: simulated {data.time:.3f}s, "
        f"max |qvel|={np.max(np.abs(data.qvel)):.4f} rad/s"
    )


def run_viewer(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    controller: JointPositionController,
    duration: float | None,
) -> None:
    import mujoco.viewer

    state = {"paused": False}

    def reset() -> None:
        mujoco.mj_resetData(model, data)
        controller.initialize_data(data)

    def on_key(keycode: int) -> None:
        pose_keys = {
            ord("1"): "home",
            ord("2"): "crouch",
            ord("3"): "tpose",
            ord("4"): "wave",
        }
        if keycode in pose_keys:
            controller.set_pose(pose_keys[keycode])
            print(f"pose -> {controller.pose_name}")
        elif keycode == ord(" "):
            state["paused"] = not state["paused"]
            print("paused" if state["paused"] else "running")
        elif keycode in (ord("R"), ord("r")):
            reset()
            print(f"reset -> {controller.pose_name}")
        elif keycode in (ord("H"), ord("h")):
            print(KEY_HELP)

    print(KEY_HELP)
    print("Close the viewer window to stop.")
    with mujoco.viewer.launch_passive(
        model, data, key_callback=on_key, show_left_ui=True, show_right_ui=True
    ) as viewer:
        viewer.cam.lookat[:] = (0.0, 0.0, 0.72)
        viewer.cam.distance = 2.25
        viewer.cam.azimuth = 145
        viewer.cam.elevation = -15
        while viewer.is_running():
            if duration is not None and data.time >= duration:
                break
            frame_start = time.perf_counter()
            if not state["paused"]:
                frame_end = data.time + 1.0 / 60.0
                while data.time < frame_end:
                    controller.apply(data)
                    mujoco.mj_step(model, data)
            viewer.sync()
            delay = 1.0 / 60.0 - (time.perf_counter() - frame_start)
            if delay > 0:
                time.sleep(delay)


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    model = load_model(free_base=args.free_base)
    errors = validate_model(model)
    if errors:
        print(validation_summary(model))
        return 2

    if args.check:
        print(validation_summary(model))
        return 0
    if args.list_joints:
        _print_joints(model)
        return 0

    overrides = dict(args.joint_overrides)
    invalid = sorted(set(overrides) - set(EXPECTED_LIMITS_DEG))
    if invalid:
        raise SystemExit(f"Unknown or fixed joint(s): {', '.join(invalid)}")

    stability_cleanup = args.free_base and not args.legacy_controller
    data, controller = _configure(
        model,
        args.pose,
        list(overrides.items()),
        stability_cleanup=stability_cleanup,
    )
    if stability_cleanup:
        print(
            "SIMULATION_STABILITY_CANDIDATE: using simulation-only pelvis-attitude "
            "feedback; values are not hardware-calibrated."
        )
    if args.headless:
        run_headless(model, data, controller, args.duration or 2.0)
    else:
        run_viewer(model, data, controller, args.duration)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
