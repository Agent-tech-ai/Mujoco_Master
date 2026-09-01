"""Plan or, only after every external gate is confirmed, run one arm test.

Default and --dry-run paths use a saved read-only snapshot and do not import ROS,
create a node, create a publisher, call a service, or connect to the robot.

The --enable-motion path is deliberately difficult to enter: it requires a live
robot environment, a completely confirmed machine-readable operator gate, zero
pre-existing arm-command publishers, absence of the native MC ROS node, and two
interactive confirmations. This script never stops/starts MC or changes modes.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys
import time

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from calibration.phase2b2_common import (
    DEFAULT_SNAPSHOT,
    FIELD_LIMITS_DEG,
    adaptive_symmetric_amplitude,
    assessments,
    load_snapshot,
)


ARM_ORDER = list(FIELD_LIMITS_DEG)
DEFAULT_GATE = Path(__file__).with_name("phase2b2_operator_gate.json")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true", help="offline plan only (default)")
    mode.add_argument("--enable-motion", action="store_true", help="unlock guarded live executor")
    parser.add_argument("--joint", choices=ARM_ORDER, default="left_wrist_roll_joint")
    parser.add_argument("--requested-amplitude-deg", type=float, default=2.0)
    parser.add_argument("--reserve-deg", type=float, default=5.0)
    parser.add_argument("--minimum-useful-amplitude-deg", type=float, default=1.0)
    parser.add_argument("--snapshot", type=Path, default=DEFAULT_SNAPSHOT)
    parser.add_argument("--gate", type=Path, default=DEFAULT_GATE)
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


def selected_plan(args: argparse.Namespace, snapshot: dict) -> dict:
    by_name = {item.name: item for item in assessments(snapshot)}
    item = by_name[args.joint]
    amplitude, reason = adaptive_symmetric_amplitude(
        item,
        requested_deg=args.requested_amplitude_deg,
        reserve_deg=args.reserve_deg,
        minimum_useful_deg=args.minimum_useful_amplitude_deg,
    )
    current_positions = {
        joint["name"]: float(joint["position"]) for joint in snapshot["joints"]
    }
    return {
        "joint": args.joint,
        "current_deg": item.current_deg,
        "lower_deg": item.lower_deg,
        "upper_deg": item.upper_deg,
        "distance_to_lower_deg": item.lower_distance_deg,
        "distance_to_upper_deg": item.upper_distance_deg,
        "selected_amplitude_deg": amplitude,
        "selection_reason": reason,
        "plus_target_deg": None if amplitude is None else item.current_deg + amplitude,
        "minus_target_deg": None if amplitude is None else item.current_deg - amplitude,
        "full_arm_current_command_rad": current_positions,
        "command_graph": snapshot["command_graph"],
        "control_state": snapshot["control_state"],
        "snapshot_time": snapshot["capture_host_time"],
        "snapshot_is_fresh_for_motion": False,
    }


def dry_run(args: argparse.Namespace) -> int:
    snapshot = load_snapshot(args.snapshot)
    plan = selected_plan(args, snapshot)
    print("PHASE2B2_MODE=DRY_RUN")
    print("ROS_IMPORTED=0")
    print("ROS_NODE_CREATED=0")
    print("PUBLISHER_CREATED=0")
    print("COMMAND_SENT=0")
    print(json.dumps(plan, indent=2, sort_keys=True))
    blockers = []
    if plan["selected_amplitude_deg"] is None:
        blockers.append(plan["selection_reason"])
    if plan["command_graph"]["publisher_count"] != 0:
        blockers.append(
            f"saved evidence has {plan['command_graph']['publisher_count']} existing "
            "arm-command publisher(s)"
        )
    if not plan["snapshot_is_fresh_for_motion"]:
        blockers.append("saved snapshot is stale by definition and cannot authorize motion")
    print("GO_NO_GO=NO-GO" if blockers else "GO_NO_GO=DRY_RUN_ONLY")
    for blocker in blockers:
        print(f"BLOCKER={blocker}")
    print("WOULD_SEND_PHASES=current,+delta,return,-delta,return")
    print("NOTE=All 14 arm entries would be held; no command object was created.")
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        lines = [
            "# Phase 2B-2 dry-run report",
            "",
            "Status: **NO-GO / DRY-RUN ONLY**",
            "",
            f"- Joint: `{plan['joint']}`",
            f"- Saved snapshot: `{plan['snapshot_time']}`",
            f"- Current: {plan['current_deg']:.6f}°",
            f"- Field limits: {plan['lower_deg']:.3f}° to {plan['upper_deg']:.3f}°",
            f"- Selected symmetric amplitude: {plan['selected_amplitude_deg'] if plan['selected_amplitude_deg'] is not None else 'SKIP'}°",
            f"- Hypothetical + target: {plan['plus_target_deg']}°",
            f"- Hypothetical - target: {plan['minus_target_deg']}°",
            f"- Saved command publisher count: {plan['command_graph']['publisher_count']} ({plan['command_graph']['publishers']})",
            f"- Saved MC action: `{plan['control_state']['mc_action_desc']}`",
            f"- Saved input source: `{plan['control_state']['input_source_name']}` (empty does not prove ownership)",
            "",
            "Dry-run invariants: ROS was not imported; no node or publisher was created; no service was called; no command was sent.",
            "",
            "Blockers:",
            "",
        ] + [f"- {blocker}" for blocker in blockers]
        args.output.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return 0


def validate_gate(path: Path) -> tuple[dict, list[str]]:
    gate = json.loads(path.read_text(encoding="utf-8"))
    blockers: list[str] = []
    if gate.get("overall_status") != "GO":
        blockers.append("overall_status must be GO")
    for name, value in gate.get("confirmations", {}).items():
        if value is not True:
            blockers.append(f"confirmation {name}=UNKNOWN/false")
    numeric = (
        "max_joint_velocity_rad_s",
        "max_joint_acceleration_rad_s2",
        "max_abs_effort_n_m",
        "max_effort_change_n_m",
        "max_state_age_s",
        "max_imu_orientation_change_deg",
    )
    for name in numeric:
        value = gate.get("limits", {}).get(name)
        if not isinstance(value, (int, float)) or value <= 0:
            blockers.append(f"limit {name} must be a positive approved number")
    ownership = gate.get("ownership", {})
    if ownership.get("method") != "DIRECT_HAL_MC_STOPPED_VENDOR_APPROVED":
        blockers.append("ownership.method is not vendor-approved direct HAL ownership")
    if ownership.get("expected_command_publishers_before_test") != 0:
        blockers.append("expected pre-test arm-command publisher count must be zero")
    if ownership.get("native_mc_stop_separately_authorized") is not True:
        blockers.append("separate native-MC stop authorization is absent")
    if ownership.get("native_mc_stop_performed_by_this_script") is not False:
        blockers.append("this script is forbidden from stopping MC")
    if ownership.get("release_or_restore_procedure") in (None, "", "UNKNOWN"):
        blockers.append("control restoration procedure is UNKNOWN")
    abort = gate.get("abort", {})
    if abort.get("approved_strategy") != "HOLD_LATEST_MEASURED_THEN_STOP_APPROVED":
        blockers.append("approved abort strategy is UNKNOWN")
    if not isinstance(abort.get("hold_duration_s"), (int, float)) or abort["hold_duration_s"] <= 0:
        blockers.append("abort hold duration is UNKNOWN")
    if abort.get("loss_of_command_behavior") in (None, "", "UNKNOWN"):
        blockers.append("loss-of-command behavior is UNKNOWN")
    params = gate.get("arm_command_parameters", {})
    if params.get("source") in (None, "", "UNKNOWN"):
        blockers.append("arm command stiffness/damping source is UNKNOWN")
    for field in ("stiffness_by_joint", "damping_by_joint"):
        values = params.get(field, {})
        missing = [name for name in ARM_ORDER if not isinstance(values.get(name), (int, float))]
        if missing:
            blockers.append(f"{field} lacks approved values for {missing}")
    return gate, blockers


def motion_enabled(args: argparse.Namespace) -> int:
    gate, blockers = validate_gate(args.gate)
    if blockers:
        print("GO_NO_GO=NO-GO", file=sys.stderr)
        for blocker in blockers:
            print(f"BLOCKER={blocker}", file=sys.stderr)
        print("ROS_IMPORTED=0\nPUBLISHER_CREATED=0\nCOMMAND_SENT=0", file=sys.stderr)
        return 3

    # Never treat saved state as execution evidence. ROS is imported only after
    # the complete external gate passes.
    confirmation = input(
        f"Type exactly 'ENABLE-MOTION {args.joint}' to begin LIVE preflight: "
    ).strip()
    if confirmation != f"ENABLE-MOTION {args.joint}":
        print("Operator confirmation rejected; no ROS import or command.", file=sys.stderr)
        return 4

    return run_live_motion(args, gate)


def run_live_motion(args: argparse.Namespace, gate: dict) -> int:
    """Live executor. It does not stop MC or change control/system modes."""

    # Imports are intentionally inside the doubly gated path.
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
    from aimdk_msgs.msg import JointCommandArray, JointCommand, JointStateArray
    from sensor_msgs.msg import Imu

    class LiveGuard(Node):
        def __init__(self) -> None:
            super().__init__("phase2b2_single_joint_guard")
            self.arm_state = None
            self.arm_state_received = 0.0
            self.chest_imu = None
            self.torso_imu = None
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(JointStateArray, "/aima/hal/joint/arm/state", self._arm, qos)
            self.create_subscription(Imu, "/aima/hal/imu/chest/state", self._chest, qos)
            self.create_subscription(Imu, "/aima/hal/imu/torso/state", self._torso, qos)
            self.publisher = None
            self.qos = qos

        def _arm(self, message) -> None:
            self.arm_state = message
            self.arm_state_received = time.monotonic()

        def _chest(self, message) -> None:
            self.chest_imu = message

        def _torso(self, message) -> None:
            self.torso_imu = message

    rclpy.init()
    node = LiveGuard()
    try:
        deadline = time.monotonic() + 5.0
        while node.arm_state is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.arm_state is None:
            print("NO-GO: no live arm state; no publisher created", file=sys.stderr)
            return 5
        names = [joint.name for joint in node.arm_state.joints]
        if names != ARM_ORDER:
            print(f"NO-GO: live arm order mismatch: {names}", file=sys.stderr)
            return 6
        publishers = node.get_publishers_info_by_topic("/aima/hal/joint/arm/command")
        if len(publishers) != 0:
            print(f"NO-GO: {len(publishers)} existing command publisher(s)", file=sys.stderr)
            return 7
        node_names = set(node.get_node_names())
        if any("mc_ros2_node" in name for name in node_names):
            print("NO-GO: native MC ROS node is still present", file=sys.stderr)
            return 8

        live_snapshot = {
            "capture_host_time": "LIVE",
            "command_graph": {"publisher_count": 0, "publishers": []},
            "control_state": {"mc_action_desc": "UNAVAILABLE_MC_STOPPED", "input_source_name": "UNAVAILABLE_MC_STOPPED"},
            "joints": [
                {
                    "name": joint.name,
                    "position": float(joint.position),
                    "velocity": float(joint.velocity),
                    "effort": float(joint.effort),
                }
                for joint in node.arm_state.joints
            ],
        }
        plan = selected_plan(args, live_snapshot)
        if plan["selected_amplitude_deg"] is None:
            print(f"NO-GO: {plan['selection_reason']}", file=sys.stderr)
            return 9
        print(json.dumps(plan, indent=2, sort_keys=True))
        final = input(
            f"Type exactly 'SEND-ONE-ROUND {args.joint} {plan['selected_amplitude_deg']:.6f}' "
            "to create the publisher: "
        ).strip()
        expected = f"SEND-ONE-ROUND {args.joint} {plan['selected_amplitude_deg']:.6f}"
        if final != expected:
            print("Final confirmation rejected; publisher was never created.", file=sys.stderr)
            return 10

        # Publisher construction occurs only after all gates and both confirmations.
        node.publisher = node.create_publisher(
            JointCommandArray, "/aima/hal/joint/arm/command", node.qos
        )
        return execute_one_round(node, args, gate, plan, JointCommandArray, JointCommand, rclpy)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _smoothstep5(value: float) -> tuple[float, float]:
    u = min(1.0, max(0.0, value))
    return 10*u**3 - 15*u**4 + 6*u**5, 30*u**2 - 60*u**3 + 30*u**4


def execute_one_round(node, args, gate, plan, array_type, command_type, rclpy_module) -> int:
    """Publish one smooth symmetric round with continuous state guards."""

    limits = gate["limits"]
    delta = math.radians(plan["selected_amplitude_deg"])
    vmax = float(limits["max_joint_velocity_rad_s"])
    amax = float(limits["max_joint_acceleration_rad_s2"])
    ramp = max(1.0, 1.875 * delta / vmax, math.sqrt(5.7736 * delta / amax))
    hold = 1.0
    segments = [
        ("pre", 0.0, 0.0, 2.0),
        ("plus", 0.0, 1.0, ramp),
        ("plus_hold", 1.0, 1.0, hold),
        ("return_plus", 1.0, 0.0, ramp),
        ("center_hold", 0.0, 0.0, hold),
        ("minus", 0.0, -1.0, ramp),
        ("minus_hold", -1.0, -1.0, hold),
        ("return_minus", -1.0, 0.0, ramp),
        ("post", 0.0, 0.0, 2.0),
    ]
    initial = [float(joint.position) for joint in node.arm_state.joints]
    initial_effort = [float(joint.effort) for joint in node.arm_state.joints]
    test_index = ARM_ORDER.index(args.joint)
    output = args.output or Path(__file__).with_name(f"{args.joint}_live.csv")
    output.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "timestamp", "joint_name", "command_position", "measured_position",
        "measured_velocity", "measured_torque", "imu_quaternion", "imu_gyro",
        "imu_accel", "coil_temp", "motor_temp", "motor_vol", "phase",
    ]
    sequence = 0
    aborted = None
    with output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        start = time.monotonic()
        for phase, begin, end, duration in segments:
            phase_start = time.monotonic()
            while True:
                now = time.monotonic()
                elapsed = now - phase_start
                if elapsed >= duration:
                    break
                rclpy_module.spin_once(node, timeout_sec=0.0)
                if node.arm_state is None or now - node.arm_state_received > limits["max_state_age_s"]:
                    aborted = "state communication timeout"
                    break
                if len(node.get_publishers_info_by_topic("/aima/hal/joint/arm/command")) != 1:
                    aborted = "command publisher count changed"
                    break
                measured = node.arm_state.joints[test_index]
                if abs(float(measured.velocity)) > limits["max_joint_velocity_rad_s"]:
                    aborted = "velocity threshold exceeded"
                    break
                if abs(float(measured.effort)) > limits["max_abs_effort_n_m"]:
                    aborted = "absolute effort threshold exceeded"
                    break
                if abs(float(measured.effort) - initial_effort[test_index]) > limits["max_effort_change_n_m"]:
                    aborted = "effort-change threshold exceeded"
                    break
                progress, derivative = _smoothstep5(elapsed / duration) if begin != end else (1.0, 0.0)
                scale = begin + (end - begin) * progress
                scale_velocity = (end - begin) * derivative / duration
                target = initial.copy()
                target[test_index] += scale * delta
                velocities = [0.0] * len(ARM_ORDER)
                velocities[test_index] = scale_velocity * delta
                message = array_type()
                message.header.stamp = node.get_clock().now().to_msg()
                message.header.sequence = sequence
                sequence += 1
                for index, name in enumerate(ARM_ORDER):
                    command = command_type()
                    command.name = name
                    command.position = target[index]
                    command.velocity = velocities[index]
                    command.effort = 0.0
                    command.stiffness = float(gate["arm_command_parameters"]["stiffness_by_joint"][name])
                    command.damping = float(gate["arm_command_parameters"]["damping_by_joint"][name])
                    message.joints.append(command)
                node.publisher.publish(message)
                imu = node.chest_imu or node.torso_imu
                quaternion = [] if imu is None else [imu.orientation.w, imu.orientation.x, imu.orientation.y, imu.orientation.z]
                gyro = [] if imu is None else [imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z]
                accel = [] if imu is None else [imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z]
                writer.writerow({
                    "timestamp": f"{now-start:.9f}",
                    "joint_name": args.joint,
                    "command_position": f"{target[test_index]:.12g}",
                    "measured_position": f"{float(measured.position):.12g}",
                    "measured_velocity": f"{float(measured.velocity):.12g}",
                    "measured_torque": f"{float(measured.effort):.12g}",
                    "imu_quaternion": json.dumps(quaternion, separators=(",", ":")),
                    "imu_gyro": json.dumps(gyro, separators=(",", ":")),
                    "imu_accel": json.dumps(accel, separators=(",", ":")),
                    "coil_temp": getattr(measured, "coil_temp", ""),
                    "motor_temp": getattr(measured, "motor_temp", ""),
                    "motor_vol": getattr(measured, "motor_vol", ""),
                    "phase": phase,
                })
                time.sleep(0.01)
            if aborted:
                break
        if aborted:
            # Approved abort behavior: hold the latest measured full-arm state briefly.
            hold_until = time.monotonic() + float(gate["abort"]["hold_duration_s"])
            while time.monotonic() < hold_until and node.arm_state is not None:
                message = array_type()
                message.header.stamp = node.get_clock().now().to_msg()
                for state, name in zip(node.arm_state.joints, ARM_ORDER):
                    command = command_type()
                    command.name = name
                    command.position = float(state.position)
                    command.velocity = 0.0
                    command.effort = 0.0
                    command.stiffness = float(gate["arm_command_parameters"]["stiffness_by_joint"][name])
                    command.damping = float(gate["arm_command_parameters"]["damping_by_joint"][name])
                    message.joints.append(command)
                node.publisher.publish(message)
                rclpy_module.spin_once(node, timeout_sec=0.0)
                time.sleep(0.01)
            print(f"ABORTED={aborted}", file=sys.stderr)
            return 11
    print(f"COMPLETED_ONE_ROUND={output.resolve()}")
    return 0


def main() -> int:
    args = parse_args()
    if args.requested_amplitude_deg <= 0 or args.reserve_deg <= 0 or args.minimum_useful_amplitude_deg <= 0:
        print("ERROR: amplitude/reserve inputs must be positive", file=sys.stderr)
        return 2
    if args.enable_motion:
        return motion_enabled(args)
    return dry_run(args)


if __name__ == "__main__":
    raise SystemExit(main())

