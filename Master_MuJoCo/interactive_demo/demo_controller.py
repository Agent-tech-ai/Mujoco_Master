"""Frozen-controller interactive simulation state machine."""

from __future__ import annotations

from dataclasses import dataclass, fields
from enum import Enum
import csv
import json
from pathlib import Path
import sys
from typing import Mapping

import mujoco
import numpy as np

from master_sim.model import load_model, validate_model
from .motion_player import MeasuredTrajectory, MotionPlayer, MotionSpec
from .safety_monitor import SafetyMonitor, SafetySnapshot


PROJECT = Path(__file__).resolve().parents[1]
P3AY_DIR = PROJECT / "calibration" / "phase3ay_motion_conditioned_balance"
if str(P3AY_DIR) not in sys.path:
    sys.path.insert(0, str(P3AY_DIR))
import phase3ay_core as AY  # noqa: E402


ACTION_SPECS = {
    "heart": MotionSpec(
        "heart", "H",
        PROJECT / "calibration" / "phase2e_replay" / "phase2e_heart_measured_reference.csv",
        5.659416987,
    ),
    "wave": MotionSpec(
        "wave", "V",
        PROJECT / "calibration" / "phase3av_validation" / "phase3av_measured_reference.csv",
        4.349152726,
    ),
    "clap": MotionSpec(
        "clap", "C",
        PROJECT / "calibration" / "phase3bv_physical_direction_validation" / "phase3bv_measured_reference.csv",
        5.443540770,
    ),
}
CANDIDATE_PATH = P3AY_DIR / "simulation_motion_conditioned_balance_candidate.json"
STANDING_SOURCE_T = -5.0
LOCOMOTION_AVAILABLE = False
SPEED_NAMES = {1: "SIMULATION_ONLY_LOW", 2: "SIMULATION_ONLY_MEDIUM", 3: "SIMULATION_ONLY_HIGH"}


class DemoState(str, Enum):
    STANDING = "STANDING"
    LOCOMOTION = "LOCOMOTION"
    ACTION_PLAYBACK = "ACTION_PLAYBACK"
    STOPPING = "STOPPING"
    RESETTING = "RESETTING"
    SAFETY_HOLD = "SAFETY_HOLD"


@dataclass
class DemoMetrics:
    steps: int = 0
    maximum_abs_roll_deg: float = 0.0
    maximum_abs_pitch_deg: float = 0.0
    minimum_pelvis_height_m: float = float("inf")
    minimum_joint_margin_rad: float = float("inf")
    maximum_saturation_fraction: float = 0.0
    maximum_saturation_duration_s: float = 0.0
    unexpected_self_contact_samples: int = 0
    expected_clap_contact_samples: int = 0
    nonfoot_ground_contact_samples: int = 0
    maximum_contact_penetration_m: float = 0.0
    maximum_left_foot_slip_m: float = 0.0
    maximum_right_foot_slip_m: float = 0.0
    safety_hold_entered: bool = False

    def add(self, snapshot: SafetySnapshot) -> None:
        self.steps += 1
        self.maximum_abs_roll_deg = max(self.maximum_abs_roll_deg, abs(snapshot.base_roll_deg))
        self.maximum_abs_pitch_deg = max(self.maximum_abs_pitch_deg, abs(snapshot.base_pitch_deg))
        self.minimum_pelvis_height_m = min(self.minimum_pelvis_height_m, snapshot.pelvis_height_m)
        self.minimum_joint_margin_rad = min(self.minimum_joint_margin_rad, snapshot.minimum_joint_margin_rad)
        self.maximum_saturation_fraction = max(self.maximum_saturation_fraction, snapshot.maximum_saturation_fraction)
        self.maximum_saturation_duration_s = max(
            self.maximum_saturation_duration_s, snapshot.maximum_saturation_duration_s
        )
        self.unexpected_self_contact_samples += int(snapshot.unexpected_self_contacts > 0)
        self.expected_clap_contact_samples += int(snapshot.expected_clap_contacts > 0)
        self.nonfoot_ground_contact_samples += int(snapshot.nonfoot_ground_contacts > 0)
        self.maximum_contact_penetration_m = max(
            self.maximum_contact_penetration_m, snapshot.maximum_contact_penetration_m
        )
        self.maximum_left_foot_slip_m = max(self.maximum_left_foot_slip_m, snapshot.left_foot_slip_m)
        self.maximum_right_foot_slip_m = max(self.maximum_right_foot_slip_m, snapshot.right_foot_slip_m)

    def as_dict(self) -> dict[str, object]:
        result = dict(self.__dict__)
        if result["minimum_pelvis_height_m"] == float("inf"):
            result["minimum_pelvis_height_m"] = None
        if result["minimum_joint_margin_rad"] == float("inf"):
            result["minimum_joint_margin_rad"] = None
        return result


def _load_design() -> AY.AYDesign:
    payload = json.loads(CANDIDATE_PATH.read_text(encoding="utf-8"))
    allowed = {item.name for item in fields(AY.AYDesign)}
    values = {name: value for name, value in payload["design"].items() if name in allowed}
    return AY.AYDesign(**values)


def _reference_pose(path: Path, target_t: float) -> dict[str, float]:
    closest: dict[str, tuple[float, float]] = {}
    with path.open("r", newline="", encoding="utf-8-sig") as stream:
        for row in csv.DictReader(stream):
            distance = abs(float(row["t"]) - target_t)
            name = row["joint_name"]
            if name not in closest or distance < closest[name][0]:
                closest[name] = (distance, float(row["position"]))
    return {name: value for name, (_distance, value) in closest.items()}


class InteractiveDemo:
    """Deterministic 1 kHz demo controller; robot/network access is absent."""

    def __init__(self, *, quiet: bool = False):
        self.quiet = quiet
        self.design = _load_design()
        self.model = load_model(free_base=True)
        errors = validate_model(self.model)
        if errors:
            raise RuntimeError("Model validation failed: " + "; ".join(errors))
        if abs(float(self.model.opt.timestep) - 0.001) > 1e-12:
            raise RuntimeError(f"Frozen scene timestep is {self.model.opt.timestep}; expected 0.001 s")
        self.data = mujoco.MjData(self.model)
        self.trajectories = {
            name: MeasuredTrajectory(spec, AY.AX.P3AR.P3A.Reference)
            for name, spec in ACTION_SPECS.items()
        }
        self.player = MotionPlayer(self.trajectories)
        self.speed_level = 2
        self.state = DemoState.RESETTING
        self.last_safety: SafetySnapshot | None = None
        self.last_safety_reason: tuple[str, ...] = ()
        self.metrics = DemoMetrics()
        self.state_history: list[str] = []
        self._locomotion_notice_printed: set[str] = set()
        self._build_controller_and_pose()
        self._set_state(DemoState.STANDING, "initialized")

    @property
    def dt(self) -> float:
        return float(self.model.opt.timestep)

    @property
    def action_name(self) -> str | None:
        return self.player.current_name

    def _say(self, message: str) -> None:
        if not self.quiet:
            print(message, flush=True)

    def _set_state(self, state: DemoState, reason: str = "") -> None:
        if state == self.state and self.state_history:
            return
        self.state = state
        self.state_history.append(state.value)
        suffix = f" ({reason})" if reason else ""
        self._say(f"[STATE] {state.value}{suffix}")

    def _build_controller_and_pose(self) -> None:
        self.controller = AY.MotionConditionedBalanceController(self.model, self.design)
        raw = _reference_pose(ACTION_SPECS["heart"].path, STANDING_SOURCE_T)
        offsets = AY.AX.standing_offsets(self.design)
        self.standing_reference: dict[str, float] = {}
        self.standing_targets: dict[str, float] = {}
        for name, joint in self.controller.by_name.items():
            reference = float(raw.get(name, 0.0))
            offset = float(offsets.get(name, 0.0))
            target = float(np.clip(reference + offset, joint.lower, joint.upper))
            self.standing_reference[name] = reference
            self.standing_targets[name] = target
            self.controller.reference_target[joint.qpos_adr] = reference
            self.controller.standing_offset[joint.qpos_adr] = offset
            self.controller.target[joint.qpos_adr] = target
            self.data.qpos[joint.qpos_adr] = reference
            self.data.qvel[joint.dof_adr] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.data.qpos[2] -= AY.AX.P3AR.P3A.foot_surface_minimum(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)
        self.controller.set_initial_foot_positions(self.data)
        self.safety = SafetyMonitor(self.model, self.controller)

    def reset_metrics(self) -> None:
        self.metrics = DemoMetrics()
        self.safety.reset()
        self.state_history = [self.state.value]

    def manual_reset(self) -> None:
        """Explicit demo reset; qpos is written only inside this reset path."""
        self._set_state(DemoState.RESETTING, "manual demo reset")
        self.player.reset()
        mujoco.mj_resetData(self.model, self.data)
        self._build_controller_and_pose()
        self.last_safety = None
        self.last_safety_reason = ()
        self._set_state(DemoState.STANDING, "reset complete")

    def current_positions(self) -> dict[str, float]:
        return {
            name: float(self.data.qpos[joint.qpos_adr])
            for name, joint in self.controller.by_name.items()
        }

    def start_action(self, name: str) -> bool:
        name = name.lower()
        if name not in self.trajectories:
            raise KeyError(name)
        if self.state == DemoState.SAFETY_HOLD:
            self._say(f"ACTION_REJECTED safety hold active: {name.upper()}")
            return False
        if self.player.active or self.state in (DemoState.ACTION_PLAYBACK, DemoState.STOPPING):
            active = self.player.current_name.upper() if self.player.current_name else self.state.value
            self._say(f"[ACTION] Busy: {active}; ignored {name.upper()}")
            return False
        self.player.start(name, self.current_positions(), self.standing_targets)
        self._say(f"[ACTION] {name.upper()}")
        self._set_state(DemoState.ACTION_PLAYBACK, name.upper())
        return True

    def controlled_stop(self) -> None:
        if self.state == DemoState.SAFETY_HOLD:
            self._say("SAFETY_HOLD remains active; press R for explicit demo reset")
            return
        if self.player.active:
            self._say("[COMMAND] STOP")
            self.player.stop()
            self._set_state(DemoState.STOPPING, "smooth return to standing")
        else:
            self._set_state(DemoState.STANDING, "hold")

    def request_locomotion(self, key: str) -> bool:
        if key not in self._locomotion_notice_printed:
            self._locomotion_notice_printed.add(key)
            self._say(f"[COMMAND] LOCOMOTION_NOT_AVAILABLE key={key}; frozen Phase 3A-X/Y integration absent")
        return False

    def set_speed_level(self, level: int) -> None:
        if level not in (1, 2, 3):
            raise ValueError(level)
        self.speed_level = level
        self._say(f"[SPEED] {SPEED_NAMES[level]} (NOT HARDWARE CALIBRATED; measured actions remain 1.0x)")

    def _requested_targets(self) -> tuple[dict[str, float], bool]:
        overrides: dict[str, float] = {}
        complete = False
        if self.player.active:
            overrides, complete = self.player.step(self.dt)
        return overrides, complete

    def step(self) -> SafetySnapshot:
        completing_action = self.player.current_name
        overrides, complete = self._requested_targets()
        if complete and self.state in (DemoState.ACTION_PLAYBACK, DemoState.STOPPING):
            if completing_action:
                self._say(f"[ACTION] {completing_action.upper()} COMPLETE")
            self._set_state(DemoState.STANDING, "action return complete")
        for name, joint in self.controller.by_name.items():
            if name in overrides:
                reference = float(overrides[name])
                offset = 0.0
                requested = reference
            else:
                reference = self.standing_reference[name]
                offset = self.standing_targets[name] - reference
                requested = self.standing_targets[name]
            self.controller.reference_target[joint.qpos_adr] = reference
            self.controller.standing_offset[joint.qpos_adr] = offset
            self.controller.update_reference_target(joint, requested, self.data, self.dt, "arm_only")
        self.controller.apply(self.data)
        mujoco.mj_step(self.model, self.data)
        snapshot = self.safety.evaluate(
            self.data,
            self.dt,
            self.player.current_name,
            self.player.motion_time_s,
        )
        self.last_safety = snapshot
        self.metrics.add(snapshot)
        if not snapshot.safe and self.state != DemoState.SAFETY_HOLD:
            self.last_safety_reason = snapshot.reasons
            self.metrics.safety_hold_entered = True
            if self.player.active:
                self.player.stop()
            self._say(f"[SAFETY] HOLD reason={','.join(snapshot.reasons)}")
            self._set_state(DemoState.SAFETY_HOLD, ",".join(snapshot.reasons))
        return snapshot

    def status_line(self) -> str:
        safety = self.last_safety
        if safety is None:
            detail = "safety=pending"
        else:
            detail = (
                f"roll={safety.base_roll_deg:+.2f}deg pitch={safety.base_pitch_deg:+.2f}deg "
                f"margin={safety.minimum_joint_margin_rad:.3f}rad sat={safety.maximum_saturation_fraction:.3f}"
            )
        action = self.action_name.upper() if self.action_name else "NONE"
        return f"state={self.state.value} action={action} speed={SPEED_NAMES[self.speed_level]} {detail}"

    def stable_standing(self) -> bool:
        if self.last_safety is None:
            return False
        return bool(
            self.state == DemoState.STANDING
            and self.last_safety.safe
            and abs(self.last_safety.base_roll_deg) < 5.0
            and abs(self.last_safety.base_pitch_deg) < 5.0
            and self.last_safety.pelvis_height_m >= SafetyMonitor.FALL_PELVIS_HEIGHT_M
        )

    def run_for(self, seconds: float) -> None:
        for _ in range(int(round(seconds / self.dt))):
            self.step()

    def run_until_idle(self, maximum_seconds: float) -> bool:
        steps = int(round(maximum_seconds / self.dt))
        for _ in range(steps):
            self.step()
            if self.state == DemoState.STANDING and not self.player.active:
                return True
            if self.state == DemoState.SAFETY_HOLD:
                return False
        return False
