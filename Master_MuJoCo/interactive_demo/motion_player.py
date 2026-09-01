"""Measured-trajectory loading and smooth action transitions."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import numpy as np
import pandas as pd


ARM_TOKENS = ("shoulder", "elbow", "wrist")


@dataclass(frozen=True)
class MotionSpec:
    name: str
    key: str
    path: Path
    duration_s: float


class MeasuredTrajectory:
    """Position-only adapter around the existing Phase 3A Reference loader."""

    def __init__(self, spec: MotionSpec, reference_factory):
        self.spec = spec
        # Load only position/velocity.  reported_effort is deliberately absent.
        frame = pd.read_csv(spec.path, usecols=["t", "joint_name", "position", "velocity"])
        frame = frame[
            frame["joint_name"].str.contains("|".join(ARM_TOKENS), regex=True)
            & frame["t"].between(0.0, spec.duration_s)
        ].copy()
        if frame.empty:
            raise RuntimeError(f"No arm samples in {spec.path}")
        self.reference = reference_factory(frame, "linear", 50.0)

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(sorted(self.reference.data))

    def sample(self, t: float) -> dict[str, float]:
        clamped = float(np.clip(t, 0.0, self.spec.duration_s))
        return {
            name: self.reference.at(name, clamped, "position")
            for name in self.reference.data
        }


def smootherstep(value: float) -> float:
    x = float(np.clip(value, 0.0, 1.0))
    return x * x * x * (x * (x * 6.0 - 15.0) + 10.0)


class MotionPlayer:
    """Blend into a measured action, play it, and blend back to standing."""

    BLEND_IN_S = 1.0
    BLEND_OUT_S = 2.0

    def __init__(self, trajectories: Mapping[str, MeasuredTrajectory]):
        self.trajectories = dict(trajectories)
        self.name: str | None = None
        self.phase = "IDLE"
        self.phase_time = 0.0
        self._from: dict[str, float] = {}
        self._last: dict[str, float] = {}
        self._standing: dict[str, float] = {}
        self.motion_time_s: float | None = None

    @property
    def active(self) -> bool:
        return self.phase != "IDLE"

    @property
    def current_name(self) -> str | None:
        return self.name

    def start(
        self,
        name: str,
        current: Mapping[str, float],
        standing: Mapping[str, float],
    ) -> None:
        if name not in self.trajectories:
            raise KeyError(name)
        trajectory = self.trajectories[name]
        self.name = name
        self.phase = "BLEND_IN"
        self.phase_time = 0.0
        self.motion_time_s = None
        self._standing = {joint: float(standing[joint]) for joint in trajectory.joint_names}
        self._from = {joint: float(current[joint]) for joint in trajectory.joint_names}
        self._last = self._from.copy()

    def stop(self) -> None:
        if self.active and self.phase != "BLEND_OUT":
            self.phase = "BLEND_OUT"
            self.phase_time = 0.0
            self.motion_time_s = None
            self._from = self._last.copy()

    def reset(self) -> None:
        self.name = None
        self.phase = "IDLE"
        self.phase_time = 0.0
        self.motion_time_s = None
        self._from.clear()
        self._last.clear()
        self._standing.clear()

    def step(self, dt: float) -> tuple[dict[str, float], bool]:
        if not self.active or self.name is None:
            return {}, True
        trajectory = self.trajectories[self.name]
        self.phase_time += dt
        complete = False
        if self.phase == "BLEND_IN":
            weight = smootherstep(self.phase_time / self.BLEND_IN_S)
            start = trajectory.sample(0.0)
            values = {
                name: (1.0 - weight) * self._from[name] + weight * start[name]
                for name in trajectory.joint_names
            }
            if self.phase_time >= self.BLEND_IN_S:
                self.phase = "PLAY"
                self.phase_time = 0.0
        elif self.phase == "PLAY":
            self.motion_time_s = min(self.phase_time, trajectory.spec.duration_s)
            values = trajectory.sample(self.motion_time_s)
            if self.phase_time >= trajectory.spec.duration_s:
                self.phase = "BLEND_OUT"
                self.phase_time = 0.0
                self.motion_time_s = None
                self._from = values.copy()
        else:
            weight = smootherstep(self.phase_time / self.BLEND_OUT_S)
            values = {
                name: (1.0 - weight) * self._from[name] + weight * self._standing[name]
                for name in trajectory.joint_names
            }
            if self.phase_time >= self.BLEND_OUT_S:
                values = self._standing.copy()
                complete = True
                self.reset()
        self._last = values.copy()
        return values, complete
