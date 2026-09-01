"""Simulation-only interactive X2 demo layer.

This package never connects to a robot and never modifies the frozen MJCF or
hardware calibration evidence.
"""

from .demo_controller import DemoState, InteractiveDemo

__all__ = ["DemoState", "InteractiveDemo"]
