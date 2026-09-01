#!/usr/bin/env python3
"""Launch the simulation-only Master X2 interactive MuJoCo demo."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
import time

import mujoco.viewer

from interactive_demo.demo_controller import ACTION_SPECS, LOCOMOTION_AVAILABLE, DemoState, InteractiveDemo
from interactive_demo.keyboard_map import ViewerKeyboard


ROOT = Path(__file__).resolve().parent
DEFAULT_RESULTS = ROOT / "interactive_demo_validation_results.json"


@dataclass
class ValidationResult:
    number: int
    name: str
    status: str
    note: str
    metrics: dict[str, object]


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _case_result(number: int, name: str, demo: InteractiveDemo, extra_ok: bool = True, note: str = "") -> ValidationResult:
    metrics = demo.metrics.as_dict()
    passed = bool(
        extra_ok
        and not demo.metrics.safety_hold_entered
        and demo.metrics.unexpected_self_contact_samples == 0
        and demo.metrics.nonfoot_ground_contact_samples == 0
        and demo.metrics.minimum_joint_margin_rad >= 0.0
        and demo.metrics.maximum_saturation_duration_s <= 0.20 + 1e-12
        and demo.stable_standing()
    )
    return ValidationResult(number, name, "PASS" if passed else "FAIL", note, metrics)


def _fresh(demo: InteractiveDemo) -> None:
    demo.manual_reset()
    demo.reset_metrics()


def run_self_test(output: Path) -> int:
    demo = InteractiveDemo(quiet=True)
    results: list[ValidationResult] = []

    _fresh(demo)
    demo.run_for(10.0)
    results.append(_case_result(1, "10 s free-base standing", demo))

    for number, name in ((2, "heart"), (3, "wave"), (4, "clap")):
        _fresh(demo)
        demo.run_for(2.0)
        started = demo.start_action(name)
        completed = demo.run_until_idle(ACTION_SPECS[name].duration_s + 5.0)
        demo.run_for(2.0)
        note = "measured trajectory -> smooth standing return"
        if name == "clap":
            note += "; expected wrist contact exception scoped to frozen closure windows"
        results.append(_case_result(number, f"{name.title()} playback and return", demo, started and completed, note))

    _fresh(demo)
    demo.run_for(2.0)
    started = demo.start_action("heart")
    demo.run_for(1.0 + ACTION_SPECS["heart"].duration_s / 2.0)
    demo.controlled_stop()
    completed = demo.run_until_idle(4.0)
    demo.run_for(2.0)
    results.append(_case_result(5, "Heart SPACE interrupt", demo, started and completed, "controlled smooth return"))

    _fresh(demo)
    demo.run_for(2.0)
    started = demo.start_action("wave")
    demo.run_for(2.0)
    demo.manual_reset()
    demo.run_for(10.0)
    results.append(_case_result(6, "Wave manual R reset", demo, started, "explicit mj_resetData demo reset only"))

    _fresh(demo)
    demo.run_for(2.0)
    sequence_ok = True
    for name in ("heart", "wave", "clap"):
        sequence_ok &= demo.start_action(name)
        sequence_ok &= demo.run_until_idle(ACTION_SPECS[name].duration_s + 5.0)
        demo.run_for(1.0)
    demo.run_for(2.0)
    results.append(_case_result(7, "Heart -> Wave -> Clap sequence", demo, sequence_ok, "no teleport between actions"))

    _fresh(demo)
    demo.run_for(10.0)
    monitor_ok = demo.safety.evaluations == int(round(10.0 / demo.dt))
    results.append(_case_result(8, "Safety monitor continuous operation", demo, monitor_ok, f"evaluations={demo.safety.evaluations}"))

    for number, name in (
        (9, "continuous forward/backward"),
        (10, "continuous lateral"),
        (11, "continuous turning"),
        (12, "locomotion stop/release"),
    ):
        results.append(ValidationResult(number, name, "SKIP", "LOCOMOTION_NOT_AVAILABLE", {}))

    frozen_sources = {
        "model": ROOT / "assets" / "Master" / "ff_master_ultra.xml",
        "limits_include": ROOT / "assets" / "Master" / "ff_master_ultra_x2_limits.xml",
        "free_scene": ROOT / "assets" / "Master" / "scene_x2_free.xml",
        "controller": ROOT / "master_sim" / "controller.py",
        "phase3ay_candidate": ROOT / "calibration" / "phase3ay_motion_conditioned_balance" / "simulation_motion_conditioned_balance_candidate.json",
        **{f"{name}_reference": spec.path for name, spec in ACTION_SPECS.items()},
    }
    payload = {
        "classification": "SIMULATION_ONLY_INTERACTIVE_DEMO_NOT_HARDWARE_CALIBRATION",
        "timestep_s": demo.dt,
        "controller_rate_hz": 1.0 / demo.dt,
        "locomotion_available": LOCOMOTION_AVAILABLE,
        "frozen_source_sha256": {name: _sha256(path) for name, path in frozen_sources.items()},
        "results": [asdict(result) for result in results],
        "gates": {
            "INTERACTIVE_DEMO_READY": "YES" if all(r.status == "PASS" for r in results[:8]) else "NO",
            "STANDING_READY": "YES" if results[0].status == "PASS" else "NO",
            "HEART_READY": "YES" if results[1].status == "PASS" else "NO",
            "WAVE_READY": "YES" if results[2].status == "PASS" else "NO",
            "CLAP_READY": "YES" if results[3].status == "PASS" else "NO",
            "LOCOMOTION_READY": "NO",
            "TURNING_READY": "NO",
            "SAFETY_MONITOR_READY": "YES" if results[7].status == "PASS" else "NO",
            "DYNAMICS_CALIBRATION_READY": "NO",
        },
    }
    output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    for result in results:
        print(f"TEST {result.number:02d} {result.status:4s} {result.name}: {result.note}")
    print(json.dumps(payload["gates"], indent=2))
    print(f"Validation evidence: {output}")
    return 0 if payload["gates"]["INTERACTIVE_DEMO_READY"] == "YES" else 1


def run_viewer(smoke_seconds: float | None = None) -> int:
    demo = InteractiveDemo()
    keyboard = ViewerKeyboard()
    print("Master X2 MuJoCo Interactive Demo - SIMULATION ONLY / NOT HARDWARE CALIBRATION")
    print("H Heart | V Wave | C Clap | SPACE stop/hold | R reset | 1/2/3 speed | ESC exit")
    print("W/S/A/D/Q/E: LOCOMOTION_NOT_AVAILABLE in the frozen Phase 3A-X/Y stack")
    start_wall = time.monotonic()
    with mujoco.viewer.launch_passive(demo.model, demo.data, key_callback=keyboard.on_key) as viewer:
        while viewer.is_running():
            iteration_wall = time.monotonic()
            down, rising = keyboard.sample()
            if "ESC" in down:
                break
            for key, action in (("H", "heart"), ("V", "wave"), ("C", "clap")):
                if key in rising:
                    demo.start_action(action)
            if "SPACE" in rising:
                demo.controlled_stop()
            if "R" in rising:
                demo.manual_reset()
            for level in (1, 2, 3):
                if str(level) in rising:
                    demo.set_speed_level(level)
            for key in ("W", "S", "A", "D", "Q", "E"):
                if key in down:
                    demo.request_locomotion(key)
            demo.step()
            viewer.sync()
            if smoke_seconds is not None and time.monotonic() - start_wall >= smoke_seconds:
                break
            remaining = demo.dt - (time.monotonic() - iteration_wall)
            if remaining > 0.0:
                time.sleep(remaining)
    print(demo.status_line())
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--self-test", action="store_true", help="run deterministic headless tests 1-12")
    parser.add_argument("--validation-output", type=Path, default=DEFAULT_RESULTS)
    parser.add_argument("--viewer-smoke-seconds", type=float, default=None, help="launch the real viewer, then close after N wall seconds")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.self_test:
        return run_self_test(args.validation_output.resolve())
    return run_viewer(args.viewer_smoke_seconds)


if __name__ == "__main__":
    raise SystemExit(main())
