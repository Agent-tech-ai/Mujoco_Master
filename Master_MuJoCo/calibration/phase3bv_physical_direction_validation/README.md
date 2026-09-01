# Phase 3B-V offline workflow

This directory validates one previously selected physical sensitivity direction:
`bs_mass_lower_plus08`. It is **not** a hardware-parameter identification workflow.

## Safety boundary

- The Python tools are offline-only and do not connect to a robot.
- The recorder wrapper only starts read-only subscriptions and never invokes a preset.
- Only an onsite operator may separately execute an already approved native-MC preset.
- No source MJCF is edited and no calibrated MJCF is created.
- `reported_effort` is excluded from validation references and metrics.

## Current state

No quality-gated third motion capture is present. The selected future validation motion is
the native-MC `clap` preset (motion 3017, area 11). Until its capture and replay pass,
generalization and new-motion safety remain pending.

## Capture (operator-controlled, not run by Codex)

Start the passive recorder from PowerShell:

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "C:\Users\xinga\Documents\Codex\2026-08-11\master-mujoco-agibot-x2-ultra-mujoco\work\run_x2_phase3bv_clap_capture_readonly.ps1" -Target "run@192.168.4.114" -CaptureSeconds 120 -OperatorSafetyConfirmed
```

Wait for `PRE-ROLL COMPLETE`. The onsite operator may then separately invoke the approved
`clap` preset once. Leave at least five seconds of stable data after completion.

## Offline processing and validation

After capture completes, run in order from the workspace root:

```powershell
python Master_MuJoCo\calibration\phase3bv_physical_direction_validation\process_phase3bv_capture.py
python Master_MuJoCo\calibration\phase3bv_physical_direction_validation\run_phase3bv_replays.py
python Master_MuJoCo\calibration\phase3bv_physical_direction_validation\analyze_phase3bv.py
```

The dual replay uses the identical frozen controller for:

1. the original physical baseline; and
2. `bs_mass_lower_plus08` as a runtime-only mass perturbation.

The final report remains conservative unless capture quality, cross-motion independence,
leg-response improvement, arm preservation, controller identity, and safety all pass.

