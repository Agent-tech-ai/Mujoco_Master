# Phase 3B-V selected blind-validation motion

| Item | Selection/evidence |
|---|---|
| Candidate motion | `clap` |
| Native MC preset | motion `3017`, area `11` |
| Catalog qualification | `physically_tested_without_parameters=True` |
| Motion hands | left + right |
| Control path | existing standing gesture wrapper → `SetMcPresetMotion(..., interrupt=False)` → native MC |
| Direct HAL | not used |
| Existing third capture | YES |
| Numerical independence from Heart/Wave | SUFFICIENTLY_INDEPENDENT_FROM_HEART_AND_WAVE |

The catalog evidence comes from the already-captured read-only Phase 2C source at `work/phase2c_agentech01_code_discovery_readonly.txt`. `clap` is selected because it is a physically-tested bilateral coordinated preset with a different preset ID and expected temporal/spatial structure from bilateral Heart and unilateral right-hand Wave.

This selection does **not** assume independence. The capture must pass active-joint, excursion-vector, duration, and left/right response checks against both prior motions.

## Accepted capture

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "C:\Users\xinga\Documents\Codex\2026-08-11\master-mujoco-agibot-x2-ultra-mujoco\work\run_x2_phase3bv_clap_capture_readonly.ps1" -Target "run@192.168.4.114" -CaptureSeconds 120 -OperatorSafetyConfirmed
```

The capture was made by the user-run read-only recorder; preset execution was external operator action through the existing MC-compatible path. Codex and the recorder did not invoke motion.
