# Interactive demo validation

Classification: **SIMULATION-ONLY INTERACTIVE DEMO — NOT HARDWARE CALIBRATION**.

Command:

```powershell
python run_interactive_demo.py --self-test
```

The tests run the same model, Phase 3A-Y controller over the frozen Phase 3A-X safety layer, trajectory player, outer safety observer, and 1 kHz `mj_step` loop used by the viewer. There are no random perturbations. Full machine-readable evidence, including source hashes and all metrics, is in `interactive_demo_validation_results.json`.

| # | Test | Result | Max roll / pitch | Min joint margin | Max saturation / persistent duration | Contact outcome | Final standing |
|---:|---|---|---:|---:|---:|---|---|
| 1 | Launch -> free-base standing 10 s | PASS | 0.50° / 3.23° | 0.050 rad | 0.360 / 0.000 s | 0 unexpected, 0 non-foot | YES |
| 2 | Heart -> standing | PASS | 0.57° / 3.62° | 0.050 rad | 0.360 / 0.000 s | 0 unexpected, 0 non-foot | YES |
| 3 | Wave -> standing | PASS | 0.50° / 9.28° | 0.050 rad | 0.510 / 0.000 s | 0 unexpected, 0 non-foot | YES |
| 4 | Clap -> standing | PASS | 0.61° / 4.25° | 0.036 rad | 0.685 / 0.000 s | 526 expected-Clap samples; 0 unexpected, 0 non-foot | YES |
| 5 | Heart -> SPACE interrupt -> standing | PASS | 0.57° / 3.42° | 0.050 rad | 0.360 / 0.000 s | 0 unexpected, 0 non-foot | YES |
| 6 | Wave -> explicit R reset -> standing 10 s | PASS | 0.50° / 5.47° | 0.050 rad | 0.380 / 0.000 s | 0 unexpected, 0 non-foot | YES |
| 7 | Heart -> Wave -> Clap | PASS | 0.60° / 9.16° | 0.036 rad | 0.685 / 0.000 s | 527 expected-Clap samples; 0 unexpected, 0 non-foot | YES |
| 8 | Safety monitor active for 10 s | PASS | 0.50° / 3.23° | 0.050 rad | 0.360 / 0.000 s | 10,000/10,000 steps evaluated | YES |
| 9 | Forward/backward -> stop | SKIP | — | — | — | `LOCOMOTION_NOT_AVAILABLE` | — |
| 10 | Lateral -> stop | SKIP | — | — | — | `LOCOMOTION_NOT_AVAILABLE` | — |
| 11 | Turn left/right | SKIP | — | — | — | `LOCOMOTION_NOT_AVAILABLE` | — |
| 12 | Locomotion release/stop | SKIP | — | — | — | `LOCOMOTION_NOT_AVAILABLE` | — |

All PASS cases had zero fall, zero joint-limit violation, zero unexpected self-collision, zero non-foot ground contact, and zero persistent saturation. Foot slip remained monitored; the largest proxy in the sequence test was approximately 0.0103 m left / 0.0106 m right.

## Clap contact classification

Only `left_wrist_roll_link <-> right_wrist_roll_link` is classified as `EXPECTED_TASK_CONTACT`, only while action=`clap`, and only in the three Phase 3B-C evidence envelopes:

- 1.601–1.761 s
- 2.777–2.962 s
- 3.976–4.163 s

The 1 ms envelope is supported by the documented <=1 ms onset variation across the frozen original/+8% and arm-only/whole-body evidence. A `1e-9 s` comparison epsilon handles floating-point boundary representation; it does not broaden the windows. All other self-collision remains a safety violation.

## Viewer smoke test

`python run_interactive_demo.py --viewer-smoke-seconds 3` launched the official `mujoco.viewer.launch_passive` window and exited normally with state `STANDING`, roll `-0.24°`, pitch `-0.02°`, minimum joint margin `0.049 rad`, and saturation fraction `0.293`.

## Gates

```text
INTERACTIVE_DEMO_READY = YES
STANDING_READY = YES
HEART_READY = YES
WAVE_READY = YES
CLAP_READY = YES
LOCOMOTION_READY = NO
TURNING_READY = NO
SAFETY_MONITOR_READY = YES
DYNAMICS_CALIBRATION_READY = NO
```
