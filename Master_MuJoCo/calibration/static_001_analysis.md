# static_001 analysis

- Evidence: `calibration/evidence/x2_phase2b_static_capture.txt`
- Capture mode: passive subscription only, approximately 30 seconds.
- Requested duration: `30.0` s; serialized interval cap: `0.02` s per topic.
- The local SSH wrapper returned code 2 only because PowerShell appended CR to the final shell `exit 0`; the Python capture produced its summary, all 8259 JSON records parsed, and no capture-failed marker or traceback exists.
- Statistics use raw source coordinates; no zero/sign/scale correction was applied.
- CSV timestamp is subscriber monotonic elapsed receive time; source `header.stamp`, `sequence`, and `meas_stamp` remain in the raw evidence.
- `effort` is reported as the raw field. AimDK documents torque/N·m, but its physical origin remains unknown.

## Joint array-index statistics

| group | index | live-confirmed name | n | mean position | std position | mean velocity | std velocity | mean effort | std effort |
|---|---:|---|---:|---:|---:|---:|---:|---:|---:|
| head | 0 | head_yaw | 1339 | 0.000143766403 | 0 | 0.00306892395 | 0 | -0.000561100765 | 0.000619178587 |
| head | 1 | head_pitch | 1339 | 0 | 0 | 0 | 0 | 0 | 0 |
| arm | 0 | left_shoulder_pitch | 1385 | 0.392981052 | 0 | -0.00610542297 | 0 | 0.460337571 | 0.0372323003 |
| arm | 1 | left_shoulder_roll | 1385 | -0.0344181061 | 0 | -0.00610542297 | 0 | 1.29312035 | 0.0464136826 |
| arm | 2 | left_shoulder_yaw | 1385 | -0.00872421265 | 0 | 0.00610542297 | 0 | 0.415058409 | 0.0360138083 |
| arm | 3 | left_elbow | 1385 | -1.16858912 | 0 | -0.00610542297 | 0 | -1.32100724 | 0.0323828551 |
| arm | 4 | left_wrist_yaw | 1385 | 0.00910758972 | 0 | 0.00610542297 | 0 | -0.29576692 | 0.0323991106 |
| arm | 5 | left_wrist_pitch | 1385 | 0.00133514404 | 0 | -0.0073261261 | 0 | -0.0854506854 | 0.00277098978 |
| arm | 6 | left_wrist_roll | 1385 | -0.00400543213 | 0 | -0.0073261261 | 0 | 0.117398567 | 0.00167008273 |
| arm | 7 | right_shoulder_pitch | 1385 | 0.37112236 | 0 | 0.00610542297 | 0 | 0.408076108 | 0.0314314006 |
| arm | 8 | right_shoulder_roll | 1385 | 0.0532093048 | 0 | -0.00610542297 | 0 | -0.69574401 | 0.0331525036 |
| arm | 9 | right_shoulder_yaw | 1385 | 0.0547432899 | 0 | 0.00610542297 | 0 | -0.569938533 | 0.0277801828 |
| arm | 10 | right_elbow | 1385 | -1.10224533 | 0 | 0.00610542297 | 0 | -1.07832702 | 0.0340636984 |
| arm | 11 | right_wrist_yaw | 1385 | 0.0215716362 | 0 | 0.00610542297 | 0 | -0.183634354 | 0.0308470771 |
| arm | 12 | right_wrist_pitch | 1385 | 0.0123977661 | 0 | 0.0073261261 | 0 | -0.117218018 | 0 |
| arm | 13 | right_wrist_roll | 1385 | -0.00667572021 | 0 | -0.0073261261 | 0 | 0.0451674892 | 0.00762257143 |
| waist | 0 | waist_yaw | 1383 | -0.00335550308 | 0 | -0.00610542297 | 0 | 0.0203627437 | 0.0852313687 |
| waist | 1 | waist_pitch | 1383 | -0.0327415996 | 0 | 9.24669669e-06 | 0 | -1.43766682 | 0.0795355337 |
| waist | 2 | waist_roll | 1383 | 0.00756687546 | 0 | 0.00491723467 | 0 | 0.667262044 | 0.0661299796 |
| leg | 0 | left_hip_pitch | 1390 | -0.248405457 | 0 | -0.00610542297 | 0 | -5.98114108 | 0.141055764 |
| leg | 1 | left_hip_roll | 1390 | -0.0311584473 | 0 | -0.00610542297 | 0 | 9.79757952 | 0.128681759 |
| leg | 2 | left_hip_yaw | 1390 | -0.018887043 | 0 | 0.00610542297 | 0 | -1.04233859 | 0.121024718 |
| leg | 3 | left_knee | 1390 | 0.523367882 | 0 | -0.00610542297 | 0 | -12.224376 | 0.1193943 |
| leg | 4 | left_ankle_pitch | 1390 | -0.330855846 | 0 | 0.00610542297 | 0 | 11.2059437 | 0.050868536 |
| leg | 5 | left_ankle_roll | 1390 | 0.0154352188 | 0 | 0.00610542297 | 0 | -0.816505213 | 0.0306848189 |
| leg | 6 | right_hip_pitch | 1390 | -0.293273449 | 0 | 0.00610542297 | 0 | 9.43450677 | 0.135161914 |
| leg | 7 | right_hip_roll | 1390 | 0.0167779922 | 0 | -0.00610542297 | 0 | -6.46016665 | 0.102338284 |
| leg | 8 | right_hip_yaw | 1390 | -0.0271320343 | 0 | 0.00610542297 | 0 | 1.93170527 | 0.122628758 |
| leg | 9 | right_knee | 1390 | 0.49652338 | 0 | 0.00610542297 | 0 | -1.19020676 | 0.0939466733 |
| leg | 10 | right_ankle_pitch | 1390 | -0.274674416 | 0 | -0.00610542297 | 0 | 10.3409107 | 0.0523758795 |
| leg | 11 | right_ankle_roll | 1390 | 0.00278043747 | 0 | 0.00610542297 | 0 | -0.107098471 | 0.0329770254 |

## Capture coverage

| topic | received callbacks | serialized samples | observed serialized time span (s) |
|---|---:|---:|---:|
| `/aima/hal/imu/chest/state` | 12835 | 1382 | 29.5751682 |
| `/aima/hal/imu/torso/state` | 12830 | 1380 | 29.5599362 |
| `/aima/hal/joint/arm/state` | 12922 | 1385 | 29.5535118 |
| `/aima/hal/joint/head/state` | 9733 | 1339 | 29.561995 |
| `/aima/hal/joint/leg/state` | 12929 | 1390 | 29.6416102 |
| `/aima/hal/joint/waist/state` | 12911 | 1383 | 29.5193768 |

## IMU statistics

| IMU | samples | mean gyro [x,y,z] (rad/s) | std gyro | mean acceleration [x,y,z] (m/s²) | std acceleration |
|---|---:|---|---|---|---|
| chest | 1382 | `[0.00010899981234346679, 6.0307007716163805e-05, -1.3598663707298282e-05]` | `[0.0022301789810080185, 0.0022889824146886265, 0.0019372366945308794]` | `[-0.6443327151970972, -0.03921786856418925, 9.806085797647667]` | `[0.007736862888325757, 0.007166764916247242, 0.009935219181266835]` |
| torso | 1380 | `[-8.586558019058474e-05, -6.268915661906873e-05, -1.008129193989324e-05]` | `[0.0019301596210894252, 0.0018089158970388312, 0.0020407827088233304]` | `[-0.6463564740810197, -0.2052478731912755, 9.799704699470446]` | `[0.007064727285388791, 0.006997246690282722, 0.008580726559969946]` |

## CONFIRMED

- Report values are direct population mean/std statistics over decoded passive samples.
- Every group index had one stable, populated live name throughout the capture.

## FIELD_TEST_EVIDENCE

- The capture describes only the robot's naturally occurring state during this window; the workflow did not request a pose change.

## INFERRED_CANDIDATE

- No label was inferred from static position similarity; array labels come from stable live `JointState.name` values.
- Hardware↔MuJoCo physical correspondence remains a candidate even where strings match exactly.

## UNKNOWN

- Static position similarity is not used to establish a mapping, sign, or zero.
- Effort physical origin and IMU gravity policy remain unknown.

## NEEDS_PHYSICAL_VERIFICATION

- Verify the robot was physically stationary throughout the capture and note any external support/contact loads.
