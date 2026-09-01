# Master MuJoCo conservative single-step validation

## Scope

- Code change: `outputs/Master_MuJoCo/demo.py` only.
- Evidence was added under `outputs/Master_MuJoCo/validation_evidence/`.
- Loaded model: `assets/Master/scene_x2_free.xml`.
- Existing `master_sim.controller.JointPositionController`, actuator mapping,
  poses, PD gains, control limits, and model joint limits are reused.
- No XML, URDF, mesh, existing controller, joint-limit file, README, or
  other project source was changed.

## Root-cause audit of the replaced gait

1. The old gait continuously advanced one phase and generated opposing
   left/right sinusoidal values (`sin(phase)` and `-sin(phase)`).
2. It did not define or track a stance foot.
3. It did not read MuJoCo foot-floor contacts.
4. It did not read the left/right foot world positions.
5. It did not perform an explicit pelvis/COM lateral transfer.
6. It did not transfer load to the stance leg before lifting the other foot.
7. It did not require stable double-foot contact after touchdown.
8. Both stance and swing leg joint targets were varied during the same
   continuous cycle.
9. Locomotion was implemented as sinusoidal overlays on hip/knee/ankle
   targets rather than a contact-gated step.
10. The old test treated target motion and very small displacement as
    success, without requiring final double support, direction/drift limits,
    long idle stability, or Viewer evidence. That is why the earlier PASS was
    invalid.

The old failure was reproduced in the real Viewer before replacement:
`old_forward_start.png`, `old_forward_mid.png`,
`old_forward_keyup.png`, `fall_example.png`, and
`old_gait_reproduction.log`. The robot rocked in place and then fell.

## Discovered contact geometry

- Floor: named geom `floor`, geom id 0, body `world`.
- Left foot: unnamed collision geoms 15 through 26, all on body
  `left_ankle_roll_link`.
- Right foot: unnamed collision geoms 38 through 49, all on body
  `right_ankle_roll_link`.

The geom ids were discovered from the loaded model. They were not guessed
from an assumed foot-geom name.

## Controller states, gates, and timing

| State | Target / exit gate | Maximum time |
|---|---|---:|
| `STAND` | Stable double-support hold; accepts one new command only after the 5 s physical startup settle | unbounded |
| `SHIFT_TO_LEFT` | Hip roll + ankle roll + small waist/shoulder compensation moves pelvis at least 20 mm toward the left support side; both contacts, swing-foot force below 100 N, lateral speed below 0.06 m/s, roll/pitch below 12 degrees for 30 ms | 1.60 s |
| `LIFT_RIGHT` | Smooth right hip/knee/ankle flexion; right foot must be contact-free for 40 ms | 1.30 s |
| `SWING_RIGHT` | Smooth right-foot pitch/roll/yaw placement; minimum 0.25 s (0.15 s in isolated lift test) | 0.70 s |
| `LAND_RIGHT` | Smooth lowering; both feet must be in contact for 100 ms | 1.00 s |
| `DOUBLE_SUPPORT` | Both contacts, roll/pitch below 12 degrees, centered pelvis, low lateral speed for 0.50 s | 2.00 s |
| `SHIFT_TO_RIGHT` | Symmetric transfer to right support side with the same pose/contact gate | 1.60 s |
| `LIFT_LEFT` | Smooth left hip/knee/ankle flexion; left foot contact-free for 40 ms | 1.30 s |
| `SWING_LEFT` | Smooth left-foot placement; minimum 0.25 s (0.15 s in isolated lift test) | 0.70 s |
| `LAND_LEFT` | Smooth lowering; both feet in contact for 100 ms | 1.00 s |
| `RECOVER` | Smooth rate-limited return; both contacts, roll/pitch below 8 degrees, height and horizontal-speed safe, joint-target error below 0.06 rad for 0.50 s | 6.00 s |

Any non-recovery timeout enters `RECOVER`. A recovery timeout freezes the
current safe actuator posture. Safety recovery begins at 20 degrees roll or
pitch, a pelvis-height drop, stance-contact loss, both contacts lost for more
than 100 ms, a non-finite state, or a target-limit violation.

The SAFE requests are forward/backward 40 mm, lateral +/-20 mm, and yaw
+/-3 degrees. The six Viewer trials had active state-machine durations:
forward 2.448 s, backward 2.686 s, left 3.094-3.128 s, right 3.332 s,
left turn 2.856 s, and right turn 2.686 s.

## Actual Viewer procedure

1. A fresh process was launched for every trial with
   `python -B demo.py --duration 24 --warmup-ready --log ...`.
2. `--warmup-ready` ran 5 s of normal MuJoCo physics before opening the
   Viewer; it did not reset or translate the free base.
3. The newly created window was found by enumerating top-level Windows
   windows. Its title had to contain `MuJoCo`.
4. `SetForegroundWindow` was called and the foreground window handle was
   checked against the target before any key was sent.
5. Win32 `keybd_event` sent a real main-keyboard key down, held each movement
   key for 4 s, and sent key up. A movement command is accepted once; new
   movement commands are ignored while its step is active.
6. The side camera was fixed at distance 2.30, azimuth 145 degrees,
   elevation -18 degrees. The high camera was fixed at distance 2.45,
   azimuth 180 degrees, elevation -55 degrees. Both use fixed lookat
   `(0, 0, 0.70)` and do not track the robot.
7. Start, mid-action, and recovered-end screenshots were captured for every
   movement trial. High view was used for lateral and turning trials.
8. No MP4 was recorded; the requested three-screenshot fallback was used.
9. O/P/K/L and motion-in-progress `0` were tested in three additional fresh
   Viewer processes. Key `5` was also sent to a foreground Viewer and printed
   the required disabled-mode message.

## Measured Viewer movement trials

`C(L/R)` is the left/right contact-time ratio from command start through the
post-command stable sample. All rows had final double contact, no interval
with both feet off, no actuator saturation, no fall, and an upright
fixed-camera visual result.

| Action | Trial | net x m | net y m | net yaw deg | max roll deg | max pitch deg | min pelvis m | C(L/R) | Visual | Result |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| Backward | 1 | -0.06135 | -0.01805 | +4.218 | 3.59 | 7.39 | 0.667 | 1.000/0.974 | upright, correct translation | PASS |
| Backward | 2 | -0.06156 | -0.01807 | +4.228 | 3.59 | 7.39 | 0.667 | 1.000/0.974 | upright, correct translation | PASS |
| Backward | 3 | -0.06042 | -0.01794 | +4.268 | 3.59 | 7.39 | 0.667 | 1.000/0.972 | upright, correct translation | PASS |
| Forward | 1 | +0.05762 | -0.01484 | +1.538 | 3.97 | 8.56 | 0.664 | 1.000/0.954 | upright, correct translation | PASS |
| Forward | 2 | +0.06640 | -0.01666 | +1.962 | 3.93 | 8.52 | 0.664 | 1.000/0.975 | upright, correct translation | PASS |
| Forward | 3 | +0.06622 | -0.01691 | +1.939 | 4.03 | 8.57 | 0.664 | 1.000/0.970 | upright, correct translation | PASS |
| Left | 1 | +0.00820 | +0.03490 | -0.738 | 2.43 | 8.31 | 0.667 | 0.929/1.000 | upright, lateral not turn | PASS |
| Left | 2 | +0.00913 | +0.03466 | -0.726 | 2.44 | 8.26 | 0.667 | 0.928/1.000 | upright, lateral not turn | PASS |
| Left | 3 | +0.00820 | +0.03490 | -0.738 | 2.43 | 8.31 | 0.667 | 0.929/1.000 | upright, lateral not turn | PASS |
| Right | 1 | +0.00109 | -0.04092 | +3.209 | 4.83 | 8.25 | 0.663 | 1.000/0.978 | upright, lateral dominant | PASS |
| Right | 2 | +0.00124 | -0.04084 | +3.216 | 4.83 | 8.25 | 0.663 | 1.000/0.977 | upright, lateral dominant | PASS |
| Right | 3 | +0.00206 | -0.04056 | +3.250 | 4.82 | 8.26 | 0.663 | 1.000/0.975 | upright, lateral dominant | PASS |
| Turn left | 1 | +0.00791 | +0.00539 | +5.291 | 2.27 | 7.70 | 0.668 | 0.938/1.000 | upright, heading changed left | PASS |
| Turn left | 2 | +0.00817 | +0.00536 | +5.318 | 2.27 | 7.70 | 0.668 | 0.936/1.000 | upright, heading changed left | PASS |
| Turn left | 3 | +0.00803 | +0.00540 | +5.292 | 2.28 | 7.72 | 0.668 | 0.937/1.000 | upright, heading changed left | PASS |
| Turn right | 1 | +0.00002 | -0.03014 | -5.414 | 4.81 | 8.38 | 0.663 | 1.000/0.978 | upright, heading changed right | PASS |
| Turn right | 2 | +0.00015 | -0.03015 | -5.426 | 4.81 | 8.38 | 0.663 | 1.000/0.978 | upright, heading changed right | PASS |
| Turn right | 3 | +0.00009 | -0.03018 | -5.404 | 4.81 | 8.40 | 0.663 | 1.000/0.978 | upright, heading changed right | PASS |

The exact pelvis and both-foot start/end world coordinates for every row are
in `movement_trial_metrics.csv`; all 30 Hz samples are in
`motion_validation.csv`.

## Idle, foot-lift, upper-body, and recenter results

- Real Viewer idle covered 23.970 s of simulation: net pelvis
  `(x, y)=(+0.000775, +0.00000045)` m, max roll 0.0024 degrees,
  max pitch 2.8606 degrees, minimum pelvis height 0.673849 m,
  left/right contact ratios 1.000/1.000, no both-feet-off interval,
  no saturation, no fall, and no non-finite value. Visual start, 10 s, and
  end frames show the robot upright. PASS.
- Stage 1 isolated right-foot lift/land: contact actually switched off and
  back on; measured right foot-body-origin rise 0.0080 m, final double
  contact, no fall. PASS against the contact-based lift/land gate.
- Stage 1 isolated left-foot lift/land: contact actually switched off and
  back on; measured left foot-body-origin rise 0.0168 m, final double
  contact, no fall. PASS.
- O/P and K/L, three Viewer repetitions: actual head yaw covered both signs
  (per-key samples at least +5.73/-3.94 degrees), actual waist yaw covered
  both signs (at least +7.94/-7.25 degrees), while total base-yaw span was
  only 0.31-0.35 degrees. No saturation or fall. PASS.
- Motion-in-progress `0`, three Viewer repetitions: every run changed
  `SHIFT_TO_LEFT -> RECOVER -> STAND`, ended with both feet in contact,
  head within 0.35 degrees of center, and waist within 0.02 degrees.
  Largest logged 30 Hz head/waist increment after `0` was below 1 degree.
  `max_ctrl` stayed approximately 9.5 rather than being zeroed. PASS.
- Key `5`: an actual foreground-Viewer key press produced a second
  `Continuous speed modes disabled until stable stepping is validated.`
  log line. PASS.
- Full stage-gated headless physics regression also passed Stage 1, Stage 2,
  Stage 3, and Stage 4. See `staged_validation.log`.

## Changes in `demo.py`

- Replaced continuous sinusoidal gait overlays with the 11-state
  contact/pose-gated single-step state machine.
- Added runtime discovery and printing of floor/foot collision geoms.
- Added every-frame foot contacts, contact forces, foot world positions,
  pelvis position/quaternion/RPY, finite-state, control, saturation, and fall
  monitoring.
- Added pre-lift lateral load transfer using hip roll, ankle roll, and small
  waist/shoulder compensation.
- Added smooth swing-foot flexion/placement, touchdown gating, explicit
  stable double support, and rate-limited recovery.
- Added sagittal ankle feedback and stance-side roll/ankle balance feedback.
- Added 20-degree gait-abort safety, contact-loss, height, finite-state,
  target-limit, and recovery-timeout protection.
- Changed 8/2/4/6/7/9 to one SAFE step per press and ignores new movement
  requests while active.
- Made `0` center head/waist, abort the active step, and smoothly recover
  without clearing actuator control.
- Kept `5` disabled with the required message until stable continuous
  walking is separately validated.
- Fixed Windows foreground handling for MuJoCo's child Viewer process and
  supports main-number and numpad virtual keys.
- Added 30 Hz CSV logging, fixed side/high validation cameras, startup
  physical settling, and stage-gated self-validation.

## Remaining limitations

- This is a SAFE single-step controller, not validated continuous walking.
  LOW/MEDIUM/HIGH modes and held-key continuous locomotion remain disabled.
- The right foot body origin rose about 8 mm in the successful physical
  lift, below the requested 15-25 mm guideline. Increasing it caused
  instability during tuning, so contact-off plus the measured rise is kept
  as the conservative Stage 1 result rather than overstating 15 mm.
- Forward's measured active state-machine interval was 2.448 s, about
  0.052 s below the requested 2.5-4.0 s guideline, although the real key was
  held for 4 s and all displacement/stability criteria passed.
- Right turns retain about 30 mm of lateral drift; backward steps retain
  about 4.2 degrees of yaw drift, and right lateral steps about 3.2 degrees.
  These passed the stated direction/yaw thresholds but are not symmetric or
  production-quality locomotion.
- Foot collision geoms are unnamed in the XML, so the controller identifies
  them by collision capability and their actual ankle-roll body membership.
- No MP4 was produced. Each movement has start/mid/end screenshots instead.

## Integrity statement

- Modified root qpos: **NO**
- Modified root velocity: **NO**
- Used teleport or free-joint reset: **NO**
- Used camera motion to fake displacement: **NO**
- Used mocap or external force to push the robot: **NO**
- Actually opened MuJoCo Viewer: **YES**
- Actually foregrounded Viewer and sent real key down/up: **YES**
- Generated visual evidence: **YES**
- Movement is produced through joint targets, the existing PD controller,
  actuator torques, MuJoCo contacts, and physics: **YES**
