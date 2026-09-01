# Phase 2D capture quality report

## Required streams

| Topic | messages | mean Hz | max gap s | duplicate source stamps | out-of-order source stamps | frame_id |
| --- | --- | --- | --- | --- | --- | --- |
| /aima/hal/joint/arm/state | 1497 | 47.35 | 0.051 | 0 | 0 | x2_arm |
| /aima/hal/joint/head/state | 1456 | 46.05 | 0.045 | 0 | 0 | x2_head |
| /aima/hal/joint/leg/state | 1494 | 47.26 | 0.058 | 0 | 0 | x2_leg |
| /aima/hal/joint/waist/state | 1495 | 47.29 | 0.049 | 0 | 0 | x2_waist |
| /aima/hal/imu/chest/state | 1470 | 47.30 | 0.040 | 0 | 0 | base_link |
| /aima/hal/imu/torso/state | 1469 | 47.26 | 0.041 | 0 | 0 | base_link |

## Acceptance

- Replay-ready decision: **YES**.
- Decision evidence: all six required streams present; no clean footer; all required streams still cover full motion and >=5 s post-roll; heart motion detected from arm velocity; pre-roll >=5 s; post-roll >=5 s; required source timestamps monotonic; required receive gaps <0.5 s.
- Recorder termination: `MANUAL_OR_CONNECTION_END_AFTER_POST_ROLL`. A clean fixed-window footer is preferred but is not required for replay readiness when every required stream continuously covers the complete detected motion plus at least 5 s before and after.
- Malformed serialized lines: 1.
- Timestamp policy: raw source header stamp, `meas_stamp`, sequence, frame_id, robot receive wall clock, and robot receive monotonic clock are preserved. Alignment uses robot receive monotonic time; no claim is made that source and receive clocks are perfectly synchronized.
- MC state samples: 311; coverage before/during/after the detected motion is evaluated from the same receive clock.
- MC state values observed in this capture: input source `app_proxy`; action description `STAND_DEFAULT`. This confirms reported state continuity only; it does not independently define the semantics of balance-controller activation.
