# Phase 3A-V measured-joint classification

All positions and velocities are measured output state: **MEASURED_REAL_TRAJECTORY**. They are not MC internal commands.

Reported effort is retained only as `reported_effort`; semantics remain UNKNOWN and it is not used for fitting.

| joint | group | classification | excursion rad | peak \|dq\| rad/s | basis |
| --- | --- | --- | --- | --- | --- |
| left_elbow_joint | arm | STATIC | 0.00115 | 0.04273 | group=arm; excursion=0.001150 rad; peak\|dq\|=0.042734 rad/s; measured state only |
| left_shoulder_pitch_joint | arm | GESTURE_SECONDARY | 0.00997 | 0.11600 | group=arm; excursion=0.009971 rad; peak\|dq\|=0.115995 rad/s; measured state only |
| left_shoulder_roll_joint | arm | STATIC | 0.00000 | 0.00611 | group=arm; excursion=0.000000 rad; peak\|dq\|=0.006105 rad/s; measured state only |
| left_shoulder_yaw_joint | arm | STATIC | 0.00058 | 0.03053 | group=arm; excursion=0.000576 rad; peak\|dq\|=0.030525 rad/s; measured state only |
| left_wrist_pitch_joint | arm | STATIC | 0.00000 | 0.00733 | group=arm; excursion=0.000000 rad; peak\|dq\|=0.007326 rad/s; measured state only |
| left_wrist_roll_joint | arm | STATIC | 0.00000 | 0.00733 | group=arm; excursion=0.000000 rad; peak\|dq\|=0.007326 rad/s; measured state only |
| left_wrist_yaw_joint | arm | STATIC | 0.00000 | 0.00611 | group=arm; excursion=0.000000 rad; peak\|dq\|=0.006105 rad/s; measured state only |
| right_elbow_joint | arm | GESTURE_SECONDARY | 0.57811 | 0.76313 | group=arm; excursion=0.578111 rad; peak\|dq\|=0.763126 rad/s; measured state only |
| right_shoulder_pitch_joint | arm | GESTURE_PRIMARY | 2.94635 | 3.31502 | group=arm; excursion=2.946352 rad; peak\|dq\|=3.315018 rad/s; measured state only |
| right_shoulder_roll_joint | arm | GESTURE_SECONDARY | 0.34667 | 0.94628 | group=arm; excursion=0.346675 rad; peak\|dq\|=0.946276 rad/s; measured state only |
| right_shoulder_yaw_joint | arm | GESTURE_PRIMARY | 0.54973 | 1.76435 | group=arm; excursion=0.549733 rad; peak\|dq\|=1.764347 rad/s; measured state only |
| right_wrist_pitch_joint | arm | STATIC | 0.00076 | 0.00733 | group=arm; excursion=0.000763 rad; peak\|dq\|=0.007326 rad/s; measured state only |
| right_wrist_roll_joint | arm | STATIC | 0.00000 | 0.00733 | group=arm; excursion=0.000000 rad; peak\|dq\|=0.007326 rad/s; measured state only |
| right_wrist_yaw_joint | arm | GESTURE_PRIMARY | 1.55620 | 1.63004 | group=arm; excursion=1.556201 rad; peak\|dq\|=1.630037 rad/s; measured state only |
| head_pitch_joint | head | STATIC | 0.00000 | 0.00000 | group=head; excursion=0.000000 rad; peak\|dq\|=0.000000 rad/s; measured state only |
| head_yaw_joint | head | STATIC | 0.00010 | 0.00307 | group=head; excursion=0.000096 rad; peak\|dq\|=0.003069 rad/s; measured state only |
| left_ankle_pitch_joint | leg | BALANCE_COMPENSATION_CANDIDATE | 0.03010 | 0.07936 | group=leg; excursion=0.030104 rad; peak\|dq\|=0.079365 rad/s; measured state only |
| left_ankle_roll_joint | leg | BALANCE_COMPENSATION_CANDIDATE | 0.00882 | 0.06716 | group=leg; excursion=0.008821 rad; peak\|dq\|=0.067156 rad/s; measured state only |
| left_hip_pitch_joint | leg | BALANCE_COMPENSATION_CANDIDATE | 0.01400 | 0.05494 | group=leg; excursion=0.013998 rad; peak\|dq\|=0.054945 rad/s; measured state only |
| left_hip_roll_joint | leg | BALANCE_COMPENSATION_CANDIDATE | 0.00997 | 0.05494 | group=leg; excursion=0.009971 rad; peak\|dq\|=0.054945 rad/s; measured state only |
| left_hip_yaw_joint | leg | STATIC | 0.00249 | 0.03053 | group=leg; excursion=0.002492 rad; peak\|dq\|=0.030525 rad/s; measured state only |
| left_knee_joint | leg | BALANCE_COMPENSATION_CANDIDATE | 0.00729 | 0.05494 | group=leg; excursion=0.007286 rad; peak\|dq\|=0.054945 rad/s; measured state only |
| right_ankle_pitch_joint | leg | BALANCE_COMPENSATION_CANDIDATE | 0.02799 | 0.07936 | group=leg; excursion=0.027995 rad; peak\|dq\|=0.079365 rad/s; measured state only |
| right_ankle_roll_joint | leg | BALANCE_COMPENSATION_CANDIDATE | 0.00940 | 0.06716 | group=leg; excursion=0.009396 rad; peak\|dq\|=0.067156 rad/s; measured state only |
| right_hip_pitch_joint | leg | BALANCE_COMPENSATION_CANDIDATE | 0.01707 | 0.05494 | group=leg; excursion=0.017066 rad; peak\|dq\|=0.054945 rad/s; measured state only |
| right_hip_roll_joint | leg | BALANCE_COMPENSATION_CANDIDATE | 0.00959 | 0.05494 | group=leg; excursion=0.009588 rad; peak\|dq\|=0.054945 rad/s; measured state only |
| right_hip_yaw_joint | leg | STATIC | 0.00249 | 0.03053 | group=leg; excursion=0.002492 rad; peak\|dq\|=0.030525 rad/s; measured state only |
| right_knee_joint | leg | BALANCE_COMPENSATION_CANDIDATE | 0.01016 | 0.05494 | group=leg; excursion=0.010162 rad; peak\|dq\|=0.054945 rad/s; measured state only |
| waist_pitch_joint | waist | BALANCE_COMPENSATION_CANDIDATE | 0.02804 | 0.09567 | group=waist; excursion=0.028043 rad; peak\|dq\|=0.095675 rad/s; measured state only |
| waist_roll_joint | waist | BALANCE_COMPENSATION_CANDIDATE | 0.03867 | 0.14330 | group=waist; excursion=0.038672 rad; peak\|dq\|=0.143302 rad/s; measured state only |
| waist_yaw_joint | waist | BALANCE_COMPENSATION_CANDIDATE | 0.04602 | 0.29914 | group=waist; excursion=0.046019 rad; peak\|dq\|=0.299145 rad/s; measured state only |

## Independence from heart

- decision: **SUFFICIENTLY_INDEPENDENT_VALIDATION_MOTION**
- active-set Jaccard: `0.500`
- excursion-vector cosine similarity: `0.335`
- left/right excursion asymmetry: `0.996`
- duration wave/heart: `4.349 / 5.640 s`
