# Phase 2E left/right measured-coordinate symmetry

All relationships below describe real measured coordinates only. They do not update hardware-to-MuJoCo sign, zero, or encoder offset.

| pair | left excursion rad | right excursion rad | same corr | mirrored corr | RMS(left+right) | RMS(left-right) | classification |
| --- | --- | --- | --- | --- | --- | --- | --- |
| elbow_joint | 0.26768 | 0.27899 | 0.999 | -0.999 | 2.61780 | 0.00623 | SAME_SIGN |
| shoulder_pitch_joint | 0.39492 | 0.40094 | 1.000 | -1.000 | 0.53246 | 0.00252 | SAME_SIGN |
| shoulder_roll_joint | 2.20534 | 2.20281 | -1.000 | 1.000 | 0.00288 | 2.89919 | MIRRORED |
| shoulder_yaw_joint | 1.58094 | 1.57313 | -1.000 | 1.000 | 0.00497 | 2.07667 | MIRRORED |
| wrist_pitch_joint | 0.00114 | 0.00076 | UNKNOWN | UNKNOWN | 0.00794 | 0.00448 | INSUFFICIENT_EVIDENCE |
| wrist_roll_joint | 1.07830 | 1.09102 | -1.000 | 1.000 | 0.00718 | 1.46164 | MIRRORED |
| wrist_yaw_joint | 1.62408 | 1.63120 | -1.000 | 1.000 | 0.00318 | 2.18479 | MIRRORED |
