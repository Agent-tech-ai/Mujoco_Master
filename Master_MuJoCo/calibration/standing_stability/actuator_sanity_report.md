# Actuator and controller sanity report

The MJCF has 30 direct-drive motors with gear 1. Standing-relevant ctrlranges are ±118 N·m at hips/knees/waist-yaw, ±36 at ankle pitch, ±24 at ankle roll, and ±48 at waist pitch/roll. All joints inherit damping 0, armature 0.03 and frictionloss 0.3.

Original control is joint PD plus `qfrc_bias`. It initializes at zero joint error and does not provide feedback for the six free-base DOFs. Consequently, it cannot actively regulate pelvis pitch/roll inside the support region. The original free run later saturates while falling; the cleanup run peak saturation fraction is 0.247 with no sustained saturation.

The previous fixed-base 2° rehearsals show a separate tracking issue: uniform frictionloss 0.3 creates approximate pure-PD deadbands of 1.43° for wrist Kp=12 and 0.45° for arm Kp=38. A smooth compensation of the model's own frictionloss reduces that infrastructure artifact. It does not identify real friction or hardware gains.

Accepted simulation cleanup: pelvis roll/pitch feedback through both ankles (pitch 200/30, roll 100/20 in simulation units) plus smooth 1.5× compensation of the existing model frictionloss. These are **SIMULATION_STABILITY_CANDIDATE** values only.
