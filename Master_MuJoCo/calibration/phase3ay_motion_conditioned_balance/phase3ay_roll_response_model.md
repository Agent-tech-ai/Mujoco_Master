# Phase 3A-Y Roll Response Model

The roll model uses arm activity, left/right asymmetry, lateral-motion proxy, pelvis roll and roll rate. Symmetric active motion allocates explicit waist-roll authority; unilateral motion returns continuously to the frozen ankle-roll distribution. A 0.10 rad/s arm-energy deadband prevents landing and base-perturbation noise from activating the gesture model.

The model restored heart waist-roll excursion from 0.012× to an order-one response without using a motion label. Safety scaling and slew limiting are applied afterward.
