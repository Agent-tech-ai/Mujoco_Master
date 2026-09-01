# Phase 3A-Y Motion-Condition Analysis

## Measured input differences

| dataset | duration_s | mean_arm_energy_rad_s | peak_arm_energy_rad_s | mean_asymmetry | p90_asymmetry | peak_arm_acceleration_rad_s2 | mean_abs_sagittal_proxy | mean_abs_lateral_proxy | peak_chest_pitch_rad | peak_chest_roll_rad | peak_chest_gyro_rad_s |
|---|---|---|---|---|---|---|---|---|---|---|---|
| heart | 5.65942 | 2.42853 | 8.77173 | 0.0283248 | 0.0812767 | 20.0232 | 0.477971 | 0.0587527 | 0.0135645 | 0.0062364 | 0.101673 |
| wave | 4.34915 | 2.02262 | 3.71597 | 0.95453 | 0.989569 | 25.0147 | 1.109 | 1.19996 | 0.0122468 | 0.0367824 | 0.354874 |

Heart is bilateral and nearly symmetric; wave is unilateral and highly asymmetric. This is measured, not inferred from the preset name: the final controller sees only live arm energy and left/right asymmetry. The two motions also differ in sagittal/lateral velocity proxies, onset, duration, and measured torso response.

## Why the response distribution differs

- Heart's symmetric, high-energy arm motion produces a measurable pitch disturbance while net left/right asymmetry remains low. The real response spreads into ankle pitch and a small but nonzero waist-roll response.
- Wave's unilateral arm motion produces high asymmetry and stronger roll coupling. The safest simulation response retains the frozen Phase 3A-X distribution; applying the heart allocation blindly causes contact.
- The large wave knee ratio is dominated by small real knee excursion plus passive simulated leg coupling. It is not evidence that the real MC commands eight times more knee motion.

Phase markers were derived from measured arm energy: `{"heart": {"arm_energy_threshold_rad_s": 0.4385866049869931, "offset_s": 4.0799999999998064, "onset_s": 0.019999999999892992, "peak_s": 3.3399999999998222}, "wave": {"arm_energy_threshold_rad_s": 0.18579828096976336, "offset_s": 4.2199999999998035, "onset_s": 0.039999999999892566, "peak_s": 3.2799999999998235}}`.
