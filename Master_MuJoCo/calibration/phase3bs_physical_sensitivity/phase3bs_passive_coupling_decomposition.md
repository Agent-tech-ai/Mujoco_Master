# Wave right-knee passive-coupling decomposition

All non-normal runs are **DIAGNOSTIC_ONLY** and are not controller candidates.

| case | fixed_base | contact_retained | balance_feedback_enabled | knee_actuator_enabled | observed_until_s | right_knee_excursion_rad_before_failure | right_knee_peak_abs_velocity_rad_s | stable_no_fall | fall_time_s | minimum_limit_margin_rad | persistent_saturation_fraction | classification |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| A_NORMAL_FREE_BASE_CONTACT_RETAINED | False | True | True | True | 4.34915 | 0.082092 | 0.138657 | True |  | 0.0462312 | 0 | DIAGNOSTIC_ONLY_NOT_CONTROLLER_CANDIDATE |
| B_FIXED_BASE_CONTACT_RETAINED | True | True | True | True | 4.34915 | 1.77013e-05 | 7.87587e-05 | True |  | 0.0463763 | 0 | DIAGNOSTIC_ONLY_NOT_CONTROLLER_CANDIDATE |
| C_BALANCE_DISABLED_DIAGNOSTIC | False | True | False | True | 1.863 | 0.0273582 | 0.172884 | False | 1.863 | -0.0397942 | 0.793872 | DIAGNOSTIC_ONLY_NOT_CONTROLLER_CANDIDATE |
| D_KNEE_ACTUATOR_CHANNEL_DISABLED | False | True | True | False | 0.393 | 0.000175692 | 0.00151665 | False | 0.393 | -0.0788489 | 0.00696379 | DIAGNOSTIC_ONLY_NOT_CONTROLLER_CANDIDATE |
| F_FIXED_BASE_CONTACT_FREE_DIAGNOSTIC | True | False | True | True | 4.34915 | 1.77013e-05 | 7.87587e-05 | True |  | 0.0463763 | 0 | DIAGNOSTIC_ONLY_NOT_CONTROLLER_CANDIDATE |

## Interpretation

- Fixing the base changes right-knee excursion from 0.082092 to 0.000018 rad (-100.0%). This quantifies the base/contact pathway under retained contacts.
- Disabling balance feedback produces 0.027358 rad before failure and then falls; the unstable tail is excluded. It proves the frozen balance loop is safety-critical and cannot serve as an alternative controller.
- Disabling the knee actuator produces only 0.000176 rad before an early failure at 0.393 s. Because the run fails before most of the Wave response develops, it confirms that the channel is safety-critical but does **not** quantify a full passive residual.
- Fixed-base contact-free gives 0.000018 rad versus 0.000018 rad with contact. The diagnostic is technically limited: fixing the base removes the global support dynamics whose contact contribution is of interest. A free-base contact-free run would simply be unsupported and is not meaningful.
- Controller reference contribution remains observable in the normal joint/command decomposition logs; MC internal command and real torque are not used.

## Causal ranking from decomposition

1. **Closed-loop base/contact/leg coupling — PRIMARY**: fixed-base changes response; disabling balance causes a fall.
2. **Direct knee balance actuation plus passive leg mechanics — SECONDARY/PARTIAL**: isolation fails too early to separate these components reliably.
3. **Pure passive mechanical residual — UNKNOWN**: the disabled-channel run is truncated before a comparable Wave interval.
4. **Contact reaction alone — UNKNOWN/PARTIAL**: fixed/contact-free comparison is conditioned on an artificial fixed base.
5. **Hip versus ankle sub-path — UNKNOWN**: not isolated independently because doing so would invalidate the safety baseline.
