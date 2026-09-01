# Phase 3A-X final gate

1. Phase 3A-R failed to generalize because fixed additive allocation had no constraint state and heart/wave required different authority distribution.
2. Additive balance lacked target envelope, contact prediction, actuator authority, slew and arbitration.
3. Pelvis/hip contact can be predicted locally before contact: YES.
4. Limit-aware alone is partial; combined warning plus hard envelope is effective.
5. Contact-aware is effective: wave arm contact samples 687 -> 0.
6. Saturation-aware alone is ineffective; combined upstream prevention is effective.
7. Pitch/roll separation improves isolation, not proven response generalization.
8. Eligibility-safe allocation is safer, but response superiority to fixed allocation is not proven.
9. Wave arm-only has true tested positive margin: 1.134 mm nominal, 0.819 mm worst perturbation.
10. Wave whole-body no longer falls and has no contact, limit violation, or persistent saturation.
11. Shoulder/wrist tracking improvement is retained: YES.
12. Heart and wave both pass hard safety/tracking, but not both pass response similarity.
13. Perturbation robustness passes locally: 8/8.
14. VALIDATED_SIM_CONTROLLER_BASELINE = NO.

`ARM_TRACKING_GENERALIZES = YES`  
`CONTACT_SAFETY_ROBUST = YES`  
`LIMIT_MANAGEMENT_ROBUST = YES`  
`SATURATION_MANAGEMENT_ROBUST = YES`  
`BALANCE_GENERALIZES_HEART_AND_WAVE = NO`  
`WHOLE_BODY_STRESS_TEST_PASSES = YES`  
`PERTURBATION_ROBUSTNESS = YES`  
`REHEARSAL_12_OF_12_SETTLED = YES`  
`VALIDATED_SIM_CONTROLLER_BASELINE = NO`  
`DYNAMICS_CALIBRATION_READY = NO`  

Do not start physical-model tuning. Phase 2H evidence gates remain unchanged.
