# Phase 3A-X failure-chain baseline

Legacy implementation is torque-additive:

```text
tau = PD(q_reference - q) + bias + friction + tau_balance
```

It lacks target-envelope, contact-distance, actuator-authority, slew and arbitration
state. Observed chain: `tracking -> contact -> limit -> balance excursion -> saturation -> fall`.

- Phase 3A-R wave whole-body fall: `7.450 s`
- legacy minimum joint margin: `-0.06282 rad`
- legacy pelvis/hip penetration: `2.575 mm`
- Phase 3A-X fall: `none`

`LEGACY_BALANCE_ARCHITECTURE` remains available for comparison; production
`master_sim/controller.py` was not edited.
