# Change Log – Soft Segment Design

This log tracks the evolution of the soft segment used in the drone arm.

---

## Version 1 — Rectangular Shape

- Simple rectangular soft piece
- No cuts or shape optimization
- Too stiff for meaningful deformation
- Did not allow sufficient morphing

---

## Version 2 — Cut Shape

![Version 2](images/soft4mm_ver2.jpg)

- Introduced cut geometry to increase flexibility
- Deformation improved significantly

Problem observed:
- When the rotor (propeller) spins, the arm becomes **extremely unstable**
- Severe shaking and oscillation due to excessive softness
- Not flyable

Conclusion:
> Structure is too soft to resist rotor-induced yaw torque and vibration

---

## Version 3 — Revised Cut Shape (soft4mm_ver3.jpg)

![Version 3](images/soft4mm_ver3.jpg)

- Geometry adjusted to recover some stiffness
- Rotor-induced vibration reduced
- Still severe shaking and oscillation
- 
Remaining issue:
- Still slightly softer than desired
- Deformation force is higher than optimal relative to available thrust

Conclusion:
> Retry with version 1 for flight experiment
