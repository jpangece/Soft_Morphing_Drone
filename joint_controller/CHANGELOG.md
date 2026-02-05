# CHANGELOG

## Current-Control Based Compliant Morphing (Major Update)

### Summary
Replaced position-based servo control with current-based control and redesigned the controller logic to achieve compliant, backdrivable behavior during external push while ensuring fast and stable return to the zero reference without oscillation.

---

### Problem Observed in Initial Implementation

1. **Position control stiffness**
   - Using `Goal_Position` in Position Mode caused strong holding torque.
   - Servo resisted manual rotation and felt very stiff.

2. **Oscillation (limit-cycle) near zero in early current-control attempt**
   - Applying continuous virtual spring-damper torque around the zero reference caused oscillatory left-right motion.
   - Gear backlash, friction, and sensor quantization amplified this effect.
   - Present_Current-based external torque estimation interfered with control stability.

---

### Root Causes

- Position mode inherently enforces holding torque, preventing backdrivability.
- In current mode, continuously applying corrective torque at the reference creates limit-cycle oscillation due to mechanical backlash.
- External force estimation using Present_Current mixes commanded current with real disturbance, degrading detection reliability.

---

### Final Solution

Implemented a state-based controller with three implicit behaviors:

1. **Push behavior (compliance)**
   - When the user pushes the arm (detected via error growth and velocity), commanded current is set to zero.
   - Enables smooth manual rotation with minimal resistance.

2. **Return behavior (fast recovery)**
   - When external push stops, a PD current controller drives the servo back to zero quickly.

3. **Latch behavior near zero (oscillation prevention)**
   - When the position crosses or approaches zero, current is set to zero.
   - Prevents continuous holding torque and eliminates oscillation.
   - Controller reactivates only if the arm drifts far from zero.

---

### Key Architectural Changes

- Switched Dynamixel Operating Mode from **Position Control** to **Current Control**.
- Replaced `Goal_Position` SyncWrite with `Goal_Current`.
- Removed continuous holding behavior near reference.
- Replaced Present_Current-based push detection with kinematic detection (error growth + velocity).
- Introduced zero-cross capture and latch window to suppress limit-cycle oscillation.

---

### Resulting Behavior

- The arm can be pushed smoothly by hand (high compliance).
- Upon release, the arm quickly returns to the zero position.
- No oscillation or hunting occurs near zero.
- Stable and predictable behavior suitable for soft–active morphing mechanism.
