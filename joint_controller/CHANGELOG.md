# CHANGELOG

## Current-Control Based Compliant Morphing (Major Update)

### Summary

Replaced position-based servo control with current-based control and redesigned the controller logic to achieve compliant, backdrivable behavior during external push while ensuring fast and stable return to the zero reference without oscillation.

---

### Problem Observed in Initial Implementation

1. **Position control stiffness**
   - Using `Goal_Position` in Position Mode caused strong holding torque.
   - The servo actively resisted manual rotation and felt mechanically stiff.

2. **Oscillation (limit-cycle) near zero in early current-control attempt**
   - Applying continuous virtual spring–damper torque around the zero reference caused oscillatory left-right motion.
   - Gear backlash, friction, and sensor quantization amplified this effect.
   - Present_Current-based external torque estimation interfered with control stability.

---

### Root Causes

- Position mode inherently enforces holding torque, preventing backdrivability.
- In current mode, continuously applying corrective torque at the reference creates a **limit-cycle oscillation** due to mechanical backlash and quantization.
- External force estimation using Present_Current mixes commanded current with real disturbance, degrading detection reliability.

---

## Mathematical and Physical Background

Dynamixel current control directly corresponds to torque control:

$$
\tau = K_t I
$$

where

- $\tau$ is motor torque (Nm)  
- $K_t$ is the torque constant (Nm/A)  
- $I$ is motor current (A)

This allows implementation of a **virtual impedance model**:

$$
\tau = -K(\theta - \theta_{ref}) - B\dot{\theta}
$$

which leads to the current command:

$$
I = \frac{-K(\theta - \theta_{ref}) - B\dot{\theta}}{K_t}
$$

This behaves as a virtual spring–damper system around the reference position.

However, in a real geared servo system with backlash, friction, and sensor quantization, continuously applying this restoring torque near

$$
\theta \approx \theta_{ref}
$$

causes the torque direction to flip repeatedly as the error sign changes, producing a **limit-cycle oscillation**.

The key insight was that **holding torque near the reference must be removed, rather than tuned**.

---

## Final Control Strategy

A state-based controller was implemented with three implicit behaviors:

### 1. Push Behavior (High Compliance)

When the user manually pushes the arm, detected via:

- increasing position error magnitude
- sufficiently large angular velocity

the commanded current is set to zero:

$$
I = 0
$$

This enables smooth manual rotation with minimal resistance (true backdrivability).

---

### 2. Return Behavior (Fast Recovery)

When the external push stops, the controller switches to virtual impedance return:

$$
I = \frac{-K(\theta - \theta_{ref}) - B\dot{\theta}}{K_t}
$$

This drives the arm quickly back to the zero reference.

---

### 3. Latch Behavior Near Zero (Oscillation Prevention)

When the position:

- crosses the zero reference, or
- enters a small neighborhood around zero with low velocity,

the controller **disables holding torque**:

$$
I = 0
$$

This prevents continuous corrective torque and eliminates limit-cycle oscillation.

The controller reactivates only if the arm drifts sufficiently far from the reference.

---

## Key Architectural Changes

- Switched Dynamixel Operating Mode from **Position Control** to **Current Control**.
- Replaced `Goal_Position` SyncWrite with `Goal_Current`.
- Removed continuous holding behavior near reference.
- Replaced Present_Current-based push detection with kinematic detection (error growth + velocity).
- Introduced zero-cross capture and latch window to suppress limit-cycle oscillation.

---

## Resulting Behavior

- The arm can be pushed smoothly by hand (high compliance).
- Upon release, the arm quickly returns to the zero position.
- No oscillation or hunting occurs near zero.
- The behavior matches the physical characteristics required for soft–active morphing mechanisms.
