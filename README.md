# Soft Half-Passive / Half-Active Morphing Drone

## Objective

This project develops a morphing quadrotor that combines:

- passive soft-arm deformation for collision and squeezing
- active servo-controlled deformation at the arm ends

The drone initially relies on passive morphing in unknown environments.  
Once interaction force feedback is available, the system will switch to
admittance-based morphology control using servos.

The servos at the arm ends also provide indirect force/torque information
through their load response.

---

## Current Mechanical Design (Jan 2026)

The arm consists of:

- A dual-layer soft segment located at half arm length
- A rigid support structure using a chain–bearing–pin linkage
- A servo position at the arm end (currently replaced by a rigid block for flight testing)

Soft segment parameters:

- Length: 5 cm
- Thickness: 4 mm
- Dual layer

---

## Key Design Insights

Two critical requirements were identified for soft drone arms:

1. The structure must resist rotor yaw torque (horizontal direction)
2. The structure must resist thrust-induced vertical deformation

Instead of using anisotropic materials, the design uses:

> fully deformable soft material + rigid skeleton to suppress vertical deformation

An important observation is:

> soft stiffness is inversely proportional to the cube of its length

This strongly affects the required deformation force.

---

## Verified Parameters

| Parameter | Value |
|---|---|
| Servo rated torque | 0.52 Nm |
| Rotor yaw torque | ~0.05 Nm |
| Current full compression force | 2–3 N |
| Equivalent torque | 0.2–0.3 Nm |
| Drone mass | 600 g |
| Maximum thrust | ~6 N |

---

## Intended Control Architecture

- PX4 handles flight stabilization
- Raspberry Pi handles servo morphology control
- External force feedback will drive admittance-based servo motion

