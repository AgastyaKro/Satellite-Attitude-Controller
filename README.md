# Satellite Attitude Controller

This project simulates a satellite's attitude stabilization using independent **PID controllers** for each rotational axis. The controller applies torque via actuators (e.g., gas thrusters) to align the satellite’s orientation with a target quaternion. Orientation is represented and integrated using quaternions to avoid gimbal lock and ensure smooth 3D rotation.

---

## Visualizations

For these visualizations, the satellite starts at 0 degrees on all axes. The target orientation is a 90-degree rotation around the y-axis.

---

### Component-Wise Orientation

<p align="center">
  <img src="assets/Satellite-Components.png" width="600"/>
</p>

## Overview

* **State Representation:**
  The system tracks:

  * Angular velocity vector `ω = [ωx, ωy, ωz]`
  * Orientation quaternion `q = [w, x, y, z]`
  * Quaternion error `q_err = q_target ⊗ q_current⁻¹`
  * Angle-axis representation extracted from `q_err` to compute PID error per axis

* **Dynamics:**
  Simulates satellite rotation using **Euler’s rotational equations** for rigid body dynamics. Quaternion orientation is integrated over time to update the satellite’s pose.

* **Controller:**
  A **separate PID controller** (Proportional-Integral-Derivative) is used for each axis (roll, pitch, yaw):

  * Proportional term reduces immediate error
  * Integral term addresses steady-state error
  * Derivative term damps the response
  * Each controller operates on the angle-axis error components

---

## Simulation Output

* The simulation logs:

  * Time
  * Orientation quaternion `[w, x, y, z]`
  * Angular velocity `[ωx, ωy, ωz]`
  * PID errors (per axis)
  * Commanded torque (per axis)
  * Cumulative orientation error


## How to Run

1. **Build the project** (CMake):

   ```bash
   mkdir build && cd build
   cmake ..
   make
   ./SatelliteController
   ```

---
