# Satellite Attitude Control Simulator

A C++ simulation of a satellite’s attitude (3D orientation) control system using quaternions, torque modeling, and a PID feedback loop. The simulator models how satellites use actuators like reaction wheels to control their orientation in space — a core feature of all spacecraft.

## Overview

This project simulates:
- The rotational dynamics of a satellite in free space
- The application of torques via virtual actuators
- A PID controller that drives the satellite to a target orientation
- Quaternion-based orientation updates to avoid gimbal lock

This kind of system is found in real spacecraft, from CubeSats to Starlink satellites, and is critical for camera aiming, antenna pointing, and solar alignment.

---

## Features

- Quaternion-based orientation tracking  
- Rigid body physics with moment of inertia  
- Time-stepped simulation loop  
- Virtual torque input  
- PID controller (in progress)  
- Optional 3D visualization (coming soon)  

---

## Core Concepts

- **Quaternions** for stable 3D orientation representation  
- **Torque → Angular Acceleration → Angular Velocity → Orientation**  
- **Moment of Inertia Tensor** to model realistic rotational inertia  
- **PID Control** to stabilize and reorient the satellite  

---

## Demo (Sample Output)

## Project Layout

SatelliteSim/
├── main.cpp               # Simulation runner
├── Satellite.hpp/.cpp     # Satellite physics and control
├── Controller.hpp/.cpp    # PID control system (coming soon)
├── Quaternion.hpp/.cpp    # Quaternion math wrapper (using Eigen)
├── utils.hpp              # Timestep helpers and math
├── CMakeLists.txt         # CMake project setup