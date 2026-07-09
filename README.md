# h1-mocap-retargeting
# H1 Thrusters

Motion-capture-based simulation and control of a thruster movement on the Unitree H1 humanoid robot.

## Overview

This project focuses on making the Unitree H1 perform a **thruster** movement in simulation.

A thruster combines:

* Squat
* Overhead press

## Project Goal

The goal is to retarget human motion capture data to the H1 robot, simulate the motion in MuJoCo, and improve its feasibility using stability analysis and optimal control.

## Pipeline

Motion capture data
        ↓
Retarget motion to H1 kinematics
        ↓
Visualize retargeted motion
        ↓
Clean and check motion feasibility
        ↓
Analyze stability using CoM / support polygon / ZMP
        ↓
Simulate in MuJoCo
        ↓
Improve motion using optimal control
        ↓
Extend from squat to full thruster


## Repository Structure

```text
h1-thrusters/
│
├── data/           # Motion capture data and processed motion files
├── models/         # Unitree H1 model files
├── scripts/        # Python scripts for processing, simulation, and analysis
├── results/        # Generated plots, videos, motions, and reports
├── docs/           # Project notes and documentation
│
├── requirements.txt
└── README.md
```
## Planned Work

### Phase 1: Squat Motion

* Collect motion capture data
* Retarget squat motion to H1 kinematics
* Visualize the retargeted motion
* Clean the motion and check feasibility
* Analyze stability using CoM, support polygon, and ZMP
* Simulate the retargeted motion in MuJoCo
* Apply optimal control between selected keyframes

### Phase 2: Full Thruster Motion

* Add the overhead press
* Include dumbbell or barbell interaction
* Repeat stability and feasibility analysis
* Improve the full motion using optimal control

## Contributors

* Rokaia Ibrahem
* Till Laube
