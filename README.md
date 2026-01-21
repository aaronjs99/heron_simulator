# HERON Simulator
**Fidelity-Adaptive Gazebo Back-End** — Simulation environment for the HERON USV, supporting multiple fidelity levels.

| | |
|:---:|:---:|
| **Package** | `heron_simulator` |
| **System** | MARINER / ORACLE |
| **Maintainer** | Aaron JS |

## Overview
This package provides a Gazebo-based digital twin of the Heron USV. It supports "Lazy Simulation" where heavy physics or sensor rendering can be toggled off for rapid mission testing, or enabled for full Hardware-In-The-Loop (HIL) verification.

## Features
- **Dynamic Spawning**: Procedurally generates inspection targets (e.g., buoys, pipes) based on `config/anchors.yaml`.
- **Mock Interfaces**: Provides service stubs for the Defect Detector when GPU resources are constrained.
- **Environment Control**: Simulates water currents and wind disturbances.
- **Sensor Simulation**: Replicates:
    - Fisheye Stereo Cameras (approximate FOV)
    - Inertial Measurement Unit (IMU)
    - GPS / Odometry

## Nodes
| Node | Description |
|------|-------------|
| `spawn_inspection_models.py` | Reads the semantic map and spawns Gazebo models (SDF) at runtime. |
| `mock_defector_service.py` | Stand-in for the heavy Deep Learning detector; returns synthetic defect detections relative to ground truth models. |
| `fake_localize.py` | Publishes perfect ground-truth odometry (for debugging state estimation). |

## Usage
```bash
# Launch full simulation with sensors
roslaunch heron_simulator simulation.launch

# Spawn models manually
rosrun heron_simulator spawn_inspection_models.py
```

## Configuration
Spawning parameters in `config/simulation.yaml`:

```yaml
water_level: 0.0
wave_amplitude: 0.1
prop_density: "medium" # low, medium, high
```
