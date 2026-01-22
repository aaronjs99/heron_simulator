# HERON Simulator
**Fidelity-Adaptive Digital Twin Architecture** — High-fidelity Gazebo simulation environment for the clear-Heron USV, supporting multi-level fidelity abstraction.

| | |
|:---:|:---:|
| **Package** | `heron_simulator` |
| **System Interface** | MARINER / ORACLE |
| **Maintainer** | Aaron JS |

## Overview
This package provides a Gazebo-based digital twin of the Heron USV platform. Architecture facilitates "Adaptive Fidelity Simulation" where computational expensive physics or high-resolution sensor rendering can be modulated based on operational requirements—ranging from rapid mission logic verification to full Hardware-In-The-Loop (HIL) validation.

## Functional Capabilities
- **Procedural Environment Generation**: Dynamically instantiates structural inspection targets (e.g., maritime infrastructure, navigational aids) based on the `anchors.yaml` semantic map.
- **On-Demand Perception Emulation**: Provides high-performance mock interfaces for vision-based defect detection when computational resources are prioritize for control or planning.
- **Stochastic Disturbance Modeling**: Simulates complex maritime environment dynamics, including fluid currents and aerodynamic wind disturbances.
- **Multi-Modal Sensor Synthesis**: 
    - Stereoscopic Vision (Fisheye geometry emulation)
    - Inertial Navigation (IMU)
    - Global Positioning and Odometry (GPS/INS)

## Analytical Modules
| Module | Category | Functional Description |
|------|----------|-------------|
| `autonomy/spawn_inspection_models.py` | Autonomy | Translates semantic map definitions into runtime SDF entities within the Gazebo world. |
| `autonomy/mock_defector_service.py` | Autonomy | Provides synthetic perception feedback derived from ground-truth Gazebo states for resource-constrained evaluation. |
| `sensors/scan_to_cloud.py` | Sensors | Facilitates the transformation of 1D laser data into 3D point cloud representations for the MARINER stack. |
| `control/cmd_drive_translate.py` | Control | Maps high-level drive efforts to metric thruster force vectors. |

## Operational Deployment
```bash
# Initialize full-fidelity simulation environment
roslaunch heron_simulator simulation.launch

# Execute manual model instantiation
rosrun heron_simulator spawn_inspection_models.py
```

## Configuration and Parameterization
Simulation parameters are governed via `config/simulation.yaml`:

```yaml
water_level: 0.0
wave_amplitude: 0.1
prop_density: "medium" # low, medium, high
```
