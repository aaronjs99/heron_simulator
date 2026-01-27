# Heron Simulation — Nodes

Detailed documentation for scripts and nodes in the `heron_simulator` package.

---

## Autonomy

### autonomy/mock_defector_service.py

**Simulated Inspection Service**

Provides a mock `/defector/capture_and_analyze` service for testing the full autonomy stack without real cameras.

#### Overview

- Simulates inspection delay (configurable)
- Returns mock detection results
- Logs "inspection" events

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~delay` | float | `2.0` | Simulated capture delay (seconds) |
| `~success_rate` | float | `1.0` | Probability of successful capture |

#### Services Provided

| Service | Type | Description |
|---------|------|-------------|
| `/defector/capture_and_analyze` | `std_srvs/Trigger` | Trigger mock inspection |

---

### autonomy/spawn_inspection_models.py

**Dynamic Model Spawner**

Spawns inspection target models (pillars, pipes) into the Gazebo world.

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~anchor_file` | string | `slam_grande/data/anchors.yaml` | Anchor definitions |
| `~spawn_delay` | float | `0.5` | Delay between spawns |

#### Models Spawned

Based on anchor types in `anchors.yaml`:
- `pillar` → Cylindrical models
- `pipe` → Pipe/tube models
- `home` → Dock marker

---

## Sensors

### sensors/scan_to_cloud.py

**LaserScan to PointCloud2 Converter**

Converts Gazebo LaserScan topics to PointCloud2 for compatibility with Mariner's costmap configuration.

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~scan_topic` | string | `/lidar_h/scan` | Input LaserScan topic |
| `~cloud_topic` | string | `/lidar_h/velodyne_points` | Output PointCloud2 topic |

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/lidar_h/scan` | `sensor_msgs/LaserScan` | Simulated horizontal lidar |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/lidar_h/velodyne_points` | `sensor_msgs/PointCloud2` | Converted point cloud |

---

### sensors/vel_cov_fixed.py

**Velocity Covariance Publisher**

Adds covariance information to velocity messages for robot_localization.

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/navsat/vel` | `geometry_msgs/TwistStamped` | GPS velocity |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/navsat/vel_cov` | `geometry_msgs/TwistWithCovarianceStamped` | Velocity with covariance |

---

### sensors/navsat_vel_translate.py

**GPS Velocity Translator**

Converts GPS velocity from NWU (North-West-Up) to ENU (East-North-Up) convention.

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/navsat/velocity` | `geometry_msgs/Vector3Stamped` | Velocity in NWU |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/navsat/vel` | `geometry_msgs/TwistStamped` | Velocity in ENU |

---

### sensors/rpy_translator.py

**Quaternion to RPY Translator**

Converts quaternion orientation to Roll-Pitch-Yaw for magnetometer calibration.

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/imu/data` | `sensor_msgs/Imu` | IMU with quaternion |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/imu/rpy` | `geometry_msgs/Vector3Stamped` | Roll, Pitch, Yaw |

---

### sensors/vector3_to_magnetic_field.py

**Magnetometer Message Converter**

Converts Vector3Stamped magnetometer data to MagneticField message.

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/imu/mag_sim` | `sensor_msgs/MagneticField` | Simulated magnetometer |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/imu/mag_raw` | `geometry_msgs/Vector3Stamped` | Raw magnetic field |
| `/imu/mag` | `sensor_msgs/MagneticField` | Calibrated field |

---

## Control

### control/cmd_drive_translate.py

**Propulsion Force Translator**

Translates normalized `cmd_drive` efforts from the `heron_controller` into physical Newtonian forces (Wrenches) applied to the Gazebo thruster links. This module implements the empirical thrust interpolation model.

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_drive` | `heron_msgs/Drive` | Normalized drive efforts [-1.0, 1.0] |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/thrusters/0/input` | `geometry_msgs/Wrench` | Calculated force for the Right thruster |
| `/thrusters/1/input` | `geometry_msgs/Wrench` | Calculated force for the Left thruster |

---

### control/twist_translate.py

**Interactive Marker Velocity Scaler**

Scales interactive marker velocities to match robot capabilities.

#### Parameters

Configured via `config/heron_controller.yaml`:
- Linear velocity: Scaled to max robot speed
- Angular velocity: Scaled to ±1.1 rad/s

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/twist_marker_server/cmd_vel` | `geometry_msgs/Twist` | Raw marker velocity |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Scaled velocity |

---

### control/activate_control_service.py

**Control Activation Service**

Enables/disables the control interface for the Heron.

#### Services Provided

| Service | Type | Description |
|---------|------|-------------|
| `/heron/activate_control` | `std_srvs/SetBool` | Enable/disable control |

---

### control/vessel_dynamics.py

**Fossen Hydrodynamic Model**

High-fidelity vessel dynamics engine implementing Fossen's equations of motion for realistic USV behavior.

#### Overview

Computes hydrodynamic forces applied to the Heron hull:
- **Damping**: Linear + quadratic drag (stiffened sway/yaw for rail-like handling)
- **Buoyancy**: Hydrostatic lift based on displaced volume
- **Coriolis**: Rigid-body coupling during turns
- **Restoring moments**: Metacentric roll/pitch stability

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~namespace` | string | `heron` | Robot namespace for topics |

#### Physical Model

```
M·dv/dt + C(v)·v + D(v)·v + g(η) = τ
```

| Coefficient | Surge | Sway | Heave | Roll | Pitch | Yaw |
|-------------|-------|------|-------|------|-------|-----|
| Linear Damping | 30 | 80 | 100 | 20 | 50 | 50 |
| Quadratic Damping | 15 | 50 | 100 | 15 | 50 | 50 |
| Added Mass | 20 | 40 | 40 | 0 | 20 | 20 |

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/ground_truth/odom` | `nav_msgs/Odometry` | Current vessel state |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/{namespace}/hydro_forces` | `geometry_msgs/Wrench` | Computed hydrodynamic forces |

---

## Launch Files

### simulation_full.launch

**Full Stack Simulation**

Launches complete simulation with all SLAM GRANDE components.

**Includes:**
- Gazebo world
- Heron spawn
- Mariner navigation
- Oracle mission planning
- Mock defector service

### spawn_heron.launch

**Robot Spawner**

Spawns the Heron model with sensors.

**Arguments:**
| Argument | Default | Description |
|----------|---------|-------------|
| `x` | `0.0` | Initial X position |
| `y` | `0.0` | Initial Y position |
| `yaw` | `0.0` | Initial heading |
| `namespace` | `` | Robot namespace |
