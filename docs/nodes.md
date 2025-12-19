# Heron Simulation — Nodes

Detailed documentation for scripts and nodes in the `heron_simulator` package.

---

## SLAM GRANDE Extensions

### mock_defector_service.py

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

### scan_to_cloud.py

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

### sim_odom_publisher.py

**Simulation Odometry Publisher**

Bridges Gazebo model state to standard odometry for the navigation stack.

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Robot odometry |

#### Published TF

| Transform | Description |
|-----------|-------------|
| `odom` → `base_link` | Robot pose from simulation |

---

### spawn_inspection_models.py

**Dynamic Model Spawner**

Spawns inspection target models (pillars, pipes) into the Gazebo world.

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `~anchor_file` | string | `oracle/data/anchors.yaml` | Anchor definitions |
| `~spawn_delay` | float | `0.5` | Delay between spawns |

#### Models Spawned

Based on anchor types in `anchors.yaml`:
- `pillar` → Cylindrical models
- `pipe` → Pipe/tube models
- `home` → Dock marker

---

### vel_cov_fixed.py

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

### verify_movement.py

**Motion Verification Utility**

Monitors robot movement for debugging navigation issues.

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/debug/is_moving` | `std_msgs/Bool` | True if robot is moving |

---

## Upstream Clearpath Scripts

Scripts from the original [heron_simulator](https://github.com/heron/heron_simulator) package.

### cmd_drive_translate

**Thruster Command Translator**

Translates `cmd_drive` topic from heron_controller to UUV Simulator thruster topics.

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_drive` | `heron_msgs/Drive` | Drive command |

#### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/thrusters/0/input` | `uuv_thruster_msgs/Float64` | Right thruster |
| `/thrusters/1/input` | `uuv_thruster_msgs/Float64` | Left thruster |

---

### navsat_vel_translate

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

### rpy_translator

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

### twist_translate

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

### vector3_to_magnetic_field

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

### activate_control_service

**Control Activation Service**

Enables/disables the control interface for the Heron.

#### Services Provided

| Service | Type | Description |
|---------|------|-------------|
| `/heron/activate_control` | `std_srvs/SetBool` | Enable/disable control |

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
