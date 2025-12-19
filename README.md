# Heron Simulation

Gazebo simulation for the Heron USV, integrated with the full SLAM GRANDE stack (Oracle, Mariner, Defector).

[![ROS](https://img.shields.io/badge/ROS-Noetic-blue)](http://wiki.ros.org/noetic)
[![Gazebo](https://img.shields.io/badge/Gazebo-11-orange)](http://gazebosim.org/)
[![License](https://img.shields.io/badge/license-BSD%20%2B%20MIT-lightgrey)]()

## Attribution

This package is based on [heron_simulator](https://github.com/heron/heron_simulator) by Clearpath Robotics. The original package provides Gazebo simulation for the Heron USV using the UUV Simulator framework.

**Original Authors:** Guy Stoppi, Yan Ma, Mike Purvis (Clearpath Robotics)

**SLAM GRANDE Extensions:** Aaron John Sabu (UCLA)

## Documentation

| Document | Description |
|----------|-------------|
| [docs/nodes.md](docs/nodes.md) | Script and node documentation |
| [docs/hydrodynamics.md](docs/hydrodynamics.md) | Physics model details |

---

## Installation

### Prerequisites

```bash
# Gazebo and ROS packages
sudo apt install ros-noetic-gazebo-ros ros-noetic-gazebo-plugins ros-noetic-joint-state-publisher

# UUV Simulator (hydrodynamics)
sudo apt install ros-noetic-uuv-simulator ros-noetic-uuv-gazebo-worlds

# Heron packages
sudo apt install ros-noetic-heron-description ros-noetic-heron-msgs ros-noetic-heron-controller

# IMU and sensors
sudo apt install ros-noetic-imu-filter-madgwick ros-noetic-imu-tools ros-noetic-hector-gazebo-plugins

# Interactive control
sudo apt install ros-noetic-interactive-marker-twist-server

# Localization
sudo apt install ros-noetic-robot-localization

# Install all dependencies at once
rosdep install --from-paths src --ignore-src --rosdistro=noetic -yr
```

**Required packages in workspace:** `heron`, `heron_controller`, `heron_desktop`, `imu_tools`

---

## Running the Simulation

### SLAM GRANDE Full Stack

```bash
roslaunch heron_simulator simulation_full.launch
```

### Upstream Clearpath Stack

```bash
# Ocean world
roslaunch heron_simulator upstream_heron_world.launch

# Lake world  
roslaunch heron_simulator upstream_heron_lake_world.launch

# Custom world from another package
roslaunch heron_simulator upstream_heron_world.launch \
    world_pkg:=uuv_descriptions world_pkg_file:=ocean_waves.launch

# Custom world from absolute path
roslaunch heron_simulator upstream_heron_world.launch \
    use_pkg_path:=0 world_file:=/path/to/world.world
```

### Available UUV Simulator Worlds

Using `world_pkg:=uuv_descriptions`:
- `ocean_waves.launch`
- `mangalia.launch`
- `munkholmen.launch`
- `lake.launch`
- `empty_underwater_world.launch` *(very dark)*
- `herkules_ship_wreck.launch` *(very dark)*

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `gui` | `true` | Show Gazebo GUI |
| `paused` | `false` | Start simulation paused |
| `x` | `0` | Initial X position (meters) |
| `y` | `0` | Initial Y position (meters) |
| `yaw` | `0` | Initial heading (radians) |
| `namespace` | `` | Robot namespace (for multi-robot) |
| `hydro_debug` | `0` | Enable hydrodynamics debug topics |
| `use_oracle` | `true` | Enable SLAM GRANDE Oracle |
| `use_mariner` | `true` | Enable SLAM GRANDE navigation |
| `llm_model` | `llama3` | LLM model for Oracle |

---

## Multi-Robot Simulation

Multiple Herons can be simulated with unique namespaces:

```bash
# Terminal 1: Launch world only
roslaunch heron_simulator gazebo_world.launch

# Terminal 2: First Heron (default namespace)
roslaunch heron_simulator upstream_spawn_heron.launch

# Terminal 3: Second Heron
roslaunch heron_simulator upstream_spawn_heron.launch namespace:=heron1 x:=5.0

# Terminal 4: Third Heron
roslaunch heron_simulator upstream_spawn_heron.launch namespace:=heron2 x:=10.0
```

**Note:** Empty namespace and `heron` namespace cannot be used simultaneously (thruster topic conflict).

---

## Control

### Control Interfaces

| Method | Topic | Type | Description |
|--------|-------|------|-------------|
| Oracle NL | `/oracle/human_query_in` | String | Natural language commands |
| Nav Goal | `/mariner/final_pose` | PoseStamped | Navigation goal |
| Velocity | `/cmd_vel` | Twist | Velocity command |
| Helm | `/cmd_helm` | Helm | Course/speed command |
| Wrench | `/cmd_wrench` | Wrench | Force/torque command |
| Direct | `/cmd_drive` | Drive | Direct thruster command |

### RViz Interactive Markers

The Heron can be controlled using interactive markers in RViz:
- One marker controls forward/backward motion
- Another marker controls rotation

**Recommended frame:** `[namespace]/base_link`
- `odom` frame shows world position but fluctuates with GPS updates
- For multi-robot, frames connect via `utm` frame

**Setup:** Add `RobotModel` and `InteractiveMarker` displays in RViz for each Heron.

---

## Topics

All topics are prefixed with `/[namespace]/` when using namespaces.

### IMU Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `imu/data_raw` | Imu | Gazebo | Raw simulated IMU |
| `imu/data` | Imu | imu_filter_madgwick | Filtered IMU |
| `imu/rpy` | Vector3Stamped | rpy_translator | Raw Roll/Pitch/Yaw |
| `imu/rpy/filtered` | Vector3Stamped | imu_filter_madgwick | Filtered Roll/Pitch/Yaw |
| `imu/mag_sim` | MagneticField | Gazebo | Raw magnetometer |
| `imu/mag_raw` | Vector3Stamped | mag_interpreter | Raw mag as Vector3 |
| `imu/mag` | MagneticField | mag_interpreter | Calibrated magnetometer |

### GPS Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `navsat/fix` | NavSatFix | Gazebo | GPS position |
| `navsat/velocity` | Vector3Stamped | Gazebo | Velocity in NWU |
| `navsat/vel` | TwistStamped | navsat_vel_translate | Velocity in ENU |
| `navsat/vel_cov` | TwistWithCovarianceStamped | vel_cov | ENU with covariance |

### Thruster Topics

| Topic | Description |
|-------|-------------|
| `thrusters/0/input` | Right thruster (-1.0 to 1.0) |
| `thrusters/1/input` | Left thruster (-1.0 to 1.0) |

### SLAM GRANDE Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/lidar_h/velodyne_points` | PointCloud2 | Horizontal lidar (converted) |
| `/lidar_v/scan` | LaserScan | Vertical lidar |
| `/sonar/scan` | PointCloud2 | Underwater sonar |
| `/odom` | Odometry | Robot odometry |

---

## Hydrodynamics

Physics based on UUV Simulator. Enable debug with `hydro_debug:=1`.

### Buoyancy

Modelled using Linear (Small Angle) Theory for box-shaped vessels ([Fossen Ch4.2](http://www.fossen.biz/wiley/Ch4.pdf)).
- **Draft:** 0.02m submerged at rest
- **Height:** 0.32m (excluding antennae)
- Metacentric heights tuned empirically

### Damping

Quasi-quadratic damping: `force = -Qv|v| - Lv`
- Coefficients manually tuned (not from measurements)

### Thrust Curve

Measured from Columbia Lake trials:

| Input | Thrust (N) | | Input | Thrust (N) |
|-------|------------|--|-------|------------|
| -1.0 | -19.88 | | 0.2 | 2.24 |
| -0.8 | -16.52 | | 0.4 | 9.52 |
| -0.6 | -12.60 | | 0.6 | 21.28 |
| -0.4 | -5.60 | | 0.8 | 28.00 |
| -0.2 | -1.40 | | 1.0 | 33.60 |
| 0.0 | 0.00 | | | |

**Note:** Thrusters "remember" last command. Send zero to stop.

---

## Launch File Architecture

### upstream_spawn_heron.launch

Nodes launched for each robot:
- **imu_filter_madgwick**: Filters IMU + magnetometer for accurate orientation
- **heron_controller**: Converts `cmd_wrench`/`cmd_helm` → `cmd_drive`
- **control.launch**: Robot localization (EKF)
- **description.launch**: Robot state publisher (URDF/TF)
- **urdf_spawner**: Spawns robot in Gazebo
- **cmd_drive_translate**: `cmd_drive` → `thrusters/*/input`
- **rpy_translator**: Quaternion → Roll/Pitch/Yaw
- **twist_translate**: Scales interactive marker velocities
- **navsat_vel_translate**: NWU → ENU velocity conversion

### upstream_heron_world.launch

Launches:
1. Gazebo with selected world
2. `upstream_spawn_heron.launch` with given pose/namespace

---

## Magnetometer Calibration

To calibrate the magnetometer using `calibrate_compass` from `heron_bringup`:

```bash
export ROBOT_NAMESPACE=heron1  # Set to your namespace
rosrun heron_bringup calibrate_compass
```

---

## Creating Custom Worlds

Use the `heron_worlds` package:
1. Create a 3D file (STL tested) representing seabed/coastline
2. Follow heron_worlds package instructions

Or create a Gazebo `.world` file directly and launch with:
```bash
roslaunch heron_simulator upstream_heron_world.launch \
    use_pkg_path:=0 world_file:=/path/to/custom.world
```

---

## Known Issues

1. **IMU Initialization**: Vessel must spawn floating and stationary. Spawning in air or with initial velocity causes bad IMU data (self-corrects eventually).

2. **Damping Instability**: Large forces can crash simulation due to damping feedback loop. Fix by temporarily reducing damping coefficients.

3. **Out-of-Water Physics**: Damping applies even when airborne (minor—Heron can't fly).

4. **EKF Teleportation**: Using Gazebo's move tool causes odometry lag (EKF doesn't expect teleportation).

5. **Namespace Conflict**: Empty namespace and `heron` namespace cannot coexist (thruster topic conflict).

---

## License

- **Clearpath code (heron_gazebo)**: BSD-3-Clause
- **SLAM GRANDE extensions**: MIT

See [heron_simulator](https://github.com/heron/heron_simulator) for original license.
