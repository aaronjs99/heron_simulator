# Heron Simulation — Hydrodynamics

Physics model documentation for the Heron USV simulation.

> **Attribution**: This documentation is derived from the original [heron_simulator](https://github.com/heron/heron_simulator) by Clearpath Robotics.

---

## Overview

The Heron simulation uses the [UUV Simulator](https://uuvsimulator.github.io/) framework for realistic marine vehicle dynamics. The physics model includes:

- Buoyancy forces
- Hydrodynamic damping
- Thrust modeling
- Wave interaction

## Buoyancy Forces

The Heron's buoyancy is modeled using **Linear (Small Angle) Theory for Box-Shaped Vessels**, described in [Fossen's Marine Control Systems](http://www.fossen.biz/wiley/Ch4.pdf), Section 4.2.

### Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `submerged_height` | 0.02 m | Draft when floating normally |
| `vessel_height` | 0.32 m | Total height (excluding antennae) |
| `metacentric_height_x` | Tuned | Roll stability (transverse) |
| `metacentric_height_y` | Tuned | Pitch stability (longitudinal) |

### Notes

- Metacentric heights were tuned empirically rather than calculated from theoretical values
- The 0.74m antenna height is ignored to improve submerged behavior
- Buoyancy forces apply even when the vessel is airborne (known limitation)

## Damping Forces

Damping is modeled as **quasi-quadratic** for each axis:

```
force = -Q * v * |v| - L * v
```

Where:
- `Q` = Quadratic damping coefficient
- `L` = Linear damping coefficient  
- `v` = Velocity along axis

### Coefficients

| Axis | Quadratic (Q) | Linear (L) | Description |
|------|---------------|------------|-------------|
| X (surge) | Tuned | Tuned | Forward/backward motion |
| Y (sway) | Tuned | Tuned | Lateral motion |
| Z (heave) | Tuned | Tuned | Vertical motion |
| Roll | Tuned | Tuned | Rotation about X |
| Pitch | Tuned | Tuned | Rotation about Y |
| Yaw | Tuned | Tuned | Rotation about Z |

### Tuning Notes

- Coefficients were manually tuned to match observed Heron behavior
- Not derived from real-world measurements
- May need adjustment for specific scenarios

### Known Issue: Amplifying Loop

Applying very large forces (via Gazebo's Apply Force tool) can cause simulation instability:

1. Large force → High velocity
2. High velocity → Large damping force
3. Large damping → Even higher velocity
4. Repeat until crash

**Solution**: Reduce damping coefficients temporarily, then gradually increase.

## Thrust Forces

Thrust is modeled using **linear interpolation** based on measured acceleration from Columbia Lake trials.

### Thrust Mapping

| Input | Output Thrust (N) |
|-------|-------------------|
| -1.0 | -19.88 |
| -0.8 | -16.52 |
| -0.6 | -12.60 |
| -0.4 | -5.60 |
| -0.2 | -1.40 |
| 0.0 | 0.00 |
| 0.2 | 2.24 |
| 0.4 | 9.52 |
| 0.6 | 21.28 |
| 0.8 | 28.00 |
| 1.0 | 33.60 |

### Maximum Thrust

| Direction | Max Thrust |
|-----------|------------|
| Forward | ~34 N |
| Reverse | ~20 N |

These values approximately match the Heron datasheet specifications.

### Thrust Persistence

The simulation "remembers" the last thrust command. Even if no new commands are sent, thrusters continue at their last setting. To stop:

- Publish zero to `/cmd_drive`
- Or use the RViz interactive markers (which handle this automatically)

## Water Current

Water current can be configured in world files.

### Default Configuration (ocean_surface.world)

```xml
<current_velocity_topic>hydrodynamics/current_velocity</current_velocity_topic>
<mean_velocity>0.0 0.0 0.0</mean_velocity>
<min_velocity>0.0 0.0 0.0</min_velocity>
<max_velocity>0.0 0.0 0.0</max_velocity>
```

### Adding Current

To add water current, modify the world file:

```xml
<mean_velocity>0.5 0.0 0.0</mean_velocity>  <!-- 0.5 m/s East -->
```

## Wave Interaction

Standard UUV Simulator wave models apply:

- Surface buoyancy effects
- Wave-induced motion
- Configurable wave parameters

### Wave Parameters (inspection_dock.world)

Currently configured for calm water to simplify testing.

## Sensor Simulation

### GPS (hector_gazebo_plugins)

- Gaussian noise model
- Configurable accuracy
- Velocity output in NWU (converted to ENU)

### IMU (hector_gazebo_plugins)

- 6-DOF acceleration and angular velocity
- Magnetometer for heading
- Requires stationary initialization

### Magnetometer

- World magnetic field simulation
- Supports calibration via `calibrate_compass` script
- Requires `ROBOT_NAMESPACE` environment variable

## IMU Initialization

The IMU filter requires the vessel to be:

1. Floating on water
2. Approximately stationary
3. Level (not pitched or rolled)

**Warning**: Spawning in the air, inside another object, or with initial velocity will cause:
- Incorrect IMU readings
- Poor heading estimation
- Delayed stabilization (may take several seconds)

## Debug Topics

Enable with `hydro_debug:=1`:

```bash
roslaunch heron_simulator simulation.launch hydro_debug:=1
```

### Debug Topics Published

| Topic | Description |
|-------|-------------|
| `/debug/buoyancy_force` | Current buoyancy forces |
| `/debug/damping_force` | Current damping forces |
| `/debug/thrust_force` | Current thrust forces |

## References

1. [Fossen's Marine Control Systems](http://www.fossen.biz/wiley/) - Buoyancy theory
2. [UUV Simulator Documentation](https://uuvsimulator.github.io/) - Simulation framework
3. Heron USV Datasheet - Thrust specifications
4. Columbia Lake Trials - Thrust calibration data
