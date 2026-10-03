# Heron Simulator

Heron Simulator provides Gazebo vehicle physics, configurable scenarios,
synthetic sensors, simulated timing, and a simulated drive-to-thruster plant.

```text
scenario and vehicle -> Gazebo -> simulated sensors and state
  -> drive command -> propulsion model -> Gazebo forces
```

## Quick start

Build this package in a ROS Noetic Catkin workspace with Gazebo available.
Choose a world and scenario from this repository's launch, config, and worlds
directories.

## Documentation

- [Architecture](docs/architecture.md) describes package components, scenario flow, ground truth, and known limitations.
- Scenario guides under config/ explain the available simulation setups.

## Simulation limits

Simulation demonstrates software behavior for a declared configuration. It does
not establish physical thrust, sensor accuracy, or field safety. The
range_marker_pool scenario provides synthetic marker targets for repeatable
multi-view tests.

## License

Inherited Clearpath code retains BSD 3-Clause terms; local extensions use MIT.
See LICENSE and preserve applicable file-level notices.

## File Structure

| File | Purpose | Dependencies | Used by |
| --- | --- | --- | --- |
| `.gitattributes` | Defines text and binary handling for simulator assets. | Git | Repository contributors |
| `.gitignore` | Excludes local build products while retaining simulator source assets. | Git | Repository contributors |
| `CMakeLists.txt` | Defines the catkin build, target-scoped Gazebo plugin linkage, installed worlds, models, launch files, executable scripts, reusable runtime package, launch-time scenario resolver, and configuration. | CMake 3.13+, catkin, pkg-config, ROS Noetic, Gazebo, setup.py | catkin build and install spaces |
| `LICENSE` | Provides the BSD-3-Clause terms for inherited code and MIT terms for locally developed extensions. | None | Repository users and redistributors |
| `package.xml` | Separates Gazebo/C++ build dependencies from simulator-only runtime integrations, including active-package scenario resolution through rospkg. | ROS Noetic, Gazebo, rospkg | catkin, rosdep |
| `setup.py` | Installs simulator model helpers, parameter validation, and scenario resolution. | catkin_pkg | CMakeLists.txt and simulator entrypoints |
