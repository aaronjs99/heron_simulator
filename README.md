# Heron Simulator

Heron Simulator provides Gazebo vehicle physics, configurable scenarios,
synthetic sensors, simulated timing, and a simulated drive-to-thruster plant.

```text
scenario and vehicle -> Gazebo -> simulated sensors and state
  -> drive command -> propulsion model -> Gazebo forces
```

Scenario resource lookup uses the active ament package index. Missing native
packages reject without searching ROS 1 manifests or inventing source roots.
This lookup change does not qualify legacy scenario-generated worlds or the
Gazebo Classic acoustic-marker spawn path for Harmonic operation.

## Quick start

Build this package in a ROS 2 Jazzy Colcon workspace with Gazebo Harmonic available. Native scenario-resolution consumers remain in progress; inspect the selected launch contract before execution.
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


# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| .gitattributes | Defines simulator text and binary path handling. | Git | Contributors |
| .gitignore | Excludes generated artifacts while retaining Gazebo models and documentation as source assets. | Git | Contributors |
| CMakeLists.txt | Builds Gazebo Harmonic systems and installs ROS 2 launch files, sensor adapters, models, worlds, and scenario resources; installs the package license notice. | CMake 3.22+, ament_cmake, ros_gz, Gazebo Harmonic | Colcon build and install spaces |
| LICENSE | Provides the BSD-3-Clause terms for retained Clearpath code and MIT terms for GRANDE-specific extensions. | None | Repository users and redistributors |
| package.xml | Declares ROS 2 and Gazebo Harmonic build/runtime dependencies and resource exports. | ROS 2 Jazzy, Gazebo Harmonic | Colcon, rosdep, ros_gz_sim |
| README.md | Describes repository scope, supported environment and current limitations. | Repository source and metadata | Repository users |
| gz_sim_resource_path.sh.in | Exports installed simulator model resources through the ament environment. | ament_cmake, Gazebo Harmonic | Installed workspace setup |
| gz_sim_system_plugin_path.sh.in | Exports installed simulator system plugins through the ament environment. | ament_cmake, Gazebo Harmonic | Installed workspace setup |
