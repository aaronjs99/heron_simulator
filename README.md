# Heron Simulator

Heron Simulator owns Gazebo vehicle physics, scenarios, synthetic sensors,
simulated timing, and the simulator-only drive-to-thruster plant. GRANDE selects
the scenario, MARINER consumes canonical surfaces, and ORACLE owns mission
meaning.

```text
scenario/vehicle -> Gazebo -> synthetic sensors -> MARINER state/navigation
  -> normalized drive -> simulator propulsion -> Gazebo forces
```

## Documentation

- [Architecture](docs/architecture.md) describes ownership, integration, ground truth, and architectural debt.

The simulator demonstrates software behavior within a declared configuration;
it does not validate physical thrust, sensor accuracy, or field safety. Markdown
is canonical and each narrative document has an adjacent PDF.

The `range_marker_pool` scenario renders RANGE_AID's provisional marker
contract for synthetic multi-view testing. Geometry, observability, and the
Ping360 negative-control boundary are defined once in the
[architecture reference](docs/architecture.md#descriptor-driven-range-marker).

# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| .gitattributes | Defines simulator text and binary path handling. | Git | Contributors |
| .gitignore | Applies the shared GRANDE exclusions while retaining Gazebo models and documentation as source assets. | Git | Contributors |
| CMakeLists.txt | Builds Gazebo Harmonic systems and installs ROS 2 launch files, sensor adapters, models, worlds, and scenario resources. | CMake 3.22+, ament_cmake, ros_gz, Gazebo Harmonic | Colcon build and install spaces |
| LICENSE | Provides the BSD-3-Clause terms for retained Clearpath code and MIT terms for GRANDE-specific extensions. | None | Repository users and redistributors |
| package.xml | Declares ROS 2 and Gazebo Harmonic build/runtime dependencies and resource exports. | ROS 2 Jazzy, Gazebo Harmonic | Colcon, rosdep, ros_gz_sim |
