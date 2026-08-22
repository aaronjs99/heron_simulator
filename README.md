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
| .gitignore | Excludes generated Gazebo and simulator artifacts. | Git | Contributors |
| CMakeLists.txt | Defines the catkin build, target-scoped Gazebo plugin linkage, installed worlds, models, launch files, executable scripts, reusable runtime package, launch-time scenario resolver, and configuration. | CMake 3.13+, catkin, pkg-config, ROS Noetic, Gazebo, setup.py | catkin build and install spaces |
| LICENSE | Provides the BSD-3-Clause terms for retained Clearpath code and MIT terms for GRANDE-specific extensions. | None | Repository users and redistributors |
| package.xml | Separates Gazebo/C++ build dependencies from simulator-only runtime integrations, including active-package scenario resolution through rospkg. | ROS Noetic, Gazebo, rospkg | catkin, rosdep |
| setup.py | Installs the reusable deterministic `models` package through the standard source/devel/install Python path. | catkin_pkg, scripts/models | CMakeLists.txt, simulator entrypoints, static validation |
