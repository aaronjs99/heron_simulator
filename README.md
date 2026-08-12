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

The `range_marker_pool` scenario excludes the legacy target field and spawns
the provisional Orion marker from RANGE_AID's single geometry and instance
records. The simulator only renders that contract as Gazebo collision and
visual geometry:

```bash
roslaunch heron_simulator heron_world.launch \
  world_name:=$(rospack find heron_simulator)/worlds/range_marker_pool.world \
  spawn_acoustic_marker:=true
```

The per-element `laser_retro` values make ray-based regression deterministic;
they are not calibrated acoustic reflectivity. The canonical provisional
instance is at map/world position $(0,0,-1.5)\ \mathrm{m}$ with a $160^\circ$
yaw. It is a DT100 multi-view scenario. The current zero-thickness horizontal
Ping360 proxy cannot observe that deep instance and is only a planar negative
control here. Full marker-pose evidence still requires multiple DT100 views.

# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| .gitattributes | Defines simulator text and binary path handling. | Git | Contributors |
| .gitignore | Excludes generated Gazebo and simulator artifacts. | Git | Contributors |
| CMakeLists.txt | Defines the catkin build and installed worlds, models, launch files, scripts, and configuration. | catkin, ROS Noetic, Gazebo | catkin build |
| package.xml | Declares simulator package metadata and ROS/Gazebo dependencies. | ROS Noetic, Gazebo | catkin, rosdep |
