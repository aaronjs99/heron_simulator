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

# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| .gitattributes | Defines simulator text and binary path handling. | Git | Contributors |
| .gitignore | Excludes generated Gazebo and simulator artifacts. | Git | Contributors |
| CMakeLists.txt | Defines the catkin build and installed worlds, models, launch files, scripts, and configuration. | catkin, ROS Noetic, Gazebo | catkin build |
| package.xml | Declares simulator package metadata and ROS/Gazebo dependencies. | ROS Noetic, Gazebo | catkin, rosdep |
