# HERON Simulator

Gazebo simulation environment for the Heron USV and the rest of the SLAM
GRANDE stack.

This package is where planning, navigation, sensing, and mission behavior can be
tested together before running on hardware.

## What The Simulator Provides

- Heron vehicle spawn and world bringup
- a synthetic harbor with a short launch dock, longer inspection piers, and moored workboats driven from `slam_grande/data/anchors_sim.yaml`
- simulated sensors that mirror the operator dashboard sensor catalog
- topic bridges needed by MARINER and ORACLE
- mock inspection/perception support for fast autonomy testing
- Gazebo plugins for custom force behavior
- the same shared ORACLE, web, RViz, and rosbag surface used by hardware bringup

## Important Runtime Conventions

- The simulation stack applies MARINER planner changes as overlays. The base
  planner profile still loads first, then sim-only TEB and local-costmap
  overrides layer on top.
- The benchmark Heron hull profile is sourced from
  `../heron/heron_description/urdf/configs/ig_handle_benchmark`, so mass,
  damping, and added-mass changes are explicit.
- The simulated sonar publishes `sensor_msgs/PointCloud2` so it can share the
  same browser viewer path as lidar.

## Common Uses

### Full stack simulation

```bash
roslaunch heron_simulator bringup_sim.launch
```

By default, the boat spawns at the tip of the short launch dock defined in
`slam_grande/data/anchors_sim.yaml`. Inspection props still spawn in the
background after startup so the autonomy stack can begin moving without waiting
on Gazebo model churn.

### Simulator-only bringup

```bash
roslaunch heron_simulator simulation.launch
```

## Important Pieces

| Path | Role |
|---|---|
| `launch/` | Main simulation entrypoints |
| `scripts/autonomy/` | Runtime helpers such as model spawning and mock services |
| `scripts/control/` | Command translation between stack layers and Gazebo |
| `scripts/sensors/` | Sensor adaptation utilities |
| `src/` | Gazebo plugins and compiled simulator support |
| `tests/` | Simulation launch, rendering, and control regressions |

## How It Fits The Workspace

- ORACLE sends missions
- MARINER produces plans and control commands
- the simulator publishes the world, vehicle state, and sensor streams those
  packages need

This package is the place to debug integration issues that only show up when the
full autonomy loop is closed.

## Shared Runtime Pattern

The simulator now mirrors `slam_grande/launch/bringup_real.launch` closely:

1. Source vehicle state and sensor topics
2. Build or load the navigation map
3. Run MARINER navigation
4. Run ORACLE and keep `/oracle/map/anchors` live
5. Expose the web dashboard and RViz
6. Record a rosbag of sensors, state, MARINER, and ORACLE topics

The main difference is the source of data:

- `heron_simulator/bringup_sim.launch` uses Gazebo and simulated sensors
- `slam_grande/bringup_real.launch` uses real sensors through `ig_handle`

Useful simulator-only knobs:

- `spawn_inspection_models:=false` to skip prop spawning entirely
- `inspection_spawn_delay_sec:=<seconds>` to delay prop spawning further

To open the shared navigation RViz layout in sim:

```bash
roslaunch heron_simulator bringup_sim.launch use_rviz:=true
```
