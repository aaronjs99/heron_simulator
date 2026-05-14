# HERON Simulator

Gazebo simulation environment for the Heron USV and the rest of the SLAM
GRANDE stack.

This package is where planning, navigation, sensing, and mission behavior can be
tested together before running on hardware.

## What The Simulator Provides

- Heron vehicle spawn and world bringup
- a simulated harbor with a short launch dock, longer inspection piers, and moored workboats driven from `slam_grande/config/anchors/harbor.yaml`
- simulated sensors that mirror the operator dashboard sensor catalog
- topic bridges needed by MARINER and ORACLE
- inspection-scene geometry for real DEFECTOR testing
- Gazebo plugins for custom force behavior
- the same shared ORACLE, web, RViz, and rosbag surface used by hardware bringup

## Important Runtime Conventions

- MARINER planner behavior is shared with hardware. Optional overlays are
  explicit navigation or scenario profiles, not hidden simulation behavior.
- The benchmark Heron hull profile is sourced from
  `../heron/heron_description/urdf/configs/ig_handle_benchmark`, so mass,
  damping, and added-mass changes are explicit.
- The simulated lidar pair uses the same ROS surface as the real VLP-16 pair:
  `/sensors/lidar/hori/points` and `/sensors/lidar/vert/points`. Each Gazebo
  ray model is configured as a 16-channel, 360 x 30 degree VLP-16-style scan at
  the 600 RPM / 10 Hz default cadence with 0.2 degree azimuth spacing and 200 m
  maximum range.
- The simulated sonar publishes `sensor_msgs/PointCloud2` on the same typed
  surface as the real DT100 adapter: `/sensors/sonar/scan` in `sonar_link`. The
  Gazebo ray model is configured as a DT100-style downward 480-beam, 120 degree
  profile at 20 Hz with a 0.5 m minimum range and 100 m slant range. It does not
  synthesize proprietary raw DT100 packets or acoustic multipath.
- The simulated F1/F2 cameras use the same canonical image and camera-info
  topics as the real Forge FG-PGE-50S5C-C-IP color inspection pair. The sim
  camera model is a geometry/topic stand-in, not a Forge driver emulation.
- Runtime cleanup is performed before launching a sim run. The in-launch
  `simulation_preflight` guard does not kill ROS nodes or port owners unless
  explicit cleanup arguments are enabled.

## Common Uses

### Full stack simulation

```bash
roslaunch slam_grande bringup.launch mode:=sim
```

By default, the boat spawns at the tip of the short launch dock defined in
`slam_grande/config/anchors/harbor.yaml`. Inspection props still spawn in the
background after startup so the autonomy stack can begin moving without waiting
on Gazebo model churn.

### Simulator-only bringup

```bash
roslaunch heron_simulator heron_world.launch
```

Most development should still use the canonical full-stack command above. The
package-level launch files are lower-level helpers for world/spawn/control
debugging.

## Important Pieces

| Path | Role |
|---|---|
| `launch/` | Main simulation entrypoints |
| `scripts/autonomy/` | Runtime helpers such as model spawning |
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

The simulator now mirrors hardware bringup closely through the shared
`slam_grande/launch/bringup.launch` selector:

1. Source vehicle state and sensor topics
2. Build or load the navigation map
3. Run MARINER navigation
4. Run ORACLE with `/oracle/world/entities` as the live semantic world
5. Expose the web dashboard and RViz
6. Record a rosbag of sensors, state, MARINER, and ORACLE topics

The main difference is the source of data:

- `mode:=sim` uses Gazebo and simulated sensors
- `mode:=real` uses real sensors through `ig_handle`

Current shared-bringup defaults:

| Setting | Sim Default | Meaning |
| --- | --- | --- |
| `use_web_viz` | `true` | starts dashboard/rosbridge/video bridge |
| `record_bags` | `true` | records runtime evidence unless disabled |
| `use_rviz` | `true` | opens the shared navigation RViz layout in sim |
| `build_map` | `true` | starts RTAB-Map loop closure/global correction when RTAB-Map is installed |

DLiO is the canonical state path in simulation and real bringup. Gazebo still
publishes simulator truth topics for diagnostics, but `slam_grande` no longer
uses simulator truth, GPS, or EKF as navigation state sources.

Useful simulator-only knobs:

- `spawn_inspection_models:=false` to skip prop spawning entirely
- `inspection_spawn_delay_sec:=<seconds>` to delay prop spawning further

To open the shared navigation RViz layout in sim:

```bash
roslaunch slam_grande bringup.launch mode:=sim use_rviz:=true
```
