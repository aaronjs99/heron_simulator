# HERON Simulator

Gazebo simulation boundary for the Heron USV.

This package owns the simulated hull, Gazebo worlds/models, sensor plugins, and
the small bridge from Heron drive commands to Gazebo thruster wrench inputs.
Planning, navigation, ORACLE behavior, semantic world reasoning, runtime
preflight, and evaluation orchestration live outside this package.

## What The Simulator Provides

- Heron vehicle spawn and world bringup
- reusable water, harbor, and tank environment assets
- simulated sensors that publish the same topic surfaces as `ig_handle`
- simulated IG Handle timing-reference topics derived from simulated sensor
  stamps
- `/cmd_drive` to Gazebo thruster wrench translation
- Gazebo plugins for custom force behavior

## Important Runtime Conventions

- MARINER planner behavior is shared with hardware. Optional overlays are
  explicit navigation or scenario profiles, not hidden simulation behavior.
- The benchmark Heron hull profile is sourced from
  `../heron/heron_description/urdf/configs/ig_handle_benchmark`, so mass,
  damping, and added-mass changes are explicit.
- Low-level PID controller gains live only in
  `heron_controller/config/heron_controller.yaml`. MARINER owns the
  `cmd_vel` to `/cmd_drive` mixer profile. The simulator owns only the
  `/cmd_drive` to Gazebo thruster bridge and
  `heron_simulator/config/thruster_dynamics.yaml`.
- The simulated lidar pair uses the same ROS surface as the real VLP-16 pair:
  `/sensors/lidar/hori/points` and `/sensors/lidar/vert/points`. Each Gazebo
  ray model is configured as a 16-channel, 360 x 30 degree VLP-16-style scan at
  the 600 RPM / 10 Hz default cadence with 0.2 degree azimuth spacing and 200 m
  maximum range. The simulator does not publish synthetic
  `velodyne_msgs/VelodyneScan` packet topics; packet-level fidelity belongs to
  real hardware or bag-backed tests because fabricated packet bytes would not
  represent the Gazebo ray model honestly.
- The simulated sonar is a multibeam echosounder path, not a direct scan
  shortcut. Gazebo produces an internal DT100-style 480-beam profile cloud,
  `multibeam_raw.py` packs it as raw 83P/profile bytes on
  `/sensors/sonar/raw`. MARINER owns raw-to-scan decoding and scan-to-map
  accumulation.
- The simulated Forge cameras use the same canonical image and camera-info
  topics and optical frame names as the real Forge FG-PGE-50S5C-C-IP color
  camera stack. Simulation enables F1/F2/F3/F4 so the full camera topic
  set is available. The sim camera model is a geometry/topic stand-in, not
  a Forge driver emulation.
- The simulator sensor URDF is only a Gazebo plugin/visual add-on. It gets all
  sensor frame and mount poses from `ig_handle/config/sensors/sensor_frames.yaml`
  through `ig_handle/scripts/frames/export.py`; do not duplicate sensor
  extrinsics in `heron_simulator`.
- `sim_ig_timing.py` mirrors the optional IG Handle timing surface by publishing
  `/sensors/pps/time` from sim time, `/sensors/camera/time` from camera image
  header stamps, and `/sensors/imu/time` from simulated IMU header stamps.

## Sensor Output Contract

`heron_simulator` publishes the same simulated sensor topics consumed by the
real stack. Sonar follows the hardware boundary: this package publishes raw
multibeam profile data only, and MARINER decodes that raw stream into a scan
cloud.

| Topic | Type |
|---|---|
| `/sensors/lidar/hori/points` | `sensor_msgs/PointCloud2` |
| `/sensors/lidar/vert/points` | `sensor_msgs/PointCloud2` |
| `/sensors/imu/data` | `sensor_msgs/Imu` |
| `/sensors/camera/f1/image_raw` | `sensor_msgs/Image` |
| `/sensors/camera/f2/image_raw` | `sensor_msgs/Image` |
| `/sensors/camera/f3/image_raw` | `sensor_msgs/Image` |
| `/sensors/camera/f4/image_raw` | `sensor_msgs/Image` |
| `/sensors/sonar/raw` | `std_msgs/UInt8MultiArray` |
| `/sensors/pps/time` | `sensor_msgs/TimeReference` |
| `/sensors/camera/time` | `sensor_msgs/TimeReference` |
| `/sensors/imu/time` | `sensor_msgs/TimeReference` |

## Common Uses

### Package-level simulation

```bash
roslaunch heron_simulator heron_world.launch
```

By default, the boat spawns with the `ig_handle_benchmark` Heron description
config so the package-level launch exposes the same IG sensor add-ons as the
full-stack integration launch. Any semantic reasoning over the scene is owned
by ORACLE and the `/oracle/world/entities` pipeline, not by Gazebo model names
or simulator scripts.

`heron_simulator` launch files are Gazebo pieces. Full-stack operation should
compose them from the active integration launch so MARINER, ORACLE, topics,
and state wiring stay consistent.

## Important Pieces

| Path | Role |
|---|---|
| `launch/` | Gazebo world/spawn launch pieces used by an integration stack |
| `config/scenarios.yaml` | Simulator scenario index used by integration launch defaults |
| `config/scenarios/` | Per-scenario world/entity/navigation profile metadata |
| `config/entities/` | Simulation semantic entity fixtures matched to the Gazebo scenes |
| `worlds/` | Thin scenario composition files for physics, lighting, GUI camera, and model includes |
| `models/` | Reusable Gazebo geometry assets such as water surfaces, tank geometry, and tank targets |
| `scripts/scenarios.py` | Minimal roslaunch resolver for scenario-specific values |
| `scripts/drive_to_thrusters.py` | `/cmd_drive` to Gazebo thruster wrench bridge |
| `scripts/multibeam_raw.py` | Gazebo multibeam sonar ray cloud to raw 83P/profile packet bridge |
| `scripts/sim_ig_timing.py` | Simulated IG Handle timing-reference topic bridge |
| `src/` | Gazebo plugins and compiled simulator support |
| `tests/` | Simulation launch, rendering, and control regressions |

## How It Fits The Workspace

- MARINER produces `/cmd_drive` through the shared navigation stack
- the simulator turns `/cmd_drive` into Gazebo thruster inputs
- Gazebo publishes simulated sensor topics that look like `ig_handle` hardware
  topics

ORACLE, MARINER, DEFECTOR, dashboards, bagging, and topic wiring should be
composed by the integration layer.

## Integration Runtime Pattern

The simulator mirrors hardware bringup most closely when composed by an
integration launch:

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

The current integration stack uses Gazebo truth as the default upstream
localization source and republishes it through the same odometry sanity filter
and `/state/odometry` topic used by the rest of the stack. DLiO launch and
configuration are owned by MARINER/integration bringup, not by
`heron_simulator`. Mocap stays outside simulator bringup as a lab
logging/comparison stream.

To open Gazebo with the package-level simulator launch:

```bash
roslaunch heron_simulator heron_world.launch gui:=true
```
