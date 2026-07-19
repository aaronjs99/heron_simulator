# HERON Simulator

`heron_simulator` provides the Gazebo Classic simulation boundary for the
Heron-style USV. It owns simulated worlds, vehicle spawn, simulated sensors,
and the bridge from Heron drive commands to Gazebo thruster wrench inputs.

It does not own mission logic, navigation policy, mapping, operator surfaces,
or evaluation orchestration. Those are composed by `grande`, ORACLE, and
MARINER.

## What It Provides

- Heron vehicle spawn and world launch files
- water, harbor, and tank environment assets
- simulated LiDAR, IMU, camera, sonar, and timing topics
- scenario metadata and semantic entity fixtures
- `/cmd_drive` to Gazebo thruster wrench conversion
- Gazebo plugins and simulation regression tests

## Quick Start

Package-level simulation:

```bash
roslaunch heron_simulator heron_world.launch gui:=true
```

Full integrated simulation:

```bash
cd ~/catkin_ws/heron_ws/src/grande/grande
python3 run.py bringup --mode sim
```

Integrated simulation uses the dedicated ROS master at port `11312`. Physical
bringup retains the Heron-facing master on port `11311`; the runner owns and
cleans up only the simulation master it starts.

Software OpenGL fallback:

```bash
roslaunch heron_simulator heron_world.launch \
  gui:=true \
  gazebo_software_rendering:=true
```

## Sensor Surface

The simulator publishes the same ROS topic surface expected from IG Handle:

| Topic | Type |
| --- | --- |
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
| `/sense` | `heron_msgs/Sense`, explicitly synthetic contract telemetry |
| `/state/sim_control_odometry_3dof` | `nav_msgs/Odometry`, simulator-only body-frame control feedback |

Camera spawn follows the same `disabled_sensor_ids` launch argument used by the
integrated sensor contract. For low-load upward-camera-off tests, launch GRANDE
with `disabled_sensor_ids:=6,7`; this removes F3/F4 from ORACLE/DEFECTOR
inspection inputs and prevents Gazebo from spawning those camera sensors.

The simulator does not fabricate Velodyne packet bytes. Packet-level fidelity
belongs to real hardware or bag-backed tests.

Simulated sensor samples are useful for integration and contract validation,
but they are not a substitute for field calibration. `/sense` carries a stable
nominal battery value. Its motor-current fields are zero under the linear plant
and carry the empirical proxy state when that proxy is explicitly enabled.
Both forms are labelled synthetic and calibration-ineligible. Simulated current,
thrust, timing, and water-load behavior must not be used to approve a real
actuator model or sensor extrinsic.

## Empirical Actuator Proxy

The normal simulator bridge applies the checked-in linear force model. For an
explicit sensitivity run, `drive_to_thrusters.py` can instead load a generated
empirical proxy derived from the canonical direct-drive electrical model:

```bash
python3 grande/run.py bringup --mode sim \
  --launch-arg sim_empirical_thruster_model_enabled:=true \
  --launch-arg sim_empirical_thruster_model_file:=/absolute/path/to/heron_simulator_actuator_proxy.json
```

The proxy preserves measured side, direction, and rising/falling current-shape
asymmetry. It uses one shared forward-current reference so those asymmetries are
not normalized away. Legacy side scaling is disabled while the proxy is active;
transient lag and slew remain separate simulator assumptions.

This is a current-shaped force proxy, not thrust calibration. Its schema marks
it `calibration_eligible=false`, the bridge verifies its contract and hash, and
any missing or malformed enabled proxy fails closed. `/sim_sense/source_status`
separately identifies synthetic `/sense` and excludes it from calibration.
The bridge publishes the proxy's current state and applied force on
`/cmd_drive_to_thrusters/actuator_state`; this is simulator provenance, not
fabricated physical telemetry.

## Simulator Control Feedback

`ground_truth_control_odometry.py` converts `/pose_gt` world-frame velocity to
the body-frame planar odometry contract used by the fast simulator controller:

```text
/pose_gt -> /state/sim_control_odometry_3dof
```

The topic is available only in simulation and is labelled
`provenance=simulator_ground_truth` and `calibration_eligible=false`. It prevents
an estimator scale error from being mistaken for actuator-model error during
controller integration. D-LIO remains active in parallel for mapping and
sim-to-real estimator diagnostics. Real bringup never selects this topic.

## Vehicle and Sensor Geometry

The benchmark Heron hull profile is sourced from:

```text
../heron/heron_description/urdf/configs/ig_handle_benchmark
```

Sensor frame and mount poses come from:

```text
ig_handle/config/sensors/sensor_frames.yaml
```

Do not duplicate sensor extrinsics inside simulator-specific files.

## Sonar Simulation

Gazebo produces an internal multibeam profile cloud. `multibeam_raw.py` packs
that profile as DT100-style raw bytes on `/sensors/sonar/raw`. MARINER owns the
decoder and map accumulation path.

This preserves the hardware boundary:

```text
simulated sonar geometry -> raw DT100-style packets -> MARINER decoder -> sonar evidence
```

## Scenarios

Scenario metadata and fixtures live in:

| Path | Purpose |
| --- | --- |
| `config/scenarios.yaml` | Scenario index |
| `config/scenarios/` | Per-scenario launch and navigation metadata |
| `config/entities/` | Semantic entities matched to simulator scenes |
| `worlds/` | Gazebo world composition files |
| `models/` | Reusable Gazebo assets |

The default harbor scenario is a compact port-style basin with wharves,
pile fields, service boats, underwater debris, and a launch dock.

## Integration Pattern

Integrated simulation should look like real bringup wherever possible:

1. Gazebo publishes simulated sensor topics.
2. MARINER runs the selected odometry backend and navigation stack.
3. RTAB-Map builds map evidence when mapping is enabled.
4. ORACLE consumes semantic and map evidence.
5. The dashboard, RViz, and recorder expose the same operator surfaces used in
   real mode.

Gazebo truth is diagnostic by default. Explicit simulator LMPC runs may select
the private simulator control-odometry topic for feedback; mapping and real
navigation contracts remain on the configured estimator topics.

## Important Files

| Path | Role |
| --- | --- |
| `launch/` | Gazebo world and spawn launch files |
| `scripts/scenarios.py` | Scenario value resolver for roslaunch |
| `scripts/drive_to_thrusters.py` | `/cmd_drive` to Gazebo wrench bridge |
| `scripts/ground_truth_control_odometry.py` | Simulator truth to body-frame planar control odometry |
| `scripts/multibeam_raw.py` | Simulated sonar profile to raw packet bridge |
| `scripts/sim_ig_timing.py` | Simulated timing-reference publisher |
| `src/` | Gazebo plugins |
| `tests/` | Simulation launch and behavior regressions |
