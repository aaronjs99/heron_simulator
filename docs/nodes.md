# Heron simulator nodes

This is the practical ownership map for `heron_simulator` nodes. Use it when
you need to locate a runtime issue quickly.

## Autonomy helpers

### `autonomy/mock_defector_service.py`

Provides mock inspection service behavior in sim.

- service: `/defector/capture_and_analyze` (`std_srvs/Trigger`)
- key params:
  - `~delay` (default `2.0`)
  - `~success_rate` (default `1.0`)

### `autonomy/spawn_inspection_models.py`

Spawns sim inspection geometry from anchor definitions.

- key params:
  - `~anchor_file` (default `slam_grande/data/anchors_sim.yaml`)
  - `~spawn_delay` (default `0.5`)

## Sensor adaptation nodes

### `sensors/scan_to_cloud.py`

Converts LaserScan to PointCloud2 for costmap compatibility.

- subscribes: `~scan_topic` (default `/lidar_h/scan`)
- publishes: `~cloud_topic` (default `/lidar_h/velodyne_points`)

### `sensors/vel_cov_fixed.py`

Adds covariance to velocity stream for downstream localization consumers.

- subscribes: `/navsat/vel`
- publishes: `/navsat/vel_cov`

### `sensors/navsat_vel_translate.py`

Converts GPS velocity convention to the format expected by the stack.

- subscribes: `/navsat/velocity`
- publishes: `/navsat/vel`

### `sensors/rpy_translator.py`

Publishes RPY orientation derived from IMU quaternion.

- subscribes: `/imu/data`
- publishes: `/imu/rpy`

### `sensors/vector3_to_magnetic_field.py`

Normalizes magnetometer message shape for consumers.

- subscribes: `/imu/mag_sim`
- publishes: `/imu/mag_raw`, `/imu/mag`

## Control and dynamics

### `control/cmd_drive_translate.py`

Maps normalized `cmd_drive` into physical thruster wrenches.

- subscribes: `/cmd_drive`
- publishes: `/thrusters/0/input`, `/thrusters/1/input`

### `control/twist_translate.py`

Scales interactive-marker velocity commands to vehicle limits.

- subscribes: `/twist_marker_server/cmd_vel`
- publishes: `/cmd_vel`

### `control/activate_control_service.py`

Control enable/disable service surface.

- service: `/heron/activate_control` (`std_srvs/SetBool`)

### `control/vessel_dynamics.py`

Applies hydrodynamic hull-force model in sim runtime.

- subscribes: `/ground_truth/odom`
- publishes: `/{namespace}/hydro_forces`
- key param: `~namespace` (default `heron`)

## Where to debug first

- no inspection response: `mock_defector_service.py`
- missing sim props: `spawn_inspection_models.py`
- bad lidar/costmap feed: `scan_to_cloud.py`
- command issued but no vessel motion: `cmd_drive_translate.py` then `vessel_dynamics.py`

For end-to-end topic rules, use `topic_contract.md`. For physics tuning, use
`hydrodynamics.md`.
