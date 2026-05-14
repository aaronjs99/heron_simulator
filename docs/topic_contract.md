# Topic Contract

The simulator now uses a module-based topic contract instead of one flat list of
hardcoded checks.

Canonical chains:

State:
- `/pose_gt`
- `/state/odometry`
- `/state/odometry/filtered`

Navigation:
- `/mariner/goal_anchor`
- `/mariner/goal`
- `/mariner/final_pose`
- `/mariner/status/nav_ok`

Actuation:
- `/cmd_vel_raw`
- `/cmd_vel`
- `/cmd_drive`
- `/thrusters/1/input`
- `/thrusters/0/input`

Additional modules:
- `simulation_truth`: disabled-by-default Gazebo reference topics such as `/pose_gt`
- `goal_routing`: the Oracle-to-Mariner navigation goal bridge
- `ig_handle_sensors`: IMU, LiDAR, sonar, camera feeds
- `dlio`: pose, path, and map outputs
- `oracle`: mission/anchor topics

Lidar simulation contract:
- Real and sim share the typed VLP-16 point-cloud surfaces:
  `/sensors/lidar/hori/points` in `lidar_h_link` and
  `/sensors/lidar/vert/points` in `lidar_v_link`.
- Each Gazebo lidar is configured as a 16-channel, 360 x 30 degree ray scan at
  10 Hz, matching the VLP-16 600 RPM default frame cadence.
- Horizontal samples are set to 1800, which gives 0.2 degree azimuth spacing at
  600 RPM and about 288000 simulated points per second in single-return mode.
- The sim range cap is 200 m. The simulator does not model VLP-16 dual returns,
  calibrated reflectivity, per-laser timing offsets, web configuration, or
  raw UDP packet bytes.

Sonar simulation contract:
- Real and sim share the typed cloud surface `/sensors/sonar/scan` in
  `sonar_link`.
- The active Gazebo sensor is a DT100-style downward profile: 480 beams over a
  120 degree cross-track fan, 20 Hz maximum update rate, 0.5 m minimum range,
  and 100 m slant range.
- The real-only raw byte stream `/sensors/sonar/raw` is not faked in sim.
  Sim publishes the post-decoder `PointCloud2` equivalent.
- Gazebo ray casting does not model water acoustics, intensity, reverberation,
  or multipath. Treat it as first-order geometry parity, not acoustic parity.

Camera/IMU simulation contract:
- Real and sim share the F1/F2 Forge FG-PGE-50S5C-C-IP color image and
  camera-info topic names used by DEFECTOR.
- The sim F1/F2 camera image size is 1280 x 1024 at 15 Hz with the canonical
  optical-frame rotation.
- The real camera native sensor is Sony IMX264 color at 2448 x 2048, 24 FPS
  standard or 33 FPS with lossless compression. The sim stream keeps the
  calibrated 1280 x 1024 profile until the camera-info calibration is updated.
- Real and sim share `/sensors/imu/data` in `imu_link`. The sim IMU is a
  Gazebo dynamics sensor configured to the same frame/topic boundary as the
  Xsens MTi-30 driver, not an Xsens firmware emulation.

Schema:
- `topics`: canonical topic keys that launch files can override centrally
- `types`: reusable message-type aliases
- `modules`: endpoint rules grouped by subsystem
- `forbidden_topics`: legacy aliases or known-bad split points
- `chains`: high-level documentation of the intended flow

Semantics:
- `/cmd_vel_raw` is planner output.
- `/cmd_vel` is the vehicle-facing command after optional safety filtering.
- `/cmd_drive` is the mixed differential drive command.
- `/state/odometry` is the canonical nav odometry consumed by planners and mission code.
- `/state/odometry/filtered` is the canonical filtered/consumer odometry topic.

Scaling pattern:
- Add a new topic key under `topics`.
- Add a type alias under `types` if needed.
- Add or extend a module under `modules`.
- Toggle that module via `/topic_contract/modules/<name>/enabled` from launch.

Enforcement:
- `slam_grande/config/contracts/topic_contract.yaml` is the canonical source of truth.
- `slam_grande/launch/bringup.launch` loads that shared contract and enables modules per stack.
- `topic_contract_guard.py` reads `/topic_contract` and validates the live ROS graph generically.
- The guard fails launch on duplicate publishers, missing chain links, bad types, or forbidden legacy aliases such as `/mariner/nav_ok`.
