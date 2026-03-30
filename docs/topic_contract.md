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
- `simulation_truth`: sim-only Gazebo truth topics such as `/pose_gt`
- `goal_routing`: the Oracle-to-Mariner navigation goal bridge
- `ig_handle_sensors`: IMU, LiDAR, sonar, camera feeds
- `dlio`: pose, path, and map outputs
- `oracle`: mission/anchor topics

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
- `slam_grande/config/topic_contract.yaml` is the canonical source of truth.
- `slam_grande/launch/bringup.launch` loads that shared contract and enables modules per stack.
- `topic_contract_guard.py` reads `/topic_contract` and validates the live ROS graph generically.
- The guard fails launch on duplicate publishers, missing chain links, bad types, or forbidden legacy aliases such as `/mariner/nav_ok`.
