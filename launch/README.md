# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| heron_world.launch | Selects the Gazebo world, display mode, ROS graph, plant profile, sensor frames, vehicle and optional marker, synthetic vehicle and payload battery topics, provisional sonar extrinsic revisions, and the simulation-only grounding-evidence provider. | gazebo_ros, heron_simulator configuration and worlds, oracle service contracts | grande/launch/bringup.launch |
| simulated_navigation.launch | Generates a truth-seeded perturbed navigation measurement, filters it through a covariance-aware planar estimator, and publishes the canonical MARINER state surface for simulation only. | simulated_navigation_state.py, robot_localization, mariner simulation navigation configuration and state adapters | GRANDE state-estimator orchestration |
| spawn_acoustic_marker.launch | Spawns one explicitly provisional simulation marker from the canonical RANGE_AID geometry and instance records. | range_aid/config/markers, gazebo_ros, spawn_acoustic_marker.py | heron_simulator/launch/heron_world.launch |
| spawn_heron.launch | Generates the Heron model from selected sensor-frame geometry and attaches sonar providers, synthetic platform and payload battery state, legacy sense/status telemetry, timing, and propulsion. | robot_state_publisher, joint_state_publisher, gazebo_ros, heron_simulator URDF and scripts, ig_handle sensor-frame export and sonar messages | heron_simulator/launch/heron_world.launch |
