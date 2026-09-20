# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| spawn_acoustic_marker.launch | Spawns one explicitly provisional simulation marker from RANGE_AID-owned geometry and the simulator-owned scenario placement. | range_aid/config/markers, heron_simulator/config/scenarios/range_marker_pool, gazebo_ros, spawn_acoustic_marker.py | heron_simulator/launch/heron_world.launch |
| spawn_heron.launch | Generates the Heron model from selected sensor-frame geometry and enables sonar ray sensors with their selected providers, synthetic platform and payload battery state, legacy sense/status telemetry, timing, and propulsion. Propulsion adaptation and housekeeping telemetry are independently optional for bounded non-actuating diagnostics. | robot_state_publisher, joint_state_publisher, gazebo_ros, heron_simulator URDF and scripts, ig_handle sensor-frame export and sonar messages | heron_simulator/launch/heron_world.launch |
| heron_world.launch | Composes the simulated environment, sensor sources and actuator sink; accepts Gazebo arguments including the random seed. | spawn_heron.launch, gazebo_ros | GRANDE bringup.launch |
