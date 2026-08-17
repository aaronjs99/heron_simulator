# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| heron_world.launch | Selects the Gazebo world, display mode, ROS graph, plant profile, canonical sensor-frame file, vehicle spawn, optional marker, and separate provisional DT100/Ping360 extrinsic revisions. | gazebo_ros, heron_simulator configuration and worlds | grande/grande/launch/bringup.launch |
| spawn_acoustic_marker.launch | Spawns one explicitly provisional simulation marker from the canonical RANGE_AID geometry and instance records. | range_aid/config/markers, gazebo_ros, spawn_acoustic_marker.py | heron_simulator/launch/heron_world.launch |
| spawn_heron.launch | Generates the Heron model from the selected sensor-frame geometry and attaches separate DT100 and Ping360 frame/revision-aware providers, synthetic `/sense` and `/status` telemetry, timing, and propulsion. | robot_state_publisher, joint_state_publisher, gazebo_ros, heron_simulator URDF and scripts, ig_handle sensor-frame export and sonar messages | heron_simulator/launch/heron_world.launch |
