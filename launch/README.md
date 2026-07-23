# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| heron_world.launch | Launches and wires the heron world ROS runtime. | heron_simulator | grande/grande/launch/bringup.launch, heron_simulator/scripts/validate_lidar_fidelity_profiles.py |
| spawn_heron.launch | Launches and wires the spawn heron ROS runtime. | robot_state_publisher, joint_state_publisher, gazebo_ros, heron_simulator | heron_simulator/launch/heron_world.launch, heron_simulator/scripts/validate_lidar_fidelity_profiles.py |
