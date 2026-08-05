# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| heron_world.launch | Selects the Gazebo world, display mode, ROS graph, plant profile, and vehicle spawn launch. | gazebo_ros, heron_simulator configuration and worlds | grande/grande/launch/bringup.launch |
| spawn_heron.launch | Spawns the Heron model and attaches synthetic sensors, timing, telemetry, and propulsion providers. | robot_state_publisher, joint_state_publisher, gazebo_ros, heron_simulator URDF and scripts | heron_simulator/launch/heron_world.launch |
