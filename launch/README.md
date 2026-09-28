# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| heron_world.launch.py | Composes the Harmonic world, ROS 2/Gazebo bridges, Heron model, synthetic sensors and simulator actuator sink. Exposes launch-time simulation controls and orderly process shutdown. | Gazebo Harmonic, ros_gz_bridge, ros_gz_sim, robot_state_publisher, heron_description, ig_handle interfaces | ROS 2 launch / simulation |
| heron_world.launch | Legacy ROS 1 composition retained as source during migration; not installed by the ROS 2 package. | gazebo_ros, spawn_heron.launch | Pending ROS 2 marker/mission integration |
| spawn_heron.launch | Legacy ROS 1 vehicle spawner retained as source during migration; not installed by the ROS 2 package. | robot_state_publisher, gazebo_ros, simulator URDF and scripts | Legacy launch only |
| spawn_acoustic_marker.launch | Legacy ROS 1 marker spawner retained as source during migration; not installed by the ROS 2 package. | range_aid/config/markers, gazebo_ros, spawn_acoustic_marker.py | Pending ROS 2 marker/mission integration |
