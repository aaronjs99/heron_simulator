# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| heron_world.launch | Launches the selected Gazebo world and simulator runtime. | heron_simulator | grande/grande/launch/bringup.launch |
| spawn_heron.launch | Spawns the Heron model, publishers, plugins, and simulated sensor bridges. | robot_state_publisher, joint_state_publisher, gazebo_ros, heron_simulator | heron_simulator/launch/heron_world.launch |
