# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| sensors.urdf.xacro | Defines synthetic LiDAR, camera, IMU, and sonar links and plugins using IG Handle-exported sensor poses. | xacro, Gazebo sensor plugins, ig_handle sensor_frames.yaml export | heron_simulator/launch/spawn_heron.launch |
