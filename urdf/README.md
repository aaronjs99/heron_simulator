# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| sensors.urdf.xacro | Defines synthetic LiDAR, camera, IMU, and separate DT100 vertical-fan and Ping360 horizontal-plane links using IG Handle-exported seed poses. | xacro, Gazebo sensor plugins, ig_handle sensor_frames.yaml export | heron_simulator/launch/spawn_heron.launch |
