# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| sensors.urdf.xacro | Defines synthetic LiDAR, camera, IMU, and separate DT100 vertical-fan and Ping360 horizontal-plane links using poses exported from the launch-selected sensor-frame YAML. | xacro, Gazebo sensor plugins, ig_handle sensor-frame export | heron_simulator/launch/spawn_heron.launch |
