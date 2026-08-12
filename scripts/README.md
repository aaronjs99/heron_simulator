# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| acoustic_marker_model.py | Consumes RANGE_AID's canonical validated sphere-marker descriptor, applies only SDF identifier/provisional gates, and deterministically renders static Gazebo SDF with ray-intensity surrogates. | range_aid.marker.model, pathlib, yaml, xml.dom | spawn_acoustic_marker.py, offline geometry checks |
| drive_to_thrusters.py | Runs the internally consistent provisional four-regime actuator plant, including deadband, voltage scaling, lag, slew, reversal blanking, and synthetic PWM/RPM/current/thrust telemetry, then applies its signed forces to Gazebo. | json, pathlib, sys, geometry_msgs | heron_simulator/CMakeLists.txt, heron_simulator/config/thruster_dynamics.yaml, heron_simulator/launch/spawn_heron.launch |
| empirical_actuator_proxy.py | Loads and interpolates the simulator-only empirical actuator proxy. | hashlib, json, math, bisect | drive_to_thrusters.py |
| four_regime_propulsion.py | Defines the nonlinear side-by-direction propulsion plant used by simulator runtime. | math | drive_to_thrusters.py |
| gazebo_with_xvfb.sh | Runs gzserver on an owned headless display, with loopback TCP fallback for WSLg's non-sticky read-only X socket directory. | Python 3, Xvfb | heron_simulator/CMakeLists.txt, heron_simulator/launch/heron_world.launch |
| multibeam_raw.py | Converts only `dt100_link` Gazebo returns into exact-length Imagenex 83P v1.10 datagrams carrying the required provisional DT100 extrinsic revision. | datetime, math, struct, rospy, ig_handle/SonarRawPacket, sensor_msgs/PointCloud2 | heron_simulator/CMakeLists.txt, heron_simulator/launch/spawn_heron.launch, mariner/scripts/mapping/sonar/sonar_raw_to_cloud.py |
| ping360_profile_model.py | Pure deterministic model for canonical Ping360 simulation profiles. | math, typing | heron_simulator/CMakeLists.txt |
| ping360_profile_sim.py | Converts only `ping360_link` Gazebo returns into normalized planar profiles carrying and hashing the required provisional Ping360 extrinsic revision. | hashlib, struct, sys, pathlib, rospy, ig_handle/SonarProfile | heron_simulator/CMakeLists.txt, heron_simulator/launch/spawn_heron.launch |
| profile_env_run.sh | Runs the profile env run shell workflow. | None | heron_simulator/CMakeLists.txt, heron_simulator/launch/spawn_heron.launch |
| scenarios.py | Resolves simulator world, entity, spawn, marker, and offset values while leaving mission and navigation policy to GRANDE profiles. | pathlib, typing, yaml | heron_simulator/CMakeLists.txt, grande/launch/bringup.launch |
| sim_ig_timing.py | Publish IG Handle-style timing topics from simulated sensor timestamps. | typing, rospy, sensor_msgs | heron_simulator/CMakeLists.txt, heron_simulator/launch/spawn_heron.launch |
| sim_sense.py | Publish an explicitly synthetic Heron `/sense` contract whose battery-side current and voltage agree with the provisional actuator state. | time, rospy, heron_msgs, std_msgs | heron_simulator/CMakeLists.txt, heron_simulator/launch/spawn_heron.launch |
| spawn_acoustic_marker.py | Resolves the canonical RANGE_AID descriptor and instance, checks revision consistency, and spawns the generated static model in Gazebo. | rospy, gazebo_msgs, geometry_msgs, yaml, acoustic_marker_model.py | heron_simulator/launch/spawn_acoustic_marker.launch |
