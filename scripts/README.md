# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| drive_to_thrusters.py | Runs the internally consistent provisional four-regime actuator plant, including deadband, voltage scaling, lag, slew, reversal blanking, and synthetic PWM/RPM/current/thrust telemetry, then applies its signed forces to Gazebo. | json, pathlib, sys, geometry_msgs | heron_simulator/CMakeLists.txt, heron_simulator/config/thruster_dynamics.yaml, heron_simulator/launch/spawn_heron.launch |
| empirical_actuator_proxy.py | Loads and interpolates the simulator-only empirical actuator proxy. | hashlib, json, math, bisect | drive_to_thrusters.py |
| four_regime_propulsion.py | Defines the nonlinear side-by-direction propulsion plant used by simulator runtime. | math | drive_to_thrusters.py |
| gazebo_with_xvfb.sh | Runs gzserver on an owned headless display, with loopback TCP fallback for WSLg's non-sticky read-only X socket directory. | Python 3, Xvfb | heron_simulator/CMakeLists.txt, heron_simulator/launch/heron_world.launch |
| multibeam_raw.py | Publish raw multibeam echosounder profile packets from Gazebo rays. | math, struct, typing, rospy | heron_simulator/CMakeLists.txt, heron_simulator/launch/spawn_heron.launch |
| ping360_profile_model.py | Pure deterministic model for canonical Ping360 simulation profiles. | math, typing | heron_simulator/CMakeLists.txt |
| ping360_profile_sim.py | Publishes canonical Ping360 profiles from a full-circle Gazebo ray cloud. | hashlib, struct, sys, pathlib | heron_simulator/CMakeLists.txt, heron_simulator/launch/spawn_heron.launch |
| profile_env_run.sh | Runs the profile env run shell workflow. | None | heron_simulator/CMakeLists.txt, heron_simulator/launch/spawn_heron.launch |
| scenarios.py | Resolves simulator world, entity, spawn, and offset values while leaving mission and navigation policy to GRANDE profiles. | pathlib, typing, yaml | heron_simulator/CMakeLists.txt, grande/launch/bringup.launch |
| sim_ig_timing.py | Publish IG Handle-style timing topics from simulated sensor timestamps. | typing, rospy, sensor_msgs | heron_simulator/CMakeLists.txt, heron_simulator/launch/spawn_heron.launch |
| sim_sense.py | Publish an explicitly synthetic Heron `/sense` contract whose battery-side current and voltage agree with the provisional actuator state. | time, rospy, heron_msgs, std_msgs | heron_simulator/CMakeLists.txt, heron_simulator/launch/spawn_heron.launch |
