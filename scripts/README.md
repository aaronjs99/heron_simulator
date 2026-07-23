# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| drive_to_thrusters.py | Bridge normalized Heron drive commands into Gazebo thruster wrench inputs. | json, pathlib, sys, geometry_msgs | heron_simulator/CMakeLists.txt, heron_simulator/config/thruster_dynamics.yaml, heron_simulator/launch/spawn_heron.launch |
| empirical_actuator_proxy.py | Pure validation and interpolation for the simulator actuator proxy. | hashlib, json, math, bisect | None |
| gazebo_with_xvfb.sh | Runs the gazebo with xvfb shell workflow. | None | heron_simulator/CMakeLists.txt, heron_simulator/launch/heron_world.launch |
| multibeam_raw.py | Publish raw multibeam echosounder profile packets from Gazebo rays. | math, struct, typing, rospy | heron_simulator/CMakeLists.txt, heron_simulator/launch/spawn_heron.launch |
| ping360_profile_model.py | Pure deterministic model for canonical Ping360 simulation profiles. | math, typing | heron_simulator/CMakeLists.txt |
| ping360_profile_sim.py | Publish canonical Ping360 profiles from a full-circle Gazebo ray cloud. | hashlib, struct, sys, pathlib | grande/grande/tests/tools/validate_ping360_contract.py, heron_simulator/CMakeLists.txt, heron_simulator/launch/spawn_heron.launch |
| profile_env_run.sh | Runs the profile env run shell workflow. | None | heron_simulator/CMakeLists.txt, heron_simulator/launch/spawn_heron.launch |
| scenarios.py | Resolve simulator scenario values for integration launch files. | pathlib, typing, yaml | heron_simulator/CMakeLists.txt |
| sim_ig_timing.py | Publish IG Handle-style timing topics from simulated sensor timestamps. | typing, rospy, sensor_msgs | heron_simulator/CMakeLists.txt, heron_simulator/launch/spawn_heron.launch |
| sim_sense.py | Publish an explicitly synthetic Heron ``/sense`` contract for simulation. | time, rospy, heron_msgs, std_msgs | heron_simulator/CMakeLists.txt, heron_simulator/launch/spawn_heron.launch |
| validate_drive_to_thrusters.py | Focused simulator actuator direction and time-epoch regression checks. | argparse, importlib, json, pathlib | heron_simulator/CMakeLists.txt |
| validate_hydrodynamics_launch.py | Verify profile-owned defaults and explicit hydrodynamic launch overrides. | json, math, subprocess, xml | heron_simulator/CMakeLists.txt |
| validate_lidar_fidelity_profiles.py | Validate full-fidelity defaults and opt-in lidar performance controls. | json, pathlib | heron_simulator/CMakeLists.txt |
| validate_ping360_profile_sim.py | Validate the deterministic Ping360 profile simulator model. | math, ping360_profile_model | heron_simulator/CMakeLists.txt |
