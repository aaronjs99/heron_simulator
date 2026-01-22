^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package heron_simulator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.0 (2026-01-22)
------------------
* Completely removed dependencies on the UUV Simulator framework.
* Implemented a custom high-fidelity Python-based Vessel Dynamics Engine (Fossen model).
* Replaced UUV Gazebo plugins with standard libgazebo_ros_force and libgazebo_ros_joint_state_publisher.
* Refactored simulation scripts into a categorized directory structure (control, sensors, autonomy, utils).
* Renamed all internal scripts to use .py extensions for consistency and ROS Noetic compliance.
* Updated documentation (hydrodynamics.md, nodes.md) to reflect the new architecture with a professional and academic tone.
* Optimized Gazebo world files (lake.world, ocean_surface.world) by removing legacy UUV plugins.
