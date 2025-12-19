cmake_minimum_required(VERSION 3.0.2)
project(heron_simulator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

catkin_package()

#############################################################################
# Install directories (config, launch, worlds, urdf, models, docs)
#############################################################################

install(DIRECTORY config launch worlds urdf models docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############################################################################
# Install Python scripts
# Includes both SLAM GRANDE additions and upstream Clearpath scripts
#############################################################################

catkin_install_python(PROGRAMS
  # SLAM GRANDE scripts
  scripts/mock_defector_service.py
  scripts/sim_odom_publisher.py
  scripts/scan_to_cloud.py
  scripts/spawn_inspection_models.py
  scripts/vel_cov_fixed.py
  scripts/verify_movement.py
  # Upstream Clearpath scripts (BSD license)
  scripts/activate_control_service
  scripts/cmd_drive_translate
  scripts/navsat_vel_translate
  scripts/rpy_translator
  scripts/twist_translate
  scripts/vector3_to_magnetic_field
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############################################################################
# Install shell scripts
#############################################################################

install(PROGRAMS
  scripts/kill_sim.sh
  scripts/launch_sim.sh
  scripts/run_headless_recording.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
