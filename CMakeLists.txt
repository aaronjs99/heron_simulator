cmake_minimum_required(VERSION 3.0.2)
project(heron_simulator)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  rospy
  roscpp
  std_srvs
  geometry_msgs
  nav_msgs
  sensor_msgs
)

find_package(gazebo REQUIRED)

catkin_package()

#############################################################################
# Install directories (config, launch, worlds, urdf, models, docs)
#############################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(PathVisualizerPlugin src/PathVisualizerPlugin.cpp)
target_link_libraries(PathVisualizerPlugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(RelativeForcePlugin src/RelativeForcePlugin.cpp)
target_link_libraries(RelativeForcePlugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

install(DIRECTORY config launch worlds urdf models docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS
  PathVisualizerPlugin
  RelativeForcePlugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#############################################################################
# Install Python scripts
# Includes both SLAM GRANDE additions and upstream Clearpath scripts
#############################################################################

catkin_install_python(PROGRAMS
  # SLAM GRANDE scripts
  scripts/autonomy/mock_defector_service.py
  scripts/sensors/scan_to_cloud.py
  scripts/autonomy/spawn_inspection_models.py
  scripts/sensors/vel_cov_fixed.py
  scripts/tools/sim_preflight.py
  scripts/tools/gzclient_plain.py
  scripts/tools/startup_unpause.py
  scripts/tools/topic_contract_guard.py
  # Upstream Clearpath scripts (BSD license)
  scripts/control/activate_control_service.py
  scripts/control/cmd_drive_translate.py
  scripts/control/vessel_dynamics.py
  scripts/sensors/navsat_vel_translate.py
  scripts/sensors/rpy_translator.py
  scripts/control/twist_translate.py
  scripts/sensors/vector3_to_magnetic_field.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############################################################################
# Install shell scripts
#############################################################################

install(PROGRAMS
  scripts/tools/reset_gazebo.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
