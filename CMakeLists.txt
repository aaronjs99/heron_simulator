cmake_minimum_required(VERSION 3.0.2)
project(heron_simulator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

find_package(gazebo REQUIRED)

catkin_package()

#############################################################################
# Install directories (config, launch, worlds, urdf, models, docs)
#############################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(RelativeForcePlugin src/RelativeForcePlugin.cpp)
target_link_libraries(RelativeForcePlugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(PassiveDisturbancePlugin src/PassiveDisturbancePlugin.cpp)
target_link_libraries(PassiveDisturbancePlugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

install(DIRECTORY config launch worlds urdf models docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS
  PassiveDisturbancePlugin
  RelativeForcePlugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#############################################################################
# Install Python scripts
#############################################################################

catkin_install_python(PROGRAMS
  scripts/drive_to_thrusters.py
  scripts/multibeam_raw.py
  scripts/scenarios.py
  scripts/sim_sense.py
  scripts/sim_ig_timing.py
  scripts/gazebo_with_xvfb.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
