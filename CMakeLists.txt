cmake_minimum_required(VERSION 3.0.2)
project(heron_simulator)

find_package(catkin REQUIRED COMPONENTS
  ig_handle
  range_aid
  gazebo_msgs
  rospy
  roscpp
  geometry_msgs
  sensor_msgs
  std_msgs
)

find_package(gazebo REQUIRED)

catkin_package(CATKIN_DEPENDS ig_handle range_aid)

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

add_library(SurfaceHydrodynamicsPlugin src/SurfaceHydrodynamicsPlugin.cpp)
target_link_libraries(SurfaceHydrodynamicsPlugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

install(DIRECTORY config launch worlds urdf models docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS
  PassiveDisturbancePlugin
  RelativeForcePlugin
  SurfaceHydrodynamicsPlugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#############################################################################
# Install Python scripts
#############################################################################

catkin_install_python(PROGRAMS
  scripts/acoustic_marker_model.py
  scripts/drive_to_thrusters.py
  scripts/multibeam_raw.py
  scripts/ping360_profile_sim.py
  scripts/scenarios.py
  scripts/sim_sense.py
  scripts/sim_ig_timing.py
  scripts/spawn_acoustic_marker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/gazebo_with_xvfb.sh
  scripts/profile_env_run.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  scripts/four_regime_propulsion.py
  scripts/ping360_profile_model.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
set(PING360_PROFILE_MODEL_DEVEL
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/ping360_profile_model.py)
add_custom_target(ping360_profile_model_devel ALL
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${CMAKE_CURRENT_SOURCE_DIR}/scripts/ping360_profile_model.py
          ${PING360_PROFILE_MODEL_DEVEL}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/ping360_profile_model.py
  VERBATIM
)

set(FOUR_REGIME_PROPULSION_DEVEL
    ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/four_regime_propulsion.py)
add_custom_target(four_regime_propulsion_devel ALL
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
          ${CMAKE_CURRENT_SOURCE_DIR}/scripts/four_regime_propulsion.py
          ${FOUR_REGIME_PROPULSION_DEVEL}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/four_regime_propulsion.py
  VERBATIM
)
