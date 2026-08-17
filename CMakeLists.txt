cmake_minimum_required(VERSION 3.13)
project(heron_simulator)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GAZEBO REQUIRED IMPORTED_TARGET gazebo)

catkin_package(CATKIN_DEPENDS geometry_msgs roscpp std_msgs)

#############################################################################
# Install directories (config, launch, worlds, urdf, models, docs)
#############################################################################
add_library(RelativeForcePlugin src/RelativeForcePlugin.cpp)
add_library(PassiveDisturbancePlugin src/PassiveDisturbancePlugin.cpp)
add_library(SurfaceHydrodynamicsPlugin src/SurfaceHydrodynamicsPlugin.cpp)

set(HERON_SIMULATOR_PLUGINS
  RelativeForcePlugin
  PassiveDisturbancePlugin
  SurfaceHydrodynamicsPlugin
)
foreach(HERON_SIMULATOR_PLUGIN ${HERON_SIMULATOR_PLUGINS})
  target_include_directories(${HERON_SIMULATOR_PLUGIN} PRIVATE
    ${catkin_INCLUDE_DIRS}
  )
  target_link_libraries(${HERON_SIMULATOR_PLUGIN}
    PkgConfig::GAZEBO
    ${catkin_LIBRARIES}
  )
endforeach()

install(DIRECTORY config docs launch models urdf worlds
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
  scripts/gazebo_with_xvfb.py
  scripts/profile_env_run.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(HERON_SIMULATOR_PYTHON_MODULES
  scripts/empirical_actuator_proxy.py
  scripts/four_regime_propulsion.py
  scripts/ping360_profile_model.py
)

install(FILES
  ${HERON_SIMULATOR_PYTHON_MODULES}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES scripts/scenarios.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)

file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})
foreach(HERON_SIMULATOR_PYTHON_MODULE ${HERON_SIMULATOR_PYTHON_MODULES})
  get_filename_component(HERON_SIMULATOR_PYTHON_MODULE_FILE
                         ${HERON_SIMULATOR_PYTHON_MODULE} NAME)
  get_filename_component(HERON_SIMULATOR_PYTHON_MODULE_NAME
                         ${HERON_SIMULATOR_PYTHON_MODULE} NAME_WE)
  set(HERON_SIMULATOR_PYTHON_MODULE_DEVEL
      ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION}/${HERON_SIMULATOR_PYTHON_MODULE_FILE})
  add_custom_target(${HERON_SIMULATOR_PYTHON_MODULE_NAME}_devel ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/${HERON_SIMULATOR_PYTHON_MODULE}
            ${HERON_SIMULATOR_PYTHON_MODULE_DEVEL}
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/${HERON_SIMULATOR_PYTHON_MODULE}
    VERBATIM
  )
endforeach()
