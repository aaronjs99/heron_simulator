cmake_minimum_required(VERSION 3.13)
project(heron_simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(oracle REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GAZEBO REQUIRED IMPORTED_TARGET gazebo)

#############################################################################
# Build the Gazebo plugins
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
  target_link_libraries(${HERON_SIMULATOR_PLUGIN} PkgConfig::GAZEBO)
  ament_target_dependencies(${HERON_SIMULATOR_PLUGIN}
    rclcpp
    geometry_msgs
    nav_msgs
    std_msgs
  )
endforeach()

#############################################################################
# Install directories (config, launch, worlds, urdf, models, docs)
#############################################################################
install(DIRECTORY config docs launch models urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  PassiveDisturbancePlugin
  RelativeForcePlugin
  SurfaceHydrodynamicsPlugin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

#############################################################################
# Install Python scripts
#############################################################################
ament_python_install_package(models PACKAGE_DIR scripts/models)

install(PROGRAMS
  scripts/acoustic_marker_model.py
  scripts/drive_to_thrusters.py
  scripts/multibeam_raw.py
  scripts/ping360_profile_sim.py
  scripts/scenarios.py
  scripts/sim_grounding_evidence_assessor.py
  scripts/sim_sense.py
  scripts/sim_ig_timing.py
  scripts/simulated_navigation_state.py
  scripts/spawn_acoustic_marker.py
  scripts/gazebo_with_xvfb.py
  scripts/profile_env_run.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES scripts/scenarios.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()