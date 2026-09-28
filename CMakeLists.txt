cmake_minimum_required(VERSION 3.22)
project(heron_simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)

configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/gz_sim_system_plugin_path.sh.in"
  "${CMAKE_CURRENT_BINARY_DIR}/gz_sim_system_plugin_path.sh"
  @ONLY
)
ament_environment_hooks("${CMAKE_CURRENT_BINARY_DIR}/gz_sim_system_plugin_path.sh")
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/gz_sim_resource_path.sh.in"
  "${CMAKE_CURRENT_BINARY_DIR}/gz_sim_resource_path.sh"
  @ONLY
)
ament_environment_hooks("${CMAKE_CURRENT_BINARY_DIR}/gz_sim_resource_path.sh")

set(HERON_SIMULATOR_PLUGINS
  RelativeForcePlugin
  PassiveDisturbancePlugin
  SurfaceHydrodynamicsPlugin
)

add_library(RelativeForcePlugin SHARED src/RelativeForcePlugin.cpp)
add_library(PassiveDisturbancePlugin SHARED src/PassiveDisturbancePlugin.cpp)
add_library(SurfaceHydrodynamicsPlugin SHARED src/SurfaceHydrodynamicsPlugin.cpp)

foreach(plugin IN LISTS HERON_SIMULATOR_PLUGINS)
  target_link_libraries(${plugin}
    gz-sim8::gz-sim8
    gz-math7::gz-math7
    gz-plugin2::gz-plugin2-register
  )
endforeach()
target_link_libraries(RelativeForcePlugin
  gz-msgs10::gz-msgs10
  gz-transport13::gz-transport13
)

install(TARGETS ${HERON_SIMULATOR_PLUGINS}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)
ament_export_libraries(${HERON_SIMULATOR_PLUGINS})

ament_python_install_package(models PACKAGE_DIR scripts/models)
ament_python_install_package(scenario PACKAGE_DIR scripts/scenario)

#############################################################################
# Install directories (config, launch, worlds, urdf, models, docs)
#############################################################################
install(DIRECTORY config docs launch models urdf worlds
  DESTINATION share/${PROJECT_NAME}
  PATTERN "*.launch" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(PROGRAMS
  scripts/acoustic_marker_model.py
  scripts/drive_to_thrusters.py
  scripts/multibeam_raw.py
  scripts/ping360_profile_sim.py
  scripts/sim_sense.py
  scripts/sim_ig_timing.py
  scripts/gazebo_with_xvfb.py
  scripts/profile_env_run.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
