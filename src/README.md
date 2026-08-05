# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| PassiveDisturbancePlugin.cpp | Applies deterministic mean and sinusoidal body-frame force and torque disturbances. | Gazebo Classic, Ignition Math | heron_simulator/CMakeLists.txt, heron_description/urdf/heron.urdf.xacro |
| RelativeForcePlugin.cpp | Applies timeout-guarded ROS wrench commands as body-relative Gazebo force and torque. | ROS, Gazebo Classic, Boost | heron_simulator/CMakeLists.txt, heron_description/urdf/heron.urdf.xacro, heron_description/urdf/snippets.xacro |
| SurfaceHydrodynamicsPlugin.cpp | Applies hydrostatic heave, roll, and pitch restoring terms plus linear and quadratic body damping. | Gazebo Classic, Ignition Math | heron_simulator/CMakeLists.txt, heron_description/urdf/heron.urdf.xacro |
