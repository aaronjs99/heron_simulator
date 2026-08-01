# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| PassiveDisturbancePlugin.cpp | Implements the PassiveDisturbancePlugin C++ component. | cmath, string, gazebo/common/Events.hh, gazebo/common/Plugin.hh | heron_simulator/CMakeLists.txt |
| RelativeForcePlugin.cpp | Implements the RelativeForcePlugin C++ component. | string, boost/bind.hpp, boost/thread.hpp, boost/thread/mutex.hpp | heron_simulator/CMakeLists.txt |
| SurfaceHydrodynamicsPlugin.cpp | Implements a stable surface-vessel hydrostatic and body-damping model without the obsolete UUV Simulator dependency. | Gazebo Classic, Ignition Math | heron_simulator/CMakeLists.txt, heron_description/urdf/heron.urdf.xacro |
