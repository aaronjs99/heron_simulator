# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| collision_avoidance_arena.world | Defines one direct-chord cylindrical obstacle inside four nonblocking registration walls for mirrored route and terminal-rotation qualification. | water_surface model | collision_avoidance_arena and collision_avoidance_arena_westbound scenario definitions |
| dock_island.world | Defines the dock and optional island geometry for S8.5 frontier scenarios. | water_surface model | S8.5 frontier scenario definitions |
| exploration_arena.world | Defines the open-water exploration arena with three finite static structures for planar LiDAR registration. | water_surface model | heron_simulator/config/scenarios/exploration/scenario.yaml |
| harbor.world | Defines the structured harbor environment, launch area, wharves, pilings, targets, and seafloor geometry. | water_surface model | heron_simulator/config/scenarios/harbor/scenario.yaml |
| open_water_course.world | Defines open-water route geometry for S8.5 navigation maneuvers. | water_surface model | S8.5 navigation scenario definitions |
| open_water.world | Defines the default open-water Gazebo environment. | ned_frame, water_surface, sand_heightmap models | heron_simulator/launch/heron_world.launch |
| range_marker_pool.world | Defines an isolated controlled tank without pool registration or inspection fixtures; launch spawns the descriptor-driven marker separately. | tank and water_surface models | heron_simulator/config/scenarios/range_marker_pool/scenario.yaml |
| tank.world | Defines the controlled tank environment and composes the tank, registration-landmark, inspection-target, and water-surface models. | tank, pool_registration_landmarks, pool_inspection_targets, water_surface models | heron_simulator/config/scenarios/pool/scenario.yaml |
| navigation_room.world | Defines a clear bounded room observable within the 6 m mapping limit for nominal translation and turn tests. | water_surface model | navigation_room scenario |
