# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| collision_avoidance_arena.world | Defines one direct-chord cylindrical obstacle and three nonblocking registration structures for bounded deployment-path collision evidence. | water_surface model | heron_simulator/config/scenarios/collision_avoidance_arena.yaml, GRANDE S5.0 campaign 0.10 |
| exploration_arena.world | Defines the open-water exploration arena with three finite static structures for planar LiDAR registration. | water_surface model | heron_simulator/config/scenarios/exploration_arena.yaml |
| harbor.world | Defines the structured harbor environment, launch area, wharves, pilings, targets, and seafloor geometry. | water_surface model | heron_simulator/config/scenarios/harbor.yaml |
| open_water.world | Defines the default open-water Gazebo environment. | ned_frame, water_surface, sand_heightmap models | heron_simulator/launch/heron_world.launch |
| range_marker_pool.world | Defines an isolated controlled tank without legacy symmetric target geometry; launch spawns the descriptor-driven marker separately. | tank and water_surface models | heron_simulator/config/scenarios/range_marker_pool.yaml |
| tank.world | Defines the controlled tank environment and includes its tank, target, and water-surface models. | tank, tank_targets, water_surface models | heron_simulator/config/scenarios/pool.yaml |
