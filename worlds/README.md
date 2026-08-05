# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| exploration_arena.world | Defines the open-water exploration arena with three finite static structures for planar LiDAR registration. | water_surface model | heron_simulator/config/scenarios/exploration_arena.yaml |
| harbor.world | Defines the structured harbor environment, launch area, wharves, pilings, targets, and seafloor geometry. | water_surface model | heron_simulator/config/scenarios/harbor.yaml |
| open_water.world | Defines the default open-water Gazebo environment. | ned_frame, water_surface, sand_heightmap models | heron_simulator/launch/heron_world.launch |
| tank.world | Defines the controlled tank environment and includes its tank, target, and water-surface models. | tank, tank_targets, water_surface models | heron_simulator/config/scenarios/pool.yaml |
