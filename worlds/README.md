# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| exploration_arena.world | Defines the open-water exploration arena with two finite static wall obstacles that constrain planar LiDAR registration. | water_surface model | heron_simulator/config/scenarios/exploration_arena.yaml |
| harbor.world | Defines the harbor Gazebo world. | None | heron_simulator/config/scenarios/harbor.yaml |
| open_water.world | Defines the open water Gazebo world. | None | heron_simulator/launch/heron_world.launch, heron_simulator/worlds/tank.world |
| tank.world | Defines the tank Gazebo world. | None | heron_simulator/config/entities/pool.yaml, heron_simulator/config/scenarios/pool.yaml |
