# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| exploration_arena.yaml | Binds the arena world and semantic entities to its map bounds and spawn state. | heron_simulator/worlds/exploration_arena.world, heron_simulator/config/entities/exploration_arena.yaml | heron_simulator/config/scenarios.yaml, GRANDE S4.9 |
| harbor.yaml | Binds the harbor world and semantic entities to map bounds and spawn state. | heron_simulator/worlds/harbor.world, heron_simulator/config/entities/harbor.yaml | heron_simulator/config/scenarios.yaml, GRANDE scenario selection |
| pool.yaml | Binds the tank world and pool entities to map bounds and spawn state. | heron_simulator/worlds/tank.world, heron_simulator/config/entities/pool.yaml | heron_simulator/config/scenarios.yaml, GRANDE scenario selection |
