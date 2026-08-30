# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| collision_avoidance_arena.yaml | Binds the static-post collision world and its immutable evaluation geometry to a coincident map frame and start pose. | heron_simulator/worlds/collision_avoidance_arena.world, heron_simulator/config/entities/collision_avoidance_arena.yaml | heron_simulator/config/scenarios.yaml, archived GRANDE S5.0 campaign 0.10 |
| collision_avoidance_arena_westbound.yaml | Binds the static-post collision world and immutable evaluation geometry to the mirrored east-anchor start pose. | heron_simulator/worlds/collision_avoidance_arena.world, heron_simulator/config/entities/collision_avoidance_arena.yaml | heron_simulator/config/scenarios.yaml, GRANDE S5.8 campaign 1.2 |
| exploration_arena.yaml | Binds the arena world and semantic entities to its map bounds and spawn state. | heron_simulator/worlds/exploration_arena.world, heron_simulator/config/entities/exploration_arena.yaml | heron_simulator/config/scenarios.yaml, GRANDE S4.9 |
| harbor.yaml | Binds the harbor world and semantic entities to map bounds and spawn state. | heron_simulator/worlds/harbor.world, heron_simulator/config/entities/harbor.yaml | heron_simulator/config/scenarios.yaml, GRANDE scenario selection |
| pool.yaml | Binds the tank world and pool entities to map bounds and spawn state. | heron_simulator/worlds/tank.world, heron_simulator/config/entities/pool.yaml | heron_simulator/config/scenarios.yaml, GRANDE scenario selection |
| range_marker_pool.yaml | Binds the target-free tank, deep canonical RANGE_AID marker instance, and initial Heron pose for DT100 multi-view characterization; Ping360 is a planar negative control in this scenario. | heron_simulator/worlds/range_marker_pool.world, range_aid/config/markers | heron_simulator/config/scenarios.yaml, GRANDE scenario selection |
