# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| collision_avoidance_arena.yaml | Freezes the exact center-post cylinder used by independent padded-hull clearance evaluation, plus nonblocking lidar registration structures. | None | heron_simulator/config/scenarios/collision_avoidance_arena.yaml, archived GRANDE S5.0 campaign 0.10 |
| exploration_arena.yaml | Defines simulator/evaluation reference truth for the home pose and three finite wall structures. | None | heron_simulator/config/scenarios/exploration_arena.yaml, offline evaluation |
| exploration_decision_target.yaml | Exposes only the declared wall target, multiple comparable sensing viewpoints, and resolution-independent decision-support elements; navigation geometry still comes from live simulated sensing and mapping. | exploration_arena.world | GRANDE S5.9 decision-directed exploration qualification |
| harbor.yaml | Defines the canonical harbor entities, zones, and semantic scenario geometry. | None | heron_simulator launch and GRANDE harbor scenario selection |
| pool.yaml | Defines the canonical pool entities, zones, and semantic scenario geometry. | None | heron_simulator launch and GRANDE pool scenario selection |
| range_marker_pool.yaml | Defines only the home and bounds of the isolated marker tank; marker geometry and placement remain in the canonical RANGE_AID descriptor and instance. | range_aid marker configuration | heron_simulator/config/scenarios/range_marker_pool.yaml |
