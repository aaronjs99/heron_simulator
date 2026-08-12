# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| exploration_arena.yaml | Defines simulator/evaluation reference truth for the home pose and two finite wall obstacles; S4.9 mapless exploration does not expose it as a planner-visible semantic fixture. | None | heron_simulator/config/scenarios/exploration_arena.yaml, offline evaluation |
| harbor.yaml | Defines the canonical harbor entities, zones, and semantic scenario geometry. | None | heron_simulator launch and GRANDE harbor scenario selection |
| pool.yaml | Defines the canonical pool entities, zones, and semantic scenario geometry. | None | heron_simulator launch and GRANDE pool scenario selection |
| range_marker_pool.yaml | Defines only the home and bounds of the isolated marker tank; marker geometry and placement remain in the canonical RANGE_AID descriptor and instance. | range_aid marker configuration | heron_simulator/config/scenarios/range_marker_pool.yaml |
