# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| grounding_evidence_models.yaml | Maps named simulation entities to declared grounding-validity outcomes; it is simulator truth, not a physical perception model. | oracle/AssessGroundingEvidence service contract | sim_grounding_evidence_assessor.py, heron_world.launch |
| scenarios.yaml | Registers named scenarios, including both collision-arena start directions and the isolated range-marker pool, and selects the default scenario. | config/scenarios/*.yaml | scripts/scenarios.py, GRANDE scenario selection |
| thruster_dynamics.yaml | Defines the simulation-only four-regime propulsion plant using the legacy full-command thrust scale plus local side/direction electrical onset and current proxies. | Legacy Heron simulator trial-run thrust table, retained electrical profile | grande/launch/bringup.launch, heron_simulator/launch/heron_world.launch, heron_simulator/launch/spawn_heron.launch |
