# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| scenarios.yaml | Defines configuration for scenarios. | None | heron_simulator/scripts/scenarios.py |
| thruster_dynamics.yaml | Defines the simulation-only four-regime propulsion coefficients, electrical telemetry limits, voltage response, lag, slew, and direction-change blanking. | Physical identification pending | grande/grande/launch/bringup.launch, heron_simulator/launch/heron_world.launch, heron_simulator/launch/spawn_heron.launch |
| thruster_dynamics_session4_proxy.yaml | Defines configuration for thruster dynamics session4 proxy. | None | grande/grande/docs/simulation_web_viz_runbook.md, grande/grande/tests/tools/run_harbor_controller_comparison.sh, grande/grande/tests/tools/validate_session4_navigation_proxy.py |
