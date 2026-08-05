# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| scenarios.yaml | Registers named scenarios and selects the default scenario. | config/scenarios/*.yaml | scripts/scenarios.py, GRANDE scenario selection |
| thruster_dynamics.yaml | Defines the simulation-only four-regime propulsion coefficients, electrical telemetry limits, voltage response, lag, slew, and direction-change blanking. | Physical identification pending | grande/grande/launch/bringup.launch, heron_simulator/launch/heron_world.launch, heron_simulator/launch/spawn_heron.launch |
| thruster_dynamics_session4_proxy.yaml | Preserves the explicitly selected Session 4 replay propulsion condition; it is not the canonical simulator plant. | Session 4 evidence proxy | drive_to_thrusters.py when passed as the selected model |
