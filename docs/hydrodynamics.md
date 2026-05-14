# Heron simulator hydrodynamics

This note explains what is actually modeled in the simulator and where to tune
it when vehicle motion looks wrong.

## Model in one line

The simulator applies thrust-generated wrench through the Gazebo Heron model and
its active hydrodynamic plugins. There is no separate `vessel_dynamics.py`
publisher in the supported bringup path.

## What is modeled

- thrust from normalized `heron_msgs/Drive` commands, mapped through an
  empirical non-linear curve
- linear and quadratic damping terms per axis
- hydrostatic buoyancy/restoring behavior for roll and pitch stability
- rigid-body coupling terms from the active Gazebo hydrodynamics plugins

## Where it is implemented

- thrust mapping and thruster wrench publication:
  `heron_simulator/scripts/control/cmd_drive_translate.py`
- benchmark hull profile used by the sim stack:
  `heron/heron_description/urdf/configs/ig_handle_benchmark`
- passive external wrench hook:
  `/hydro_forces` remains in the URDF for Gazebo compatibility, but no
  supported launch publishes to it by default

## High-impact tuning order

If the boat feels unrealistic, tune in this order:

1. `cmd_drive_translate.py` thrust mapping (first-order effect on surge/yaw)
2. active Gazebo hydrodynamic plugin parameters in the Heron model
3. hull/inertia profile in `ig_handle_benchmark` (global behavior shift)

Do not tune all three at once; isolate one layer per run.

## Quick validation checks

Run simulator and watch command-to-motion response:

```bash
rostopic hz /cmd_drive
rostopic hz /thrusters/0/input
rostopic hz /thrusters/1/input
rostopic echo -n 5 /state/odometry
```

If `/cmd_drive` changes but thruster wrench topics do not, start with
`cmd_drive_translate.py`. If wrench topics move but odometry response is
unrealistic, inspect the active Gazebo hydrodynamic plugins and hull profile
values.

## Scope boundary

This file is about simulator physics only. For integration bringup, use
`slam_grande/docs/runtime.md`. For simulator node ownership and topic contracts,
use `heron_simulator/docs/nodes.md` and `heron_simulator/docs/topic_contract.md`.
