# HERON Simulator

Gazebo simulation environment for the Heron USV and the rest of the SLAM
GRANDE stack.

This package is where planning, navigation, sensing, and mission behavior can be
tested together before running on hardware.

## What The Simulator Provides

- Heron vehicle spawn and world bringup
- synthetic maritime environments and inspection targets
- simulated sensors
- topic bridges needed by MARINER and ORACLE
- mock inspection/perception support for fast autonomy testing
- Gazebo plugins for visualization and custom force behavior

## Common Uses

### Full stack simulation

```bash
roslaunch heron_simulator simulation_full.launch
```

### Simulator-only bringup

```bash
roslaunch heron_simulator simulation.launch
```

## Important Pieces

| Path | Role |
|---|---|
| `launch/` | Main simulation entrypoints |
| `scripts/autonomy/` | Runtime helpers such as model spawning and mock services |
| `scripts/control/` | Command translation between stack layers and Gazebo |
| `scripts/sensors/` | Sensor adaptation utilities |
| `src/` | Gazebo plugins and compiled simulator support |

## How It Fits The Workspace

- ORACLE sends missions
- MARINER produces plans and control commands
- the simulator publishes the world, vehicle state, and sensor streams those
  packages need

This package is the place to debug integration issues that only show up when the
full autonomy loop is closed.
