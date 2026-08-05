# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| model.config | Declares the tank-target model name, version, author, and SDF entrypoint for Gazebo discovery. | model.sdf | Gazebo model discovery, heron_simulator/worlds/tank.world |
| model.sdf | Defines the static submerged target geometry used by the tank scenario. | Gazebo SDF | heron_simulator/worlds/tank.world through model://tank_targets |
