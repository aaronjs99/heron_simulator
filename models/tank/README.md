# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| model.config | Declares the tank model name, version, author, and SDF entrypoint for Gazebo discovery. | model.sdf | Gazebo model discovery, heron_simulator/worlds/tank.world |
| model.sdf | Defines the static tank shell, collision geometry, and visual material. | Gazebo SDF | heron_simulator/worlds/tank.world through model://tank |
