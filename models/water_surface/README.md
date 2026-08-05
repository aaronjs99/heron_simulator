# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| model.config | Declares the water-surface model name, version, author, and SDF entrypoint for Gazebo discovery. | model.sdf | Gazebo model discovery, simulator worlds |
| model.sdf | Defines the static water-surface visual and material reference. | Gazebo SDF | exploration_arena.world, harbor.world, open_water.world, tank.world through model://water_surface |
