# Navigation room

A nominal navigation fixture with four visible walls, each 3.1 m from the origin to its inner face. The existing simulated LiDAR and IMU build the map and odometry; this fixture supplies no occupancy map. Wall returns around the two central test motions lie within the shared 6 m mapping limit. It isolates ordinary navigation from frontier coverage and tight-wall edge cases.

## File Structure

| File | Purpose |
|---|---|
| README.md | Scope and fixture interpretation. |
| scenario.yaml | Existing simulator scenario interface: world, bounds and spawn. |
| entities.yaml | Empty semantic catalog; navigation uses measured relative poses. |
