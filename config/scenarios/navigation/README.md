# File Structure

| File | Relevance | Dependencies | Used by |
| --- | --- | --- | --- |
| entities.yaml | Declares an empty semantic catalog so navigation uses measured relative poses. | Scenario schema | Navigation-room scenario |
| scenario.yaml | Selects a bounded room whose wall inner faces are 3.1 m from the origin, within the 6 m mapping range; simulated LiDAR/IMU build the map and state without a privileged occupancy map. | navigation_room.world, simulator scenario interface | S8.5 nominal navigation cases |
