# Multi-Map Navigation ROS Package

## Overview
This ROS 2 package implements a multi-map navigation system for a robot operating in a facility with multiple rooms, as specified in the ANSCER Robotics ROS Assignment. The system enables a robot to navigate between separately mapped rooms using a "wormhole" mechanism, which represents overlapping regions between maps. Wormhole positions are stored in an SQLite database, and an action server handles navigation goals, supporting both same-map and cross-map navigation. The implementation uses C++ with Object-Oriented Programming (OOP) principles, modular node design, and ROS 2 Humble conventions.

## Features
- **Multi-Map Navigation**: Seamlessly navigates between different maps using wormholes.
- **Wormhole Mechanism**: Uses overlapping regions to transition between maps, with positions stored in an SQLite database.
- **Action Server**: Handles navigation goals with a custom `MultiMapNavigateToPose` action, integrating with `move_base` for navigation.
- **Modular Design**: Implements OOP with separate header/source files and avoids hardcoded paths/topics for flexibility.

## Dependencies
- ROS 2 Humble
- TurtleBot3 (for simulation and testing)
- SQLite3 (for wormhole database)
- `nav2_msgs` (for integration with `move_base`)
- Other ROS 2 dependencies: `rclcpp`, `rclcpp_action`, `geometry_msgs`, `nav_msgs`, `std_msgs`

## Installation
1. **Clone the Repository**:
   ```bash
   cd ~/turtlebot3_ws/src
   git clone https://github.com/<your-username>/multi_map_nav.git
   ```

2. **Build the Workspace**:
   ```bash
   cd ~/turtlebot3_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

3. **Set Up the SQLite Database**:
   - The node automatically creates a `wormholes.db` file in the workspace root upon first execution.
   - Populate the database with wormhole positions as needed (see [Database Schema](#database-schema)).

4. **Prepare Map Files**:
   - Place map files (`.yaml` and `.pgm`) in the directory specified by the `map_directory` parameter (default: `/home/<user>/turtlebot3_ws/maps`).
   - Ensure maps are created for each room using a mapping tool like `slam_toolbox` or `cartographer`.

## Usage
1. **Launch the TurtleBot3 Simulation** (for testing):
   ```bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. **Launch the Navigation Stack**:
   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true
   ```

3. **Run the Multi-Map Navigation Node**:
   ```bash
   ros2 run multi_map_nav multi_map_nav_node
   ```

4. **Send a Navigation Goal**:
   - Use the ROS 2 action CLI to send a goal to the action server:
     ```bash
     ros2 action send_goal /multi_map_navigate_to_pose multi_map_nav/action/MultiMapNavigateToPose \
       "target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}, target_map: 'room1'"
     ```

5. **Testing Map Switching**:
   - Ensure the `wormholes.db` contains entries for transitions (e.g., from `room1` to `room2`).
   - Send a goal with a different `target_map` to trigger wormhole navigation.

## Pseudocode
The core navigation algorithm handles both same-map and cross-map navigation:

```plaintext
Algorithm MultiMapNavigateToPose(goal_pose, target_map):
  Input: goal_pose (PoseStamped), target_map (string)
  Output: Success or failure of navigation

  if current_map == target_map:
    Send move_base goal to goal_pose using nav2_msgs/NavigateToPose
    Return result
  else:
    Query wormhole position from database for (current_map, target_map)
    if wormhole found:
      Navigate to wormhole_pose using move_base
      if navigation to wormhole succeeds:
        Switch to target_map (update map server)
        Initialize localization in target_map
        Navigate to goal_pose using move_base
        Return result
      else:
        Abort goal
    else:
      Abort goal with error "No wormhole found"
```

## Database Schema
The SQLite database stores wormhole positions to facilitate map transitions:

```sql
CREATE TABLE wormholes (
  id INTEGER PRIMARY KEY,
  map_from TEXT,          -- Source map name
  map_to TEXT,            -- Destination map name
  x REAL, y REAL, z REAL, -- Position of the wormhole
  orientation_x REAL, orientation_y REAL, orientation_z REAL, orientation_w REAL -- Orientation quaternion
);
```

**Example Insertion**:
```sql
INSERT INTO wormholes (map_from, map_to, x, y, z, orientation_x, orientation_y, orientation_z, orientation_w)
VALUES ('room1', 'room2', 1.5, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0);
```

## Directory Structure
```plaintext
multi_map_nav/
├── action/
│   └── MultiMapNavigateToPose.action
├── src/
│   └── multi_map_nav_node.cpp
├── include/
│   └── multi_map_nav/
│       └── multi_map_nav_node.hpp 
├── config/
│   └── params.yaml 
├── maps/
│   └── (store .yaml and .pgm map )
├── launch/
│   └── multi_map_nav_launch.py 
├── CMakeLists.txt
├── package.xml
├── README.md
```
## References
- [ROS 2 Actionlib Tutorials](https://wiki.ros.org/actionlib#Tutorials)
- [ROS 2 Database Interface](https://wiki.ros.org/database_interface)
- [ROS 2 Move Base](https://wiki.ros.org/move_base)
- [Anscer Robotics AR100](http://wiki.ros.org/AnscerRobotics/AR100)

## License
This project is licensed under the Apache-2.0 License.
