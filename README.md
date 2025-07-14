# Multi-Map Navigation ROS Package

## Overview
This ROS 2 package implements a multi-map navigation system for a robot operating in a facility with multiple rooms, as specified in the ANSCER Robotics ROS Assignment. The system enables seamless navigation between separately mapped rooms using a "wormhole" mechanism, which represents overlapping regions between maps. Wormhole positions are stored in an SQLite database, and a custom action server handles navigation goals, supporting both same-map and cross-map navigation. The implementation adheres to Object-Oriented Programming (OOP) principles, uses modular node design, avoids hardcoded paths/topics, and includes comprehensive documentation for integration into existing ROS systems.

## Features
- **Multi-Map Navigation**: Navigates between different rooms, each with its own map, using wormholes for transitions.
- **Wormhole Mechanism**: Defines overlapping regions between maps, stored in an SQLite database for robust map switching.
- **Action Server**: Implements a custom `MultiMapNavigateToPose` action to handle navigation goals, integrating with the `move_base` action server (`nav2_msgs/NavigateToPose`).
- **Modular Design**: Uses C++ with separate header/source files, ROS parameters for configuration, and a flexible node structure for easy integration and future enhancements.
- **Database Integration**: Stores wormhole positions in an SQLite database with a clear schema for map transitions.
- **Trajectory Management**: Collects, stores, and processes navigation trajectories via the action server, with logging for visualization and debugging.

## Dependencies
- **ROS 2 Humble**: Core framework for the package.
- **TurtleBot3**: Used for simulation and testing (e.g., `turtlebot3_gazebo`, `turtlebot3_navigation2`).
- **SQLite3**: For storing wormhole positions.
- **ROS 2 Packages**:
  - `rclcpp`, `rclcpp_action`: For node and action server/client functionality.
  - `geometry_msgs`, `nav_msgs`, `std_msgs`: For pose and map data.
  - `nav2_msgs`: For integration with `move_base` navigation.
  - `sqlite3`, `sqlite3_vendor`: For database operations.
  - `rosidl_default_generators`, `rosidl_default_runtime`: For action interface generation.

Install dependencies:
```bash
sudo apt update
sudo apt install ros-humble-nav2-msgs ros-humble-sqlite3-vendor
rosdep install --from-paths src --ignore-src -r -y
```

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
   - The node automatically creates a `wormholes.db` file in the workspace root (`~/turtlebot3_ws`) upon first execution.
   - Populate the database with wormhole positions using SQL commands (see [Database Schema](#database-schema)).

4. **Prepare Map Files**:
   - Create separate maps for each room using a mapping tool like `slam_toolbox` or `cartographer`.
   - Place map files (`.yaml` and `.pgm`) in the directory specified by the `map_directory` parameter (default: `/home/<user>/turtlebot3_ws/maps`).
   - Example mapping command:
     ```bash
     ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=true
     ```

5. **Configure Parameters** (optional):
   - Edit `config/params.yaml` to set the `map_directory` parameter if different from the default.
   - Example `params.yaml`:
     ```yaml
     multi_map_nav_node:
       ros__parameters:
         map_directory: "/home/<user>/turtlebot3_ws/maps"
     ```

## Usage
1. **Launch the TurtleBot3 Simulation** (for testing):
   ```bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. **Launch the Navigation Stack**:
   - Ensure the `move_base` action server is running:
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

5. **Test Map Switching**:
   - Ensure `wormholes.db` contains entries for transitions (e.g., from `room1` to `room2`).
   - Send a goal with a different `target_map` to trigger wormhole navigation and map switching.
   - Example:
     ```bash
     ros2 action send_goal /multi_map_navigate_to_pose multi_map_nav/action/MultiMapNavigateToPose \
       "target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}, target_map: 'room2'"
     ```

6. **Visualize Trajectories**:
   - Use RViz to visualize the robot’s trajectory:
     ```bash
     ros2 run rviz2 rviz2 -d $(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/rviz/turtlebot3_navigation2.rviz
     ```
   - Add a `Path` display subscribed to `/plan` (published by `move_base`) to visualize the navigation path.
   - The node logs trajectory points (poses) to the console, which can be used for post-processing or visualization.

## Pseudocode
The core algorithm for multi-map navigation, including trajectory collection and map switching, is as follows:

```plaintext
Algorithm MultiMapNavigateToPose(goal_pose, target_map):
  Input: goal_pose (PoseStamped), target_map (string)
  Output: Success or failure of navigation

  Initialize:
    trajectory = []  // List to store trajectory points
    current_map = get_current_map()

  if current_map == target_map:
    Send move_base goal to goal_pose using nav2_msgs/NavigateToPose
    while goal not reached:
      Collect current_pose from odometry
      Append current_pose to trajectory
    Store trajectory in memory for logging
    Return result
  else:
    Query wormhole position from database for (current_map, target_map)
    if wormhole found:
      Send move_base goal to wormhole_pose
      while navigating to wormhole:
        Collect current_pose from odometry
        Append current_pose to trajectory
      if navigation to wormhole succeeds:
        Switch to target_map (update map server)
        Initialize localization (e.g., reset AMCL)
        Send move_base goal to goal_pose
        while navigating to goal_pose:
          Collect current_pose from odometry
          Append current_pose to trajectory
        Store trajectory in memory for logging
        Return result
      else:
        Abort goal with error "Failed to reach wormhole"
    else:
      Abort goal with error "No wormhole found"
```

**Trajectory Collection and Storage**:
- **Collection**: The node subscribes to odometry or pose topics to collect the robot’s position during navigation.
- **Storage**: Trajectory points are stored in memory as a list of `geometry_msgs/PoseStamped` messages and logged to the console.
- **Visualization**: Trajectories are visualized in RViz via the `/plan` topic or logged poses, which can be exported to a file for further analysis.

## Database Schema
The SQLite database stores wormhole positions to facilitate map transitions:

```sql
CREATE TABLE wormholes (
  id INTEGER PRIMARY KEY,
  map_from TEXT,          -- Source map name (e.g., 'room1')
  map_to TEXT,            -- Destination map name (e.g., 'room2')
  x REAL, y REAL, z REAL, -- Position of the wormhole in map_from's frame
  orientation_x REAL, orientation_y REAL, orientation_z REAL, orientation_w REAL -- Orientation quaternion
);
```

**Example Insertion**:
```sql
INSERT INTO wormholes (map_from, map_to, x, y, z, orientation_x, orientation_y, orientation_z, orientation_w)
VALUES ('room1', 'room2', 1.5, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0);
```

**Populating the Database**:
- Use a script or manual SQL commands to insert wormhole data:
  ```bash
  sqlite3 ~/turtlebot3_ws/wormholes.db
  sqlite> INSERT INTO wormholes (map_from, map_to, x, y, z, orientation_x, orientation_y, orientation_z, orientation_w)
  ...> VALUES ('room1', 'room2', 1.5, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  ```

## Directory Structure
```plaintext
multi_map_nav/
├── action/
│   └── MultiMapNavigateToPose.action  # Custom action definition
├── src/
│   └── multi_map_nav_node.cpp         # Main node implementation
├── include/
│   └── multi_map_nav/
│       └── multi_map_nav_node.hpp     # Header file (optional for OOP)
├── config/
│   └── params.yaml                    # ROS parameters (e.g., map_directory)
├── maps/
│   ├── room1.yaml                     # Map files for each room
│   ├── room1.pgm
│   ├── room2.yaml
│   └── room2.pgm
├── launch/
│   └── multi_map_nav_launch.py        # Launch file (optional)
├── CMakeLists.txt                     # Build configuration
├── package.xml                        # Package metadata and dependencies
├── README.md                          # This file
```

## Testing
1. **Create Maps**:
   - Use `slam_toolbox` or `cartographer` to map each room separately.
   - Save maps in the `maps/` directory with unique names (e.g., `room1.yaml`, `room2.yaml`).

2. **Populate Wormholes**:
   - Identify overlapping regions between rooms and insert their poses into `wormholes.db`.

3. **Run Tests**:
   - Launch the simulation, navigation stack, and node as described in [Usage](#usage).
   - Test same-map navigation:
     ```bash
     ros2 action send_goal /multi_map_navigate_to_pose multi_map_nav/action/MultiMapNavigateToPose \
       "target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}, target_map: 'room1'"
     ```
   - Test cross-map navigation:
     ```bash
     ros2 action send_goal /multi_map_navigate_to_pose multi_map_nav/action/MultiMapNavigateToPose \
       "target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}, target_map: 'room2'"
     ```

4. **Visualize in RViz**:
   - Launch RViz to visualize maps, trajectories, and robot pose:
     ```bash
     ros2 run rviz2 rviz2 -d $(ros2 pkg prefix turtlebot3_navigation2)/share/turtlebot3_navigation2/rviz/turtlebot3_navigation2.rviz
     ```
   - Add displays for `Map` (`/map`), `Path` (`/plan`), and `Pose` (`/odom`).

## References
- [ROS 2 Actionlib Tutorials](https://wiki.ros.org/actionlib#Tutorials)
- [ROS 2 Database Interface](https://wiki.ros.org/database_interface)
- [ROS 2 Move Base](https://wiki.ros.org/move_base)
- [Anscer Robotics AR100](http://wiki.ros.org/AnscerRobotics/AR100)
- [TurtleBot3 Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)

## License
This project is licensed under the Apache-2.0 License.


