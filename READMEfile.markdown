# Multi-Map Navigation ROS Package

## Overview
This ROS 2 Humble package, developed for the ANSCER Robotics assignment, implements a multi-map navigation system for a robot operating in multiple rooms. It features:

- **Separate Room Mapping**: Each room is mapped individually using TurtleBot3 simulation and Nav2's SLAM capabilities.
- **Wormhole Mechanism**: Overlapping regions (wormholes) enable seamless map transitions, with positions stored in an SQLite database.
- **C++ Action Server**: A modular action server handles navigation goals, directing the robot to navigate within the same map or through wormholes to a target map.
- **Documentation**: Clear pseudocode and modular design for easy integration and future enhancements.

The package is built for ROS 2 Humble on Ubuntu 22.04 LTS, using TurtleBot3 for simulation and Nav2 for navigation.

## Prerequisites
- **OS**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill
- **Dependencies**:
  - `ros-humble-turtlebot3`
  - `ros-humble-turtlebot3-gazebo`
  - `ros-humble-nav2-bringup`
  - `libsqlite3-dev`

## Setup Instructions
1. **Create Workspace**:
   ```bash
   mkdir -p ~/RobotMovement/src
   cd ~/RobotMovement
   colcon build
   source install/setup.bash
   ```

2. **Clone Repository**:
   ```bash
   cd ~/RobotMovement/src
   git clone <your-github-repo-url>
   ```

3. **Install Dependencies**:
   ```bash
   sudo apt update
   sudo apt install -y ros-humble-turtlebot3 ros-humble-turtlebot3-gazebo ros-humble-nav2-bringup libsqlite3-dev
   ```

4. **Build Package**:
   ```bash
   cd ~/RobotMovement
   colcon build
   source install/setup.bash
   ```

5. **Set Up Environment**:
   Add to `~/.bashrc`:
   ```bash
   echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
   echo "source ~/RobotMovement/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

## Mapping Rooms
1. **Launch Simulation for Room 1**:
   ```bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. **Map Room 1**:
   In another terminal:
   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true slam:=true
   ros2 run nav2_map_server map_saver_cli -f ~/RobotMovement/src/multi_map_navigation/maps/room1
   ```

3. **Map Room 2**:
   Stop Gazebo (Ctrl+C), then:
   ```bash
   ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true slam:=true
   ros2 run nav2_map_server map_saver_cli -f ~/RobotMovement/src/multi_map_navigation/maps/room2
   ```

## Database Setup
The SQLite database stores wormhole positions linking maps.

1. **Initialize Database**:
   ```bash
   sqlite3 ~/RobotMovement/src/multi_map_navigation/maps/wormholes.db < ~/RobotMovement/src/multi_map_navigation/sql/init_db.sql
   ```

## Algorithm Pseudocode
The navigation algorithm handles trajectory collection, storage, and execution as follows:

```
Initialize:
  - Load SQLite database for wormhole positions
  - Start action server for NavigateMultiMap
  - Set current_map to initial map (e.g., room1)

On receiving NavigateMultiMap goal (goal_pose, target_map):
  If target_map equals current_map:
    - Send goal_pose to Nav2's navigate_to_pose action
  Else:
    - Query wormhole position (x, y) from database for map_from=current_map, map_to=target_map
    - If wormhole exists:
      - Create PoseStamped for wormhole position
      - Send wormhole pose to Nav2
      - Update current_map to target_map
      - Send goal_pose to Nav2
    - Else:
      - Abort with error "No wormhole found"
  Publish feedback during navigation
  Return result "Navigation completed" on success
```

## Usage
1. **Launch Simulation**:
   ```bash
   export TURTLEBOT3_MODEL=burger
   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
   ```

2. **Launch Navigation**:
   ```bash
   ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=~/RobotMovement/src/multi_map_navigation/maps/room1.yaml
   ```

3. **Launch Action Server**:
   ```bash
   ros2 launch multi_map_navigation navigation.launch.py
   ```

4. **Send Navigation Goal**:
   ```bash
   ros2 action send_goal /navigate_multi_map multi_map_navigation/action/NavigateMultiMap "{goal_pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}, target_map: 'room1'}"
   ```
   - Replace `target_map: 'room2'` to test map switching.

## Nodes
- **navigation_server**: A C++ node implementing the `NavigateMultiMap` action server. It:
  - Queries wormhole positions from SQLite.
  - Sends navigation goals to Nav2.
  - Handles map transitions via wormholes.

## Directory Structure
```
multi_map_navigation/
├── action/
│   └── NavigateMultiMap.action
├── launch/
│   └── navigation.launch.py
├── maps/
│   ├── room1.yaml
│   ├── room1.pgm
│   ├── room2.yaml
│   ├── room2.pgm
│   └── wormholes.db
├── sql/
│   └── init_db.sql
├── src/
│   └── navigation_server.cpp
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Additional Notes
- **Modularity**: The `NavigationServer` class uses OOP principles, with separate header/source potential for extension. Topics and paths are configurable via parameters (though hardcoded for simplicity in this version).
- **Documentation**: Code includes comments, and this README provides setup and usage details.
- **Testing**: Use `simplescreenrecorder` to record simulation and navigation demos:
  ```bash
  sudo apt install -y simplescreenrecorder
  simplescreenrecorder
  ```
- **Submission**: Package the project:
  ```bash
  cd ~/RobotMovement
  tar -czvf multi_map_navigation.tar.gz src/multi_map_navigation
  ```
  Submit the `.tar.gz` and video via the provided form.

## References
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [TurtleBot3 Documentation](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
- [Nav2 Documentation](https://navigation.ros.org/)
- [ROS 2 Action Tutorials](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-An-Action-Server-Client-CPP.html)