# DevRob Waypoints – ROS 2 + MoveIt 2 Assessment

### Overview
This package demonstrates commanding a robot arm through multiple waypoints using **MoveIt 2**.
It connects to an existing MoveIt 2 environment (Panda demo) and executes a sequence of poses
defined in a YAML file.

### Stack
- **ROS 2 Humble**
- **MoveIt 2**
- **Panda arm (moveit_resources_panda_moveit_config)**
- **C++ MoveGroupInterface**
- **YAML-CPP**

### Features
✅ Loads waypoints from YAML  
✅ Copies robot parameters from the `move_group` node  
✅ Plans and executes trajectories sequentially  
✅ Logs planning/execution status

### Repository structure
devrob_waypoints/
├── CMakeLists.txt
├── package.xml
├── launch/
│ └── waypoints_demo.launch.py
├── config/
│ └── waypoints.yaml
└── src/
└── waypoint_node.cpp

bash
Copy code

### Build
```bash
cd ~/devrob_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select devrob_waypoints --symlink-install
source install/setup.bash
Run
Terminal 1 – Bring up MoveIt + Panda

bash
Copy code
source /opt/ros/humble/setup.bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
Terminal 2 – Run waypoint node

bash
Copy code
cd ~/devrob_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run devrob_waypoints waypoint_node --ros-args \
  -p planning_group:=panda_arm \
  -p waypoints_file:=/home/yash/devrob_ws/install/devrob_waypoints/share/devrob_waypoints/config/waypoints.yaml
Example Output
vbnet
Copy code
Copied string parameter 'robot_description' from move_group
Connected to MoveGroupInterface
Planning to waypoint 0: 'home'
Plan to 'home' succeeded. Executing...
Finished executing all waypoints.
Notes
Waypoints are defined in config/waypoints.yaml

Robot model parameters are automatically copied from /move_group

Any MoveIt-supported robot can be used by changing the planning group and config files
