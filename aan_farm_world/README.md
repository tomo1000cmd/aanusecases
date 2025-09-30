# AAN farm world

This package contains the modeled farm world used in AAN and the launch configurations to start it in Gazebo. 
Tested for ROS 2 Humble and Gazebo Fortress. 

### Dependencies:
Make sure the following packages are installed:

```bash
sudo apt install ignition-fortress
sudo apt install ros-humble-ros-gz
```

### Install
Clone this package to the source folder of your ROS 2 workspace. Navigate to your ROS 2 workspace folder and colcon build:
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install 
source install/setup.bash
```

## Launch world

Launch the world with:
```bash
ros2 launch aan_farm_world launch_world.launch.py
```