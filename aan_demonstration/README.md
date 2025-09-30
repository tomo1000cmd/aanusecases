# AAN demonstration

This package contains launch configurations for AAN demonstrations. 
It uses:
- The simulated farm from `aan_farm_world`
- Simulated SMART diffbot(s) from `smart_diffbot`
- Clients that send instructions to the robot(s) from `aan_navigation_clients`


## Farm with single robot

### Launch

Launch the farm world with a single robot:
```bash
ros2 launch aan_demonstration single_robot.launch.py
```

RViz can be used to monitor the robot:
```bash
ros2 launch aan_demonstration rviz.launch.py
```

To view the map, switch on the AerialMapDisplay in the Displays window of RViz.

### Demos 

#### Open field navigation
```bash
ros2 run aan_navigation_clients field_cover_client 
```

#### Row following
```bash
ros2 run aan_navigation_clients row_follow_client 
```

#### Docking 
```bash
ros2 run aan_navigation_clients docking_client
```

#### Full demo (all three scenarios sequentially)
```bash
ros2 run aan_navigation_clients full_demo
```

## Farm with two robots 

### Launch

Launch the farm world with two robots:
```bash
ros2 launch aan_demonstration multi_robot.launch.py
```
RViz can be used to monitor the robots:
```bash
ros2 launch aan_demonstration rviz.launch.py config:=multi
```

To view the map, switch on the AerialMapDisplay in the Displays window of RViz.

### Demos 
The same navigation clients can be used as with a single robot as long as the robot name is given to the node. For example:
```bash
ros2 run aan_navigation_clients field_cover_client diffbot_2
```

Be aware that at this moment only diffbot_1 is equipped a camera, so diffbot_2 can only execute tasks that do not require camera images. The main purpose of this configuration is to demonstrate namespacing of robots. 