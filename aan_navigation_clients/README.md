# AAN navigation clients

Package containing simple examples of Python ROS 2 clients to send instructions to the smart_diffbot robot created for the Autonomous Agricultural Navigation project. Running one of these clients will result in instructions (action goals) being sent to the Nav2 stack of the smart_diffbot running in simulation. 

## Field coverage
A field cover request is sent to the robot by running:

```bash
ros2 run aan_navigation_clients field_cover_client
```

## Row following
Row following is started by running:

```bash
ros2 run aan_navigation_clients row_follow_client
```

## Docking 
Docking can be achieved by running the docking client:

```bash
ros2 run aan_navigation_clients docking_client
```