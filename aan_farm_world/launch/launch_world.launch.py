from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os

world = 'aan_farm'

def generate_launch_description():
    
    ## Arguments
    headless_arg = DeclareLaunchArgument(name='headless', default_value='false', choices=['true', 'false'],
                                    description='Set to true of headless rendering is desired')

    ## Gazebo world 
    world_file = os.path.join(get_package_share_directory('aan_farm_world'), 'world', world+'.sdf')

    # Start Gazebo 
    gazebo_launch_file = os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
    gazebo_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_file]),
        condition=IfCondition(LaunchConfiguration('headless')),
        launch_arguments=[('gz_args', [' -r -v 0 -s --headless-rendering ' + world_file])],
    )
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch_file]),
        condition=UnlessCondition(LaunchConfiguration('headless')),
        launch_arguments=[('gz_args', [' -r -v 0 ' + world_file])],
    )

    # Bridge clock 
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_clock_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
    )


    ## Launch description
    return LaunchDescription([

        # Arguments
        headless_arg,

        # Launch
        gazebo, # OR
        gazebo_headless,

        # Nodes
        gz_bridge_node,

    ])
    