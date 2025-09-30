from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ## Arguments
    config_arg = DeclareLaunchArgument(name='config', default_value='single', choices=['single', 'multi'],
                                    description='Configuration: "single" or "multi" robots.')

    ## Nodes
    # Rviz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', [get_package_share_directory('aan_demonstration'), '/launch', '/rviz/', LaunchConfiguration('config'), '_robot.rviz']],
    )

    ## Launch description
    return LaunchDescription([

        # Arguments
        config_arg,
        
        # Nodes
        rviz

    ])
    
