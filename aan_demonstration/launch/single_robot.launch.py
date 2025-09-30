from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
        
    # Launch AAN farm world
    launch_farm_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('aan_farm_world'), 'launch', 'launch_world.launch.py')]),
    )
    
    # Launch robot
    launch_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('smart_diffbot_bringup'), 'launch', 'main.launch.py')]),
        launch_arguments=[  ('costmap_path', os.path.join(get_package_share_directory('aan_farm_world'), 'world', 'costmap', 'aan_farm.yaml')),
                            ('camera', 'true'),
                            ('x', '30'),
                            ('y', '3'),
                        ]
    )

    ## Launch description
    return LaunchDescription([

        # Launch
        launch_farm_world,
        launch_robot,

    ])
    
