import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    
    param_file_name = "nav2_params" + '.yaml'

    map_name = LaunchConfiguration("map_name")

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('mybot_navigation'),
            'config',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz')

    navigation_config_arg = DeclareLaunchArgument(
        'params_file',
        default_value=param_dir,
        description='Full path to param file to load'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="small_house"
    )

    map_path = PathJoinSubstitution([
        get_package_share_directory("mybot_navigation"),
        "maps",
        map_name,
        "map.yaml"
    ])

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    return LaunchDescription([
        map_name_arg,
        navigation_config_arg,
        use_sim_time_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_path,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'),
    ])