import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    
    param_file_name = "nav2_params" + '.yaml'

    map_name = LaunchConfiguration("map_name")

    use_amcl = LaunchConfiguration("use_amcl")

    use_amcl_arg = DeclareLaunchArgument(
        "use_amcl",
        default_value="false"
    )

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('mybot_navigation'),
            'config',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    navigation_config_arg = DeclareLaunchArgument(
        'params_file',
        default_value=param_dir,
        description='Full path to param file to load'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="real_house"
    )

    map_path = PathJoinSubstitution([
        get_package_share_directory("mybot_navigation"),
        "maps",
        map_name,
        "map.yaml"
    ])

    use_sim_time = LaunchConfiguration('use_sim_time')

    navigation_amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
        launch_arguments={
            'map': map_path,
            'use_sim_time': use_sim_time,
            'params_file': param_dir}.items(),
        condition = IfCondition(use_amcl)
        
    )

    navigation_neo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("nav2_bringup"),
            "launch",
            "navigation_launch.py"
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': param_dir}.items(),
        condition=UnlessCondition(use_amcl)
    )

    neo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("neo_localization2"),
            "launch",
            "test_setup.launch.py"
        ),
        condition=UnlessCondition(use_amcl)
    )

    return LaunchDescription([
        map_name_arg,
        navigation_config_arg,
        use_sim_time_arg,
        use_amcl_arg,
        navigation_amcl,
        navigation_neo,
        neo,
    ])