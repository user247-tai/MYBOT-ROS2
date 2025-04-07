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
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    use_rviz = LaunchConfiguration('use_rviz', default='true')

    map_name = LaunchConfiguration("map_name")

    use_filters = LaunchConfiguration("use_filters")

    map_path = PathJoinSubstitution([
        get_package_share_directory("mybot_navigation"),
        "maps",
        map_name,
        "map.yaml"
    ])

    use_amcl = LaunchConfiguration("use_amcl")


    costmap_filter_dir = get_package_share_directory('nav2_costmap_filters_demo')
    cosmap_filter_launch_dir = os.path.join(costmap_filter_dir, 'launch')

    param_file_name = "nav2_params" + '.yaml'
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

    return LaunchDescription([

        DeclareLaunchArgument(
            "map_name",
            default_value="small_house",
            description="Map name"),

        DeclareLaunchArgument(
            "use_amcl",
            default_value="false",
            description="Use AMCL or another for localization"),      

        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'use_filters',
            default_value='true',
            description='Use filters in map'),

        #AMCL_Navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map_path,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
            condition = IfCondition(use_amcl)
        ),

        #Neo Navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_launch.py']),
            launch_arguments={'params_file': param_dir}.items(),
            condition=UnlessCondition(use_amcl)
        ),

        #Neo
        IncludeLaunchDescription(
            os.path.join(
                get_package_share_directory("neo_localization2"),
                "launch",
                "test_setup.launch.py"
            ),
            condition=UnlessCondition(use_amcl)
        ),

        # #Keepout filter
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(cosmap_filter_launch_dir, 'costmap_filter_info.launch.py')),
        #     launch_arguments={
        #         'params_file': '/home/tai/mybot_workspace/src/nav2_costmap_filters_demo/params/keepout_params.yaml',
        #         'mask': '/home/tai/mybot_workspace/src/mybot_navigation/filter/keepout_mask.yaml',
        #     }.items(),
        #     condition=IfCondition(use_filters)
        # ),

        # #Speed filter
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(cosmap_filter_launch_dir, 'costmap_filter_info.launch.py')),
        #     launch_arguments={
        #         'params_file': '/home/tai/mybot_workspace/src/nav2_costmap_filters_demo/params/speed_params.yaml',
        #         'mask': '/home/tai/mybot_workspace/src/mybot_navigation/filter/speed_mask.yaml',
        #     }.items(),
        #     condition=IfCondition(use_filters)
        # ),
        
        #Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
            output='screen'),
    ])