import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")
    use_amcl = LaunchConfiguration("use_amcl")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    use_amcl_arg = DeclareLaunchArgument(
        "use_amcl",
        default_value="false"
    )

    param_file_name = "neo_navigation_params" + '.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('mybot_bringup'),
            'config',
            param_file_name))

    gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mybot_description"),
            "launch",
            "gazebo.launch.py"
        )
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mybot_controller"),
            "launch",
            "controller.launch.py"
        )
    )
    
    nav2_launch_file_dir = os.path.join(get_package_share_directory('mybot_navigation'), 'launch')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation2.launch.py']),
        launch_arguments={
            'use_amcl': use_amcl
            }.items(),
        condition = UnlessCondition(use_slam)
        
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mybot_slam"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    rviz_config_dir = os.path.join(
    get_package_share_directory('nav2_bringup'),
    'rviz',
    'nav2_default_view.rviz')

    rviz_navigation = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=UnlessCondition(use_slam)
    )

    rviz_slam = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("mybot_bringup"),
                "rviz",
                "rviz_slam.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_slam)
    )
    
    return LaunchDescription([
        use_amcl_arg,
        use_slam_arg,
        gazebo,
        controller,
        slam,
        navigation,
        rviz_navigation,
        rviz_slam
    ])