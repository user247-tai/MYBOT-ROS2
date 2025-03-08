import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")
    use_navigation = LaunchConfiguration("use_navigation")
    use_navslam = LaunchConfiguration("use_navslam")
    use_amcl = LaunchConfiguration("use_amcl")
    use_filters = LaunchConfiguration("use_filters")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    use_navigation_arg = DeclareLaunchArgument(
        "use_navigation",
        default_value="false"
    )

    use_navslam_arg = DeclareLaunchArgument(
        "use_navslam",
        default_value="false"
    )

    use_amcl_arg = DeclareLaunchArgument(
        "use_amcl",
        default_value="false"
    )

    use_filters_arg = DeclareLaunchArgument(
        'use_filters',
        default_value='true'
    )

    param_file_name = "navslam_params" + '.yaml'
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

    # localization = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("mybot_localization"),
    #         "launch",
    #         "global_localization.launch.py"
    #     ),
    #     condition=UnlessCondition(use_slam)
    # )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mybot_navigation"),
            "launch",
            "navigation2.launch.py"
        ),
        launch_arguments={
            'use_amcl': use_amcl,
            'use_filters': use_filters}.items()
        ,
        condition=IfCondition(use_navigation)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mybot_slam"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    navslam_navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("nav2_bringup"),
            "launch",
            "navigation_launch.py"
        ),
        condition=IfCondition(use_navslam),
        launch_arguments={'params_file': param_dir}.items()

    )

    navslam_slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("slam_toolbox"),
            "launch",
            "online_async_launch.py"
        ),
        condition=IfCondition(use_navslam)
    ) 

    # rviz_localization = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     arguments=["-d", os.path.join(
    #             get_package_share_directory("mybot_localization"),
    #             "rviz",
    #             "global_localization.rviz"
    #         )
    #     ],
    #     output="screen",
    #     parameters=[{"use_sim_time": True}],
    #     condition=UnlessCondition(use_slam)
    # )

    rviz_navslam = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                get_package_share_directory("mybot_bringup"),
                "rviz",
                "rviz_navslam.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_navslam)
    )
    
    return LaunchDescription([
        use_slam_arg,
        use_navigation_arg,
        use_navslam_arg,
        use_amcl_arg,
        use_filters_arg,
        gazebo,
        controller,
        slam,
        rviz_navslam,
        navigation,
        navslam_navigation,
        navslam_slam
    ])