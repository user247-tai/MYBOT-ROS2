import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    use_slam = LaunchConfiguration("use_slam")
    use_navigation = LaunchConfiguration("use_navigation")
    use_navslam = LaunchConfiguration("use_navslam")
    use_param = LaunchConfiguration("params_file")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    use_navigation_arg = DeclareLaunchArgument(
        "use_navigation",
        default_value="true"
    )

    use_navslam_arg = DeclareLaunchArgument(
        "use_navslam",
        default_value="false"
    )

    param_file_name = "navslam_params" + '.yaml'
    param_dir_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            get_package_share_directory('mybot_bringup'),
            'config',
            param_file_name)
    )

    control = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mybot_control"),
            "launch",
            "mybot_hw_control.launch.py"
        ),
    )

    convert = Node(
        package="mybot_controller",
        executable="mybot_convert",
        name="mybot_convert"
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mybot_slam"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mybot_navigation"),
            "launch",
            "navigation2.launch.py"
        ),
        condition=IfCondition(use_navigation)
    )

    # navslam_navigation = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("nav2_bringup"),
    #         "launch",
    #         "navigation_launch.py"
    #     ),
    #     condition=IfCondition(use_navslam),
    #     launch_arguments={'params_file': '/home/tai/mybot_workspace/src/mybot_bringup/config/navslam_params.yaml'}.items()

    # )

    # navslam_slam = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("slam_toolbox"),
    #         "launch",
    #         "online_async_launch.py"
    #     ),
    #     condition=IfCondition(use_navslam)
    # ) 

    # navslam_slam = GroupAction([
    #     TimerAction(
    #         period=5.0,
    #         actions=[
    #             IncludeLaunchDescription(
    #                 PythonLaunchDescriptionSource(os.path.join(
    #                     get_package_share_directory("slam_toolbox"),
    #                     "launch",
    #                     "online_async_launch.py"
    #                 ))
    #             )
    #         ]
    #     )
    # ], condition=IfCondition(use_navslam))

    return LaunchDescription([
        use_slam_arg,
        use_navigation_arg,
        use_navslam_arg,
        param_dir_arg,
        control,
        convert,
        slam,
        navigation,
        # navslam_slam,
        # navslam_navigation,
    ])