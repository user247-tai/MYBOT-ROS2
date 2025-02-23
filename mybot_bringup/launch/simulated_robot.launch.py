import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

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
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mybot_slam"),
            "launch",
            "slam.launch.py"
        ),
        condition=IfCondition(use_slam)
    )
    
    return LaunchDescription([
        use_slam_arg,
        gazebo,
        controller,
        slam,
        navigation
    ])