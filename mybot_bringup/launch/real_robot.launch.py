import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # use_slam_arg = DeclareLaunchArgument(
    #     "use_slam",
    #     default_value="false"
    # )

    # use_slam = LaunchConfiguration("use_slam")

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mybot_hw_interface"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("mybot_controller"),
            "launch",
            "controller.launch.py"
        ),
    )

    # slam = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("mybot_slam"),
    #         "launch",
    #         "slam.launch.py"
    #     ),
    #     launch_arguments={
    #         "use_sim_time": "false"
    #     }.items(),
    #     condition=IfCondition(use_slam)
    #)

    # navigation = IncludeLaunchDescription(
    #     os.path.join(
    #         get_package_share_directory("mybot_navigation"),
    #         "launch",
    #         "navigation2.launch.py"
    #     ),
    #     launch_arguments={
    #         "use_sim_time": "false"
    #     }.items(),
    #     condition=UnlessCondition(use_slam)
    # )

    return LaunchDescription([
        #use_slam_arg,
        hardware_interface,
        controller#,
        # slam,
        # navigation
    ])