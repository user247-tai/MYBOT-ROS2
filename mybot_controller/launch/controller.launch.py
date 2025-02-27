from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node


def generate_launch_description():

    convert = Node(
        package="mybot_controller",
        executable="mybot_convert",
        name="mybot_convert"
    )

    return LaunchDescription(
        [ convert ]
    )