from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node


def generate_launch_description():
    
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033",
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.3",
    )

    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "joint_state_broadcaster",
    #         "--controller-manager",
    #         "/controller_manager",
    #     ]
    # )

    # wheel_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["mybot_controller", 
    #                "--controller-manager", 
    #                "/controller_manager"
    #     ]
    # )

    convert = Node(
        package="mybot_controller",
        executable="mybot_convert",
        name="mybot_convert"
    )

    return LaunchDescription(
        [
            # wheel_radius_arg,
            # wheel_separation_arg,
            # joint_state_broadcaster_spawner,
            # wheel_controller_spawner,
            convert
        ]
    )