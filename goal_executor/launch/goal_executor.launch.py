# imu.launch.py
import launch
import launch_ros.actions

def generate_launch_description():

    # Define node
    goal_executor_node = launch_ros.actions.Node(
        package='goal_executor',
        executable='goal_executor',
        name='goal_executor',
        output='screen'
    )

    # Create launch description
    return launch.LaunchDescription([
        goal_executor_node
    ])