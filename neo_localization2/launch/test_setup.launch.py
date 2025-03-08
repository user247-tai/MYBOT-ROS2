import launch
import launch.actions
import launch.substitutions
import os
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    config = os.path.join(get_package_share_directory('neo_localization2'),'launch','test_setup.yaml')
    map_name = LaunchConfiguration("map_name")
    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="small_house"
    )

    lifecycle_nodes = ["map_server"]

    map_path = PathJoinSubstitution([
        get_package_share_directory("neo_localization2"),
        "maps",
        map_name,
        "map.yaml"
    ])

    return launch.LaunchDescription([
        map_name_arg,
        
        launch_ros.actions.Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": True},
            {"autostart": True}],),

        launch_ros.actions.Node(
            package='neo_localization2', executable='neo_localization_node', output='screen',
            name='neo_localization2_node', parameters = [config]),

        launch_ros.actions.Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_path},
            {"use_sim_time": True}])
    ])