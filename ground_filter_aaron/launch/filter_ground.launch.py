from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration("config")
    namespace = LaunchConfiguration("namespace")
    node_name = LaunchConfiguration("node_name")

    default_config = PathJoinSubstitution([
        FindPackageShare("ground_filter_aaron"),
        "config",
        "filter_ground.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument("node_name", default_value="filter_ground"),
        Node(
            package="ground_filter_aaron",
            executable="filter_ground",
            name=node_name,
            namespace=namespace,
            output="screen",
            parameters=[config],
        ),
    ])
