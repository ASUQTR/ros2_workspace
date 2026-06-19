"""Launch the complete manual-assisted stack for an in-water pool test."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_1 = LaunchConfiguration("camera_1")
    camera_2 = LaunchConfiguration("camera_2")
    dashboard_port = LaunchConfiguration("dashboard_port")
    save_video = LaunchConfiguration("save_video")
    output = LaunchConfiguration("output")
    label = LaunchConfiguration("label")
    control_mode = LaunchConfiguration("control_mode")

    core_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sub_launch"), "launch", "sub.launch.yaml"]
            )
        ),
        launch_arguments={"control_mode": control_mode}.items(),
    )

    dashboard = Node(
        package="sub_hardware",
        executable="camera_dashboard_server.py",
        name="camera_dashboard_server",
        output="screen",
        arguments=[
            "--camera-1", camera_1,
            "--camera-2", camera_2,
            "--port", dashboard_port,
            "--save-video", save_video,
            "--output", output,
            "--label", label,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_1", default_value="0"),
            DeclareLaunchArgument("camera_2", default_value="4"),
            DeclareLaunchArgument("dashboard_port", default_value="6969"),
            DeclareLaunchArgument("save_video", default_value="false"),
            DeclareLaunchArgument("output", default_value="/workspace/output/manual_assisted"),
            DeclareLaunchArgument("label", default_value="pool-test"),
            DeclareLaunchArgument("control_mode", default_value="manual_assisted"),
            core_launch,
            dashboard,
        ]
    )
