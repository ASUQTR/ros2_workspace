"""Launch manual-assisted pool testing with the no-magnetometer EKF stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_1 = LaunchConfiguration("camera_1")
    camera_2 = LaunchConfiguration("camera_2")
    dashboard_port = LaunchConfiguration("dashboard_port")
    save_video = LaunchConfiguration("save_video")
    output = LaunchConfiguration("output")
    label = LaunchConfiguration("label")
    control_mode = LaunchConfiguration("control_mode")
    ekf_config_file = LaunchConfiguration("ekf_config_file")
    require_kill_switch_armed = LaunchConfiguration("require_kill_switch_armed")
    magnetic_play_enabled = LaunchConfiguration("magnetic_play_enabled")
    lookahead_distance = LaunchConfiguration("lookahead_distance")
    path_completion_tolerance = LaunchConfiguration("path_completion_tolerance")

    core_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("sub_launch"), "launch", "sub.launch.yaml"]
            )
        ),
        launch_arguments={
            "control_mode": control_mode,
            "ekf_config_file": ekf_config_file,
        }.items(),
    )

    playback = Node(
        package="sub_playback",
        executable="playback_node.py",
        name="playback_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("sub_playback"), "config", "params.yaml"]
            ),
            {
                "require_kill_switch_armed": ParameterValue(
                    require_kill_switch_armed, value_type=bool
                )
            },
            {
                "magnetic_play_enabled": ParameterValue(
                    magnetic_play_enabled, value_type=bool
                )
            },
            {
                "lookahead_distance_m": ParameterValue(
                    lookahead_distance, value_type=float
                )
            },
            {
                "path_completion_tolerance_m": ParameterValue(
                    path_completion_tolerance, value_type=float
                )
            },
        ],
    )

    dashboard = Node(
        package="sub_hardware",
        executable="camera_dashboard_server.py",
        name="camera_dashboard_server",
        output="screen",
        arguments=[
            "--camera-1",
            camera_1,
            "--camera-2",
            camera_2,
            "--port",
            dashboard_port,
            "--save-video",
            save_video,
            "--output",
            output,
            "--label",
            label,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "ekf_config_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("sub_control"),
                        "config",
                        "EKF_no_mag_yaw.yaml",
                    ]
                ),
                description=(
                    "Robot localization config. Defaults to Pete's no-mag yaw "
                    "pool-test EKF."
                ),
            ),
            DeclareLaunchArgument("control_mode", default_value="manual_assisted"),
            DeclareLaunchArgument("camera_1", default_value="0"),
            DeclareLaunchArgument("camera_2", default_value="4"),
            DeclareLaunchArgument("dashboard_port", default_value="6969"),
            DeclareLaunchArgument("save_video", default_value="false"),
            DeclareLaunchArgument(
                "output", default_value="/workspace/output/manual_assisted"
            ),
            DeclareLaunchArgument("label", default_value="nomag-pool-test"),
            DeclareLaunchArgument("require_kill_switch_armed", default_value="true"),
            DeclareLaunchArgument("magnetic_play_enabled", default_value="true"),
            DeclareLaunchArgument("lookahead_distance", default_value="0.35"),
            DeclareLaunchArgument("path_completion_tolerance", default_value="0.25"),
            core_launch,
            playback,
            dashboard,
        ]
    )
