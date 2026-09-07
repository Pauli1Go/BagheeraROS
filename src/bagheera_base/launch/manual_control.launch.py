from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory("bagheera_base")) / "config" / "base.yaml")

    arguments = [
        DeclareLaunchArgument("serial_port", default_value="/dev/mowgli"),
        DeclareLaunchArgument("joystick_index", default_value="0"),
        DeclareLaunchArgument("deadman_button", default_value="4"),
        DeclareLaunchArgument("throttle_axis", default_value="3"),
        DeclareLaunchArgument("steering_axis", default_value="2"),
    ]

    mowgli_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("mowgli_bringup"))
                / "launch"
                / "mowgli.launch.py"
            )
        ),
        launch_arguments={
            "serial_port": LaunchConfiguration("serial_port"),
            "use_sim_time": "false",
        }.items(),
    )
    controller = Node(
        package="bagheera_base",
        executable="bagheera_controller",
        name="bagheera_controller",
        output="screen",
        parameters=[
            config,
            {
                "joystick_index": LaunchConfiguration("joystick_index"),
                "deadman_button": LaunchConfiguration("deadman_button"),
                "throttle_axis": LaunchConfiguration("throttle_axis"),
                "steering_axis": LaunchConfiguration("steering_axis"),
            },
        ],
    )
    mode = Node(
        package="bagheera_base",
        executable="bagheera_manual_mode",
        name="bagheera_mode",
        output="screen",
    )

    return LaunchDescription(arguments + [mowgli_launch, mode, controller])
