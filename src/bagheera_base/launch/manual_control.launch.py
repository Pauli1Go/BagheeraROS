from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory("bagheera_base")) / "config" / "base.yaml")

    arguments = [
        DeclareLaunchArgument("serial_port", default_value="/dev/mowgli"),
        DeclareLaunchArgument("allow_unsafe_firmware", default_value="false"),
        DeclareLaunchArgument("joystick_index", default_value="0"),
        DeclareLaunchArgument("deadman_button", default_value="4"),
        DeclareLaunchArgument("throttle_axis", default_value="3"),
        DeclareLaunchArgument("steering_axis", default_value="0"),
    ]

    driver = Node(
        package="bagheera_base",
        executable="bagheera_base_driver",
        name="bagheera_base_driver",
        output="screen",
        parameters=[
            config,
            {
                "port": LaunchConfiguration("serial_port"),
                "allow_unsafe_firmware": LaunchConfiguration("allow_unsafe_firmware"),
            },
        ],
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

    return LaunchDescription(arguments + [driver, controller])
