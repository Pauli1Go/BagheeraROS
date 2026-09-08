from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory("bagheera_base")) / "config" / "base.yaml")
    sensor_config = str(
        Path(get_package_share_directory("bagheera_base")) / "config" / "sensors.yaml"
    )

    arguments = [
        DeclareLaunchArgument("serial_port", default_value="/dev/mowgli"),
        DeclareLaunchArgument("joystick_index", default_value="0"),
        DeclareLaunchArgument("deadman_button", default_value="4"),
        DeclareLaunchArgument("throttle_axis", default_value="3"),
        DeclareLaunchArgument("steering_axis", default_value="2"),
        DeclareLaunchArgument("use_lidar", default_value="true"),
        DeclareLaunchArgument("use_optical_flow", default_value="true"),
        DeclareLaunchArgument("use_camera", default_value="true"),
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

    lidar = Node(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node",
        output="screen",
        parameters=[sensor_config],
        condition=IfCondition(LaunchConfiguration("use_lidar")),
    )
    optical_flow = Node(
        package="bagheera_base",
        executable="bagheera_optical_flow",
        name="bagheera_optical_flow",
        output="screen",
        parameters=[sensor_config],
        condition=IfCondition(LaunchConfiguration("use_optical_flow")),
    )
    camera = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera",
        output="screen",
        parameters=[sensor_config],
        condition=IfCondition(LaunchConfiguration("use_camera")),
    )

    return LaunchDescription(
        arguments + [mowgli_launch, mode, controller, lidar, optical_flow, camera]
    )
