from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def _robot_description(package_share: Path, robot_config: Path) -> dict:
    with robot_config.open(encoding="utf-8") as handle:
        values = yaml.safe_load(handle)["bagheera"]["ros__parameters"]

    xacro_file = package_share / "urdf" / "bagheera.urdf.xacro"
    argument_names = (
        "chassis_length",
        "chassis_width",
        "chassis_height",
        "chassis_center_x",
        "chassis_mass_kg",
        "wheel_radius",
        "wheel_width",
        "wheel_track",
        "caster_radius",
        "caster_track",
        "imu_x",
        "imu_y",
        "imu_z",
        "imu_roll",
        "imu_pitch",
        "imu_yaw",
        "lidar_x",
        "lidar_y",
        "lidar_z",
        "lidar_yaw",
        "optical_flow_x",
        "optical_flow_y",
        "optical_flow_z",
        "camera_x",
        "camera_y",
        "camera_z",
        "camera_roll",
        "camera_pitch",
        "camera_yaw",
    )
    command = [FindExecutable(name="xacro"), " ", str(xacro_file)]
    for name in argument_names:
        command.extend((f" {name}:=", str(values[name])))
    return {"robot_description": ParameterValue(Command(command), value_type=str)}


def generate_launch_description():
    package_share = Path(get_package_share_directory("bagheera_base"))
    mowgli_share = Path(get_package_share_directory("mowgli_bringup"))
    base_config = str(package_share / "config" / "base.yaml")
    sensor_config = str(package_share / "config" / "sensors.yaml")
    robot_config = package_share / "config" / "robot.yaml"
    localization_config = str(package_share / "config" / "localization.yaml")

    serial_port = LaunchConfiguration("serial_port")
    use_lidar = LaunchConfiguration("use_lidar")
    use_optical_flow = LaunchConfiguration("use_optical_flow")
    use_camera = LaunchConfiguration("use_camera")
    use_sensor_fusion = LaunchConfiguration("use_sensor_fusion")
    use_foxglove = LaunchConfiguration("use_foxglove")

    arguments = [
        DeclareLaunchArgument("serial_port", default_value="/dev/mowgli"),
        DeclareLaunchArgument("joystick_index", default_value="0"),
        DeclareLaunchArgument("deadman_button", default_value="4"),
        DeclareLaunchArgument("throttle_axis", default_value="3"),
        DeclareLaunchArgument("steering_axis", default_value="2"),
        DeclareLaunchArgument("use_lidar", default_value="true"),
        DeclareLaunchArgument("use_optical_flow", default_value="true"),
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("use_sensor_fusion", default_value="true"),
        DeclareLaunchArgument("use_foxglove", default_value="true"),
        DeclareLaunchArgument("foxglove_address", default_value="0.0.0.0"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
    ]

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[_robot_description(package_share, robot_config)],
    )
    hardware_bridge = Node(
        package="mowgli_hardware",
        executable="hardware_bridge_node",
        name="hardware_bridge",
        output="screen",
        parameters=[
            str(mowgli_share / "config" / "hardware_bridge.yaml"),
            str(robot_config),
            {"serial_port": serial_port},
        ],
        remappings=[
            ("~/imu/data_raw", "/imu/data_raw"),
            ("~/imu/mag_raw", "/imu/mag_raw"),
            ("~/wheel_odom", "/wheel_odom_raw"),
            ("~/wheel_ticks", "/wheel_ticks"),
            ("~/emergency", "/hardware_bridge/emergency"),
            ("~/power", "/hardware_bridge/power"),
            ("~/status", "/hardware_bridge/status"),
            ("~/cmd_vel", "/cmd_vel"),
            ("~/dock_heading", "/gnss/heading"),
        ],
    )
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[str(mowgli_share / "config" / "twist_mux.yaml")],
        remappings=[("cmd_vel_out", "/cmd_vel")],
    )

    controller = Node(
        package="bagheera_base",
        executable="bagheera_controller",
        name="bagheera_controller",
        output="screen",
        parameters=[
            base_config,
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
    measurement_normalizer = Node(
        package="bagheera_base",
        executable="bagheera_measurement_normalizer",
        name="bagheera_measurement_normalizer",
        output="screen",
        parameters=[sensor_config],
    )

    lidar = Node(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node",
        output="screen",
        parameters=[sensor_config],
        condition=IfCondition(use_lidar),
    )
    optical_flow = Node(
        package="bagheera_base",
        executable="bagheera_optical_flow",
        name="bagheera_optical_flow",
        output="screen",
        parameters=[sensor_config],
        condition=IfCondition(use_optical_flow),
    )
    camera = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera",
        output="screen",
        parameters=[sensor_config],
        remappings=[("/camera/camera_info", "/camera/camera_info_raw")],
        condition=IfCondition(use_camera),
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[localization_config],
        remappings=[("odometry/filtered", "/odometry/filtered")],
        condition=IfCondition(use_sensor_fusion),
    )
    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[
            {
                "address": LaunchConfiguration("foxglove_address"),
                "port": ParameterValue(
                    LaunchConfiguration("foxglove_port"), value_type=int
                ),
                "send_buffer_limit": 10_000_000,
                "num_threads": 0,
            }
        ],
        condition=IfCondition(use_foxglove),
    )

    return LaunchDescription(
        arguments
        + [
            robot_state_publisher,
            hardware_bridge,
            twist_mux,
            mode,
            controller,
            measurement_normalizer,
            lidar,
            optical_flow,
            camera,
            ekf,
            foxglove,
        ]
    )
