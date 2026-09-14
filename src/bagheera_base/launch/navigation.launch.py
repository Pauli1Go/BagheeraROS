from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("bagheera_base"))
    params = str(package_share / "config" / "nav2_navigation.yaml")

    common = {
        "output": "screen",
        "parameters": [params],
    }

    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        remappings=[("cmd_vel", "/cmd_vel_nav")],
        **common,
    )
    planner = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        **common,
    )
    behaviors = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        remappings=[("cmd_vel", "/cmd_vel_nav")],
        **common,
    )
    navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        **common,
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        remappings=[
            ("cmd_vel", "/cmd_vel_nav"),
            ("cmd_vel_smoothed", "/cmd_vel_monitored"),
        ],
        **common,
    )
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "velocity_smoother",
                    "bt_navigator",
                ],
                "bond_timeout": 4.0,
            }
        ],
    )
    goal_bridge = Node(
        package="bagheera_base",
        executable="bagheera_goal_pose_bridge",
        name="bagheera_goal_pose_bridge",
        output="screen",
    )

    return LaunchDescription(
        [
            controller,
            planner,
            behaviors,
            navigator,
            velocity_smoother,
            lifecycle_manager,
            goal_bridge,
        ]
    )
