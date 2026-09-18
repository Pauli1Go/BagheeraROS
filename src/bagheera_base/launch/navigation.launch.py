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
    keepout_mask_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="keepout_filter_mask_server",
        output="screen",
        parameters=[params],
    )
    localization_mask_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="localization_exclusion_mask_server",
        output="screen",
        parameters=[params],
    )
    filter_info_server = Node(
        package="nav2_map_server",
        executable="costmap_filter_info_server",
        name="costmap_filter_info_server",
        output="screen",
        parameters=[params],
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
        output="screen",
        parameters=[params, {
            "default_nav_to_pose_bt_xml": str(
                package_share / "behavior_trees" / "navigate_no_spin.xml"
            ),
        }],
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        remappings=[
            ("cmd_vel", "/cmd_vel_nav"),
            # The autonomous dock guard owns the final navigation lane. The
            # collision monitor remains removed; collision avoidance lives in
            # RPP's footprint checks, the costmaps and BT replanning.
            ("cmd_vel_smoothed", "/cmd_vel_automatic_raw"),
        ],
        **common,
    )
    dock_guard = Node(
        package="bagheera_base",
        executable="bagheera_autonomy_dock_guard",
        name="bagheera_autonomy_dock_guard",
        output="screen",
        parameters=[params],
    )
    localization_guard = Node(
        package="bagheera_base",
        executable="bagheera_localization_exclusion_guard",
        name="bagheera_localization_exclusion_guard",
        output="screen",
        parameters=[params],
    )
    keepout_relay = Node(
        package="bagheera_base",
        executable="bagheera_keepout_mask_relay",
        name="bagheera_keepout_mask_relay",
        output="screen",
        parameters=[params],
    )
    filter_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_costmap_filters",
        output="screen",
        parameters=[
            {
                "autostart": True,
                "node_names": [
                    "keepout_filter_mask_server",
                    "localization_exclusion_mask_server",
                    "costmap_filter_info_server",
                ],
                "bond_timeout": 4.0,
            }
        ],
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
            keepout_mask_server,
            localization_mask_server,
            filter_info_server,
            behaviors,
            navigator,
            velocity_smoother,
            dock_guard,
            localization_guard,
            keepout_relay,
            filter_lifecycle_manager,
            lifecycle_manager,
            goal_bridge,
        ]
    )
