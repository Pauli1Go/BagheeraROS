from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


# Conservative autonomous base speeds (proven mapping tune).
# enable_higher_speeds:=true doubles linear and adds 50 % angular.
# Docking, undock guard and teleop limits keep their own parameters.
BASE_LINEAR_SPEED = 0.16
BASE_ANGULAR_SPEED = 0.30


def _launch_setup(context):
    package_share = Path(get_package_share_directory("bagheera_base"))
    params = str(package_share / "config" / "nav2_navigation.yaml")

    high_speed = (
        context.perform_substitution(
            LaunchConfiguration("enable_higher_speeds")
        ).lower()
        == "true"
    )
    linear_speed = BASE_LINEAR_SPEED * 2.0 if high_speed else BASE_LINEAR_SPEED
    angular_speed = BASE_ANGULAR_SPEED * 1.5 if high_speed else BASE_ANGULAR_SPEED

    # Autonomous Nav2 driving only. Approach speeds of the docking
    # controller, the undock manoeuvre and manual limits are untouched.
    controller_overrides = {
        "FollowPath.desired_linear_vel": linear_speed,
        "FollowPath.rotate_to_heading_angular_vel": angular_speed,
        "FollowPath.min_approach_linear_velocity": (
            0.10 if high_speed else 0.05
        ),
        "FollowPath.regulated_linear_scaling_min_speed": (
            0.10 if high_speed else 0.05
        ),
    }
    behavior_overrides = {
        "max_rotational_vel": angular_speed,
        "min_rotational_vel": 0.15 if high_speed else 0.10,
    }
    smoother_overrides = {
        "max_velocity": [linear_speed, 0.0, angular_speed],
        "min_velocity": [
            -0.16 if high_speed else -0.08,
            0.0,
            -angular_speed,
        ],
    }

    # Nav2's official components share one DDS participant. This removes the
    # per-process discovery/executor overhead without changing any algorithms,
    # costmap parameters, sensor rates or safety checks.
    nav2_components = LoadComposableNodes(
        target_container="nav2_container",
        composable_node_descriptions=[
            ComposableNode(
                package="nav2_controller",
                plugin="nav2_controller::ControllerServer",
                name="controller_server",
                parameters=[params, controller_overrides],
                remappings=[("cmd_vel", "/cmd_vel_nav")],
            ),
            ComposableNode(
                package="nav2_planner",
                plugin="nav2_planner::PlannerServer",
                name="planner_server",
                parameters=[params],
            ),
            ComposableNode(
                package="nav2_map_server",
                plugin="nav2_map_server::MapServer",
                name="keepout_filter_mask_server",
                parameters=[params],
            ),
            ComposableNode(
                package="nav2_map_server",
                plugin="nav2_map_server::MapServer",
                name="localization_exclusion_mask_server",
                parameters=[params],
            ),
            ComposableNode(
                package="nav2_map_server",
                plugin="nav2_map_server::CostmapFilterInfoServer",
                name="costmap_filter_info_server",
                parameters=[params],
            ),
            ComposableNode(
                package="nav2_behaviors",
                plugin="behavior_server::BehaviorServer",
                name="behavior_server",
                parameters=[params, behavior_overrides],
                remappings=[("cmd_vel", "/cmd_vel_nav")],
            ),
            ComposableNode(
                package="nav2_bt_navigator",
                plugin="nav2_bt_navigator::BtNavigator",
                name="bt_navigator",
                parameters=[
                    params,
                    {"default_nav_to_pose_bt_xml": str(
                        package_share / "behavior_trees" / "navigate_no_spin.xml"
                    )},
                ],
            ),
            ComposableNode(
                package="nav2_velocity_smoother",
                plugin="nav2_velocity_smoother::VelocitySmoother",
                name="velocity_smoother",
                parameters=[params, smoother_overrides],
                remappings=[
                    ("cmd_vel", "/cmd_vel_nav"),
                    # Collision checking stays in RPP, costmaps and the BT.
                    ("cmd_vel_smoothed", "/cmd_vel_automatic_raw"),
                ],
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_costmap_filters",
                parameters=[{
                    "autostart": True,
                    "node_names": [
                        "keepout_filter_mask_server",
                        "localization_exclusion_mask_server",
                        "costmap_filter_info_server",
                    ],
                    "bond_timeout": 12.0,
                    "bond_heartbeat_period": 1.0,
                    "service_timeout": 10.0,
                }],
            ),
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_navigation",
                parameters=[{
                    "autostart": True,
                    "node_names": [
                        "controller_server",
                        "planner_server",
                        "behavior_server",
                        "velocity_smoother",
                        "bt_navigator",
                    ],
                    "bond_timeout": 12.0,
                    "bond_heartbeat_period": 1.0,
                    "service_timeout": 10.0,
                }],
            ),
        ],
    )
    dock_guard = Node(
        package="bagheera_base",
        executable="bagheera_autonomy_dock_guard",
        name="bagheera_autonomy_dock_guard",
        output="screen",
        parameters=[params],
    )
    dock_apriltag = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="dock_apriltag",
        output="screen",
        parameters=[params],
        remappings=[
            # One newest frame at a time from bagheera_dock_frame_gate; the
            # direct camera topic queued five frames (~0.8 s extra latency).
            ("image_rect", "/dock/camera/image_raw"),
            ("camera_info", "/dock/camera/camera_info"),
            ("detections", "/dock/tags"),
            # apriltag_ros Kilted insists on pose estimation. Its pinhole TF
            # is intentionally isolated; bagheera_dock_tag_pose solves the
            # fisheye-corrected corners instead.
            ("/tf", "/dock/tag_tf_unused"),
        ],
    )
    dock_frame_gate = Node(
        package="bagheera_base",
        executable="bagheera_dock_frame_gate",
        name="bagheera_dock_frame_gate",
        output="screen",
        parameters=[params],
    )
    dock_tag_pose = Node(
        package="bagheera_base",
        executable="bagheera_dock_tag_pose",
        name="bagheera_dock_tag_pose",
        output="screen",
        parameters=[params],
    )
    # Separate process and lifecycle manager: a docking plugin failure must
    # not take down the Nav2 navigation container.
    docking_server = Node(
        package="opennav_docking",
        executable="opennav_docking",
        name="docking_server",
        output="screen",
        parameters=[params],
        remappings=[("cmd_vel", "/cmd_vel_docking")],
    )
    lifecycle_manager_docking = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_docking",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ["docking_server"],
            "bond_timeout": 12.0,
            "bond_heartbeat_period": 1.0,
            "service_timeout": 10.0,
        }],
    )
    dock_trigger = Node(
        package="bagheera_base",
        executable="bagheera_dock_trigger",
        name="bagheera_dock_trigger",
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
    goal_bridge = Node(
        package="bagheera_base",
        executable="bagheera_goal_pose_bridge",
        name="bagheera_goal_pose_bridge",
        output="screen",
        parameters=[params],
    )

    return [
        nav2_components,
        dock_guard,
        dock_frame_gate,
        dock_apriltag,
        dock_tag_pose,
        docking_server,
        lifecycle_manager_docking,
        dock_trigger,
        localization_guard,
        keepout_relay,
        goal_bridge,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_higher_speeds",
                default_value="true",
                description=(
                    "Double autonomous Nav2 linear speed and add 50 % "
                    "angular speed. Docking, undock and teleop untouched."
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
