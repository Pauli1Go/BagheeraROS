// Nav2 goal checker that keeps "position reached" across the path updates of
// the 1 Hz replanning (see latched_goal.hpp).
#include <limits>
#include <memory>
#include <string>

#include "bagheera_navigation/latched_goal.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2/utils.hpp"

namespace bagheera_navigation
{

class LatchedGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override
  {
    auto node = parent.lock();
    LatchedGoalParams params;
    const auto declare = [&](const std::string & name, double & value) {
        nav2_util::declare_parameter_if_not_declared(
          node, plugin_name + "." + name, rclcpp::ParameterValue(value));
        node->get_parameter(plugin_name + "." + name, value);
      };
    declare("xy_goal_tolerance", params.xy_goal_tolerance);
    declare("yaw_goal_tolerance", params.yaw_goal_tolerance);
    declare("xy_release_tolerance", params.xy_release_tolerance);
    declare("same_goal_xy", params.same_goal_xy);
    declare("same_goal_yaw", params.same_goal_yaw);
    if (params.xy_release_tolerance < params.xy_goal_tolerance) {
      RCLCPP_WARN(
        node->get_logger(), "%s: xy_release_tolerance below xy_goal_tolerance; using %.3f",
        plugin_name.c_str(), params.xy_goal_tolerance);
      params.xy_release_tolerance = params.xy_goal_tolerance;
    }
    logic_.setParams(params);
  }

  // Called for every new path. The latch survives; a different goal or a
  // robot beyond the release distance clears it in isGoalReached().
  void reset() override {}

  bool isGoalReached(
    const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
    const geometry_msgs::msg::Twist &) override
  {
    return logic_.isReached(
      Pose2D{query_pose.position.x, query_pose.position.y, tf2::getYaw(query_pose.orientation)},
      Pose2D{goal_pose.position.x, goal_pose.position.y, tf2::getYaw(goal_pose.orientation)});
  }

  bool getTolerances(
    geometry_msgs::msg::Pose & pose_tolerance,
    geometry_msgs::msg::Twist & vel_tolerance) override
  {
    const double invalid = std::numeric_limits<double>::lowest();
    const double xy = logic_.reportedXYTolerance();
    pose_tolerance.position.x = xy;
    pose_tolerance.position.y = xy;
    pose_tolerance.position.z = invalid;
    pose_tolerance.orientation =
      nav2_util::geometry_utils::orientationAroundZAxis(logic_.params().yaw_goal_tolerance);
    vel_tolerance.linear.x = invalid;
    vel_tolerance.linear.y = invalid;
    vel_tolerance.linear.z = invalid;
    vel_tolerance.angular.x = invalid;
    vel_tolerance.angular.y = invalid;
    vel_tolerance.angular.z = invalid;
    return true;
  }

private:
  LatchedGoal logic_;
};

}  // namespace bagheera_navigation

PLUGINLIB_EXPORT_CLASS(bagheera_navigation::LatchedGoalChecker, nav2_core::GoalChecker)
