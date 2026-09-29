// ROS-free core of LatchedGoalChecker, kept separate so it can be unit tested.
#pragma once

#include <cmath>

namespace bagheera_navigation
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct LatchedGoalParams
{
  // Reaching this distance latches "position reached" (like SimpleGoalChecker).
  double xy_goal_tolerance{0.25};
  double yaw_goal_tolerance{0.25};
  // Once latched, only a larger distance releases the latch again.
  double xy_release_tolerance{0.35};
  // A goal that moved less than this counts as the same goal.
  double same_goal_xy{0.10};
  double same_goal_yaw{0.20};
};

inline double normalizeAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

// Nav2 resets the goal checker (and RPP its own "xy reached" state) with every
// new path. With the 1 Hz replanning the robot then dropped out of its final
// turn whenever it stood at the edge of the xy tolerance. This keeps the
// latch across new paths as long as the goal stays the same and the robot
// stays within the release distance.
class LatchedGoal
{
public:
  explicit LatchedGoal(const LatchedGoalParams & params = LatchedGoalParams())
  : params_(params) {}

  void setParams(const LatchedGoalParams & params) {params_ = params;}
  const LatchedGoalParams & params() const {return params_;}

  bool isReached(const Pose2D & robot, const Pose2D & goal)
  {
    if (has_goal_ && !sameGoal(goal)) {
      latched_ = false;
    }
    goal_ = goal;
    has_goal_ = true;

    const double distance = std::hypot(robot.x - goal.x, robot.y - goal.y);
    if (!latched_ && distance <= params_.xy_goal_tolerance) {
      latched_ = true;
    } else if (latched_ && distance > params_.xy_release_tolerance) {
      latched_ = false;
    }

    const bool reached = latched_ &&
      std::fabs(normalizeAngle(goal.yaw - robot.yaw)) <= params_.yaw_goal_tolerance;
    if (reached) {
      // The controller stops at success; the next goal starts unlatched.
      latched_ = false;
      has_goal_ = false;
    }
    return reached;
  }

  // XY tolerance reported to the controller (RPP rotates to the goal heading
  // within it). Larger while latched, so RPP keeps its final turn after the
  // reset that every new path causes.
  double reportedXYTolerance() const
  {
    return latched_ ? params_.xy_release_tolerance : params_.xy_goal_tolerance;
  }

  bool latched() const {return latched_;}

  void clear()
  {
    latched_ = false;
    has_goal_ = false;
  }

private:
  bool sameGoal(const Pose2D & goal) const
  {
    return std::hypot(goal.x - goal_.x, goal.y - goal_.y) <= params_.same_goal_xy &&
           std::fabs(normalizeAngle(goal.yaw - goal_.yaw)) <= params_.same_goal_yaw;
  }

  LatchedGoalParams params_;
  Pose2D goal_;
  bool has_goal_{false};
  bool latched_{false};
};

}  // namespace bagheera_navigation
