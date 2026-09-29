#include <gtest/gtest.h>

#include <cmath>

#include "bagheera_navigation/latched_goal.hpp"

using bagheera_navigation::LatchedGoal;
using bagheera_navigation::LatchedGoalParams;
using bagheera_navigation::Pose2D;

namespace
{

LatchedGoalParams precise()
{
  LatchedGoalParams params;
  params.xy_goal_tolerance = 0.05;
  params.yaw_goal_tolerance = 5.0 * M_PI / 180.0;
  params.xy_release_tolerance = 0.12;
  params.same_goal_xy = 0.10;
  params.same_goal_yaw = 0.20;
  return params;
}

const Pose2D kGoal{1.30, 1.26, 1.607};

Pose2D robotAt(double distance, double yaw_error)
{
  return Pose2D{kGoal.x + distance, kGoal.y, kGoal.yaw + yaw_error};
}

}  // namespace

TEST(LatchedGoal, NotReachedOutsideTolerance)
{
  LatchedGoal goal(precise());
  EXPECT_FALSE(goal.isReached(robotAt(0.08, 0.0), kGoal));
  EXPECT_FALSE(goal.latched());
  EXPECT_DOUBLE_EQ(goal.reportedXYTolerance(), 0.05);
}

TEST(LatchedGoal, ReachedWithinBothTolerances)
{
  LatchedGoal goal(precise());
  EXPECT_TRUE(goal.isReached(robotAt(0.04, 0.02), kGoal));
}

TEST(LatchedGoal, LatchSurvivesDriftDuringFinalTurn)
{
  // Arrival at the edge, then the turn moves base_link a few cm outward.
  LatchedGoal goal(precise());
  EXPECT_FALSE(goal.isReached(robotAt(0.048, 1.2), kGoal));
  EXPECT_TRUE(goal.latched());
  EXPECT_DOUBLE_EQ(goal.reportedXYTolerance(), 0.12);
  EXPECT_FALSE(goal.isReached(robotAt(0.07, 0.6), kGoal));
  EXPECT_TRUE(goal.isReached(robotAt(0.08, 0.03), kGoal));
}

TEST(LatchedGoal, ResetDoesNotDropTheLatchForTheSameGoal)
{
  // A replanned path re-sends the same goal, shifted slightly by AMCL.
  LatchedGoal goal(precise());
  EXPECT_FALSE(goal.isReached(robotAt(0.045, 1.0), kGoal));
  const Pose2D replanned{kGoal.x + 0.02, kGoal.y - 0.01, kGoal.yaw + 0.01};
  EXPECT_FALSE(goal.isReached(robotAt(0.07, 0.8), replanned));
  EXPECT_TRUE(goal.latched());
}

TEST(LatchedGoal, ReleasedBeyondReleaseDistance)
{
  LatchedGoal goal(precise());
  EXPECT_FALSE(goal.isReached(robotAt(0.04, 1.0), kGoal));
  EXPECT_FALSE(goal.isReached(robotAt(0.13, 0.0), kGoal));
  EXPECT_FALSE(goal.latched());
  EXPECT_DOUBLE_EQ(goal.reportedXYTolerance(), 0.05);
}

TEST(LatchedGoal, DifferentGoalClearsTheLatch)
{
  LatchedGoal goal(precise());
  EXPECT_FALSE(goal.isReached(robotAt(0.04, 1.0), kGoal));
  const Pose2D other{kGoal.x + 0.30, kGoal.y, kGoal.yaw};
  EXPECT_FALSE(goal.isReached(robotAt(0.04, 0.0), other));
  EXPECT_FALSE(goal.latched());
}

TEST(LatchedGoal, SameSpotNewHeadingIsANewGoal)
{
  LatchedGoal goal(precise());
  EXPECT_FALSE(goal.isReached(robotAt(0.04, 1.0), kGoal));
  const Pose2D turned{kGoal.x, kGoal.y, kGoal.yaw + 1.0};
  // 0.04 m is within the tolerance, so it latches again for the new goal,
  // but the robot still faces the old heading (1 rad off the new one).
  EXPECT_FALSE(goal.isReached(robotAt(0.04, 0.0), turned));
  EXPECT_TRUE(goal.latched());
}

TEST(LatchedGoal, SuccessClearsTheLatchForTheNextGoal)
{
  LatchedGoal goal(precise());
  EXPECT_TRUE(goal.isReached(robotAt(0.03, 0.0), kGoal));
  EXPECT_FALSE(goal.latched());
  EXPECT_DOUBLE_EQ(goal.reportedXYTolerance(), 0.05);
}

TEST(LatchedGoal, YawToleranceWrapsAround)
{
  LatchedGoalParams params = precise();
  const Pose2D goal_pose{0.0, 0.0, M_PI - 0.01};
  LatchedGoal goal(params);
  EXPECT_TRUE(goal.isReached(Pose2D{0.01, 0.0, -M_PI + 0.02}, goal_pose));
}
