#include <gtest/gtest.h>

#include <cmath>

#include "fer_moveit/core/follow_law.hpp"

namespace fer_moveit
{

namespace
{

Eigen::Isometry3d at(double x, double y, double z)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, y, z);
  return pose;
}

}  // namespace

TEST(FollowLaw, NoMotionAtAStillGoal)
{
  const auto twist = follow_twist(at(0.4, 0, 0.5), at(0.4, 0, 0.5), Eigen::Vector3d::Zero(), {});
  EXPECT_NEAR(twist.norm(), 0.0, 1e-12);
}

TEST(FollowLaw, VelocityIsGainTimesError)
{
  // 10 cm behind, gain 2/s: 0.2 m/s towards the goal, below the 0.25 m/s cap.
  const auto twist = follow_twist(at(0.4, 0, 0.5), at(0.5, 0, 0.5), Eigen::Vector3d::Zero(), {});
  EXPECT_NEAR(twist(0), 0.2, 1e-12);
  EXPECT_NEAR(twist(1), 0.0, 1e-12);
  EXPECT_NEAR(twist(2), 0.0, 1e-12);
}

TEST(FollowLaw, SpeedIsCappedKeepingTheDirection)
{
  // 30 cm in x and 40 cm in y: 1 m/s uncapped, 0.25 m/s capped, direction (0.6, 0.8).
  const auto twist = follow_twist(at(0, 0, 0), at(0.3, 0.4, 0), Eigen::Vector3d::Zero(), {});
  EXPECT_NEAR(twist.head<3>().norm(), 0.25, 1e-12);
  EXPECT_NEAR(twist(0), 0.15, 1e-12);
  EXPECT_NEAR(twist(1), 0.20, 1e-12);
}

TEST(FollowLaw, FeedforwardKeepsUpWithAMovingGoal)
{
  // On the goal, the goal moves at 0.1 m/s in y: the TCP moves with it, no lag.
  const auto twist = follow_twist(at(0.4, 0, 0.5), at(0.4, 0, 0.5), Eigen::Vector3d(0, 0.1, 0), {});
  EXPECT_NEAR(twist(1), 0.1, 1e-12);
  EXPECT_NEAR(twist(0), 0.0, 1e-12);
}

TEST(FollowLaw, RotatesTowardsTheGoalOrientation)
{
  Eigen::Isometry3d goal = at(0.4, 0, 0.5);
  goal.rotate(Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ()));
  const auto twist = follow_twist(at(0.4, 0, 0.5), goal, Eigen::Vector3d::Zero(), {});
  EXPECT_NEAR(twist(5), 0.2, 1e-12);  // 0.1 rad about z, gain 2/s
  EXPECT_NEAR(twist.head<3>().norm(), 0.0, 1e-12);
}

TEST(FollowLaw, AngularSpeedIsCapped)
{
  Eigen::Isometry3d goal = at(0, 0, 0);
  goal.rotate(Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitX()));
  const auto twist = follow_twist(at(0, 0, 0), goal, Eigen::Vector3d::Zero(), {});
  EXPECT_NEAR(twist.tail<3>().norm(), 0.5, 1e-12);
  EXPECT_GT(twist(3), 0.0);
}

TEST(VelocityEstimator, ZeroUntilTwoSamples)
{
  VelocityEstimator estimator(0.0);
  EXPECT_TRUE(estimator.update(0.0, Eigen::Vector3d(1, 2, 3)));
  EXPECT_NEAR(estimator.velocity().norm(), 0.0, 1e-12);
}

TEST(VelocityEstimator, FindsAConstantVelocity)
{
  VelocityEstimator estimator(0.05);
  for (int i = 0; i <= 100; ++i)  // 1 s at 100 Hz, moving at 0.1 m/s in y
  {
    const double t = 0.01 * i;
    estimator.update(t, Eigen::Vector3d(0.5, 0.1 * t, 0.5));
  }
  EXPECT_NEAR(estimator.velocity().y(), 0.1, 1e-6);
  EXPECT_NEAR(estimator.velocity().x(), 0.0, 1e-12);
}

TEST(VelocityEstimator, IgnoresSamplesThatAreNotNewer)
{
  VelocityEstimator estimator(0.0);
  estimator.update(1.0, Eigen::Vector3d(0, 0, 0));
  estimator.update(1.1, Eigen::Vector3d(0.01, 0, 0));
  EXPECT_FALSE(estimator.update(1.1, Eigen::Vector3d(5, 5, 5)));  // same stamp again
  EXPECT_FALSE(estimator.update(1.0, Eigen::Vector3d(5, 5, 5)));  // older
  EXPECT_NEAR(estimator.velocity().x(), 0.1, 1e-9);
}

TEST(VelocityEstimator, ResetForgetsTheSamples)
{
  VelocityEstimator estimator(0.0);
  estimator.update(0.0, Eigen::Vector3d(0, 0, 0));
  estimator.update(0.1, Eigen::Vector3d(0.01, 0, 0));
  estimator.reset();
  EXPECT_NEAR(estimator.velocity().norm(), 0.0, 1e-12);
  EXPECT_TRUE(estimator.update(0.0, Eigen::Vector3d(9, 9, 9)));  // old stamps are fine again
  EXPECT_NEAR(estimator.velocity().norm(), 0.0, 1e-12);
}

}  // namespace fer_moveit
