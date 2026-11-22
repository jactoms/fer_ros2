#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

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

TEST(Workspace, ContainsItsInsideAndItsWall)
{
  const auto box = Workspace::from({0.25, -0.3, 0.25}, {0.6, 0.3, 0.5});
  EXPECT_TRUE(box.contains({0.4, 0.1, 0.4}));
  EXPECT_TRUE(box.contains({0.6, -0.3, 0.5}));  // a corner
}

TEST(Workspace, DoesNotContainWhatIsOutsideOnAnyAxis)
{
  const auto box = Workspace::from({0.25, -0.3, 0.25}, {0.6, 0.3, 0.5});
  EXPECT_FALSE(box.contains({0.7, 0.0, 0.4}));
  EXPECT_FALSE(box.contains({0.4, -0.31, 0.4}));
  EXPECT_FALSE(box.contains({0.4, 0.0, 0.51}));
}

TEST(Workspace, RejectsABoxThatIsNotOne)
{
  EXPECT_THROW(Workspace::from({0.0, 0.0}, {1.0, 1.0, 1.0}), std::invalid_argument);
  EXPECT_THROW(Workspace::from({0.0, 0.0, 0.5}, {1.0, 1.0, 0.5}), std::invalid_argument);
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

namespace
{

/// Feed \p detector at 100 Hz from \p from to \p to (s): commanded speed \p command along x,
/// the TCP moving along x at \p speed from \p x0. \return the first time update() was true, or
/// -1.
double run_stall(
  StallDetector & detector, double from, double to, double command, double speed, double x0 = 0.0)
{
  for (int i = static_cast<int>(std::lround(from * 100.0)); i <= std::lround(to * 100.0); ++i)
  {
    const double t = 0.01 * i;
    if (detector.update(
          t, Eigen::Vector3d(command, 0.0, 0.0),
          Eigen::Vector3d(x0 + speed * (t - from), 0.0, 0.5)))
    {
      return t;
    }
  }
  return -1.0;
}

}  // namespace

TEST(StallDetector, TcpThatMovesAsCommandedIsNotStalled)
{
  StallDetector detector({});
  EXPECT_LT(run_stall(detector, 0.0, 5.0, 0.1, 0.1), 0.0);
  EXPECT_NEAR(detector.commanded_speed(), 0.1, 1e-9);
  EXPECT_NEAR(detector.measured_speed(), 0.1, 1e-6);
}

TEST(StallDetector, StillTcpUnderACommandIsStalledAfterWindowAndTimeout)
{
  StallDetector detector({});  // window 0.5 s, timeout 1.5 s
  const double stalled_at = run_stall(detector, 0.0, 5.0, 0.1, 0.0);
  EXPECT_NEAR(stalled_at, 2.0, 0.015);  // judged from 0.5 s on, stalled 1.5 s later
  EXPECT_NEAR(detector.measured_speed(), 0.0, 1e-12);
}

TEST(StallDetector, CrawlBelowTheRatioIsStalledAboveItIsNot)
{
  StallDetector crawl({});
  EXPECT_GT(run_stall(crawl, 0.0, 5.0, 0.1, 0.01), 0.0);  // 10 % of the command
  StallDetector slow({});
  EXPECT_LT(run_stall(slow, 0.0, 5.0, 0.1, 0.03), 0.0);  // 30 %: slow, but it follows
}

TEST(StallDetector, BackAndForthWithAPeriodOfTheWindowIsNotStalled)
{
  // A waving hand at 2 Hz (period = window): the TCP follows it, so it ends every window where
  // it started. The command nets out to zero as well, so nothing is judged.
  StallDetector detector({});
  const double w = 2.0 * M_PI * 2.0, a = 0.02;  // 2 Hz, +-2 cm: up to 0.25 m/s
  for (int i = 0; i <= 500; ++i)
  {
    const double t = 0.01 * i;
    EXPECT_FALSE(detector.update(
      t, Eigen::Vector3d(0.0, a * w * std::cos(w * t), 0.0),
      Eigen::Vector3d(0.4, a * std::sin(w * t), 0.5)))
      << "at t = " << t;
  }
  EXPECT_LT(detector.commanded_speed(), 0.03);
}

TEST(StallDetector, BackAndForthAroundAStuckTcpThatIsCommandedAwayIsStalled)
{
  // The same wave, on top of a steady 0.1 m/s command in x that the TCP never follows.
  StallDetector detector({});
  const double w = 2.0 * M_PI * 2.0, a = 0.02;
  double fired = -1.0;
  for (int i = 0; i <= 500 && fired < 0.0; ++i)
  {
    const double t = 0.01 * i;
    if (detector.update(
          t, Eigen::Vector3d(0.1, a * w * std::cos(w * t), 0.0), Eigen::Vector3d(0.4, 0.0, 0.5)))
    {
      fired = t;
    }
  }
  EXPECT_NEAR(fired, 2.0, 0.015);
}

TEST(StallDetector, MotionAcrossTheCommandIsNoProgress)
{
  // Commanded along x, the TCP slides along y as fast: it does not get where it is sent.
  StallDetector detector({});
  double fired = -1.0;
  for (int i = 0; i <= 500 && fired < 0.0; ++i)
  {
    const double t = 0.01 * i;
    if (detector.update(t, Eigen::Vector3d(0.1, 0.0, 0.0), Eigen::Vector3d(0.4, 0.1 * t, 0.5)))
    {
      fired = t;
    }
  }
  EXPECT_NEAR(fired, 2.0, 0.015);
  EXPECT_NEAR(detector.measured_speed(), 0.0, 1e-9);
}

TEST(StallDetector, CommandsBelowMinSpeedAreNotJudged)
{
  StallDetector detector({});  // min_speed 0.03 m/s
  EXPECT_LT(run_stall(detector, 0.0, 5.0, 0.02, 0.0), 0.0);
}

TEST(StallDetector, ShortStallIsRiddenOutAndTheTimerStartsAgain)
{
  StallDetector detector({});
  EXPECT_LT(run_stall(detector, 0.0, 1.0, 0.1, 0.1), 0.0);        // moving
  EXPECT_LT(run_stall(detector, 1.01, 2.0, 0.1, 0.0, 0.1), 0.0);  // still for 1 s
  EXPECT_LT(run_stall(detector, 2.01, 3.0, 0.1, 0.1, 0.1), 0.0);  // moving again
  // Still again: the 1.5 s count starts anew, once the window shows the stop (~0.4 s).
  const double stalled_at = run_stall(detector, 3.01, 6.0, 0.1, 0.0, 0.2);
  EXPECT_GT(stalled_at, 3.01 + 1.5);
  EXPECT_LT(stalled_at, 3.01 + 0.5 + 1.5 + 0.02);
}

TEST(StallDetector, ResetForgetsTheHistory)
{
  StallDetector detector({});
  run_stall(detector, 0.0, 1.9, 0.1, 0.0);  // almost stalled
  detector.reset();
  EXPECT_LT(run_stall(detector, 2.0, 2.4, 0.1, 0.0), 0.0);  // window not full again yet
  EXPECT_NEAR(detector.commanded_speed(), 0.0, 1e-12);
}

TEST(StallDetector, RejectsInvalidLimits)
{
  StallLimits no_window;
  no_window.window = 0.0;
  EXPECT_THROW(StallDetector{no_window}, std::invalid_argument);
  StallLimits ratio_one;
  ratio_one.min_ratio = 1.0;
  EXPECT_THROW(StallDetector{ratio_one}, std::invalid_argument);
  StallLimits no_timeout;
  no_timeout.timeout = -1.0;
  EXPECT_THROW(StallDetector{no_timeout}, std::invalid_argument);
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

TEST(VelocityFade, FullUpToHoldThenLinearToZero)
{
  const VelocityFade fade{0.05, 0.10};
  EXPECT_DOUBLE_EQ(fade.weight(-0.01), 1.0);  // a sample stamped after "now": fresh
  EXPECT_DOUBLE_EQ(fade.weight(0.0), 1.0);
  EXPECT_DOUBLE_EQ(fade.weight(0.05), 1.0);
  EXPECT_NEAR(fade.weight(0.075), 0.75, 1e-12);
  EXPECT_NEAR(fade.weight(0.10), 0.5, 1e-12);
  EXPECT_NEAR(fade.weight(0.15), 0.0, 1e-12);  // 0.05 + 0.10 is not exactly 0.15 in binary
  EXPECT_DOUBLE_EQ(fade.weight(0.16), 0.0);
  EXPECT_DOUBLE_EQ(fade.weight(10.0), 0.0);
}

TEST(VelocityFade, ZeroFadeCutsAtHold)
{
  const VelocityFade fade{0.05, 0.0};
  EXPECT_DOUBLE_EQ(fade.weight(0.05), 1.0);
  EXPECT_DOUBLE_EQ(fade.weight(0.0501), 0.0);
  const VelocityFade none{0.0, 0.0};
  EXPECT_DOUBLE_EQ(none.weight(0.0), 1.0);
  EXPECT_DOUBLE_EQ(none.weight(1e-9), 0.0);
}

TEST(VelocityFade, RejectsNegativeValues)
{
  EXPECT_NO_THROW((VelocityFade{0.0, 0.0}.validate()));
  EXPECT_THROW((VelocityFade{-0.01, 0.05}.validate()), std::invalid_argument);
  EXPECT_THROW((VelocityFade{0.05, -0.01}.validate()), std::invalid_argument);
}

namespace
{

// Kinematic loop at 100 Hz: the TCP moves exactly as commanded. The target moves at 0.1 m/s in
// x, sampled every cycle, and stops sending at t = 1 s (last position x = 0.1). The loop runs
// on to t = 1.5 s, where target_timeout 0.5 s would end Follow.
// \return how far the TCP got past the last target position (m).
double overshoot_after_dropout(const VelocityFade & fade)
{
  const FollowGains gains;  // 2/s, 0.25 m/s
  const double dt = 0.01;
  VelocityEstimator estimator(0.05);
  double tcp = 0.0;
  double target = 0.0;
  double seen = 0.0;
  double furthest = 0.0;
  for (int i = 0; i <= 150; ++i)
  {
    const double t = i * dt;
    if (t <= 1.0 + 1e-9)
    {
      target = 0.1 * t;
      estimator.update(t, Eigen::Vector3d(target, 0, 0));
      seen = t;
    }
    const Eigen::Vector3d velocity = fade.weight(t - seen) * estimator.velocity();
    tcp += follow_twist(at(tcp, 0, 0), at(target, 0, 0), velocity, gains)(0) * dt;
    furthest = std::max(furthest, tcp);
  }
  return furthest - target;
}

}  // namespace

TEST(VelocityFade, StaleVelocityNoLongerCarriesTheTcpOn)
{
  // Before the fade: the full velocity until target_timeout (same as a 0.5 s hold, no fade).
  const double before = overshoot_after_dropout(VelocityFade{0.5, 0.0});
  // Default fade: full for 0.05 s, gone after 0.1 s; at 0.1 m/s that is < 1 cm at most.
  const double after = overshoot_after_dropout(VelocityFade{0.05, 0.05});
  EXPECT_GT(before, 0.025);
  EXPECT_LT(after, 0.01);
  EXPECT_GT(after, 0.0);  // the first 0.05 s are still fed forward (a late sample is no dropout)
}

namespace
{

// A fixed 6 x 7 Jacobian of full rank (one-dimensional null space, as on the FER).
Eigen::MatrixXd seven_joint_jacobian()
{
  Eigen::MatrixXd jacobian(6, 7);
  jacobian << 0.0, 0.3, 0.0, -0.2, 0.0, 0.1, 0.0,  //
    0.4, 0.0, 0.4, 0.0, 0.1, 0.0, 0.0,             //
    0.0, -0.4, 0.0, 0.3, 0.0, 0.1, 0.0,            //
    0.0, 0.0, 0.7, 0.0, -0.7, 0.0, 0.3,            //
    0.0, 1.0, 0.0, -1.0, 0.0, 1.0, 0.0,            //
    1.0, 0.0, 0.7, 0.0, 0.7, 0.0, -0.9;
  return jacobian;
}

}  // namespace

TEST(PostureVelocity, ZeroGainIsOff)
{
  const Eigen::VectorXd q = Eigen::VectorXd::Constant(7, 0.3);
  const auto v = posture_velocity(seven_joint_jacobian(), q, Eigen::VectorXd::Zero(7), 0.0, 1.0);
  EXPECT_EQ(v.size(), 7);
  EXPECT_NEAR(v.norm(), 0.0, 1e-12);
}

TEST(PostureVelocity, DoesNotMoveTheTcp)
{
  const auto jacobian = seven_joint_jacobian();
  const Eigen::VectorXd q = (Eigen::VectorXd(7) << 0.2, -0.5, 0.4, -2.0, 0.1, 1.6, 0.5).finished();
  const auto v = posture_velocity(jacobian, q, Eigen::VectorXd::Zero(7), 1.0, 10.0);
  EXPECT_GT(v.norm(), 1e-3);                      // there is a null-space part to correct
  EXPECT_NEAR((jacobian * v).norm(), 0.0, 1e-9);  // and correcting it moves the TCP by nothing
}

TEST(PostureVelocity, TurnsTheArmBackToRest)
{
  // Integrate with a fixed Jacobian: the null-space part of q - rest decays like exp(-gain t);
  // the rest (what the TCP needs) stays.
  const auto jacobian = seven_joint_jacobian();
  const Eigen::MatrixXd null =
    Eigen::MatrixXd::Identity(7, 7) -
    jacobian.completeOrthogonalDecomposition().pseudoInverse() * jacobian;
  const Eigen::VectorXd rest = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd q = null * Eigen::VectorXd::Constant(7, 0.5) + Eigen::VectorXd::Constant(7, 0.1);
  const double before = (null * (q - rest)).norm();
  const Eigen::VectorXd tcp_part = q - null * q;
  for (int i = 0; i < 300; ++i)  // 3 s at 10 ms, gain 1/s
  {
    q += posture_velocity(jacobian, q, rest, 1.0, 10.0) * 0.01;
  }
  EXPECT_LT((null * (q - rest)).norm(), 0.06 * before);  // exp(-3) = 0.05
  EXPECT_NEAR((q - null * q - tcp_part).norm(), 0.0, 1e-9);
}

TEST(PostureVelocity, IsCapped)
{
  const Eigen::VectorXd q = (Eigen::VectorXd(7) << 2.0, -0.5, 2.0, -2.0, 1.0, 1.6, 2.0).finished();
  const auto v = posture_velocity(seven_joint_jacobian(), q, Eigen::VectorXd::Zero(7), 50.0, 0.2);
  EXPECT_NEAR(v.norm(), 0.2, 1e-9);
}

}  // namespace fer_moveit
