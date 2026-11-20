#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <vector>

#include "fer_moveit/core/joint_shaper.hpp"

namespace fer_moveit
{

namespace
{

constexpr std::size_t JOINTS = 7;
constexpr double PERIOD = 0.01;

/// The `conservative` profile of fer_moveit_motion_server.yaml.
ShapingLimits conservative()
{
  ShapingLimits limits;
  limits.max_velocity = {2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61};
  limits.max_acceleration = {3.75, 1.875, 2.5, 3.125, 3.75, 5.0, 5.0};
  limits.max_jerk = {37.5, 18.75, 25.0, 31.25, 37.5, 50.0, 50.0};
  return limits;
}

Eigen::VectorXd vec(const std::vector<double> & v)
{
  return Eigen::Map<const Eigen::VectorXd>(v.data(), static_cast<Eigen::Index>(v.size()));
}

/// Time-optimal time for one joint to change its velocity by \p dv from zero acceleration to
/// zero acceleration with limits \p a and \p j.
double change_time(double dv, double a, double j)
{
  dv = std::abs(dv);
  return dv >= a * a / j ? a / j + dv / a : 2.0 * std::sqrt(dv / j);
}

double slowest_change(const Eigen::VectorXd & from, const Eigen::VectorXd & to)
{
  const auto limits = conservative();
  double t = 0.0;
  for (std::size_t i = 0; i < JOINTS; ++i)
  {
    t = std::max(t, change_time(to[i] - from[i], limits.max_acceleration[i], limits.max_jerk[i]));
  }
  return t;
}

}  // namespace

TEST(JointShaper, RandomJumpyTargetsNeverExceedALimit)
{
  const auto limits = conservative();
  JointShaper shaper(limits, JOINTS, PERIOD);
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> unit(-1.5, 1.5);  // up to 1.5x the velocity limit
  std::uniform_int_distribution<int> hold(1, 50);          // a new target every 1..50 periods
  std::bernoulli_distribution zero(0.15);

  Eigen::VectorXd target = Eigen::VectorXd::Zero(JOINTS);
  JointShaper::State prev = shaper.state();
  int left = 0;
  for (int k = 0; k < 10000; ++k)
  {
    if (left-- <= 0)
    {
      for (std::size_t i = 0; i < JOINTS; ++i)
      {
        target[i] = zero(rng) ? 0.0 : unit(rng) * limits.max_velocity[i];
      }
      left = hold(rng);
    }
    const auto s = shaper.step(target);
    for (std::size_t i = 0; i < JOINTS; ++i)
    {
      ASSERT_LE(std::abs(s.velocity[i]), limits.max_velocity[i] * (1 + 1e-9)) << "k " << k;
      ASSERT_LE(std::abs(s.acceleration[i]), limits.max_acceleration[i] * (1 + 1e-6) + 1e-9)
        << "joint " << i << " k " << k;
      const double jerk = std::abs(s.acceleration[i] - prev.acceleration[i]) / PERIOD;
      ASSERT_LE(jerk, limits.max_jerk[i] * (1 + 1e-6) + 1e-6) << "joint " << i << " k " << k;
      // the position is the integral of the velocity: the streamed command is consistent
      const double mean_v = 0.5 * (s.velocity[i] + prev.velocity[i]);
      ASSERT_NEAR(s.position[i] - prev.position[i], mean_v * PERIOD, 1e-4) << "k " << k;
    }
    prev = s;
  }
}

TEST(JointShaper, ConstantTargetIsReachedFullyInTheOptimalTime)
{
  JointShaper shaper(conservative(), JOINTS, PERIOD);
  const Eigen::VectorXd target = vec({0.3, -0.5, 0.8, -1.0, 0.2, 1.2, -0.6});
  const double bound = slowest_change(Eigen::VectorXd::Zero(JOINTS), target) + 2 * PERIOD;
  int k = 0;
  while ((shaper.state().velocity - target).cwiseAbs().maxCoeff() > 1e-9)
  {
    shaper.step(target);
    ASSERT_LE(++k * PERIOD, bound) << "target not reached in time";
  }
  for (int i = 0; i < 100; ++i)
  {
    const auto & s = shaper.step(target);
    EXPECT_LT((s.velocity - target).cwiseAbs().maxCoeff(), 1e-9);  // all of it, kept
    EXPECT_LT(s.acceleration.cwiseAbs().maxCoeff(), 1e-9);
  }
}

TEST(JointShaper, StopsFromFullSpeedWithinTheBrakingTime)
{
  const auto limits = conservative();
  JointShaper shaper(limits, JOINTS, PERIOD);
  const Eigen::VectorXd fast = 0.9 * vec(limits.max_velocity);
  shaper.reset(Eigen::VectorXd::Zero(JOINTS), fast);
  const double bound = slowest_change(fast, Eigen::VectorXd::Zero(JOINTS)) + 2 * PERIOD;
  int k = 0;
  while (!shaper.at_rest())
  {
    shaper.step(Eigen::VectorXd::Zero(JOINTS));
    ASSERT_LE(++k * PERIOD, bound) << "did not stop in time";
  }
  EXPECT_GT(k, 0);
}

TEST(JointShaper, ZeroTargetFromRestStaysAtRest)
{
  JointShaper shaper(conservative(), JOINTS, PERIOD);
  const Eigen::VectorXd start = vec({0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785});
  shaper.reset(start, Eigen::VectorXd::Zero(JOINTS));
  for (int k = 0; k < 100; ++k)
  {
    shaper.step(Eigen::VectorXd::Zero(JOINTS));
  }
  EXPECT_LT((shaper.state().position - start).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_TRUE(shaper.at_rest());
}

TEST(JointShaper, JointsArriveTogether)
{
  JointShaper shaper(conservative(), JOINTS, PERIOD);
  const Eigen::VectorXd target = vec({0.1, -0.4, 0.9, -1.5, 0.05, 2.0, -0.3});
  std::vector<int> arrived(JOINTS, -1);
  for (int k = 0; k < 300; ++k)
  {
    const auto & s = shaper.step(target);
    for (std::size_t i = 0; i < JOINTS; ++i)
    {
      if (arrived[i] < 0 && std::abs(s.velocity[i] - target[i]) < 1e-6)
      {
        arrived[i] = k;
      }
    }
  }
  const auto [lo, hi] = std::minmax_element(arrived.begin(), arrived.end());
  ASSERT_GE(*lo, 0);
  EXPECT_LE(*hi - *lo, 1) << "joints did not reach the target velocity together";
}

TEST(JointShaper, TargetOverTheLimitIsScaledAsAWhole)
{
  const auto limits = conservative();
  JointShaper shaper(limits, JOINTS, PERIOD);
  Eigen::VectorXd target = Eigen::VectorXd::Zero(JOINTS);
  target[0] = 2.0 * limits.max_velocity[0];  // twice the limit
  target[1] = 0.5 * limits.max_velocity[1];
  for (int k = 0; k < 500; ++k)
  {
    shaper.step(target);
  }
  const auto & v = shaper.state().velocity;
  EXPECT_NEAR(v[0], limits.max_velocity[0], 1e-9);
  EXPECT_NEAR(v[1], 0.25 * limits.max_velocity[1], 1e-9);  // same ratio as asked
}

TEST(JointShaper, RejectsInvalidInput)
{
  auto limits = conservative();
  EXPECT_THROW(limits.validate(6), std::invalid_argument);
  limits.max_jerk[3] = 0.0;
  EXPECT_THROW(JointShaper(limits, JOINTS, PERIOD), std::invalid_argument);
  limits = conservative();
  limits.max_acceleration[0] = -1.0;
  EXPECT_THROW(limits.validate(JOINTS), std::invalid_argument);
  limits = conservative();
  limits.max_velocity[6] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(limits.validate(JOINTS), std::invalid_argument);
  EXPECT_THROW(JointShaper(conservative(), JOINTS, 0.0), std::invalid_argument);

  JointShaper shaper(conservative(), JOINTS, PERIOD);
  EXPECT_THROW(
    shaper.reset(Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(JOINTS)), std::invalid_argument);
  Eigen::VectorXd bad = Eigen::VectorXd::Zero(JOINTS);
  bad[2] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(shaper.step(bad), std::invalid_argument);
}

}  // namespace fer_moveit
