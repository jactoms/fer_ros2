#include "fer_moveit/core/follow_law.hpp"

#include <stdexcept>

namespace fer_moveit
{

namespace
{

// Scale \p v down to length \p limit if it is longer.
Eigen::Vector3d capped(const Eigen::Vector3d & v, double limit)
{
  const double norm = v.norm();
  return norm > limit ? Eigen::Vector3d(v * (limit / norm)) : v;
}

}  // namespace

Workspace Workspace::from(const std::vector<double> & min, const std::vector<double> & max)
{
  if (min.size() != 3 || max.size() != 3)
  {
    throw std::invalid_argument("workspace min and max need 3 values (x, y, z)");
  }
  Workspace workspace{{min[0], min[1], min[2]}, {max[0], max[1], max[2]}};
  if (!(workspace.min.array() < workspace.max.array()).all())
  {
    throw std::invalid_argument("workspace min must be below max on every axis");
  }
  return workspace;
}

ClampedGoal clamp_to_workspace(
  const Eigen::Vector3d & position, const Eigen::Vector3d & velocity, const Workspace & workspace)
{
  ClampedGoal goal{position.cwiseMax(workspace.min).cwiseMin(workspace.max), velocity, false};
  for (Eigen::Index i = 0; i < 3; ++i)
  {
    if (goal.position(i) != position(i))
    {
      goal.velocity(i) = 0.0;
      goal.clamped = true;
    }
  }
  return goal;
}

Eigen::Matrix<double, 6, 1> follow_twist(
  const Eigen::Isometry3d & tcp, const Eigen::Isometry3d & goal,
  const Eigen::Vector3d & goal_velocity, const FollowGains & gains)
{
  const Eigen::Vector3d linear =
    goal_velocity + gains.gain * (goal.translation() - tcp.translation());

  // Rotation that turns the TCP orientation into the goal orientation, in the base frame.
  const Eigen::AngleAxisd error(goal.rotation() * tcp.rotation().transpose());
  const Eigen::Vector3d angular = gains.rotation_gain * error.angle() * error.axis();

  Eigen::Matrix<double, 6, 1> twist;
  twist << capped(linear, gains.max_speed), capped(angular, gains.max_rotation_speed);
  return twist;
}

VelocityEstimator::VelocityEstimator(double time_constant) : time_constant_(time_constant) {}

bool VelocityEstimator::update(double t, const Eigen::Vector3d & position)
{
  if (has_sample_ && t <= last_t_)
  {
    return false;
  }
  if (has_sample_)
  {
    const double dt = t - last_t_;
    const Eigen::Vector3d raw = (position - last_position_) / dt;
    const double alpha = dt / (time_constant_ + dt);
    velocity_ += alpha * (raw - velocity_);
  }
  has_sample_ = true;
  last_t_ = t;
  last_position_ = position;
  return true;
}

void VelocityEstimator::reset()
{
  has_sample_ = false;
  velocity_.setZero();
}

}  // namespace fer_moveit
