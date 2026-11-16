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

bool Workspace::contains(const Eigen::Vector3d & point) const
{
  return (point.array() >= min.array()).all() && (point.array() <= max.array()).all();
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

void VelocityFade::validate() const
{
  if (!(hold >= 0.0) || !(fade >= 0.0))
  {
    throw std::invalid_argument("velocity fade hold and fade must be >= 0");
  }
}

double VelocityFade::weight(double age) const
{
  if (age <= hold)
  {
    return 1.0;
  }
  if (age >= hold + fade)
  {
    return 0.0;
  }
  return 1.0 - (age - hold) / fade;
}

void StallLimits::validate() const
{
  if (!(window > 0.0) || !(min_speed > 0.0) || !(timeout > 0.0))
  {
    throw std::invalid_argument("stall window, min_speed and timeout must be > 0");
  }
  if (!(min_ratio > 0.0 && min_ratio < 1.0))
  {
    throw std::invalid_argument("stall min_ratio must be in (0, 1)");
  }
}

StallDetector::StallDetector(const StallLimits & limits) : limits_(limits) { limits_.validate(); }

bool StallDetector::update(
  double t, const Eigen::Vector3d & commanded_velocity, const Eigen::Vector3d & tcp)
{
  samples_.push_back({t, commanded_velocity, tcp});
  // Keep one sample at or before t - window, so the displacement spans the whole window.
  while (samples_.size() > 2 && samples_[1].t <= t - limits_.window)
  {
    samples_.pop_front();
  }
  const double span = t - samples_.front().t;
  if (span < limits_.window)
  {
    return false;  // not enough history yet
  }

  // Each command acts until the next cycle; the newest one has not acted yet.
  Eigen::Vector3d commanded = Eigen::Vector3d::Zero();
  for (std::size_t i = 1; i < samples_.size(); ++i)
  {
    commanded += samples_[i - 1].commanded_velocity * (samples_[i].t - samples_[i - 1].t);
  }
  const double commanded_norm = commanded.norm();
  commanded_ = commanded_norm / span;
  measured_ = commanded_norm > 0.0
                ? (tcp - samples_.front().tcp).dot(commanded / commanded_norm) / span
                : 0.0;

  const bool stalled =
    commanded_ >= limits_.min_speed && measured_ < limits_.min_ratio * commanded_;
  if (!stalled)
  {
    stalled_since_.reset();
    return false;
  }
  if (!stalled_since_)
  {
    stalled_since_ = t;
  }
  return t - *stalled_since_ >= limits_.timeout;
}

void StallDetector::reset()
{
  samples_.clear();
  stalled_since_.reset();
  commanded_ = 0.0;
  measured_ = 0.0;
}

}  // namespace fer_moveit
