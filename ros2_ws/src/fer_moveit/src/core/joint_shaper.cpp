#include "fer_moveit/core/joint_shaper.hpp"

#include <algorithm>
#include <cmath>
#include <ruckig/ruckig.hpp>
#include <stdexcept>
#include <string>

namespace fer_moveit
{

namespace
{

void check_list(const std::vector<double> & values, std::size_t joints, const char * name)
{
  if (values.size() != joints)
  {
    throw std::invalid_argument(
      std::string(name) + ": " + std::to_string(values.size()) + " values for " +
      std::to_string(joints) + " joints");
  }
  for (const double value : values)
  {
    if (!std::isfinite(value) || value <= 0.0)
    {
      throw std::invalid_argument(std::string(name) + ": every value must be > 0");
    }
  }
}

void check_vector(const Eigen::VectorXd & values, std::size_t joints, const char * name)
{
  if (static_cast<std::size_t>(values.size()) != joints || !values.allFinite())
  {
    throw std::invalid_argument(
      std::string(name) + ": need " + std::to_string(joints) + " finite values");
  }
}

std::vector<double> to_std(const Eigen::VectorXd & v) { return {v.data(), v.data() + v.size()}; }

Eigen::VectorXd to_eigen(const std::vector<double> & v)
{
  return Eigen::Map<const Eigen::VectorXd>(v.data(), static_cast<Eigen::Index>(v.size()));
}

}  // namespace

void ShapingLimits::validate(std::size_t joints) const
{
  if (joints == 0)
  {
    throw std::invalid_argument("shaping limits: no joints");
  }
  check_list(max_velocity, joints, "max_velocity");
  check_list(max_acceleration, joints, "max_acceleration");
  check_list(max_jerk, joints, "max_jerk");
}

struct JointShaper::Impl
{
  Impl(std::size_t joints, double period) : otg(joints, period), input(joints), output(joints) {}

  ruckig::Ruckig<ruckig::DynamicDOFs> otg;
  ruckig::InputParameter<ruckig::DynamicDOFs> input;
  ruckig::OutputParameter<ruckig::DynamicDOFs> output;
};

JointShaper::JointShaper(const ShapingLimits & limits, std::size_t joints, double period)
: limits_(limits), joints_(joints), period_(period)
{
  limits_.validate(joints_);
  if (!std::isfinite(period_) || period_ <= 0.0)
  {
    throw std::invalid_argument("shaping period must be > 0");
  }
  impl_ = std::make_unique<Impl>(joints_, period_);
  auto & in = impl_->input;
  in.control_interface = ruckig::ControlInterface::Velocity;
  in.synchronization = ruckig::Synchronization::Time;
  in.max_velocity = limits_.max_velocity;
  in.max_acceleration = limits_.max_acceleration;
  in.max_jerk = limits_.max_jerk;
  reset(Eigen::VectorXd::Zero(joints_), Eigen::VectorXd::Zero(joints_));
}

JointShaper::~JointShaper() = default;

void JointShaper::reset(const Eigen::VectorXd & position, const Eigen::VectorXd & velocity)
{
  check_vector(position, joints_, "position");
  check_vector(velocity, joints_, "velocity");
  state_.position = position;
  state_.velocity = velocity;
  state_.acceleration = Eigen::VectorXd::Zero(joints_);
}

const JointShaper::State & JointShaper::step(const Eigen::VectorXd & target_velocity)
{
  check_vector(target_velocity, joints_, "target velocity");
  // Scale the whole vector into the velocity limits: the joints keep their ratio, so the TCP
  // keeps its direction.
  double over = 1.0;
  for (std::size_t i = 0; i < joints_; ++i)
  {
    over = std::max(over, std::abs(target_velocity[i]) / limits_.max_velocity[i]);
  }
  const Eigen::VectorXd target = target_velocity / over;

  auto & in = impl_->input;
  in.current_position = to_std(state_.position);
  in.current_velocity = to_std(state_.velocity);
  in.current_acceleration = to_std(state_.acceleration);
  in.target_velocity = to_std(target);
  in.target_acceleration.assign(joints_, 0.0);

  const auto result = impl_->otg.update(in, impl_->output);
  if (result != ruckig::Result::Working && result != ruckig::Result::Finished)
  {
    throw std::runtime_error("Ruckig found no trajectory (result " + std::to_string(result) + ")");
  }
  state_.position = to_eigen(impl_->output.new_position);
  state_.velocity = to_eigen(impl_->output.new_velocity);
  state_.acceleration = to_eigen(impl_->output.new_acceleration);
  return state_;
}

bool JointShaper::at_rest(double velocity, double acceleration) const
{
  return state_.velocity.cwiseAbs().maxCoeff() < velocity &&
         state_.acceleration.cwiseAbs().maxCoeff() < acceleration;
}

}  // namespace fer_moveit
