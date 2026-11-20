#ifndef FER_MOVEIT__CORE__JOINT_SHAPER_HPP_
#define FER_MOVEIT__CORE__JOINT_SHAPER_HPP_

#include <Eigen/Core>
#include <cstddef>
#include <memory>
#include <vector>

namespace fer_moveit
{

/// \brief Velocity, acceleration and jerk limits per joint, in the joint group's order.
struct ShapingLimits
{
  std::vector<double> max_velocity;      ///< rad/s
  std::vector<double> max_acceleration;  ///< rad/s^2
  std::vector<double> max_jerk;          ///< rad/s^3

  /// \throw std::invalid_argument unless every list has \p joints finite values > 0.
  void validate(std::size_t joints) const;
};

/// \brief Jerk-limited shaping of a joint velocity command, one control period at a time.
///
/// Each step() moves the joint state one period towards a target velocity without exceeding
/// the velocity, acceleration or jerk limit of any joint (Ruckig, velocity control, all joints
/// time-synchronised so they reach the target together and the TCP keeps its direction). The
/// target is recomputed every period, so the command can change at any time. The position is
/// the integral of the shaped velocity: the joint command stays consistent.
class JointShaper
{
public:
  /// \brief Shaped joint state.
  struct State
  {
    Eigen::VectorXd position;      ///< rad
    Eigen::VectorXd velocity;      ///< rad/s
    Eigen::VectorXd acceleration;  ///< rad/s^2
  };

  /// \throw std::invalid_argument for invalid \p limits or \p period <= 0.
  JointShaper(const ShapingLimits & limits, std::size_t joints, double period);
  ~JointShaper();
  JointShaper(const JointShaper &) = delete;
  JointShaper & operator=(const JointShaper &) = delete;

  /// \brief Start from \p position and \p velocity with zero acceleration.
  /// \throw std::invalid_argument for wrong sizes or non-finite values.
  void reset(const Eigen::VectorXd & position, const Eigen::VectorXd & velocity);

  /// \brief One period towards \p target_velocity.
  ///
  /// A target above a joint's velocity limit is scaled down as a whole vector (the direction is
  /// kept). \throw std::invalid_argument for a wrong size or non-finite values;
  /// std::runtime_error if Ruckig finds no trajectory (the state is then unchanged).
  const State & step(const Eigen::VectorXd & target_velocity);

  /// \brief True if every joint is below \p velocity (rad/s) and \p acceleration (rad/s^2).
  bool at_rest(double velocity = 1e-3, double acceleration = 1e-2) const;

  const State & state() const { return state_; }
  double period() const { return period_; }

private:
  struct Impl;  // Ruckig stays out of this header
  std::unique_ptr<Impl> impl_;
  ShapingLimits limits_;
  std::size_t joints_;
  double period_;
  State state_;
};

}  // namespace fer_moveit

#endif  // FER_MOVEIT__CORE__JOINT_SHAPER_HPP_
