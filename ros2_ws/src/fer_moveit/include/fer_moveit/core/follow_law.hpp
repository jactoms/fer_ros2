#ifndef FER_MOVEIT__CORE__FOLLOW_LAW_HPP_
#define FER_MOVEIT__CORE__FOLLOW_LAW_HPP_

#include <Eigen/Geometry>
#include <vector>

namespace fer_moveit
{

/// \brief Gains and limits of the follow law.
struct FollowGains
{
  double gain{2.0};                ///< 1/s, position error -> TCP velocity
  double max_speed{0.25};          ///< m/s, cap of the TCP speed
  double rotation_gain{2.0};       ///< 1/s, orientation error -> TCP angular velocity
  double max_rotation_speed{0.5};  ///< rad/s, cap of the TCP angular speed
};

/// \brief Axis-aligned box (base frame) the follow goal is kept in.
struct Workspace
{
  Eigen::Vector3d min;
  Eigen::Vector3d max;

  /// \throw std::invalid_argument unless both have 3 values and min < max on every axis.
  static Workspace from(const std::vector<double> & min, const std::vector<double> & max);
};

/// \brief A goal point after clamp_to_workspace().
struct ClampedGoal
{
  Eigen::Vector3d position;  ///< inside the workspace
  Eigen::Vector3d velocity;  ///< goal velocity, zero on every clamped axis
  bool clamped;              ///< true if any axis was clamped
};

/// \brief Clamp \p position into \p workspace.
///
/// On a clamped axis the goal velocity is dropped too, so the TCP rests at the wall instead of
/// pushing against it with the feedforward.
ClampedGoal clamp_to_workspace(
  const Eigen::Vector3d & position, const Eigen::Vector3d & velocity, const Workspace & workspace);

/// \brief TCP velocity (base frame) that brings \p tcp to \p goal while the goal moves.
///
/// linear  = goal_velocity + gain * (goal - tcp), capped at max_speed;
/// angular = rotation_gain * (rotation from tcp to goal as axis * angle), capped.
/// The feedforward goal_velocity keeps the TCP up with a moving goal instead of trailing it.
/// \return linear (m/s) then angular (rad/s).
Eigen::Matrix<double, 6, 1> follow_twist(
  const Eigen::Isometry3d & tcp, const Eigen::Isometry3d & goal,
  const Eigen::Vector3d & goal_velocity, const FollowGains & gains);

/// \brief Velocity of a moving point from its position samples.
///
/// Finite difference of consecutive samples, smoothed by a first-order low-pass, as the
/// fer_track_hand prototype did. Times are the samples' own stamps, so the estimate does not
/// depend on the clock of whoever reads it.
class VelocityEstimator
{
public:
  /// \param time_constant s, of the low-pass; 0 = no smoothing.
  explicit VelocityEstimator(double time_constant);

  /// \brief Add the sample at time \p t (s).
  /// \return false, and no change, if \p t is not newer than the last sample.
  bool update(double t, const Eigen::Vector3d & position);

  /// \brief Current estimate; zero until two samples arrived.
  const Eigen::Vector3d & velocity() const { return velocity_; }

  /// \brief Forget all samples.
  void reset();

private:
  double time_constant_;
  bool has_sample_{false};
  double last_t_{0.0};
  Eigen::Vector3d last_position_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d velocity_{Eigen::Vector3d::Zero()};
};

}  // namespace fer_moveit

#endif  // FER_MOVEIT__CORE__FOLLOW_LAW_HPP_
