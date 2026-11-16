#ifndef FER_MOVEIT__CORE__FOLLOW_LAW_HPP_
#define FER_MOVEIT__CORE__FOLLOW_LAW_HPP_

#include <Eigen/Geometry>
#include <deque>
#include <optional>
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

  /// \brief True if \p point is inside the box or on its wall.
  bool contains(const Eigen::Vector3d & point) const;
};

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

/// \brief Limits of StallDetector.
struct StallLimits
{
  double window{0.5};      ///< s, the motion is judged over this
  double min_speed{0.03};  ///< m/s, slower net commands are not judged (TCP about on target)
  double min_ratio{0.2};   ///< progress below this fraction of the command = stalled
  double timeout{1.5};     ///< s, stalled this long without a break = update() true

  /// \throw std::invalid_argument unless window, min_speed and timeout are > 0 and min_ratio
  /// is in (0, 1).
  void validate() const;
};

/// \brief Whether the TCP moves as fast as it is commanded to.
///
/// Catches an arm that is commanded but does not follow, whatever the reason: Servo slowing
/// towards a singularity without ever reporting a halt, a collision or a joint limit slowing it
/// down, a controller that ignores the commands. Judges the linear motion only.
///
/// Over the last `window` seconds: the commanded displacement is the sum of the commanded
/// velocities times their cycle time (a vector), the progress the TCP displacement along it.
/// Both divided by the time give the commanded speed and the measured speed. A cycle is
/// stalled if the commanded speed is at least min_speed and the measured speed below min_ratio
/// of it. Net displacements, not mean speeds: a command that goes back and forth (a waving or
/// trembling hand) nets out to about nothing and is not judged, whereas a TCP that does follow
/// it would show about no net displacement either.
class StallDetector
{
public:
  /// \throw std::invalid_argument for invalid \p limits.
  explicit StallDetector(const StallLimits & limits);

  /// \brief Add one cycle: time \p t (s), commanded TCP velocity (m/s), measured TCP position.
  /// \return true once the cycles have been stalled for limits.timeout without a break.
  bool update(double t, const Eigen::Vector3d & commanded_velocity, const Eigen::Vector3d & tcp);

  /// \brief Net commanded speed over the window (m/s); 0 until the window is full.
  double commanded_speed() const { return commanded_; }
  /// \brief TCP speed along the commanded direction over the window (m/s, negative: away from
  /// it); 0 until the window is full or while the command nets out to zero.
  double measured_speed() const { return measured_; }

  /// \brief Forget all cycles.
  void reset();

private:
  struct Sample
  {
    double t;
    Eigen::Vector3d commanded_velocity;
    Eigen::Vector3d tcp;
  };

  StallLimits limits_;
  std::deque<Sample> samples_;
  std::optional<double> stalled_since_;
  double commanded_{0.0};
  double measured_{0.0};
};

}  // namespace fer_moveit

#endif  // FER_MOVEIT__CORE__FOLLOW_LAW_HPP_
