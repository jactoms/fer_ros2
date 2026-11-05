#ifndef FER_MOVEIT__ADAPTERS__CARTESIAN_STREAMER_HPP_
#define FER_MOVEIT__ADAPTERS__CARTESIAN_STREAMER_HPP_

#include <Eigen/Core>
#include <string>

namespace fer_moveit
{

/// \brief Result of one streaming cycle.
struct StreamStatus
{
  enum class Code
  {
    OK,      ///< moving as commanded
    SLOWED,  ///< moving slower than commanded: near a singularity or a collision
    HALTED,  ///< not moving: collision, singularity or joint bound ahead
    INVALID  ///< no joint command could be computed
  };

  Code code{Code::OK};
  std::string message;
};

/// \brief The Cartesian -> joint stage of /motion/follow.
///
/// Turns a TCP velocity into joint commands for the arm's trajectory controller, one control
/// cycle at a time. The controller is never switched; it must be active already.
///
/// ServoStreamer implements it with MoveIt Servo. Joint-space shaping (Ruckig) goes behind the
/// same interface, so /motion/follow does not change when the stage is replaced.
class CartesianStreamer
{
public:
  /// Linear (m/s) then angular (rad/s) TCP velocity, in the base frame.
  using Twist = Eigen::Matrix<double, 6, 1>;

  virtual ~CartesianStreamer() = default;

  /// \brief Control period in s: call step() or halt() once per period.
  virtual double period() const = 0;

  /// \brief Begin a stream at the arm's current joint state.
  /// \throws MotionError TIMEOUT if no current joint state arrives.
  virtual void start() = 0;

  /// \brief One cycle: move the TCP with \p twist and send the joint command.
  virtual StreamStatus step(const Twist & twist) = 0;

  /// \brief One cycle of stopping.
  /// \return true once the arm is commanded to rest.
  virtual bool halt() = 0;
};

}  // namespace fer_moveit

#endif  // FER_MOVEIT__ADAPTERS__CARTESIAN_STREAMER_HPP_
