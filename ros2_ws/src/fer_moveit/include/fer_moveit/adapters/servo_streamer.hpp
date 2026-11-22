#ifndef FER_MOVEIT__ADAPTERS__SERVO_STREAMER_HPP_
#define FER_MOVEIT__ADAPTERS__SERVO_STREAMER_HPP_

#include <deque>
#include <memory>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit_servo/servo.hpp>

#include "fer_moveit/adapters/cartesian_streamer.hpp"
#include "fer_moveit/core/joint_shaper.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace fer_moveit
{

/// \brief CartesianStreamer with MoveIt Servo as a library, in the motion server's process.
///
/// Servo computes the joint state for each cycle; this class shapes it (JointShaper: Ruckig,
/// within the velocity, acceleration and jerk limits of the selected profile) and sends it as a
/// short joint trajectory on the controller's topic (`moveit_servo.command_out_topic`). Servo
/// integrates its next state from the shaped one, so its collision and singularity checks see
/// what is really commanded. Profile `off`: Servo's state is sent unshaped. With
/// `follow_posture_gain` > 0, a null-space term (core/follow_law posture_velocity) is added to
/// Servo's state before shaping: it turns the elbow back towards `follow_posture_rest` without
/// moving the TCP, so long periodic Follow runs do not drift into a singular posture.
///
/// Reads the `moveit_servo.*` parameters of the node, `follow_shaping_profile` and
/// `follow_shaping.<profile>.max_velocity|max_acceleration|max_jerk` (one value per joint of the
/// group), `follow_posture_gain|max_speed|rest`, and needs `robot_description`, `robot_description_semantic`,
/// `robot_description_kinematics` and `robot_description_planning` on it, as move_group does.
class ServoStreamer : public CartesianStreamer
{
public:
  /// \throws std::runtime_error if the Servo or shaping parameters are invalid.
  explicit ServoStreamer(const rclcpp::Node::SharedPtr & node);

  double period() const override;
  void start() override;
  StreamStatus step(const Twist & twist) override;
  bool halt() override;

private:
  /// Add \p state to the trajectory window, publish the window, remember \p state.
  void send(moveit_servo::KinematicState & state);
  /// Add the posture term (follow_posture_*) to \p state: one period of null-space motion
  /// towards follow_posture_rest, which leaves the TCP where Servo put it.
  void add_posture(moveit_servo::KinematicState & state);
  /// Replace \p state's motion by one shaper period towards its velocity relative to last_.
  void shape(moveit_servo::KinematicState & state);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<const servo::ParamListener> params_;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;
  std::unique_ptr<moveit_servo::Servo> servo_;
  moveit::core::RobotStatePtr robot_state_;
  const moveit::core::JointModelGroup * group_{nullptr};
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;

  moveit_servo::KinematicState last_;                // last commanded joint state
  std::deque<moveit_servo::KinematicState> window_;  // the states the next message carries
  std::unique_ptr<JointShaper> shaper_;              // null for profile "off"
  rclcpp::Time stamp_;                               // time of the last command (period grid)
  double posture_gain_{0.0};                         // 1/s, 0 = no posture term
  double posture_max_speed_{0.2};                    // rad/s
  Eigen::VectorXd posture_rest_;                     // rad, the posture it pulls towards
};

}  // namespace fer_moveit

#endif  // FER_MOVEIT__ADAPTERS__SERVO_STREAMER_HPP_
