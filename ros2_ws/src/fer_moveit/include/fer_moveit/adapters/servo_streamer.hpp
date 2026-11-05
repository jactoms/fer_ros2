#ifndef FER_MOVEIT__ADAPTERS__SERVO_STREAMER_HPP_
#define FER_MOVEIT__ADAPTERS__SERVO_STREAMER_HPP_

#include <deque>
#include <memory>
#include <moveit/planning_scene_monitor/planning_scene_monitor.hpp>
#include <moveit_servo/servo.hpp>

#include "fer_moveit/adapters/cartesian_streamer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace fer_moveit
{

/// \brief CartesianStreamer with MoveIt Servo as a library, in the motion server's process.
///
/// Servo computes the joint state for each cycle; this class sends it as a short joint
/// trajectory on the controller's topic (`moveit_servo.command_out_topic`). Servo's own node is
/// not used, so the joint command passes through here: that is where shaping plugs in.
///
/// Reads the `moveit_servo.*` parameters of the node, and needs `robot_description`,
/// `robot_description_semantic`, `robot_description_kinematics` and
/// `robot_description_planning` on it, as move_group does.
class ServoStreamer : public CartesianStreamer
{
public:
  /// \throws std::runtime_error if the Servo parameters are invalid.
  explicit ServoStreamer(const rclcpp::Node::SharedPtr & node);

  double period() const override;
  void start() override;
  StreamStatus step(const Twist & twist) override;
  bool halt() override;

private:
  /// Add \p state to the trajectory window, publish the window, remember \p state.
  void send(moveit_servo::KinematicState & state);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<const servo::ParamListener> params_;
  planning_scene_monitor::PlanningSceneMonitorPtr scene_monitor_;
  std::unique_ptr<moveit_servo::Servo> servo_;
  moveit::core::RobotStatePtr robot_state_;
  const moveit::core::JointModelGroup * group_{nullptr};
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;

  moveit_servo::KinematicState last_;                // last commanded joint state
  std::deque<moveit_servo::KinematicState> window_;  // the states the next message carries
};

}  // namespace fer_moveit

#endif  // FER_MOVEIT__ADAPTERS__SERVO_STREAMER_HPP_
