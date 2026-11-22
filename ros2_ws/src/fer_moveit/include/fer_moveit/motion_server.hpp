#ifndef FER_MOVEIT__MOTION_SERVER_HPP_
#define FER_MOVEIT__MOTION_SERVER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "fer_interfaces/action/follow.hpp"
#include "fer_interfaces/action/move_to_joints.hpp"
#include "fer_interfaces/action/move_to_pose.hpp"
#include "fer_interfaces/msg/pose_target.hpp"
#include "fer_interfaces/msg/world_object.hpp"
#include "fer_interfaces/srv/check_reachable.hpp"
#include "fer_moveit/adapters/cartesian_streamer.hpp"
#include "fer_moveit/adapters/move_group_client.hpp"
#include "fer_moveit/adapters/robot_mode_monitor.hpp"
#include "fer_moveit/adapters/world_model_client.hpp"
#include "fer_moveit/core/follow_law.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace fer_moveit
{

/// \brief Serves /motion/move_to_pose, /motion/move_to_joints, /motion/follow and
/// /motion/check_reachable.
///
/// One motion runs at a time; a newer goal stops the running one, which ends CANCELLED once
/// the arm is at rest.
class MotionServer
{
public:
  /// Creates the Cartesian stage of /motion/follow on its first goal.
  using StreamerFactory = std::function<std::unique_ptr<CartesianStreamer>()>;

  /// \param robot_mode null when the hardware reports no robot mode (MuJoCo, mock).
  /// \param make_streamer empty: /motion/follow answers UNSUPPORTED.
  MotionServer(
    const rclcpp::Node::SharedPtr & node, WorldModelClient & world_model,
    MoveGroupClient & move_group, const RobotModeMonitor * robot_mode,
    const rclcpp::CallbackGroup::SharedPtr & group, StreamerFactory make_streamer = {});

  /// \brief True once move_group is reachable and a joint state has arrived.
  bool ready() const { return ready_; }

private:
  using MoveToPose = fer_interfaces::action::MoveToPose;
  using MoveToJoints = fer_interfaces::action::MoveToJoints;
  using Follow = fer_interfaces::action::Follow;
  using CheckReachable = fer_interfaces::srv::CheckReachable;

  template <typename ActionT>
  typename rclcpp_action::Server<ActionT>::SharedPtr create_action(const std::string & name);

  template <typename ActionT>
  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle);

  /// End the goal with \p result and release it.
  template <typename ActionT>
  void finish(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle,
    const std::shared_ptr<typename ActionT::Result> & result);

  void execute_follow(const std::shared_ptr<rclcpp_action::ServerGoalHandle<Follow>> & goal_handle);
  /// The follow loop; returns only by throwing (Interrupted, MotionError).
  /// \p on_started is called once the arm is commanded, \p on_distance with the feedback.
  void follow(
    const Follow::Goal & goal, CartesianStreamer & streamer,
    const std::function<bool()> & interrupted, const std::function<void()> & on_started,
    const std::function<void(float)> & on_distance);
  CartesianStreamer & streamer();
  /// Brake the stream until the arm rests; false if it does not within rest_timeout.
  bool halt(CartesianStreamer & streamer);
  /// Sleep until \p time on the node clock.
  /// \throws MotionError TIMEOUT if that clock (sim time) stops advancing.
  void sleep_until(const rclcpp::Time & time);
  Eigen::Isometry3d lookup(const std::string & frame) const;

  MotionTarget target_of(const MoveToPose::Goal & goal) const;
  MotionTarget target_of(const MoveToJoints::Goal & goal) const;
  MotionTarget pose_target(const fer_interfaces::msg::PoseTarget & target) const;
  void check_speed(double speed_scaling) const;
  void check_robot_mode() const;
  bool is_active(const rclcpp_action::GoalUUID & goal_id);
  bool wait_for_rest();
  std::optional<sensor_msgs::msg::JointState> current_joints();

  void on_check_reachable(
    const std::shared_ptr<CheckReachable::Request> request,
    std::shared_ptr<CheckReachable::Response> response);
  void on_joint_state(const sensor_msgs::msg::JointState::ConstSharedPtr & msg);
  void on_startup();

  rclcpp::Node::SharedPtr node_;
  WorldModelClient & world_model_;
  MoveGroupClient & move_group_;
  const RobotModeMonitor * robot_mode_;
  rclcpp::CallbackGroup::SharedPtr group_;

  double tf_timeout_;
  double rest_velocity_;
  double rest_timeout_;
  double startup_timeout_;
  FollowGains follow_gains_;
  double follow_velocity_time_constant_;
  VelocityFade follow_velocity_fade_;
  double follow_feedback_period_;
  Workspace follow_workspace_;
  double follow_halt_timeout_;
  StallLimits follow_stall_;

  StreamerFactory make_streamer_;
  std::unique_ptr<CartesianStreamer> streamer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp_action::Server<MoveToPose>::SharedPtr pose_server_;
  rclcpp_action::Server<MoveToJoints>::SharedPtr joints_server_;
  rclcpp_action::Server<Follow>::SharedPtr follow_server_;

  rclcpp::Service<CheckReachable>::SharedPtr reachable_service_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  std::chrono::steady_clock::time_point startup_begin_;

  std::atomic<bool> ready_{false};
  std::mutex goal_mutex_;
  std::optional<rclcpp_action::GoalUUID> active_goal_;
  // Held by the goal that plans and moves; a replaced goal releases it once the arm rests.
  std::mutex motion_mutex_;

  std::mutex joint_mutex_;
  std::vector<double> positions_;
  std::vector<double> velocities_;
  rclcpp::Time joint_stamp_;
  uint64_t joint_count_{0};
};

}  // namespace fer_moveit

#endif  // FER_MOVEIT__MOTION_SERVER_HPP_
