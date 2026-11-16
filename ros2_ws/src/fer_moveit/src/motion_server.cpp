#include "fer_moveit/motion_server.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "fer_interfaces/msg/joint_configuration.hpp"
#include "fer_interfaces/msg/outcome.hpp"
#include "fer_moveit/core/checks.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fer_moveit
{

using fer_interfaces::msg::JointConfiguration;
using fer_interfaces::msg::Outcome;
using fer_interfaces::msg::PoseTarget;
using fer_interfaces::msg::WorldObject;

namespace
{

constexpr char BASE_FRAME[] = "base";
constexpr char TCP_FRAME[] = "fer_hand_tcp";

Outcome make_outcome(uint8_t code, const std::string & message)
{
  Outcome outcome;
  outcome.code = code;
  outcome.message = message;
  return outcome;
}

void check_ids(const std::vector<WorldObject> & objects, const std::vector<std::string> & ids)
{
  for (const auto & id : ids)
  {
    const bool known = std::any_of(
      objects.begin(), objects.end(), [&id](const WorldObject & o) { return o.id == id; });
    if (!known)
    {
      throw MotionError(Outcome::NOT_FOUND, "unknown object '" + id + "' in may_touch");
    }
  }
}

Eigen::Isometry3d to_eigen(const geometry_msgs::msg::Transform & t)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(t.translation.x, t.translation.y, t.translation.z);
  pose.linear() = Eigen::Quaterniond(t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z)
                    .normalized()
                    .toRotationMatrix();
  return pose;
}

Eigen::Isometry3d to_eigen(const geometry_msgs::msg::Pose & p)
{
  geometry_msgs::msg::Transform t;
  t.translation.x = p.position.x;
  t.translation.y = p.position.y;
  t.translation.z = p.position.z;
  t.rotation = p.orientation;
  return to_eigen(t);
}

}  // namespace

template <typename ActionT>
typename rclcpp_action::Server<ActionT>::SharedPtr MotionServer::create_action(
  const std::string & name)
{
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  return rclcpp_action::create_server<ActionT>(
    node_, name,
    [this](
      const rclcpp_action::GoalUUID & /*goal_id*/,
      std::shared_ptr<const typename ActionT::Goal> /*goal*/)
    {
      return ready_ ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                    : rclcpp_action::GoalResponse::REJECT;
    },
    [](const std::shared_ptr<GoalHandle> /*goal_handle*/)
    { return rclcpp_action::CancelResponse::ACCEPT; },
    [this](const std::shared_ptr<GoalHandle> goal_handle)
    {
      {
        std::lock_guard<std::mutex> lock(goal_mutex_);
        active_goal_ = goal_handle->get_goal_id();
      }
      std::thread(
        [this, goal_handle]
        {
          if constexpr (std::is_same_v<ActionT, Follow>)
          {
            execute_follow(goal_handle);
          }
          else
          {
            execute<ActionT>(goal_handle);
          }
        })
        .detach();
    },
    rcl_action_server_get_default_options(), group_);
}

template <typename ActionT>
void MotionServer::execute(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle)
{
  std::lock_guard<std::mutex> motion(motion_mutex_);
  const auto goal = goal_handle->get_goal();
  const auto goal_id = goal_handle->get_goal_id();
  auto result = std::make_shared<typename ActionT::Result>();
  auto feedback = std::make_shared<typename ActionT::Feedback>();
  const std::function<bool()> interrupted = [this, goal_handle, goal_id]
  { return goal_handle->is_canceling() || !is_active(goal_id); };
  bool moving = false;
  try
  {
    if (interrupted())
    {
      throw Interrupted();
    }
    const MotionTarget target = target_of(*goal);
    check_robot_mode();
    feedback->phase = ActionT::Feedback::PLANNING;
    goal_handle->publish_feedback(feedback);

    const auto objects = world_model_.snapshot();
    check_ids(objects, target.may_touch);
    move_group_.sync_scene(objects);
    const auto trajectory =
      move_group_.plan(target, goal->speed_scaling, std::nullopt, interrupted);

    feedback->phase = ActionT::Feedback::EXECUTING;
    moving = true;
    move_group_.execute(
      trajectory, interrupted,
      [&feedback, &goal_handle](double progress)
      {
        feedback->progress = static_cast<float>(progress);
        goal_handle->publish_feedback(feedback);
      });
    result->outcome = make_outcome(Outcome::OK, "");
  }
  catch (const Interrupted &)
  {
    const std::string reason =
      goal_handle->is_canceling() ? "cancelled by the client" : "replaced by a newer goal";
    if (moving && !wait_for_rest())
    {
      result->outcome =
        make_outcome(Outcome::EXECUTION_FAILED, reason + ", but the arm did not come to rest");
    }
    else
    {
      result->outcome = make_outcome(Outcome::CANCELLED, reason);
    }
  }
  catch (const MotionError & e)
  {
    result->outcome = make_outcome(e.code(), e.what());
    if (moving && e.code() == Outcome::EXECUTION_FAILED && robot_mode_)
    {
      if (const auto error = robot_mode_->error())
      {
        result->outcome = make_outcome(Outcome::ROBOT_ERROR, *error);
      }
    }
  }
  catch (const std::exception & e)
  {
    result->outcome = make_outcome(Outcome::EXECUTION_FAILED, e.what());
  }
  finish<ActionT>(goal_handle, result);
}

template <typename ActionT>
void MotionServer::finish(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle,
  const std::shared_ptr<typename ActionT::Result> & result)
{
  if (result->outcome.code == Outcome::OK)
  {
    goal_handle->succeed(result);
  }
  else if (result->outcome.code == Outcome::CANCELLED && goal_handle->is_canceling())
  {
    goal_handle->canceled(result);
  }
  else
  {
    goal_handle->abort(result);
  }
  std::lock_guard<std::mutex> lock(goal_mutex_);
  if (active_goal_ == goal_handle->get_goal_id())
  {
    active_goal_.reset();
  }
}

void MotionServer::execute_follow(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<Follow>> & goal_handle)
{
  std::lock_guard<std::mutex> motion(motion_mutex_);
  const auto goal = goal_handle->get_goal();
  const auto goal_id = goal_handle->get_goal_id();
  auto result = std::make_shared<Follow::Result>();
  auto feedback = std::make_shared<Follow::Feedback>();
  const std::function<bool()> interrupted = [this, goal_handle, goal_id]
  { return goal_handle->is_canceling() || !is_active(goal_id); };
  CartesianStreamer * moving = nullptr;  // set once the arm is commanded
  const auto stop = [this, &moving, &result](uint8_t code, const std::string & message)
  {
    if (moving && !halt(*moving))
    {
      result->outcome =
        make_outcome(Outcome::EXECUTION_FAILED, message + ", but the arm did not come to rest");
      return;
    }
    result->outcome = make_outcome(code, message);
  };
  try
  {
    if (interrupted())
    {
      throw Interrupted();
    }
    check_speed(goal->speed_scaling);
    if (goal->target_frame.empty())
    {
      throw MotionError(Outcome::INVALID_GOAL, "target_frame is empty");
    }
    if (!(goal->target_timeout > 0.0))
    {
      throw MotionError(Outcome::INVALID_GOAL, "target_timeout must be > 0");
    }
    if (goal->follow_orientation)
    {
      throw MotionError(Outcome::UNSUPPORTED, "follow_orientation: position only for now");
    }
    check_robot_mode();
    auto & stream = streamer();
    follow(
      *goal, stream, interrupted, [&moving, &stream] { moving = &stream; },
      [&feedback, &goal_handle](float distance)
      {
        feedback->distance = distance;
        goal_handle->publish_feedback(feedback);
      });
  }
  catch (const Interrupted &)
  {
    stop(
      Outcome::CANCELLED,
      goal_handle->is_canceling() ? "cancelled by the client" : "replaced by a newer goal");
  }
  catch (const MotionError & e)
  {
    auto code = e.code();
    std::string message = e.what();
    if (code == Outcome::EXECUTION_FAILED && robot_mode_)
    {
      if (const auto error = robot_mode_->error())
      {
        code = Outcome::ROBOT_ERROR;
        message = *error;
      }
    }
    stop(code, message);
  }
  catch (const std::exception & e)
  {
    stop(Outcome::EXECUTION_FAILED, e.what());
  }
  finish<Follow>(goal_handle, result);
}

void MotionServer::follow(
  const Follow::Goal & goal, CartesianStreamer & streamer,
  const std::function<bool()> & interrupted, const std::function<void()> & on_started,
  const std::function<void(float)> & on_distance)
{
  FollowGains gains = follow_gains_;
  gains.max_speed *= goal.speed_scaling;
  gains.max_rotation_speed *= goal.speed_scaling;
  const Eigen::Isometry3d offset = to_eigen(goal.offset);

  geometry_msgs::msg::TransformStamped target;
  try
  {
    target = tf_buffer_->lookupTransform(BASE_FRAME, goal.target_frame, tf2::TimePointZero);
  }
  catch (const tf2::TransformException & e)
  {
    throw MotionError(
      Outcome::NOT_FOUND, "target frame '" + goal.target_frame + "' not in TF: " + e.what());
  }
  // follow_orientation false: the TCP keeps the orientation it has now.
  const Eigen::Isometry3d start = lookup(TCP_FRAME);
  const Eigen::Matrix3d orientation = start.linear();

  VelocityEstimator target_velocity(follow_velocity_time_constant_);
  streamer.start();
  on_started();
  const auto period = rclcpp::Duration::from_seconds(streamer.period());
  auto next = node_->now();
  auto target_seen = next;  // last time the target had a new sample
  auto feedback_due = next;
  auto last_status = StreamStatus::Code::OK;
  std::optional<rclcpp::Time> halted_since;  // the stream has reported HALTED since then
  // Commanded, but the TCP does not follow. A low speed_scaling must not put every command
  // below min_speed, which would switch the watchdog off.
  StallLimits stall_limits = follow_stall_;
  stall_limits.min_speed = std::min(stall_limits.min_speed, 0.5 * gains.max_speed);
  StallDetector stall(stall_limits);
  Eigen::Vector3d held = start.translation();  // last goal inside the workspace
  bool holding = false;
  while (true)
  {
    if (interrupted())
    {
      throw Interrupted();
    }
    check_robot_mode();

    try
    {
      target = tf_buffer_->lookupTransform(BASE_FRAME, goal.target_frame, tf2::TimePointZero);
    }
    catch (const tf2::TransformException &)
    {
      throw MotionError(Outcome::NOT_FOUND, "target frame '" + goal.target_frame + "' lost");
    }
    const Eigen::Isometry3d target_pose = to_eigen(target.transform);
    // Stamp 0: a static frame, always current. Otherwise the target's own stamps count, so
    // the target may run on another clock (the hand simulation does).
    const double stamp = rclcpp::Time(target.header.stamp).seconds();
    const auto now = node_->now();
    if (stamp == 0.0 || target_velocity.update(stamp, target_pose.translation()))
    {
      target_seen = now;
    }
    else if ((now - target_seen).seconds() > goal.target_timeout)
    {
      throw MotionError(
        Outcome::NOT_FOUND, "target frame '" + goal.target_frame + "' not updated for " +
                              std::to_string(goal.target_timeout) + " s");
    }

    Eigen::Isometry3d goal_pose = target_pose * offset;
    goal_pose.linear() = orientation;
    // Follow only while the goal is in the workspace, away from the arm's singular regions.
    // Outside it, hold the last goal that was inside and wait: sliding along the wall instead
    // could bring the TCP onto the target (a goal above the hand, held at the box top, would
    // be at the hand).
    const bool inside = follow_workspace_.contains(goal_pose.translation());
    if (inside)
    {
      held = goal_pose.translation();
    }
    if (inside == holding)
    {
      RCLCPP_INFO(
        node_->get_logger(), "follow: %s",
        inside ? "goal inside the workspace again" : "goal outside the workspace, holding");
      holding = !inside;
    }
    Eigen::Isometry3d command = goal_pose;
    command.translation() = held;
    // Feed the target velocity forward only while its samples are fresh: a tracker that stopped
    // sending would otherwise carry the TCP past the last known position until target_timeout.
    const double feedforward =
      inside ? follow_velocity_fade_.weight((now - target_seen).seconds()) : 0.0;
    const Eigen::Vector3d velocity = feedforward * target_velocity.velocity();
    const Eigen::Isometry3d tcp = lookup(TCP_FRAME);
    const auto twist = follow_twist(tcp, command, velocity, gains);
    const auto status = streamer.step(twist);
    if (status.code == StreamStatus::Code::INVALID)
    {
      throw MotionError(Outcome::EXECUTION_FAILED, "Cartesian stream: " + status.message);
    }
    if (status.code != last_status)
    {
      RCLCPP_INFO(node_->get_logger(), "follow: %s", status.message.c_str());
      last_status = status.code;
    }
    // A halt that lasts (e.g. Servo at a singularity: it then blocks every direction) does
    // not end by itself; give the arm back to the caller, who can recover with a planned move.
    if (status.code != StreamStatus::Code::HALTED)
    {
      halted_since.reset();
    }
    else if (!halted_since)
    {
      halted_since = now;
    }
    else if ((now - *halted_since).seconds() >= follow_halt_timeout_)
    {
      throw MotionError(
        Outcome::UNREACHABLE, "Cartesian stream halted for " +
                                std::to_string(follow_halt_timeout_) + " s: " + status.message);
    }
    // A stream can also slow down almost to zero without ever reporting a halt (Servo near a
    // singularity: SLOWED down to ~0 % of the command). Judge what the TCP really does.
    if (stall.update(now.seconds(), twist.head<3>(), tcp.translation()))
    {
      char speeds[96];
      std::snprintf(
        speeds, sizeof(speeds), "TCP moved %.3f m/s along the commanded %.3f m/s, for %.1f s",
        stall.measured_speed(), stall.commanded_speed(), follow_stall_.timeout);
      throw MotionError(Outcome::UNREACHABLE, std::string(speeds) + " (" + status.message + ")");
    }
    if (now >= feedback_due)
    {
      on_distance(static_cast<float>((goal_pose.translation() - tcp.translation()).norm()));
      feedback_due = now + rclcpp::Duration::from_seconds(follow_feedback_period_);
    }

    next += period;
    if (node_->now() - next > period)
    {
      next = node_->now();  // fell behind: do not try to catch up
    }
    sleep_until(next);
  }
}

CartesianStreamer & MotionServer::streamer()
{
  if (!streamer_)
  {
    if (!make_streamer_)
    {
      throw MotionError(Outcome::UNSUPPORTED, "no Cartesian streamer configured");
    }
    try
    {
      streamer_ = make_streamer_();
    }
    catch (const std::exception & e)
    {
      throw MotionError(Outcome::UNSUPPORTED, std::string("Cartesian streamer: ") + e.what());
    }
  }
  return *streamer_;
}

bool MotionServer::halt(CartesianStreamer & streamer)
{
  try
  {
    const auto period = rclcpp::Duration::from_seconds(streamer.period());
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(rest_timeout_);
    auto next = node_->now();
    while (!streamer.halt())
    {
      if (std::chrono::steady_clock::now() > deadline)
      {
        return false;
      }
      next += period;
      sleep_until(next);
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(node_->get_logger(), "follow: stopping failed: %s", e.what());
    return false;
  }
  return wait_for_rest();
}

void MotionServer::sleep_until(const rclcpp::Time & time)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (rclcpp::ok() && node_->now() < time)
  {
    if (std::chrono::steady_clock::now() > deadline)
    {
      throw MotionError(Outcome::TIMEOUT, "the clock stopped advancing");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

Eigen::Isometry3d MotionServer::lookup(const std::string & frame) const
{
  try
  {
    return to_eigen(tf_buffer_->lookupTransform(BASE_FRAME, frame, tf2::TimePointZero).transform);
  }
  catch (const tf2::TransformException & e)
  {
    throw MotionError(Outcome::EXECUTION_FAILED, "no TF base -> " + frame + ": " + e.what());
  }
}

MotionServer::MotionServer(
  const rclcpp::Node::SharedPtr & node, WorldModelClient & world_model,
  MoveGroupClient & move_group, const RobotModeMonitor * robot_mode,
  const rclcpp::CallbackGroup::SharedPtr & group, StreamerFactory make_streamer)
: node_(node),
  world_model_(world_model),
  move_group_(move_group),
  robot_mode_(robot_mode),
  group_(group),
  tf_timeout_(node->declare_parameter("tf_timeout", 0.2)),
  rest_velocity_(node->declare_parameter("rest_velocity", 0.01)),
  rest_timeout_(node->declare_parameter("rest_timeout", 3.0)),
  startup_timeout_(node->declare_parameter("startup_timeout", 10.0)),
  follow_gains_{
    node->declare_parameter("follow_gain", 2.0), node->declare_parameter("follow_max_speed", 0.25),
    node->declare_parameter("follow_rotation_gain", 2.0),
    node->declare_parameter("follow_max_rotation_speed", 0.5)},
  follow_velocity_time_constant_(node->declare_parameter("follow_velocity_time_constant", 0.05)),
  follow_velocity_fade_{
    node->declare_parameter("follow_velocity_hold", 0.05),
    node->declare_parameter("follow_velocity_fade", 0.05)},
  follow_feedback_period_(node->declare_parameter("follow_feedback_period", 0.1)),
  follow_workspace_(Workspace::from(
    node->declare_parameter("follow_workspace_min", std::vector<double>{0.25, -0.30, 0.25}),
    node->declare_parameter("follow_workspace_max", std::vector<double>{0.60, 0.30, 0.50}))),
  follow_halt_timeout_(node->declare_parameter("follow_halt_timeout", 1.0)),
  follow_stall_{
    node->declare_parameter("follow_stall_window", 0.5),
    node->declare_parameter("follow_stall_min_speed", 0.03),
    node->declare_parameter("follow_stall_ratio", 0.2),
    node->declare_parameter("follow_stall_timeout", 1.5)},
  make_streamer_(std::move(make_streamer)),
  startup_begin_(std::chrono::steady_clock::now())
{
  // Fail at startup, not on the first Follow goal.
  follow_velocity_fade_.validate();
  follow_stall_.validate();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node);

  rclcpp::SubscriptionOptions options;
  options.callback_group = group;
  joint_state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", rclcpp::QoS(10),
    [this](const sensor_msgs::msg::JointState::ConstSharedPtr & msg) { on_joint_state(msg); },
    options);

  pose_server_ = create_action<MoveToPose>("/motion/move_to_pose");
  joints_server_ = create_action<MoveToJoints>("/motion/move_to_joints");
  follow_server_ = create_action<Follow>("/motion/follow");
  reachable_service_ = node->create_service<CheckReachable>(
    "/motion/check_reachable",
    [this](
      const std::shared_ptr<CheckReachable::Request> request,
      std::shared_ptr<CheckReachable::Response> response)
    { on_check_reachable(request, response); },
    rclcpp::ServicesQoS(), group);

  startup_timer_ =
    node->create_wall_timer(std::chrono::milliseconds(100), [this] { on_startup(); }, group);
}

MotionTarget MotionServer::target_of(const MoveToPose::Goal & goal) const
{
  check_speed(goal.speed_scaling);
  return pose_target(goal.target);
}

MotionTarget MotionServer::target_of(const MoveToJoints::Goal & goal) const
{
  check_speed(goal.speed_scaling);
  MotionTarget target;
  target.kind = MotionTarget::Kind::JOINTS;
  target.joints.assign(goal.joints.positions.begin(), goal.joints.positions.end());
  return target;
}

MotionTarget MotionServer::pose_target(const PoseTarget & target) const
{
  if (target.path != PoseTarget::PATH_FREE && target.path != PoseTarget::PATH_STRAIGHT)
  {
    throw MotionError(Outcome::INVALID_GOAL, "unknown path type " + std::to_string(target.path));
  }
  if (target.pose.header.frame_id.empty())
  {
    throw MotionError(Outcome::INVALID_GOAL, "pose has no frame_id");
  }
  auto pose = target.pose;
  pose.header.stamp = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  MotionTarget result;
  try
  {
    result.pose = tf_buffer_->transform(pose, BASE_FRAME, tf2::durationFromSec(tf_timeout_));
  }
  catch (const tf2::TransformException & e)
  {
    throw MotionError(
      Outcome::INVALID_GOAL, "cannot transform '" + target.pose.header.frame_id + "' to '" +
                               BASE_FRAME + "': " + e.what());
  }
  result.kind = target.path == PoseTarget::PATH_STRAIGHT ? MotionTarget::Kind::POSE_STRAIGHT
                                                         : MotionTarget::Kind::POSE_FREE;
  result.may_touch = target.may_touch;
  return result;
}

void MotionServer::check_speed(double speed_scaling) const
{
  if (!speed_valid(speed_scaling))
  {
    throw MotionError(
      Outcome::INVALID_GOAL,
      "speed_scaling must be in (0, 1], got " + std::to_string(speed_scaling));
  }
}

void MotionServer::check_robot_mode() const
{
  if (robot_mode_)
  {
    if (const auto error = robot_mode_->error())
    {
      throw MotionError(Outcome::ROBOT_ERROR, *error);
    }
  }
}

bool MotionServer::is_active(const rclcpp_action::GoalUUID & goal_id)
{
  std::lock_guard<std::mutex> lock(goal_mutex_);
  return active_goal_ == goal_id;
}

bool MotionServer::wait_for_rest()
{
  uint64_t seen;
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    seen = joint_count_;
  }
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::duration<double>(rest_timeout_);
  while (std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(POLL_PERIOD);
    std::lock_guard<std::mutex> lock(joint_mutex_);
    if (joint_count_ > seen && at_rest(velocities_, rest_velocity_))
    {
      return true;
    }
  }
  return false;
}

std::optional<sensor_msgs::msg::JointState> MotionServer::current_joints()
{
  std::lock_guard<std::mutex> lock(joint_mutex_);
  if (joint_count_ == 0)
  {
    return std::nullopt;
  }
  sensor_msgs::msg::JointState state;
  state.name = move_group_.joint_names();
  state.position = positions_;
  return state;
}

void MotionServer::on_joint_state(const sensor_msgs::msg::JointState::ConstSharedPtr & msg)
{
  const auto & names = move_group_.joint_names();
  std::vector<double> positions;
  std::vector<double> velocities;
  for (const auto & name : names)
  {
    const auto it = std::find(msg->name.begin(), msg->name.end(), name);
    if (it == msg->name.end())
    {
      return;
    }
    const auto i = static_cast<size_t>(it - msg->name.begin());
    positions.push_back(msg->position.at(i));
    if (msg->velocity.size() == msg->name.size())
    {
      velocities.push_back(msg->velocity[i]);
    }
  }
  const rclcpp::Time stamp(msg->header.stamp, node_->get_clock()->get_clock_type());
  std::lock_guard<std::mutex> lock(joint_mutex_);
  if (velocities.size() != positions.size())
  {
    // No velocities published: estimate them from consecutive positions.
    const double dt = joint_count_ > 0 ? (stamp - joint_stamp_).seconds() : 0.0;
    velocities.assign(positions.size(), std::numeric_limits<double>::infinity());
    if (dt > 0.0 && positions_.size() == positions.size())
    {
      for (size_t i = 0; i < positions.size(); ++i)
      {
        velocities[i] = (positions[i] - positions_[i]) / dt;
      }
    }
  }
  positions_ = std::move(positions);
  velocities_ = std::move(velocities);
  joint_stamp_ = stamp;
  ++joint_count_;
}

void MotionServer::on_check_reachable(
  const std::shared_ptr<CheckReachable::Request> request,
  std::shared_ptr<CheckReachable::Response> response)
{
  response->outcome = make_outcome(Outcome::OK, "");
  const auto & names = move_group_.joint_names();
  size_t index = 0;
  try
  {
    if (!ready_)
    {
      throw MotionError(Outcome::TIMEOUT, "motion server not ready");
    }
    if (request->targets.empty())
    {
      throw MotionError(Outcome::INVALID_GOAL, "no targets");
    }
    check_speed(request->speed_scaling);

    std::vector<MotionTarget> targets;
    for (index = 0; index < request->targets.size(); ++index)
    {
      targets.push_back(pose_target(request->targets[index]));
    }
    index = 0;
    const auto objects = world_model_.snapshot();
    for (index = 0; index < targets.size(); ++index)
    {
      check_ids(objects, targets[index].may_touch);
    }
    index = 0;
    move_group_.sync_scene(objects);
    auto start = current_joints();
    for (index = 0; index < targets.size(); ++index)
    {
      const auto trajectory =
        move_group_.plan(targets[index], request->speed_scaling, start, [] { return false; });
      const auto & joint_trajectory = trajectory.joint_trajectory;
      sensor_msgs::msg::JointState reached;
      reached.name = joint_trajectory.joint_names;
      reached.position = joint_trajectory.points.back().positions;
      JointConfiguration configuration;
      for (size_t j = 0; j < names.size() && j < configuration.positions.size(); ++j)
      {
        const auto it = std::find(reached.name.begin(), reached.name.end(), names[j]);
        if (it != reached.name.end())
        {
          configuration.positions[j] =
            reached.position.at(static_cast<size_t>(it - reached.name.begin()));
        }
      }
      response->configurations.push_back(configuration);
      start = reached;
    }
  }
  catch (const MotionError & e)
  {
    response->outcome = make_outcome(e.code(), e.what());
    response->failed_index = static_cast<uint32_t>(index);
  }
}

void MotionServer::on_startup()
{
  if (ready_)
  {
    return;
  }
  bool joint_state_seen;
  {
    std::lock_guard<std::mutex> lock(joint_mutex_);
    joint_state_seen = joint_count_ > 0;
  }
  if (joint_state_seen && move_group_.ready())
  {
    ready_ = true;
    startup_timer_->cancel();
    RCLCPP_INFO(node_->get_logger(), "Motion server ready");
    return;
  }
  const std::chrono::duration<double> waited = std::chrono::steady_clock::now() - startup_begin_;
  if (waited.count() > startup_timeout_)
  {
    startup_timer_->cancel();
    RCLCPP_FATAL(
      node_->get_logger(),
      "move_group (/move_action, /execute_trajectory, /apply_planning_scene, "
      "/get_planning_scene, /trajectory_execution_event) or /joint_states not available after "
      "%.1f s",
      startup_timeout_);
    node_->get_node_base_interface()->get_context()->shutdown("motion server startup failed");
  }
}

}  // namespace fer_moveit
