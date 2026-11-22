#include "fer_moveit/adapters/servo_streamer.hpp"

#include <chrono>
#include <cmath>
#include <moveit_servo/utils/common.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "fer_interfaces/msg/outcome.hpp"
#include "fer_moveit/core/checks.hpp"
#include "fer_moveit/core/follow_law.hpp"

namespace fer_moveit
{

using fer_interfaces::msg::Outcome;
using moveit_servo::KinematicState;
using moveit_servo::StatusCode;

namespace
{

constexpr char BASE_FRAME[] = "base";       // the TF root; twists are given in it
constexpr double STATE_TIMEOUT = 1.0;       // s, wait for a current joint state at start()
constexpr double SUBSCRIBER_TIMEOUT = 2.0;  // s, wait for the controller's subscription at start()
constexpr char NO_SHAPING[] = "off";        // follow_shaping_profile without Ruckig

StreamStatus::Code classify(StatusCode code)
{
  switch (code)
  {
    case StatusCode::NO_WARNING:
      return StreamStatus::Code::OK;
    case StatusCode::DECELERATE_FOR_APPROACHING_SINGULARITY:
    case StatusCode::DECELERATE_FOR_LEAVING_SINGULARITY:
    case StatusCode::DECELERATE_FOR_COLLISION:
      return StreamStatus::Code::SLOWED;
    case StatusCode::HALT_FOR_SINGULARITY:
    case StatusCode::HALT_FOR_COLLISION:
    case StatusCode::JOINT_BOUND:
      return StreamStatus::Code::HALTED;
    case StatusCode::INVALID:
    default:
      return StreamStatus::Code::INVALID;
  }
}

/// Limits of follow_shaping.<profile>; an empty or missing list fails ShapingLimits::validate().
ShapingLimits read_limits(const rclcpp::Node::SharedPtr & node, const std::string & profile)
{
  const auto list = [&](const std::string & name)
  {
    const auto full = "follow_shaping." + profile + "." + name;
    if (!node->has_parameter(full))
    {
      node->declare_parameter(full, std::vector<double>{});
    }
    return node->get_parameter(full).as_double_array();
  };
  ShapingLimits limits;
  limits.max_velocity = list("max_velocity");
  limits.max_acceleration = list("max_acceleration");
  limits.max_jerk = list("max_jerk");
  return limits;
}

}  // namespace

ServoStreamer::ServoStreamer(const rclcpp::Node::SharedPtr & node)
: node_(node), params_(std::make_shared<const servo::ParamListener>(node, "moveit_servo"))
{
  const auto params = params_->get_params();
  scene_monitor_ = moveit_servo::createPlanningSceneMonitor(node_, params);
  servo_ = std::make_unique<moveit_servo::Servo>(node_, params_, scene_monitor_);
  servo_->setCommandType(moveit_servo::CommandType::TWIST);

  robot_state_ = scene_monitor_->getStateMonitor()->getCurrentState();
  group_ = robot_state_->getJointModelGroup(params.move_group_name);
  publisher_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    params.command_out_topic, rclcpp::SystemDefaultsQoS());

  // Posture term (follow_posture_*): read before the shaping block, which returns early for off.
  const auto declared = [&](const std::string & name, const rclcpp::ParameterValue & value)
  {
    if (!node_->has_parameter(name))
    {
      node_->declare_parameter(name, value);
    }
    return node_->get_parameter(name);
  };
  posture_gain_ = declared("follow_posture_gain", rclcpp::ParameterValue(0.0)).as_double();
  posture_max_speed_ =
    declared("follow_posture_max_speed", rclcpp::ParameterValue(0.2)).as_double();
  const auto rest = declared("follow_posture_rest", rclcpp::ParameterValue(std::vector<double>{}))
                      .as_double_array();
  const auto joints = group_->getActiveJointModelNames().size();
  if (posture_gain_ < 0.0 || posture_max_speed_ <= 0.0)
  {
    throw std::runtime_error("follow_posture_gain must be >= 0 and follow_posture_max_speed > 0");
  }
  if (posture_gain_ > 0.0)
  {
    if (rest.size() != joints)
    {
      throw std::runtime_error(
        "follow_posture_rest needs one value per joint of " + params.move_group_name);
    }
    posture_rest_ = Eigen::Map<const Eigen::VectorXd>(rest.data(), rest.size());
    RCLCPP_INFO(
      node_->get_logger(), "follow: posture term on (gain %.2f 1/s, max %.2f rad/s)", posture_gain_,
      posture_max_speed_);
  }

  if (!node_->has_parameter("follow_shaping_profile"))
  {
    // Any type: YAML reads an unquoted `off` as the boolean false.
    rcl_interfaces::msg::ParameterDescriptor any_type;
    any_type.dynamic_typing = true;
    node_->declare_parameter(
      "follow_shaping_profile", rclcpp::ParameterValue(std::string("conservative")), any_type);
  }
  const auto value = node_->get_parameter("follow_shaping_profile");
  std::string profile;
  if (value.get_type() == rclcpp::ParameterType::PARAMETER_STRING)
  {
    profile = value.as_string();
  }
  else if (value.get_type() == rclcpp::ParameterType::PARAMETER_BOOL && !value.as_bool())
  {
    profile = NO_SHAPING;
  }
  else
  {
    throw std::runtime_error("follow_shaping_profile must be a profile name or off");
  }
  if (profile == NO_SHAPING)
  {
    RCLCPP_WARN(node_->get_logger(), "follow: no joint shaping (follow_shaping_profile off)");
    return;
  }
  try
  {
    shaper_ =
      std::make_unique<JointShaper>(read_limits(node_, profile), joints, params.publish_period);
  }
  catch (const std::invalid_argument & e)
  {
    throw std::runtime_error("follow_shaping." + profile + ": " + e.what());
  }
  RCLCPP_INFO(node_->get_logger(), "follow: joint shaping with profile '%s'", profile.c_str());
}

double ServoStreamer::period() const { return servo_->getParams().publish_period; }

void ServoStreamer::start()
{
  // Catches a controller that is not loaded. A loaded but inactive JTC still subscribes and
  // ignores the commands; that shows as an arm that does not move. The publisher was created
  // just before (first Follow goal, streamer check), and DDS discovery of the controller can
  // still be running then: wait for it briefly instead of failing at once.
  const auto begin = std::chrono::steady_clock::now();
  const auto deadline = begin + std::chrono::duration<double>(SUBSCRIBER_TIMEOUT);
  bool waited = false;
  while (publisher_->get_subscription_count() == 0)
  {
    if (std::chrono::steady_clock::now() >= deadline)
    {
      throw MotionError(
        Outcome::EXECUTION_FAILED, "nobody listens on " +
                                     std::string(publisher_->get_topic_name()) +
                                     ": is the arm's trajectory controller loaded and active?");
    }
    waited = true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (waited)
  {
    RCLCPP_INFO(
      node_->get_logger(), "follow: trajectory controller found after %.2f s (discovery)",
      std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count());
  }
  if (!scene_monitor_->getStateMonitor()->waitForCurrentState(node_->now(), STATE_TIMEOUT))
  {
    throw MotionError(Outcome::TIMEOUT, "no current joint state for the arm");
  }
  // Integrate from the measured state once; after that from what was commanded, as Servo's
  // own node does, so measurement noise does not enter the command.
  last_ = servo_->getCurrentRobotState(false);
  servo_->resetSmoothing(last_);
  if (shaper_)
  {
    shaper_->reset(last_.positions, last_.velocities);
  }
  window_.clear();
  stamp_ = node_->now();
  moveit_servo::updateSlidingWindow(
    last_, window_, servo_->getParams().max_expected_latency, stamp_);
}

StreamStatus ServoStreamer::step(const Twist & twist)
{
  robot_state_->setJointGroupActivePositions(group_, last_.positions);
  robot_state_->setJointGroupVelocities(group_, last_.velocities);

  auto next =
    servo_->getNextJointState(robot_state_, moveit_servo::TwistCommand{BASE_FRAME, twist});
  const auto code = classify(servo_->getStatus());
  if (code != StreamStatus::Code::INVALID)
  {
    if (code != StreamStatus::Code::HALTED)
    {
      add_posture(next);  // not while Servo holds the arm (collision, singularity, joint bound)
    }
    try
    {
      shape(next);
    }
    catch (const std::exception & e)
    {
      return {StreamStatus::Code::INVALID, std::string("joint shaping: ") + e.what()};
    }
    send(next);
  }
  return {code, servo_->getStatusMessage()};
}

bool ServoStreamer::halt()
{
  if (!shaper_)
  {
    auto [stopped, next] = servo_->smoothHalt(last_);
    send(next);
    return stopped;
  }
  // Brake within the limits: target velocity zero.
  auto next = last_;
  const auto & shaped = shaper_->step(Eigen::VectorXd::Zero(last_.positions.size()));
  next.positions = shaped.position;
  next.velocities = shaped.velocity;
  next.accelerations = shaped.acceleration;
  send(next);
  return shaper_->at_rest();
}

void ServoStreamer::add_posture(KinematicState & state)
{
  if (posture_gain_ == 0.0)
  {
    return;
  }
  robot_state_->setJointGroupActivePositions(group_, last_.positions);
  robot_state_->updateLinkTransforms();
  const auto velocity = posture_velocity(
    robot_state_->getJacobian(group_), last_.positions, posture_rest_, posture_gain_,
    posture_max_speed_);
  state.positions += velocity * period();
  state.velocities += velocity;
}

void ServoStreamer::shape(KinematicState & state)
{
  if (!shaper_)
  {
    return;
  }
  const Eigen::VectorXd target = (state.positions - last_.positions) / period();
  const auto & shaped = shaper_->step(target);
  state.positions = shaped.position;
  state.velocities = shaped.velocity;
  state.accelerations = shaped.acceleration;
}

void ServoStreamer::send(KinematicState & state)
{
  const auto & params = servo_->getParams();
  // Commands are one period apart in position, so stamp them one period apart too. Stamping
  // with now() put the loop's wake-up jitter (sim time: 5..15 ms) into the controller's
  // reference: a 5 ms gap doubled the commanded velocity for that segment. Back to the clock
  // only if the loop fell behind (or the clock jumped).
  const auto now = node_->now();
  const auto period = rclcpp::Duration::from_seconds(params.publish_period);
  stamp_ += period;
  if (std::abs((now - stamp_).seconds()) > 2.0 * params.publish_period)
  {
    stamp_ = now;
  }
  moveit_servo::updateSlidingWindow(state, window_, params.max_expected_latency, stamp_);
  if (const auto msg = moveit_servo::composeTrajectoryMessage(params, window_))
  {
    publisher_->publish(*msg);
  }
  last_ = state;
}

}  // namespace fer_moveit
