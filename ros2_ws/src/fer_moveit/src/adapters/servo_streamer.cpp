#include "fer_moveit/adapters/servo_streamer.hpp"

#include <moveit_servo/utils/common.hpp>
#include <string>

#include "fer_interfaces/msg/outcome.hpp"
#include "fer_moveit/core/checks.hpp"

namespace fer_moveit
{

using fer_interfaces::msg::Outcome;
using moveit_servo::KinematicState;
using moveit_servo::StatusCode;

namespace
{

constexpr char BASE_FRAME[] = "base";  // the TF root; twists are given in it
constexpr double STATE_TIMEOUT = 1.0;  // s, wait for a current joint state at start()

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
}

double ServoStreamer::period() const { return servo_->getParams().publish_period; }

void ServoStreamer::start()
{
  // Catches a controller that is not loaded. A loaded but inactive JTC still subscribes and
  // ignores the commands; that shows as an arm that does not move.
  if (publisher_->get_subscription_count() == 0)
  {
    throw MotionError(
      Outcome::EXECUTION_FAILED, "nobody listens on " + std::string(publisher_->get_topic_name()) +
                                   ": is the arm's trajectory controller loaded and active?");
  }
  if (!scene_monitor_->getStateMonitor()->waitForCurrentState(node_->now(), STATE_TIMEOUT))
  {
    throw MotionError(Outcome::TIMEOUT, "no current joint state for the arm");
  }
  // Integrate from the measured state once; after that from what was commanded, as Servo's
  // own node does, so measurement noise does not enter the command.
  last_ = servo_->getCurrentRobotState(false);
  servo_->resetSmoothing(last_);
  window_.clear();
  moveit_servo::updateSlidingWindow(
    last_, window_, servo_->getParams().max_expected_latency, node_->now());
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
    send(next);
  }
  return {code, servo_->getStatusMessage()};
}

bool ServoStreamer::halt()
{
  auto [stopped, next] = servo_->smoothHalt(last_);
  send(next);
  return stopped;
}

void ServoStreamer::send(KinematicState & state)
{
  const auto & params = servo_->getParams();
  moveit_servo::updateSlidingWindow(state, window_, params.max_expected_latency, node_->now());
  if (const auto msg = moveit_servo::composeTrajectoryMessage(params, window_))
  {
    publisher_->publish(*msg);
  }
  last_ = state;
}

}  // namespace fer_moveit
