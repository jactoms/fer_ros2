// Check of the Cartesian streamer (WP2 step 2): the TCP moves `distance` along `axis` of the
// base frame at `speed`, then the stream halts. Passes if the TCP covers 80-120 % of the
// distance and drifts less than 1 cm on the other axes. Start with fer_streamer_check.launch.py.

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "fer_moveit/adapters/servo_streamer.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace
{

using fer_moveit::StreamStatus;

constexpr char BASE_FRAME[] = "base";
constexpr char TCP_FRAME[] = "fer_hand_tcp";
constexpr int MAX_HALT_CYCLES = 300;

const char * name_of(StreamStatus::Code code)
{
  switch (code)
  {
    case StreamStatus::Code::OK:
      return "OK";
    case StreamStatus::Code::SLOWED:
      return "SLOWED";
    case StreamStatus::Code::HALTED:
      return "HALTED";
    default:
      return "INVALID";
  }
}

Eigen::Vector3d tcp_position(const tf2_ros::Buffer & tf)
{
  const auto t = tf.lookupTransform(BASE_FRAME, TCP_FRAME, tf2::TimePointZero).transform;
  return {t.translation.x, t.translation.y, t.translation.z};
}

// Sleep on the node clock (sim time in MuJoCo).
void sleep_until(const rclcpp::Node & node, const rclcpp::Time & time)
{
  while (rclcpp::ok() && node.now() < time)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

int run(const rclcpp::Node::SharedPtr & node)
{
  const auto axis = node->declare_parameter("axis", std::string("x"));
  const double distance = node->declare_parameter("distance", 0.05);
  const double speed = node->declare_parameter("speed", 0.05);
  const auto i = std::string("xyz").find(axis);
  if (axis.size() != 1 || i == std::string::npos || speed <= 0.0 || distance == 0.0)
  {
    throw std::invalid_argument("need axis x|y|z, speed > 0 and distance != 0");
  }

  tf2_ros::Buffer tf(node->get_clock());
  tf2_ros::TransformListener listener(tf);
  fer_moveit::ServoStreamer streamer(node);
  std::string error;
  if (!tf.canTransform(
        BASE_FRAME, TCP_FRAME, tf2::TimePointZero, tf2::durationFromSec(5.0), &error))
  {
    throw std::runtime_error("no TF base -> fer_hand_tcp: " + error);
  }
  const Eigen::Vector3d start = tcp_position(tf);
  streamer.start();

  fer_moveit::CartesianStreamer::Twist twist = fer_moveit::CartesianStreamer::Twist::Zero();
  twist(static_cast<Eigen::Index>(i)) = std::copysign(speed, distance);
  const auto period = rclcpp::Duration::from_seconds(streamer.period());
  auto next = node->now();
  const auto end = next + rclcpp::Duration::from_seconds(std::abs(distance) / speed);
  RCLCPP_INFO(
    node->get_logger(), "moving %.3f m along %s at %.3f m/s", distance, axis.c_str(), speed);

  std::map<StreamStatus::Code, int> cycles;
  auto last = StreamStatus::Code::OK;
  while (rclcpp::ok() && node->now() < end)
  {
    const auto status = streamer.step(twist);
    ++cycles[status.code];
    if (status.code != last)
    {
      RCLCPP_INFO(
        node->get_logger(), "stream %s: %s", name_of(status.code), status.message.c_str());
      last = status.code;
    }
    if (status.code == StreamStatus::Code::INVALID)
    {
      throw std::runtime_error("Servo computed no joint command: " + status.message);
    }
    next += period;
    sleep_until(*node, next);
  }

  int halt_cycles = 0;
  while (rclcpp::ok() && !streamer.halt() && ++halt_cycles < MAX_HALT_CYCLES)
  {
    next += period;
    sleep_until(*node, next);
  }
  sleep_until(*node, node->now() + rclcpp::Duration::from_seconds(1.0));  // let the arm settle

  const Eigen::Vector3d moved = tcp_position(tf) - start;
  Eigen::Vector3d drift = moved;
  drift(static_cast<Eigen::Index>(i)) = 0.0;
  const double reached = moved(static_cast<Eigen::Index>(i)) / distance;
  const bool pass = reached > 0.8 && reached < 1.2 && drift.norm() < 0.01;
  RCLCPP_INFO(
    node->get_logger(),
    "moved x %+.4f y %+.4f z %+.4f m: %.0f %% of the distance, %.1f mm off-axis; "
    "cycles OK %d SLOWED %d HALTED %d, halted after %d cycles: %s",
    moved.x(), moved.y(), moved.z(), 100.0 * reached, 1000.0 * drift.norm(),
    cycles[StreamStatus::Code::OK], cycles[StreamStatus::Code::SLOWED],
    cycles[StreamStatus::Code::HALTED], halt_cycles, pass ? "PASS" : "FAIL");
  return pass ? 0 : 1;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("fer_moveit_streamer_check");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor] { executor.spin(); });

  int code = 1;
  try
  {
    code = run(node);
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
  }
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return code;
}
