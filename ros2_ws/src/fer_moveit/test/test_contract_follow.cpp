// Contract of /motion/follow: the real MotionServer with a FakeStreamer (no Servo, no arm).
// The TCP sits still at (0.3, 0, 0.5) in base; the static target "goal" is 10 cm ahead in x,
// "high" is above the workspace (default box z <= 0.50). follow_halt_timeout is 0.3 s; the
// stall watchdog ends a goal that commands a motion after 1.3 s (contract_fixture.hpp).

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include "contract_fixture.hpp"
#include "fer_interfaces/action/follow.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace fer_moveit
{

using fer_interfaces::action::Follow;

class FollowContract : public Contract
{
protected:
  /// \param with_streamer false: the server gets no streamer factory.
  void start_follow(bool with_streamer = true, uint8_t start_error = 0)
  {
    if (with_streamer)
    {
      make_streamer_ = [this, start_error]
      {
        auto streamer = std::make_unique<FakeStreamer>(start_error);
        streamer_ = streamer.get();
        return streamer;
      };
    }
    ASSERT_NO_FATAL_FAILURE(start());
    frames_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(fakes_);
    frames_->sendTransform(
      {frame(fakes_, "fer_hand_tcp", 0.3, 0.0, 0.5),
       frame(fakes_, "goal", 0.4, 0.0, 0.5, goal_yaw_), frame(fakes_, "high", 0.4, 0.0, 0.9)});
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(fakes_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    ASSERT_TRUE(wait_until([this] { return can_see("fer_hand_tcp") && can_see("goal"); }));
    std::this_thread::sleep_for(200ms);  // the server's listener gets the same messages
    follow_client_ = rclcpp_action::create_client<Follow>(fakes_, "/motion/follow");
    ASSERT_TRUE(follow_client_->wait_for_action_server(5s));
  }

  bool can_see(const std::string & child)
  {
    return tf_buffer_->canTransform("base", child, tf2::TimePointZero);
  }

  Follow::Goal follow_goal(const std::string & frame_id = "goal")
  {
    Follow::Goal goal;
    goal.target_frame = frame_id;
    goal.speed_scaling = 0.5;
    return goal;
  }

  /// Send \p goal; the latest feedback distance goes to distance_.
  rclcpp_action::ClientGoalHandle<Follow>::SharedPtr send_follow(const Follow::Goal & goal)
  {
    rclcpp_action::Client<Follow>::SendGoalOptions options;
    options.feedback_callback = [this](
                                  rclcpp_action::ClientGoalHandle<Follow>::SharedPtr,
                                  const std::shared_ptr<const Follow::Feedback> feedback)
    { distance_ = feedback->distance; };
    auto future = follow_client_->async_send_goal(goal, options);
    EXPECT_TRUE(wait(future));
    auto handle = future.get();
    EXPECT_TRUE(handle);
    return handle;
  }

  bool streaming(int steps)
  {
    return wait_until(
      [this, steps]
      {
        const auto * s = streamer_.load();
        return s && s->steps.load() > steps;
      });
  }

  int starts() const
  {
    const auto * s = streamer_.load();
    return s ? s->starts.load() : 0;
  }
  int halts() const
  {
    const auto * s = streamer_.load();
    return s ? s->halts.load() : 0;
  }

  double goal_yaw_{0.0};
  std::atomic<FakeStreamer *> streamer_{nullptr};  // owned by the server
  std::atomic<float> distance_{-1.0f};
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> frames_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp_action::Client<Follow>::SharedPtr follow_client_;
};

TEST_F(FollowContract, InvalidGoalsAreRejectedBeforeTheArmMoves)
{
  start_follow();
  auto bad_speed = follow_goal();
  bad_speed.speed_scaling = 1.5;
  auto no_frame = follow_goal("");
  auto no_timeout = follow_goal();
  no_timeout.target_timeout = 0.0;
  for (const auto & goal : {bad_speed, no_frame, no_timeout})
  {
    const auto result = run<Follow>(follow_client_, goal);
    EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
    EXPECT_EQ(result.result->outcome.code, Outcome::INVALID_GOAL);
  }
  auto orientation = follow_goal();
  orientation.follow_orientation = true;
  EXPECT_EQ(run<Follow>(follow_client_, orientation).result->outcome.code, Outcome::UNSUPPORTED);
  EXPECT_EQ(starts(), 0);
}

TEST_F(FollowContract, WithoutStreamerFollowIsUnsupported)
{
  start_follow(false);
  EXPECT_EQ(run<Follow>(follow_client_, follow_goal()).result->outcome.code, Outcome::UNSUPPORTED);
}

TEST_F(FollowContract, UnknownTargetIsNotFoundWithoutMoving)
{
  start_follow();
  const auto result = run<Follow>(follow_client_, follow_goal("nowhere"));
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.result->outcome.code, Outcome::NOT_FOUND);
  EXPECT_EQ(starts(), 0);
  EXPECT_EQ(halts(), 0);
}

TEST_F(FollowContract, StreamThatCannotStartEndsWithItsErrorWithoutBraking)
{
  start_follow(true, Outcome::TIMEOUT);
  EXPECT_EQ(run<Follow>(follow_client_, follow_goal()).result->outcome.code, Outcome::TIMEOUT);
  EXPECT_EQ(halts(), 0);  // never commanded, so nothing to stop
}

TEST_F(FollowContract, StreamsTowardsTheTargetUntilCancelled)
{
  start_follow();
  auto handle = send_follow(follow_goal());
  auto result = follow_client_->async_get_result(handle);
  ASSERT_TRUE(streaming(20));

  // 10 cm behind in x, gain 2/s: 0.2 m/s, capped at 0.25 m/s * speed_scaling 0.5.
  const auto twist = streamer_.load()->last_twist();
  EXPECT_NEAR(twist(0), 0.125, 1e-6);
  EXPECT_NEAR(twist(1), 0.0, 1e-6);
  EXPECT_NEAR(twist(2), 0.0, 1e-6);
  EXPECT_NEAR(twist.tail<3>().norm(), 0.0, 1e-6);  // orientation held
  ASSERT_TRUE(wait_until([this] { return distance_.load() >= 0.0f; }));
  EXPECT_NEAR(distance_.load(), 0.1, 1e-4);

  follow_client_->async_cancel_goal(handle);
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(result.get().result->outcome.code, Outcome::CANCELLED);
  EXPECT_EQ(starts(), 1);
  EXPECT_GE(halts(), 1);
}

TEST_F(FollowContract, OffsetIsInTheTargetFrameAndTheOrientationIsHeld)
{
  goal_yaw_ = M_PI / 2.0;  // target turned 90 deg about z: its x axis is base y
  start_follow();
  auto goal = follow_goal();
  goal.speed_scaling = 1.0;
  goal.offset.position.x = -0.1;  // 10 cm behind the target, in the target's frame
  auto handle = send_follow(goal);
  auto result = follow_client_->async_get_result(handle);
  ASSERT_TRUE(streaming(20));

  // Goal (0.4, -0.1, 0.5), TCP (0.3, 0, 0.5): error (0.1, -0.1), gain 2 -> 0.28 m/s, capped 0.25.
  const auto twist = streamer_.load()->last_twist();
  EXPECT_NEAR(twist(0), 0.25 / std::sqrt(2.0), 1e-6);
  EXPECT_NEAR(twist(1), -0.25 / std::sqrt(2.0), 1e-6);
  EXPECT_NEAR(twist.tail<3>().norm(), 0.0, 1e-6);  // the target is turned, the TCP is not

  follow_client_->async_cancel_goal(handle);
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().result->outcome.code, Outcome::CANCELLED);
}

TEST_F(FollowContract, TargetThatStopsUpdatingIsNotFoundAndTheArmStops)
{
  start_follow();
  MovingFrame hand(fakes_, frame(fakes_, "hand", 0.4, 0.0, 0.5));
  ASSERT_TRUE(wait_until([this] { return can_see("hand"); }));
  auto goal = follow_goal("hand");
  goal.target_timeout = 0.3;
  auto handle = send_follow(goal);
  auto result = follow_client_->async_get_result(handle);
  ASSERT_TRUE(streaming(10));

  hand.freeze();
  const auto frozen_at = std::chrono::steady_clock::now();
  ASSERT_TRUE(wait(result));
  EXPECT_GE(std::chrono::steady_clock::now() - frozen_at, 250ms);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.get().result->outcome.code, Outcome::NOT_FOUND);
  EXPECT_NE(result.get().result->outcome.message.find("not updated"), std::string::npos);
  EXPECT_GE(halts(), 1);
}

TEST_F(FollowContract, InvalidStreamIsExecutionFailedAndTheArmStops)
{
  start_follow();
  auto handle = send_follow(follow_goal());
  auto result = follow_client_->async_get_result(handle);
  ASSERT_TRUE(streaming(5));
  streamer_.load()->step_code = StreamStatus::Code::INVALID;
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().result->outcome.code, Outcome::EXECUTION_FAILED);
  EXPECT_GE(halts(), 1);
}

TEST_F(FollowContract, GoalOutsideTheWorkspaceFromTheStartHoldsTheArm)
{
  start_follow();
  auto handle = send_follow(follow_goal("high"));
  auto result = follow_client_->async_get_result(handle);
  ASSERT_TRUE(streaming(20));

  // (0.4, 0, 0.9) is above the box (z <= 0.50): the TCP holds where it started.
  EXPECT_NEAR(streamer_.load()->last_twist().norm(), 0.0, 1e-6);
  // The feedback is the distance to the real goal, so it shows the goal is out of reach.
  ASSERT_TRUE(wait_until([this] { return distance_.load() >= 0.0f; }));
  EXPECT_NEAR(distance_.load(), std::hypot(0.1, 0.4), 1e-4);

  follow_client_->async_cancel_goal(handle);
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().result->outcome.code, Outcome::CANCELLED);
}

TEST_F(FollowContract, GoalThatLeavesTheWorkspaceIsHeldWhereItLeft)
{
  start_follow();
  auto goal = follow_goal();
  goal.speed_scaling = 1.0;
  auto handle = send_follow(goal);
  auto result = follow_client_->async_get_result(handle);
  ASSERT_TRUE(streaming(20));
  EXPECT_NEAR(streamer_.load()->last_twist()(0), 0.2, 1e-6);  // 2/s * 0.1 m to (0.4, 0, 0.5)

  // The target jumps to (0.5, 0, 0.9), above the box. Sliding along the wall would aim at
  // (0.5, 0, 0.5) (0.25 m/s after the cap); holding keeps aiming at (0.4, 0, 0.5).
  frames_->sendTransform(frame(fakes_, "goal", 0.5, 0.0, 0.9));
  ASSERT_TRUE(wait_until([this] { return distance_.load() > 0.4f; }));
  ASSERT_TRUE(streaming(streamer_.load()->steps.load() + 5));
  const auto twist = streamer_.load()->last_twist();
  EXPECT_NEAR(twist(0), 0.2, 1e-6);
  EXPECT_NEAR(twist(2), 0.0, 1e-6);

  follow_client_->async_cancel_goal(handle);
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().result->outcome.code, Outcome::CANCELLED);
}

TEST_F(FollowContract, StreamHaltedTooLongIsUnreachableAndTheArmStops)
{
  start_follow();
  auto handle = send_follow(follow_goal());
  auto result = follow_client_->async_get_result(handle);
  ASSERT_TRUE(streaming(5));
  streamer_.load()->step_code = StreamStatus::Code::HALTED;
  const auto halted_at = std::chrono::steady_clock::now();
  ASSERT_TRUE(wait(result));
  EXPECT_GE(std::chrono::steady_clock::now() - halted_at, 250ms);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.get().result->outcome.code, Outcome::UNREACHABLE);
  EXPECT_NE(result.get().result->outcome.message.find("halted"), std::string::npos);
  EXPECT_GE(halts(), 1);
}

TEST_F(FollowContract, ShortHaltIsRiddenOut)
{
  start_follow();
  auto handle = send_follow(follow_goal());
  auto result = follow_client_->async_get_result(handle);
  ASSERT_TRUE(streaming(5));
  streamer_.load()->step_code = StreamStatus::Code::HALTED;
  std::this_thread::sleep_for(100ms);
  streamer_.load()->step_code = StreamStatus::Code::OK;
  std::this_thread::sleep_for(500ms);                           // longer than the halt timeout
  EXPECT_EQ(result.wait_for(0s), std::future_status::timeout);  // still following

  follow_client_->async_cancel_goal(handle);
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().result->outcome.code, Outcome::CANCELLED);
}

TEST_F(FollowContract, StreamThatCrawlsWithoutHaltingIsUnreachableAndTheArmStops)
{
  // Servo near a singularity: SLOWED in every cycle, never HALTED, so the halt watchdog does
  // not fire. The fake TCP does not move while 0.125 m/s is commanded: the stall watchdog
  // ends the goal after window + timeout (1.3 s here).
  start_follow();
  auto handle = send_follow(follow_goal());
  auto result = follow_client_->async_get_result(handle);
  ASSERT_TRUE(streaming(5));
  streamer_.load()->step_code = StreamStatus::Code::SLOWED;
  const auto slowed_at = std::chrono::steady_clock::now();
  ASSERT_TRUE(wait(result));
  EXPECT_GE(std::chrono::steady_clock::now() - slowed_at, 1000ms);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.get().result->outcome.code, Outcome::UNREACHABLE);
  EXPECT_NE(result.get().result->outcome.message.find("commanded"), std::string::npos);
  EXPECT_GE(halts(), 1);
}

TEST_F(FollowContract, TcpOnTheGoalIsNotAStall)
{
  // The goal is the TCP itself: nothing to command, so a TCP at rest is fine however long.
  start_follow();
  auto handle = send_follow(follow_goal("fer_hand_tcp"));
  auto result = follow_client_->async_get_result(handle);
  ASSERT_TRUE(streaming(5));
  std::this_thread::sleep_for(2000ms);                          // longer than window + timeout
  EXPECT_EQ(result.wait_for(0s), std::future_status::timeout);  // still following

  follow_client_->async_cancel_goal(handle);
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().result->outcome.code, Outcome::CANCELLED);
}

TEST_F(FollowContract, NewGoalReplacesFollow)
{
  start_follow();
  move_group_fake_->trajectory_duration = 0.3;
  auto handle = send_follow(follow_goal());
  auto result = follow_client_->async_get_result(handle);
  ASSERT_TRUE(streaming(5));

  auto joints = send<MoveToJoints>(joints_client_, joints_goal());
  auto joints_result = joints_client_->async_get_result(joints);
  ASSERT_TRUE(wait(result));
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.get().result->outcome.code, Outcome::CANCELLED);
  EXPECT_EQ(result.get().result->outcome.message, "replaced by a newer goal");
  EXPECT_GE(halts(), 1);
  ASSERT_TRUE(wait(joints_result));
  EXPECT_EQ(joints_result.get().result->outcome.code, Outcome::OK);
}

}  // namespace fer_moveit
