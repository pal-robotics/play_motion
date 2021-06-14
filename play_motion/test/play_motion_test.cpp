///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Paul Mathieu

#include <chrono>
#include <functional>
#include <memory>
#include <thread>

#include "gtest/gtest.h"

#include "play_motion_msgs/action/play_motion.hpp"

#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_action/create_client.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

using PlayMotionResult = play_motion_msgs::action::PlayMotion::Result;

class PlayMotionTestClient : public rclcpp::Node
{
  using PlayMotionAction = play_motion_msgs::action::PlayMotion;
  using ActionClientPtr = rclcpp_action::Client<PlayMotionAction>::SharedPtr;
  using ActionGoal = play_motion_msgs::action::PlayMotion_Goal;
  using ActionGoalResult = rclcpp_action::ResultCode;
  using IsReadyService = std_srvs::srv::Trigger;

public:
  PlayMotionTestClient()
  : rclcpp::Node("play_motion_test_client")
  {
    ac_ = rclcpp_action::create_client<PlayMotionAction>(this, "play_motion");

    js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&PlayMotionTestClient::jsCb, this, _1));

    is_ready_client_ = create_client<IsReadyService>("/play_motion/is_ready");
  }

  bool is_ready()
  {
    if (!is_ready_client_->wait_for_service()) {
      RCLCPP_ERROR_STREAM(get_logger(), "play_motion 'is_ready' service not available");
      return false;
    }

    auto request = std::make_shared<IsReadyService::Request>();
    for (auto i = 0u; i < 5; ++i) {
      auto result = is_ready_client_->async_send_request(request);
      if (result.wait_for(1s) == std::future_status::ready) {
        if (result.get()->success) {
          RCLCPP_INFO_STREAM(get_logger(), "play_motion is ready");
          return true;
        }
      }
      std::this_thread::sleep_for(1s);
    }

    RCLCPP_ERROR_STREAM(get_logger(), "play_motion not ready after 5 tries");
    return false;
  }

  bool playMotion(const std::string & motion_name, bool skip_planning)
  {
    if (!ac_->wait_for_action_server(10s)) {
      RCLCPP_ERROR_STREAM(get_logger(), "play_motion server not available");
      return false;
    }

    if (!is_ready()) {
      return false;
    }

    bool is_goal_completed = false;

    // set up goal
    ActionGoal goal;
    goal.motion_name = motion_name;
    goal.skip_planning = skip_planning;

    // set up goal options
    auto goal_options = rclcpp_action::Client<PlayMotionAction>::SendGoalOptions();
    goal_options.goal_response_callback = [&](auto future)
      {
        auto goal_handle = future.get();
        if (!goal_handle) {
          goal_accepted_ = false;
          RCLCPP_ERROR_STREAM(get_logger(), "Goal was rejected by server");
        } else {
          goal_accepted_ = true;
          RCLCPP_INFO_STREAM(get_logger(), "Goal accepted by server, waiting for result");
        }
      };
    goal_options.result_callback = [&](const auto & result)
      {
        is_goal_completed = true;
        goal_result_ = result.code;
        play_motion_result_ = result.result->error_code;
      };

    // send goal and wait for it to complete
    RCLCPP_INFO_STREAM(get_logger(), "Sending goal");
    auto goal_future = ac_->async_send_goal(goal, goal_options);

    RCLCPP_INFO_STREAM(get_logger(), "Wait for goal to complete");
    /// @todo add timeout here?
    while (!is_goal_completed) {
      std::this_thread::sleep_for(500ms);
    }
    RCLCPP_INFO_STREAM(get_logger(), "Goal completed");
    return true;
  }

  double getJointPos(const std::string & joint_name)
  {
    unsigned int i;
    for (i = 0; i < js_.name.size(); ++i) {
      if (js_.name[i] == joint_name) {
        return js_.position[i];
      }
    }

    return std::numeric_limits<double>::quiet_NaN();
  }

  void shouldFinishWith(ActionGoalResult goal_status, int goal_result)
  {
    EXPECT_TRUE(goal_accepted_);
    EXPECT_EQ(goal_status, goal_result_);
    EXPECT_EQ(goal_result, play_motion_result_);
  }

  void shouldFailWithCode(int goal_result)
  {
    shouldFinishWith(ActionGoalResult::ABORTED, goal_result);
  }

  void shouldSucceed()
  {
    shouldFinishWith(
      ActionGoalResult::SUCCEEDED,
      play_motion_msgs::action::PlayMotion_Result::SUCCEEDED);
  }

  void jsCb(const sensor_msgs::msg::JointState::SharedPtr js)
  {
    js_ = *js;
  }

private:
  bool goal_accepted_;
  ActionGoalResult goal_result_;
  int play_motion_result_;

  ActionClientPtr ac_;
  sensor_msgs::msg::JointState js_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Client<IsReadyService>::SharedPtr is_ready_client_;
};

TEST(PlayMotionTest, basicReachPose)
{
  auto pmtc = std::make_shared<PlayMotionTestClient>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pmtc);
  auto runner = std::thread([&]() {executor.spin();});

  auto result = pmtc->playMotion("pose1", true);
  EXPECT_TRUE(result);
  if (result) {
    pmtc->shouldSucceed();
    double final_pos = pmtc->getJointPos("joint1");
    EXPECT_NEAR(final_pos, 1.8, 0.01);
  }

  executor.cancel();
  runner.join();
}

TEST(PlayMotionTest, rejectSecondGoal)
{
  auto pmtc1 = std::make_shared<PlayMotionTestClient>();
  auto pmtc2 = std::make_shared<PlayMotionTestClient>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pmtc1);
  executor.add_node(pmtc2);
  auto runner = std::thread([&]() {executor.spin();});

  bool result1;
  std::thread play_thread([&]() {result1 = pmtc1->playMotion("home", true);});

  std::this_thread::sleep_for(300ms);

  auto result2 = pmtc2->playMotion("pose1", true);
  EXPECT_TRUE(result2);
  if (result2) {
    pmtc2->shouldFailWithCode(PlayMotionResult::CONTROLLER_BUSY);
  }

  play_thread.join();
  EXPECT_TRUE(result1);
  if (result1) {
    pmtc1->shouldSucceed();
  }

  executor.cancel();
  runner.join();
}

TEST(PlayMotionTest, badMotionName)
{
  auto pmtc = std::make_shared<PlayMotionTestClient>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pmtc);
  auto runner = std::thread([&]() {executor.spin();});

  auto result = pmtc->playMotion("inexistant_motion", true);
  EXPECT_TRUE(result);
  if (result) {
    pmtc->shouldFailWithCode(PlayMotionResult::MOTION_NOT_FOUND);
  }

  result = pmtc->playMotion("", true);
  EXPECT_TRUE(result);
  if (result) {
    pmtc->shouldFailWithCode(PlayMotionResult::MOTION_NOT_FOUND);
  }

  executor.cancel();
  runner.join();
}

TEST(PlayMotionTest, malformedPose)
{
  auto pmtc = std::make_shared<PlayMotionTestClient>();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(pmtc);
  auto runner = std::thread([&]() {executor.spin();});

  auto result = pmtc->playMotion("malformed_pose", true);
  EXPECT_TRUE(result);
  if (result) {
    /// @todo should be INVALID_MOTION
    pmtc->shouldFailWithCode(PlayMotionResult::MOTION_NOT_FOUND);
  }

  executor.cancel();
  runner.join();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int ret = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return ret;
}
