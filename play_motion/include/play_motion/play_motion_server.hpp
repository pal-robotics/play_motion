// Copyright 2021 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/** \author Adolfo Rodriguez Tsouroukdissian. */
/** \author Paul Mathieu.                     */

#ifndef PLAYMOTIONSERVER_H
#define PLAYMOTIONSERVER_H

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "play_motion/play_motion.hpp"
#include "play_motion_msgs/action/play_motion.hpp"
#include "play_motion_msgs/srv/list_motions.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/server.hpp"

namespace play_motion
{
class PlayMotionServer
{
private:
  using PlayMotionAction = play_motion_msgs::action::PlayMotion;
  using GoalHandlePlayMotionAction = rclcpp_action::ServerGoalHandle<PlayMotionAction>;

  using PlayMotionPtr = std::shared_ptr<PlayMotion>;

public:
  PlayMotionServer(const PlayMotionPtr & pm);
  virtual ~PlayMotionServer();

private:
  void playMotionCb(const PlayMotion::GoalHandle & goal_hdl);

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PlayMotionAction::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandlePlayMotionAction> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandlePlayMotionAction> goal_handle);

  bool findGoalId(const GoalHandlePlayMotionAction & gh, PlayMotion::GoalHandle & goal_id);

  bool listMotions(
    const play_motion_msgs::srv::ListMotions::Request::SharedPtr req,
    play_motion_msgs::srv::ListMotions::Response::SharedPtr resp);

  void publishDiagnostics() const;

  std::vector<std::string> clist_;
  PlayMotionPtr pm_;
  rclcpp::Logger logger_;
  rclcpp_action::Server<PlayMotionAction>::SharedPtr al_server_;
  std::map<PlayMotion::GoalHandle, std::shared_ptr<GoalHandlePlayMotionAction>> al_goals_;
  rclcpp::Service<play_motion_msgs::srv::ListMotions>::SharedPtr list_motions_srv_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};
}

#endif
