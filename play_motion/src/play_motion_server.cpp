/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2013, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PAL Robotics, S.L. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Paul Mathieu. */

#include "play_motion/play_motion_server.hpp"

#include <chrono>
#include <functional>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_updater/diagnostic_status_wrapper.hpp"

#include "play_motion/play_motion.hpp"
#include "play_motion/play_motion_helpers.hpp"

#include "rclcpp_action/create_server.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

namespace play_motion
{
using namespace std::chrono_literals;
using namespace std::placeholders;

PlayMotionServer::PlayMotionServer(const PlayMotionPtr & pm)
: pm_(pm),
  logger_(pm_->get_logger().get_child("play_motion_server"))
{
  al_server_ = rclcpp_action::create_server<PlayMotionAction>(
    pm_,
    "play_motion",
    std::bind(&PlayMotionServer::handleGoal, this, _1, _2),
    std::bind(&PlayMotionServer::handleCancel, this, _1),
    std::bind(&PlayMotionServer::handleAccepted, this, _1));

  list_motions_srv_ =
    pm_->create_service<play_motion_msgs::srv::ListMotions>(
    std::string(pm_->get_name()) + "/list_motions",
    std::bind(&PlayMotionServer::listMotions, this, _1, _2));

  diagnostic_pub_ = pm_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 1);
  diagnostic_timer_ =
    pm_->create_wall_timer(1s, std::bind(&PlayMotionServer::publishDiagnostics, this));
}

PlayMotionServer::~PlayMotionServer()
{}

bool PlayMotionServer::findGoalId(
  const GoalHandlePlayMotionAction & gh,
  PlayMotion::GoalHandle & goal_hdl)
{
  for (const auto & p : al_goals_) {
    if (p.second->get_goal_id() == gh.get_goal_id()) {
      goal_hdl = p.first;
      return true;
    }
  }
  return false;
}

void PlayMotionServer::playMotionCb(const PlayMotion::GoalHandle & goal_hdl)
{
  auto result = std::make_shared<PlayMotionResult>();
  result->error_code = goal_hdl->error_code;
  result->error_string = goal_hdl->error_string;

  if (result->error_code == PlayMotionResult::SUCCEEDED) {
    RCLCPP_INFO_STREAM(logger_, "Motion played successfully.");
    al_goals_[goal_hdl]->succeed(result);
  } else {
    if (result->error_code == 0) {
      RCLCPP_ERROR(
        logger_,
        "Motion ended with INVALID ERROR code %d and description '%s'", result->error_code,
        result->error_string.c_str());
    } else {
      RCLCPP_WARN(
        logger_,
        "Motion ended with an error code %d and description '%s'", result->error_code,
        result->error_string.c_str());
    }
    al_goals_[goal_hdl]->abort(result);
  }
  al_goals_.erase(goal_hdl);
}

rclcpp_action::GoalResponse PlayMotionServer::handleGoal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const PlayMotionAction::Goal> goal)
{
  RCLCPP_INFO_STREAM(
    logger_,
    "Received goal request with motion name '" << goal->motion_name << "'");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlayMotionServer::handleCancel(
  const std::shared_ptr<GoalHandlePlayMotionAction> goal_handle)
{
  RCLCPP_INFO_STREAM(logger_, "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;

  /// @todo move this to the goal handling callback? most likely!
//  PlayMotion::GoalHandle goal_hdl;
//  if (findGoalId(*goal_handle.get(), goal_hdl)) {
//    goal_hdl->cancel();   //should not be needed
//  } else {
//    RCLCPP_ERROR_STREAM(logger_, "Cancel request could not be fulfilled. Goal not running?.");
//  }

//  al_goals_.erase(goal_hdl);

//  PlayMotionResult::SharedPtr result;
//  goal_handle->canceled(result);
}

void PlayMotionServer::handleAccepted(const std::shared_ptr<GoalHandlePlayMotionAction> goal_handle)
{
  /// @warning According to documentation, this function should return quickly. Run this async?

  RCLCPP_INFO_STREAM(logger_, "Executing goal");

  auto goal = goal_handle->get_goal();
  auto result = std::make_shared<PlayMotionResult>();

  PlayMotion::GoalHandle goal_hdl;

  if (!pm_->run(
      goal->motion_name,
      goal->skip_planning,
      goal_hdl,
      std::bind(&PlayMotionServer::playMotionCb, this, _1)))
  {
    result->error_code = goal_hdl->error_code;
    result->error_string = goal_hdl->error_string;
    if (!result->error_string.empty()) {
      RCLCPP_ERROR_STREAM(logger_, result->error_string);
    }
    goal_handle->abort(result);
    RCLCPP_ERROR_STREAM(logger_, "Motion '" << goal->motion_name << "' could not be played.");

    /// @todo this must be done somehow in the handleGoal callback,
    /// rejects are signeld before the goal is accepted
    // gh.setRejected(r);
    return;
  }

  /// @todo this must be done somehow in the handleGoal callback,
  /// acceptance are signeld before the goal is accepted
  // gh.setAccepted();

  // what's the point of this map?
  al_goals_[goal_hdl] = goal_handle;
}

bool PlayMotionServer::listMotions(
  const play_motion_msgs::srv::ListMotions::Request::SharedPtr /*req*/,
  play_motion_msgs::srv::ListMotions::Response::SharedPtr resp)
{
  try {
    MotionNames motions;
    getMotionIds(pm_.get(), motions);
    for (const std::string & motion : motions) {
      play_motion_msgs::msg::MotionInfo info;
      info.name = motion;
      getMotionJoints(pm_.get(), motion, info.joints);
      getMotionDuration(pm_.get(), motion);
      resp->motions.push_back(info);
    }
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, e.what());
    return false;
  }
  return true;
}

void PlayMotionServer::publishDiagnostics() const
{
  diagnostic_msgs::msg::DiagnosticArray array;
  diagnostic_updater::DiagnosticStatusWrapper status;
  status.name = "Functionality: Play Motion";
  for (auto it = al_goals_.cbegin(); it != al_goals_.cend(); ++it) {
    const auto & hdl = it->second;
    status.add("Executing motion", hdl->get_goal()->motion_name);
  }
  if (al_goals_.empty()) {
    status.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Not executing any motion");
  } else {
    status.mergeSummary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Executing motions");
  }

  array.status.push_back(status);
  diagnostic_pub_->publish(array);
}
}
