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

/** \author Adolfo Rodriguez Tsouroukdissian. */
/** \author Paul Mathieu.                     */

#ifndef PLAYMOTIONSERVER_H
#define PLAYMOTIONSERVER_H

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include "play_motion/play_motion.h"
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
    PlayMotionServer(const PlayMotionPtr& pm);
    virtual ~PlayMotionServer();

  private:
    void playMotionCb(const PlayMotion::GoalHandle& goal_hdl);

    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const PlayMotionAction::Goal> goal);
    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandlePlayMotionAction> goal_handle);
    void handleAccepted(const std::shared_ptr<GoalHandlePlayMotionAction> goal_handle);

    bool findGoalId(const GoalHandlePlayMotionAction & gh, PlayMotion::GoalHandle& goal_id);

    bool listMotions(const play_motion_msgs::srv::ListMotions::Request::SharedPtr req,
                     play_motion_msgs::srv::ListMotions::Response::SharedPtr resp);

    void publishDiagnostics() const;

    std::vector<std::string>                               clist_;
    PlayMotionPtr                                          pm_;
    rclcpp::Logger logger_;
    rclcpp_action::Server<PlayMotionAction>::SharedPtr al_server_;
    std::map<PlayMotion::GoalHandle, std::shared_ptr<GoalHandlePlayMotionAction>> al_goals_;
    rclcpp::Service<play_motion_msgs::srv::ListMotions>::SharedPtr list_motions_srv_;

    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostic_pub_;
    rclcpp::TimerBase::SharedPtr diagnostic_timer_;
  };
}

#endif
