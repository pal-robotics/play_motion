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

#ifndef MOVEJOINTGROUP_H
#define MOVEJOINTGROUP_H

#include <functional>
#include <string>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"

#include "play_motion/datatypes.hpp"

#include "rclcpp_action/client.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/timer.hpp"

namespace play_motion
{
  /** Move a joint group to a given pose.
   * The pose will be reached within a specified duration with zero velocity.
   */
  class MoveJointGroup
  {
  private:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using ActionClient = rclcpp_action::Client<FollowJointTrajectory>;
    using ActionClientPtr = ActionClient::SharedPtr;
    using ActionGoal = control_msgs::action::FollowJointTrajectory::Goal;
    using ActionResult = control_msgs::action::FollowJointTrajectory::Result;
    using ActionResultPtr = ActionResult::SharedPtr;

    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    using Callback = std::function<void(int)>;

  public:
    MoveJointGroup(const rclcpp::Node::SharedPtr & node, const std::string& controller_name, const JointNames& joint_names);

    /**
     * \brief Send a trajectory goal to the associated controller.
     * \param traj The trajectory to send
     */
    bool sendGoal(const std::vector<TrajPoint>& traj);

    /**
     * \brief Returns true if the specified joint is controlled by the controller.
     * \pram joint_name Joint name
     */
    bool isControllingJoint(const std::string& joint_name);

    /**
     * \brief Returns true if the MoveJointGroup is ready to accept a new goal.
     * \note This is independent from the state of the controller, but if the
     *       controller cannot be reached, isIdle() will always return false.
     */
    bool isIdle() const;

    /**
     * \brief Cancel the current goal
     */
    void cancel();
    
    /**
     * \brief Cancel the current goal, stop tracking it and call the callback
     *        to notify an error
     */
    void abort();

    /**
     * \brief Register the callback to be called when the action is finished.
     * \param cb The callback
     */
    void setCallback(const Callback& cb);

    /**
     * \brief Returns the list of associated joints
     */
    const JointNames& getJointNames() const;

    /**
     * \brief Returns the action client state
     */
    int8_t getState();


    /**
     * \brief Returns the name that was used when creating the MoveJointGroup
     */
    const std::string& getName() const;

  private:
    void resultCallback(const GoalHandleFollowJointTrajectory::WrappedResult & result);

    bool            busy_;
    rclcpp::Node::SharedPtr node_;               ///< Default node handle.
    rclcpp::Logger logger_;
    std::string     controller_name_;  ///< Controller name. XXX: is this needed?
    JointNames      joint_names_;      ///< Names of controller joints.
    ActionClientPtr    client_;           ///< Action client used to trigger motions.
    Callback        active_cb_;        ///< Call this when we are called back from the controller
    rclcpp::TimerBase::SharedPtr      configure_timer_;  ///< To periodically check for controller actionlib server

    std::shared_future<GoalHandleFollowJointTrajectory::SharedPtr> goal_future_;

    rclcpp_action::ResultCode last_result_code_;
  };

}

#endif
