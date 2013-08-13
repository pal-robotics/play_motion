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

#include "play_motion/reach_pose.h"

#include <cassert>
#include <iostream>
#include <sstream>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

#include "play_motion/move_joint_group.h"

#define foreach BOOST_FOREACH

namespace play_motion
{
  ReachPose::ReachPose(ros::NodeHandle& nh) :
    nh_(nh), joint_states_sub_(nh_.subscribe("joint_states", 10, &ReachPose::jointStateCb, this))
  {}

  void ReachPose::setControllerList(const ControllerList& controller_list)
  {
    move_joint_groups_.clear();
    foreach (const std::string& controller_name, controller_list)
      move_joint_groups_.push_back(MoveJointGroupPtr(new MoveJointGroup(controller_name)));
  }

  void ReachPose::controllerCb(bool success)
  {
    ROS_DEBUG("return from joint group, %d active controllers", current_active_controllers_-1);
    current_success_ &= success;
    if (--current_active_controllers_ < 1)
      client_cb_(current_success_);
  };

  void ReachPose::jointStateCb(const sensor_msgs::JointStatePtr& msg)
  {
    joint_states_.clear();
    for (uint32_t i=0; i < msg->name.size(); ++i)
      joint_states_.push_back(JointState(msg->name[i], msg->position[i]));
  }

  bool ReachPose::run(const std::string& pose, const ros::Duration& duration)
  {
    typedef std::vector<std::string> JointNames;
    typedef std::vector<double> JointPositions;

    std::vector<JointPositions> joint_group_pose; // Will contain desired pose split into joint groups

    // Verify that the pose specification exists
    if (!nh_.hasParam("poses/" + pose))
    {
      ROS_ERROR_STREAM("Pose \'" << pose << "\' does not exist.");
      return false;
    }

    if (move_joint_groups_.size() < 1)
      return false;

    // Seed target pose with current joint state
    foreach (MoveJointGroupPtr move_joint_group, move_joint_groups_)
    {
      JointNames     group_joint_names = move_joint_group->getJointNames();
      JointPositions group_joint_positions(group_joint_names.size());

      for (std::size_t i = 0; i < group_joint_names.size(); ++i)
      {
        // Fetch joint position value from pose specification
        ROS_DEBUG_STREAM("fetching param: " << pose + group_joint_names[i]);
        if (!nh_.getParamCached("poses/" + pose + "/" + group_joint_names[i], group_joint_positions[i]))
        {
          //...and if unspecified, use current joint position
          bool found = false;
          foreach (JointState& js, joint_states_)
          {
            if (group_joint_names[i] == js.first)
            {
              group_joint_positions[i] = js.second;
              found = true;
              break;
            }
          }
          if (!found)
          {
            ROS_ERROR_STREAM("Could not get current position of joint \'" << group_joint_names[i] << "\'.");
            return false;
          }
        }
      }

      // Set pose goal for current joint group
      joint_group_pose.push_back(group_joint_positions);
    }

    // Send pose commands
    for (std::size_t i = 0; i < move_joint_groups_.size(); ++i)
    {
      if (joint_group_pose[i].size() < 1)
        continue;

      if (!move_joint_groups_[i]->sendGoal(joint_group_pose[i], duration,
            boost::bind(&ReachPose::controllerCb, this, _1)))
        return false;
    }
    current_active_controllers_ = move_joint_groups_.size();
    current_success_= true;

    return true;
  }
}
