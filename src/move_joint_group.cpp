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

#include "play_motion/move_joint_group.h"

#include <ros/ros.h>

namespace play_motion
{
  MoveJointGroup::MoveJointGroup(const std::string& controller_name)
    : controller_name_(controller_name),
    client_(controller_name_ + "/follow_joint_trajectory")
  {
      configure();
  }

  void MoveJointGroup::configure()
  {
    if (!client_.isServerConnected())
    {
      configure_timer_ = nh_.createTimer(ros::Duration(1.0), boost::bind(&MoveJointGroup::configure, this), true);
      return;
    }

    // Get list of joints used by the controller
    joint_names_.clear();
    using namespace XmlRpc;
    XmlRpcValue joint_names;
    ros::NodeHandle nh(controller_name_);
    if (!nh.getParam("joints", joint_names))
    {
      ROS_ERROR("No joints given. (namespace: %s)", nh.getNamespace().c_str());
      return;
    }
    if (joint_names.getType() != XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Malformed joint specification.  (namespace: %s)", nh.getNamespace().c_str());
      return;
    }
    for (int i = 0; i < joint_names.size(); ++i)
    {
      XmlRpcValue &name_value = joint_names[i];
      if (name_value.getType() != XmlRpcValue::TypeString)
      {
        ROS_ERROR("Array of joint names should contain all strings.  (namespace: %s)",
            nh.getNamespace().c_str());
        return;
      }
      joint_names_.push_back(static_cast<std::string>(name_value));
    }
    ROS_INFO_STREAM("controller '" << controller_name_ << "' configured");
  }

  bool MoveJointGroup::sendGoal(const std::vector<double>& pose, const ros::Duration& duration, const Callback& cb)
  {
    ROS_DEBUG_STREAM("sending trajectory goal to " << controller_name_);

    if (joint_names_.size() < 1) // empty vector, nothing to send (controller might not even be connected)
      return false;

    // Goal pose for right_arm_torso group
    if (pose.size() != joint_names_.size())
    {
      ROS_ERROR_STREAM("Pose size mismatch. Expected: " << joint_names_.size() << ", got: " << pose.size() << ".");
      return false;
    }
    ActionGoal goal;
    goal.trajectory.joint_names = joint_names_;
    goal.trajectory.points.resize(1);
    goal.trajectory.points[0].positions = pose;                            // Reach these joint positions...
    goal.trajectory.points[0].velocities.resize(joint_names_.size(), 0.0); // ...with zero-velocity
    goal.trajectory.points[0].time_from_start = duration;                  // ...in this time

    active_cb_ = cb;
    client_.sendGoal(goal, boost::bind(&MoveJointGroup::alCallback, this));
    return true;
  }
}
