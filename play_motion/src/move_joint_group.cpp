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
#include <play_motion_msgs/PlayMotionResult.h>

#include <ros/ros.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

namespace play_motion
{
  MoveJointGroup::MoveJointGroup(const std::string& controller_name, const JointNames& joint_names)
    : busy_(false),
      controller_name_(controller_name),
      joint_names_(joint_names),
      client_(controller_name_ + "/follow_joint_trajectory", false)
  { }

  void MoveJointGroup::alCallback()
  {
    busy_ = false;
    ActionResultPtr r = client_.getResult();
    active_cb_(r->error_code);
    active_cb_.clear();
  }

  bool MoveJointGroup::isIdle() const
  {
    return !busy_;
  }

  void MoveJointGroup::cancel()
  {
    busy_ = false;
    client_.cancelAllGoals();
  }
  
  void MoveJointGroup::abort()
  {
    if (busy_)
    {
      client_.cancelAllGoals();
      client_.stopTrackingGoal(); 
    }
    if (active_cb_)
      active_cb_(play_motion_msgs::PlayMotionResult::OTHER_ERROR);
  }

  void MoveJointGroup::setCallback(const Callback& cb)
  {
    active_cb_ = cb;
  }

  const JointNames& MoveJointGroup::getJointNames() const
  {
    return joint_names_;
  }

  actionlib::SimpleClientGoalState MoveJointGroup::getState()
  {
    return client_.getState();
  }

  const std::string& MoveJointGroup::getName() const
  {
    return controller_name_;
  }

  bool MoveJointGroup::isControllingJoint(const std::string& joint_name)
  {
    if (!client_.isServerConnected())
      return false;

    foreach (const std::string& jn, joint_names_)
      if (joint_name == jn)
        return true;

    return false;
  }

  bool MoveJointGroup::sendGoal(const std::vector<TrajPoint>& traj)
  {
    ROS_DEBUG_STREAM("Sending trajectory goal to " << controller_name_ << ".");

    ActionGoal goal;
    goal.trajectory.joint_names = joint_names_;
    goal.trajectory.points.reserve(traj.size());

    foreach (const TrajPoint& p, traj)
    {
      if (p.positions.size() != joint_names_.size())
      {
        ROS_ERROR_STREAM("Pose size mismatch. Expected: " << joint_names_.size()
                         << ", got: " << p.positions.size() << ".");
        return false;
      }
      trajectory_msgs::JointTrajectoryPoint point;

      point.positions     = p.positions;         // Reach these joint positions...
      point.velocities    = p.velocities;        // ...with a given velocity (may be unset)
      point.accelerations = p.accelerations;     // ...and acceleration      (may be unset)

      point.time_from_start = p.time_from_start; // ...in this time

      goal.trajectory.points.push_back(point);
    }
    client_.sendGoal(goal, boost::bind(&MoveJointGroup::alCallback, this));
    busy_ = true;

    return true;
  }
}
