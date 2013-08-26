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

#include "play_motion/play_motion.h"

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

#include "play_motion/move_joint_group.h"
#include "play_motion/rethrow.h"
#include "play_motion/xmlrpc_helpers.h"

#define foreach BOOST_FOREACH

namespace play_motion
{
  int PlayMotion::goal_next_id = 37; // or whatever.

  PlayMotion::PlayMotion(ros::NodeHandle& nh) :
    nh_(nh), joint_states_sub_(nh_.subscribe("joint_states", 10, &PlayMotion::jointStateCb, this))
  {}

  void PlayMotion::setControllerList(const ControllerList& controller_list)
  {
    move_joint_groups_.clear();
    foreach (const std::string& controller_name, controller_list)
      move_joint_groups_.push_back(MoveJointGroupPtr(new MoveJointGroup(controller_name)));
  }

  void PlayMotion::controllerCb(bool success, int goal_id)
  {
    Goal& goal = goals_[goal_id];
    ROS_DEBUG("return from joint group, %d active controllers", goal.active_controllers - 1);
    goal.success &= success;
    if (--goal.active_controllers < 1)
      goal.cb(goal.success);
  };

  void PlayMotion::jointStateCb(const sensor_msgs::JointStatePtr& msg)
  {
    joint_states_.clear();
    for (uint32_t i=0; i < msg->name.size(); ++i)
      joint_states_[msg->name[i]] = msg->position[i];
  }

  bool PlayMotion::getGroupTraj(MoveJointGroupPtr move_joint_group,
      const std::vector<std::string>& motion_joints,
      const Trajectory& motion_points, Trajectory& traj_group)
  {
    std::vector<std::string>   group_joint_names = move_joint_group->getJointNames();
    std::vector<double>        joint_states;
    std::map<std::string, int> joint_index;

    traj_group.clear();
    traj_group.reserve(motion_points.size());

    foreach (const std::string& jn, group_joint_names)
    {
      // store the index of this joint in the given motion
      int index = std::find(motion_joints.begin(), motion_joints.end(), jn) - motion_joints.begin();
      joint_index[jn] = index;

      // retrieve joint state,  we should have it from the joint_states subscriber
      if (joint_states_.find(jn) == joint_states_.end())
      {
        ROS_ERROR_STREAM("Could not get current position of joint \'" << jn << "\'.");
        return false;
      }
      joint_states.push_back(joint_states_[jn]);
    }

    foreach (const TrajPoint& p, motion_points)
    {
      bool has_velocities = !p.velocities.empty();
      TrajPoint point;
      point.positions.resize(group_joint_names.size());
      if (has_velocities)
        point.velocities.resize(group_joint_names.size(), 0);
      point.time_from_start = p.time_from_start;

      for (std::size_t i = 0; i < group_joint_names.size(); ++i)
      {
        // first assignment, overriden by given motion if joint specified
        point.positions[i] = joint_states[i];

        size_t index = joint_index[group_joint_names[i]];
        if (index < motion_joints.size())
        {
          point.positions[i] = p.positions[index];
          if (has_velocities)
            point.velocities[i] = p.velocities[index];
        }
      }
      traj_group.push_back(point);
    }
    return true;
  }

  bool PlayMotion::getMotionJoints(const std::string& motion_name, std::vector<std::string>& motion_joints)
  {
    using namespace XmlRpc;
    ros::NodeHandle nh("~");
    XmlRpcValue joint_names;

    RETHROW(xh::fetchParam(nh, "motions/" + motion_name + "/joints", XmlRpcValue::TypeArray, joint_names));

    motion_joints.clear();
    motion_joints.resize(joint_names.size());
    for (int i = 0; i < joint_names.size(); ++i)
      RETHROW(xh::getArrayItem(joint_names, i, XmlRpcValue::TypeString, motion_joints[i]));

    return true;
  }

  bool PlayMotion::getMotionPoints(const std::string& motion_name, Trajectory& motion_points)
  {
    using namespace XmlRpc;
    ros::NodeHandle nh("~");
    XmlRpcValue traj_points;

    RETHROW(xh::fetchParam(nh, "motions/" + motion_name + "/points", XmlRpcValue::TypeArray, traj_points));

    motion_points.clear();
    motion_points.reserve(traj_points.size());
    for (int i = 0; i < traj_points.size(); ++i)
    {
      XmlRpcValue &name_value = traj_points[i];
      TrajPoint point;
      RETHROW(xh::getStructMember(name_value, "time_from_start",
            XmlRpcValue::TypeDouble, point.time_from_start));

      XmlRpcValue positions;
      RETHROW(xh::getStructMember(name_value, "positions", XmlRpcValue::TypeArray, positions));
      point.positions.resize(positions.size());
      for (int j = 0; j < positions.size(); ++j)
        RETHROW(xh::getArrayItem(positions, j, XmlRpcValue::TypeDouble, point.positions[j]));
      if (name_value.hasMember("velocities"))
      {
        XmlRpcValue velocities;
        RETHROW(xh::getStructMember(name_value, "velocities", XmlRpcValue::TypeArray, velocities));
        point.velocities.resize(velocities.size());
        for (int j = 0; j < velocities.size(); ++j)
          RETHROW(xh::getArrayItem(velocities, j, XmlRpcValue::TypeDouble, point.velocities[j]));
      }
      motion_points.push_back(point);
    }
    return true;
  }

  bool PlayMotion::checkControllers(const std::vector<std::string>& motion_joints)
  {
    foreach (const std::string& jn, motion_joints)
    {
      foreach (MoveJointGroupPtr ctrlr, move_joint_groups_)
        if (ctrlr->isControllingJoint(jn))
          goto next_joint;

      ROS_ERROR_STREAM("no controller was found for joint '" << jn << "'");
      return false;
next_joint:;
    }
    return true;
  }

  template <class T>
  bool hasNonNullIntersection(const std::vector<T>& v1, const std::vector<T>& v2)
  {
    foreach (const T& e1, v1)
      foreach (const T& e2, v2)
        if (e1 == e2)
          return true;
    return false;
  }

  void PlayMotion::setAlCb(int goal_id, const Callback& cb)
  {
    if (goals_.count(goal_id) > 0)
      goals_[goal_id].cb = cb;
  }

  void PlayMotion::cancel(int goal_id)
  {
    if (goals_.count(goal_id) < 1)
      return;

    foreach (MoveJointGroupPtr mjg, goals_[goal_id].controllers)
      mjg->cancel();

    goals_.erase(goal_id);
  }

  bool PlayMotion::run(const std::string& motion_name, const ros::Duration& duration, int& goal_id)
  {
    std::vector<std::string>                motion_joints;
    Trajectory                              motion_points;
    std::map<MoveJointGroupPtr, Trajectory> joint_group_traj;

    RETHROW(getMotionJoints(motion_name, motion_joints));
    RETHROW(checkControllers(motion_joints));
    RETHROW(getMotionPoints(motion_name, motion_points));

    // Seed target pose with current joint state
    foreach (MoveJointGroupPtr move_joint_group, move_joint_groups_)
    {
      if (!hasNonNullIntersection(motion_joints, move_joint_group->getJointNames()))
        continue;
      RETHROW(getGroupTraj(move_joint_group, motion_joints, motion_points, joint_group_traj[move_joint_group]));
      if (!move_joint_group->isIdle())
        return false;
    }
    if (joint_group_traj.empty())
      return false;

    goal_id = PlayMotion::goal_next_id++;
    Goal& goal = goals_[goal_id];

    // Send pose commands
    typedef std::pair<MoveJointGroupPtr, Trajectory> traj_pair_t;
    foreach (const traj_pair_t& p, joint_group_traj)
    {
      goal.addController(p.first);
      p.first->setCallback(boost::bind(&PlayMotion::controllerCb, this, _1, goal_id));
      if (!p.first->sendGoal(p.second, duration))
      {
        ROS_ERROR_STREAM("controller '" << p.first->getName() << "' did not accept trajectory, "
                         "canceling everything");
        cancel(goal_id);
        return false;
      }
    }
    return true;
  }
}
