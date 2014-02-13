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
#include "play_motion/play_motion_helpers.h"
#include <cassert>

#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <sensor_msgs/JointState.h>

#include "play_motion/move_joint_group.h"
#include "play_motion/xmlrpc_helpers.h"

#define foreach BOOST_FOREACH

namespace play_motion
{
  PlayMotion::PlayMotion(ros::NodeHandle& nh) :
    nh_(nh),
    joint_states_sub_(nh_.subscribe("joint_states", 10, &PlayMotion::jointStateCb, this)),
    ctrlr_updater_(nh_)
  {
    ctrlr_updater_.registerUpdateCb(boost::bind(&PlayMotion::updateControllersCb, this, _1, _2));
  }

  void PlayMotion::Goal::cancel()
  {
    foreach (MoveJointGroupPtr mjg, controllers)
      mjg->cancel();
  }

  void PlayMotion::Goal::addController(const MoveJointGroupPtr& ctrl)
  {
    controllers.push_back(ctrl);
    active_controllers++;
  }

  PlayMotion::Goal::~Goal()
  {
    if (active_controllers)
      cancel();
  }

  void PlayMotion::updateControllersCb(const ControllerUpdater::ControllerStates& states,
                                       const ControllerUpdater::ControllerJoints& joints)
  {
    typedef std::pair<std::string, ControllerUpdater::ControllerState> ctrlr_state_pair_t;
    move_joint_groups_.clear();
    foreach (const ctrlr_state_pair_t& p, states)
    {
      if (p.second != ControllerUpdater::RUNNING)
        continue;
      move_joint_groups_.push_back(MoveJointGroupPtr(new MoveJointGroup(p.first, joints.at(p.first))));
      ROS_DEBUG_STREAM("controller '" << p.first << "' with " << joints.at(p.first).size() << " joints");
    }
  }

  static void generateErrorCode(PlayMotion::GoalHandle goal_hdl, int error_code)
  {
    typedef control_msgs::FollowJointTrajectoryResult JTR;
    switch (error_code)
    {
      case JTR::PATH_TOLERANCE_VIOLATED:
        goal_hdl->error_code = PMR::TRAJECTORY_ERROR;
        break;
      case JTR::GOAL_TOLERANCE_VIOLATED:
        goal_hdl->error_code = PMR::GOAL_NOT_REACHED;
        break;
      default:
        std::ostringstream os;
        goal_hdl->error_code = PMR::OTHER_ERROR;
        os << "got error code " << error_code << ", motion aborted";
        goal_hdl->error_string = os.str();
    }
  }

  void PlayMotion::controllerCb(int error_code, GoalHandle goal_hdl)
  {
    if (goal_hdl->active_controllers < 1)
      return;

    ROS_DEBUG("return from joint group, %d active controllers, error: %d",
              goal_hdl->active_controllers - 1, error_code);

    if (error_code != 0)
    {
      generateErrorCode(goal_hdl, error_code);
      goal_hdl->cancel();
      goal_hdl->active_controllers = 1; // terminate goal immediately
    }

    goal_hdl->active_controllers--;
    if (goal_hdl->active_controllers == 0)
    {
      if (!goal_hdl->error_code)
        goal_hdl->error_code = PMR::SUCCEEDED;
      goal_hdl->cb(goal_hdl);
    }
  };

  void PlayMotion::jointStateCb(const sensor_msgs::JointStatePtr& msg)
  {
    joint_states_.clear();
    for (uint32_t i=0; i < msg->name.size(); ++i)
      joint_states_[msg->name[i]] = msg->position[i];
  }

  bool PlayMotion::getGroupTraj(MoveJointGroupPtr move_joint_group,
                                const JointNames& motion_joints,
                                const Trajectory& motion_points, Trajectory& traj_group)
  {
    JointNames                 group_joint_names = move_joint_group->getJointNames();
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

  void PlayMotion::getMotionJoints(const std::string& motion_name, JointNames& motion_joints)
  {
    try
    {
      ::play_motion::getMotionJoints(ros::NodeHandle("~"), motion_name, motion_joints);
    }
    catch (const xh::XmlrpcHelperException& e)
    {
      std::ostringstream error_msg;
      error_msg << "could not parse motion '" << motion_name << "': " << e.what();
      throw PMException(error_msg.str(), PMR::MOTION_NOT_FOUND);
    }
  }

  void PlayMotion::getMotionPoints(const std::string& motion_name, Trajectory& motion_points)
  {
    try
    {
      ::play_motion::getMotionPoints(ros::NodeHandle("~"), motion_name, motion_points);
    }
    catch (const xh::XmlrpcHelperException& e)
    {
      std::ostringstream error_msg;
      error_msg << "could not parse motion '" << motion_name << "': " << e.what();
      throw PMException(error_msg.str(), PMR::MOTION_NOT_FOUND);
    }
  }

  void PlayMotion::checkControllers(const JointNames& motion_joints)
  {
    foreach (const std::string& jn, motion_joints)
    {
      foreach (MoveJointGroupPtr ctrlr, move_joint_groups_)
        if (ctrlr->isControllingJoint(jn))
          goto next_joint;

      throw PMException("no controller was found for joint '" + jn + "'", PMR::MISSING_CONTROLLER);
next_joint:;
    }
  }

  void PlayMotion::populateVelocities(const Trajectory& traj_in, Trajectory& traj_out)
  {
    ::play_motion::populateVelocities (traj_in, traj_out);
  }


  template <class T>
  static bool hasNonNullIntersection(const std::vector<T>& v1, const std::vector<T>& v2)
  {
    foreach (const T& e1, v1)
      foreach (const T& e2, v2)
        if (e1 == e2)
          return true;
    return false;
  }

  bool PlayMotion::run(const std::string& motion_name, const ros::Duration& duration,
                       GoalHandle& goal_hdl, const Callback& cb)
  {
    JointNames                              motion_joints;
    Trajectory                              motion_points;
    std::map<MoveJointGroupPtr, Trajectory> joint_group_traj;

    goal_hdl = GoalHandle(new Goal(cb));

    try
    {
      double shortest_time = 1.0e-2;
      if (duration.toSec() < shortest_time)
        throw PMException("reach time too small", PMR::INFEASIBLE_REACH_TIME);
      getMotionJoints(motion_name, motion_joints);
      checkControllers(motion_joints);
      getMotionPoints(motion_name, motion_points);
      populateVelocities(motion_points, motion_points);

      // Seed target pose with current joint state
      foreach (MoveJointGroupPtr move_joint_group, move_joint_groups_)
      {
        if (!hasNonNullIntersection(motion_joints, move_joint_group->getJointNames()))
          continue;
        if(!getGroupTraj(move_joint_group, motion_joints, motion_points,
                         joint_group_traj[move_joint_group]))
          throw PMException("missing joint state for joint in controller '"
                            + move_joint_group->getName() + "'");
        if(!move_joint_group->isIdle())
          throw PMException("controller '" + move_joint_group->getName()
                            + "' is busy", PMR::CONTROLLER_BUSY);
      }
      if (joint_group_traj.empty())
        throw PMException("nothing to send to controllers");

      // Send pose commands
      typedef std::pair<MoveJointGroupPtr, Trajectory> traj_pair_t;
      foreach (const traj_pair_t& p, joint_group_traj)
      {
        goal_hdl->addController(p.first);
        p.first->setCallback(boost::bind(&PlayMotion::controllerCb, this, _1, goal_hdl));
        if (!p.first->sendGoal(p.second, duration))
          throw PMException("controller '" + p.first->getName() + "' did not accept trajectory, "
                            "canceling everything");
      }
    }
    catch (const PMException& e)
    {
      goal_hdl->error_string = e.what();
      goal_hdl->error_code = e.error_code();
      return false;
    }
    return true;
  }
}
