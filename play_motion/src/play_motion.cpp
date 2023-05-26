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

#include "play_motion/approach_planner.h"
#include "play_motion/move_joint_group.h"
#include "play_motion/xmlrpc_helpers.h"

#define foreach BOOST_FOREACH

namespace
{
  typedef play_motion::PlayMotion::GoalHandle            GoalHandle;
  typedef boost::shared_ptr<play_motion::MoveJointGroup> MoveJointGroupPtr;
  typedef boost::weak_ptr<play_motion::MoveJointGroup>   MoveJointGroupWeakPtr;
  typedef std::list<MoveJointGroupPtr>                   ControllerList;
  typedef play_motion::PMR                               PMR;
  typedef actionlib::SimpleClientGoalState               SCGS;

  void generateErrorCode(GoalHandle goal_hdl, int error_code, SCGS ctrl_state)
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
        os << "Got error code " << error_code << ", motion aborted.";
        goal_hdl->error_string = os.str();
    }
    //TODO: add handling for controller state
  }

  void controllerCb(int error_code, GoalHandle goal_hdl, const MoveJointGroupWeakPtr& weak_ctrl)
  {
    // If we don't use a weak ptr, using a boost::bind on controllerCb keeps a copy
    // of the shared ptr and keeps it alive.
    if (weak_ctrl.expired())
    {
      ROS_ERROR_STREAM("Got callback on expired MoveJointGroup, ignoring it");
      return;      
    }
    MoveJointGroupPtr ctrl = weak_ctrl.lock();
    ControllerList::iterator it = std::find(goal_hdl->controllers.begin(),
                                            goal_hdl->controllers.end(), ctrl);
    if (it == goal_hdl->controllers.end())
    {
      ROS_ERROR_STREAM("Something is wrong in the controller callback handling. "
                       << ctrl->getName() << " called a goal callback while no "
                       "motion goal was alive for it.");
      return;
    }
    goal_hdl->controllers.erase(it);

    ROS_DEBUG_STREAM("Return from joint group " << ctrl->getName() << ", "
                     << goal_hdl->controllers.size() << " active controllers, "
                     "error: " << error_code);

    if (goal_hdl->canceled)
    {
      ROS_DEBUG("The Goal was canceled, not calling Motion callback.");
      return;
    }

    goal_hdl->error_code = PMR::SUCCEEDED;
    if (error_code != 0 || ctrl->getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_ERROR_STREAM("Controller " << ctrl->getName() << " aborted.");
      goal_hdl->cancel();
      generateErrorCode(goal_hdl, error_code, ctrl->getState());
      goal_hdl->cb(goal_hdl);
      return;
    }

    if (goal_hdl->controllers.empty())
      goal_hdl->cb(goal_hdl);
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
} // unnamed namespace

namespace play_motion
{
  PlayMotion::PlayMotion(ros::NodeHandle& nh) :
    nh_(nh),
    joint_states_sub_(nh_.subscribe("joint_states", 10, &PlayMotion::jointStateCb, this)),
    ctrlr_updater_(nh_)
  {
    ctrlr_updater_.registerUpdateCb(boost::bind(&PlayMotion::updateControllersCb, this, _1, _2));

    ros::NodeHandle private_nh("~");
    approach_planner_.reset(new ApproachPlanner(private_nh));
  }

  PlayMotion::Goal::Goal(const Callback& cbk)
    : error_code(0)
    , active_controllers(0)
    , cb(cbk)
    , canceled(false)
  {}

  void PlayMotion::Goal::cancel()
  {
    canceled = true;
    foreach (MoveJointGroupPtr mjg, controllers)
      mjg->cancel();
  }

  void PlayMotion::Goal::addController(const MoveJointGroupPtr& ctrl)
  {
    controllers.push_back(ctrl);
  }

  PlayMotion::Goal::~Goal()
  {
    cancel();
  }

  void PlayMotion::updateControllersCb(const ControllerUpdater::ControllerStates& states,
                                       const ControllerUpdater::ControllerJoints& joints)
  {
    typedef std::pair<std::string, ControllerUpdater::ControllerState> ctrlr_state_pair_t;
    
    ROS_INFO_STREAM("Controllers have changed, cancelling all active goals");
    foreach (MoveJointGroupPtr mjg, move_joint_groups_)
    {
      // Deleting the groups isn't enough, because they are referenced by
      // the goalhandles. They will only be destroyed when the action ends,
      // which will crash because you cannot destroy actionclient from within a callback
      // We must abort, so the goalhandle is destroyed
      mjg->abort();
    }
    move_joint_groups_.clear();
    foreach (const ctrlr_state_pair_t& p, states)
    {
      if (p.second != ControllerUpdater::RUNNING)
        continue;
      move_joint_groups_.push_back(MoveJointGroupPtr(new MoveJointGroup(p.first, joints.at(p.first))));
      ROS_DEBUG_STREAM("Controller '" << p.first << "' with " << joints.at(p.first).size() << " joints.");
    }
  }

  void PlayMotion::jointStateCb(const sensor_msgs::JointStatePtr& msg)
  {
    last_joint_state_timestamp_ = msg->header.stamp;
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
      bool has_velocities    = !p.velocities.empty();
      bool has_accelerations = !p.accelerations.empty();
      TrajPoint point;
      point.positions.resize(group_joint_names.size());
      if (has_velocities)
        point.velocities.resize(group_joint_names.size(), 0);
      if (has_accelerations)
        point.accelerations.resize(group_joint_names.size(), 0);
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
          if (has_accelerations)
            point.accelerations[i] = p.accelerations[index];
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
      error_msg << "Could not parse motion '" << motion_name << "': " << e.what();
      throw PMException(error_msg.str(), PMR::MOTION_NOT_FOUND);
    }
    catch (const ros::Exception& e)
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
      error_msg << "Could not parse motion '" << motion_name << "': " << e.what();
      throw PMException(error_msg.str(), PMR::MOTION_NOT_FOUND);
    }
    catch (const ros::Exception& e)
    {
      std::ostringstream error_msg;
      error_msg << "could not parse motion '" << motion_name << "': " << e.what();
      throw PMException(error_msg.str(), PMR::MOTION_NOT_FOUND);
    }
  }

  ControllerList PlayMotion::getMotionControllers(const JointNames& motion_joints)
  {
    // Populate list of controllers containing at least one motion joint,...
    ControllerList ctrlr_list;
    foreach (MoveJointGroupPtr move_joint_group, move_joint_groups_)
    {
      if (hasNonNullIntersection(motion_joints, move_joint_group->getJointNames()))
        ctrlr_list.push_back(move_joint_group);
    }

    // ...check that all motion joints are contained in this list...
    foreach (const std::string& jn, motion_joints)
    {
      foreach (MoveJointGroupPtr ctrlr, ctrlr_list)
        if (ctrlr->isControllingJoint(jn))
          goto next_joint;

      throw PMException("No controller was found for joint '" + jn + "'", PMR::MISSING_CONTROLLER);
next_joint:;
    }

    // ...and that no controller in the list is busy executing another goal
    foreach (MoveJointGroupPtr move_joint_group, ctrlr_list)
    {
      if(!move_joint_group->isIdle())
        throw PMException("Controller '" + move_joint_group->getName() + "' is busy", PMR::CONTROLLER_BUSY);
    }

    return ctrlr_list;
  }

  bool PlayMotion::run(const std::string& motion_name,
                       bool               skip_planning,
                       GoalHandle&        goal_hdl,
                       const Callback&    cb)
  {
    JointNames                              motion_joints;
    Trajectory                              motion_points;
    std::map<MoveJointGroupPtr, Trajectory> joint_group_traj;

    goal_hdl = GoalHandle(new Goal(cb));

    try
    {
      getMotionJoints(motion_name, motion_joints);
      ControllerList groups = getMotionControllers(motion_joints); // Checks many preconditions
      getMotionPoints(motion_name, motion_points);

      if ((ros::Time::now() - last_joint_state_timestamp_).toSec() > 0.5)
      {
        throw PMException("Unable to update the current state of the motion joints! Last joint state is older than 0.5sec!",
                          PMR::OTHER_ERROR);
      }
      std::vector<double> curr_pos;  // Current position of motion joints
      foreach (const std::string& motion_joint, motion_joints)
      {
        if (joint_states_.count(motion_joint) == 0)
          throw PMException("Error playing the motion : " + motion_name +
                                "!. Couldn't find the joint : " + motion_joint +
                                " in the joint states!",
                            PMR::OTHER_ERROR);
        curr_pos.push_back(joint_states_[motion_joint]);  // TODO: What if motion joint does not exist?
      }

      // Approach trajectory
      Trajectory motion_points_safe;
      if (!approach_planner_->prependApproach(motion_joints, curr_pos,
                                              skip_planning,
                                              motion_points, motion_points_safe))
        throw PMException("Approach motion planning failed", PMR::NO_PLAN_FOUND);// TODO: Expose descriptive error string from approach_planner

      // TODO: Resample and validate output trajectory
      try
      {
        populateVelocities(motion_points_safe, motion_points_safe);
      }
      catch (const ros::Exception& e){
          throw PMException(e.what(), PMR::OTHER_ERROR);
      }

      // Seed target pose with current joint state
      foreach (MoveJointGroupPtr move_joint_group, groups)
      {
        if(!getGroupTraj(move_joint_group, motion_joints, motion_points_safe,
                         joint_group_traj[move_joint_group]))
          throw PMException("Missing joint state for joint in controller '"
                            + move_joint_group->getName() + "'");
      }
      if (joint_group_traj.empty())
        throw PMException("Nothing to send to controllers");

      // Send pose commands
      typedef std::pair<MoveJointGroupPtr, Trajectory> traj_pair_t;
      foreach (const traj_pair_t& p, joint_group_traj)
      {
        goal_hdl->addController(p.first);
        p.first->setCallback(boost::bind(controllerCb, _1, goal_hdl, MoveJointGroupWeakPtr(p.first)));
        if (!p.first->sendGoal(p.second))
          throw PMException("Controller '" + p.first->getName() + "' did not accept trajectory, "
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
