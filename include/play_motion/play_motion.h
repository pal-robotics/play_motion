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

#ifndef REACHPOSE_H
#define REACHPOSE_H

#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>

#include "play_motion/move_joint_group.h"
#include "play_motion/controller_updater.h"

namespace sensor_msgs
{ ROS_DECLARE_MESSAGE(JointState); }

namespace play_motion
{
  /** Move robot joints to a given pose.
   * Poses are specified in the parameter server, and are identified by name.
   */
  class PlayMotion
  {
    private:
      typedef boost::shared_ptr<MoveJointGroup>     MoveJointGroupPtr;
      typedef std::vector<std::string>              ControllerList;
      typedef boost::function<void(bool)>           Callback;
      typedef std::vector<Callback>                 CallbackList;
      typedef MoveJointGroup::TrajPoint             TrajPoint;
      typedef std::vector<TrajPoint>                Trajectory;

      struct Goal
      {
        bool                           success;
        int                            active_controllers;
        Callback                       cb;
        std::vector<MoveJointGroupPtr> controllers;

        Goal() : success(true), active_controllers(0) {}
        void addController(const MoveJointGroupPtr& ctrl)
        { controllers.push_back(ctrl); active_controllers++; }
      };

    public:
      PlayMotion(ros::NodeHandle& nh);

      /// \brief Send motion goal request
      /// \param motion_name Name of motion to execute.
      /// \param duration Motion duration.
      /// \param[out] goal_id contains the goal ID if function returns true
      bool run(const std::string& motion_name, const ros::Duration& duration, int& goal_id);
      void cancel(int goal_id);
      void setAlCb(int goal_id, const Callback& cb);

    private:
      void jointStateCb(const sensor_msgs::JointStatePtr& msg);
      void controllerCb(bool success, int goal_id);

      bool getGroupTraj(MoveJointGroupPtr move_joint_group,
          const std::vector<std::string>& motion_joints,
          const Trajectory& motion_points, Trajectory& traj_group);
      bool getMotionJoints(const std::string& motion_name, std::vector<std::string>& motion_joints);
      bool getMotionPoints(const std::string& motion_name, Trajectory& motion_points);
      bool checkControllers(const std::vector<std::string>& motion_joints);
      void updateControllersCb(const ControllerUpdater::ControllerStates& states,
          const ControllerUpdater::ControllerJoints& joints);

      static int goal_next_id;

      ros::NodeHandle                  nh_;
      std::vector<MoveJointGroupPtr>   move_joint_groups_;
      std::map<std::string, double>    joint_states_;
      ros::Subscriber                  joint_states_sub_;
      std::map<int, Goal>              goals_;
      ControllerUpdater                ctrlr_updater_;
  };
}

#endif
