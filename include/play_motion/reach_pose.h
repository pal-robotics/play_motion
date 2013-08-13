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
#include <ros/ros.h>

namespace sensor_msgs
{ ROS_DECLARE_MESSAGE(JointState); }

namespace play_motion
{
  class MoveJointGroup;

  /** Move robot joints to a given pose.
   * Poses are specified in the parameter server, and are identified by name.
   */
  class ReachPose
  {
    private:
      typedef boost::shared_ptr<MoveJointGroup>     MoveJointGroupPtr;
      typedef std::vector<std::string>              ControllerList;
      typedef std::pair<std::string, double>        JointState;
      typedef boost::function<void(bool)>           Callback;
      typedef std::vector<Callback>                 CallbackList;
      typedef std::map<std::string, double>         Pose;

    public:
      ReachPose(ros::NodeHandle& nh);

      /// \brief Send pose goal request and \e block until execution completes.
      /// \param pose Name of pose to execute.
      /// \param duration Motion duration.
      bool run(const std::string& pose, const ros::Duration& duration);
      void setAlCb(const Callback& cb) { client_cb_ = cb; }
      void setControllerList(const ControllerList& controller_list);

    private:
      void jointStateCb(const sensor_msgs::JointStatePtr& msg);
      void controllerCb(bool success);

      ros::NodeHandle                  nh_;
      std::vector<MoveJointGroupPtr>   move_joint_groups_;
      ControllerList                   controller_list_;
      std::vector<JointState>          joint_states_;
      ros::Subscriber                  joint_states_sub_;
      Callback                         client_cb_;
      std::map<std::string, Pose>      poses_;

      CallbackList                     controller_cb_list_;
      int                              current_active_controllers_;
      bool                             current_success_;
  };
}

#endif
