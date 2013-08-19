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

#include "play_motion/play_motion_server.h"

#include <boost/foreach.hpp>

#include "play_motion/reach_pose.h"

#define foreach BOOST_FOREACH

namespace play_motion
{
  PlayMotionServer::PlayMotionServer(const ros::NodeHandle& nh, const ReachPosePtr& rp) :
    nh_(nh),
    rp_(rp),
    al_server_(nh_, "play_motion", false)
  {
    initControllerList();
    if (!ros::ok())
      return;
    rp->setAlCb(boost::bind(&PlayMotionServer::reachPoseCb, this, _1));
    al_server_.registerGoalCallback(boost::bind(&PlayMotionServer::alCallback, this));
    al_server_.start();
  }

  PlayMotionServer::~PlayMotionServer()
  {}

  void PlayMotionServer::reachPoseCb(bool success)
  {
    if (success)
    {
      ROS_INFO("motion played sucecssfully");
      al_server_.setSucceeded();
    }
    else
    {
      ROS_WARN("motion ended with an error");
      al_server_.setAborted();
    }
  }

  void PlayMotionServer::alCallback()
  {
    const AlServer::GoalConstPtr& goal = al_server_.acceptNewGoal();
    ROS_INFO_STREAM("sending pose '" << goal-> motion_name << "' to controllers");
    if (!rp_->run(goal->motion_name, goal->duration))
      al_server_.setAborted(AlServer::Result());
  }

  void PlayMotionServer::initControllerList()
  {
    ros::NodeHandle nh_controllers("~");
    std::vector<std::string> clist;

    XmlRpc::XmlRpcValue controller_names;
    if (!nh_controllers.getParam("controllers", controller_names))
    {
      ROS_FATAL("no controllers could be loaded, taking the node down");
      ros::shutdown();
      return;
    }

    ROS_ASSERT(controller_names.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < controller_names.size(); ++i)
    {
      ROS_ASSERT(controller_names[i].getType() == XmlRpc::XmlRpcValue::TypeString);
      clist.push_back(static_cast<std::string>(controller_names[i]));
      ROS_INFO_STREAM("adding a controller: " << clist.back());
    }

    rp_->setControllerList(clist);
  }
}
