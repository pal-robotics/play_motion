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

#include "play_motion/play_motion.h"
#include "play_motion/play_motion_helpers.h"
#include "play_motion/xmlrpc_helpers.h"

#define foreach BOOST_FOREACH

namespace play_motion
{
  PlayMotionServer::PlayMotionServer(const ros::NodeHandle& nh, const PlayMotionPtr& pm) :
    nh_(nh),
    pm_(pm),
    al_server_(nh_, "play_motion", false)
  {
    al_server_.registerGoalCallback(boost::bind(&PlayMotionServer::alGoalCb, this, _1));
    al_server_.registerCancelCallback(boost::bind(&PlayMotionServer::alCancelCb, this, _1));
    al_server_.start();

    list_motions_srv_ = ros::NodeHandle("~").advertiseService("list_motions",
                                                              &PlayMotionServer::listMotions,
                                                              this);
  }

  PlayMotionServer::~PlayMotionServer()
  {}

  bool PlayMotionServer::findGoalId(AlServer::GoalHandle gh, PlayMotion::GoalHandle& goal_hdl)
  {
    typedef std::pair<PlayMotion::GoalHandle, AlServer::GoalHandle> goal_pair_t;
    foreach (const goal_pair_t& p, al_goals_)
      if (goal_hdl = p.first, p.second == gh)
        return true;

    return false;
  }

  void PlayMotionServer::playMotionCb(const PlayMotion::GoalHandle& goal_hdl)
  {
    PMR r;
    r.error_code = goal_hdl->error_code;
    r.error_string = goal_hdl->error_string;

    if (r.error_code == PMR::SUCCEEDED)
    {
      ROS_INFO("Motion played successfully.");
      al_goals_[goal_hdl].setSucceeded(r);
    }
    else
    {
      ROS_WARN("Motion ended with an error.");
      al_goals_[goal_hdl].setAborted(r);
    }
    al_goals_.erase(goal_hdl);
  }

  void PlayMotionServer::alCancelCb(AlServer::GoalHandle gh)
  {
    PlayMotion::GoalHandle goal_hdl;
    if (findGoalId(gh, goal_hdl))
      goal_hdl->cancel(); //should not be needed
    else
      ROS_ERROR("Cancel request could not be fulfilled. Goal not running?.");

    al_goals_.erase(goal_hdl);
    gh.setCanceled();
  }

  void PlayMotionServer::alGoalCb(AlServer::GoalHandle gh)
  {
    AlServer::GoalConstPtr goal = gh.getGoal(); //XXX: can this fail? should we check it?
    ROS_INFO_STREAM("Received request to play motion '" << goal->motion_name << "'.");
    PlayMotion::GoalHandle goal_hdl;
    if (!pm_->run(goal->motion_name,
                  goal->skip_planning,
                  goal_hdl,
                  boost::bind(&PlayMotionServer::playMotionCb, this, _1)))
    {
      PMR r;
      r.error_code = goal_hdl->error_code;
      r.error_string = goal_hdl->error_string;
      if (!r.error_string.empty())
        ROS_ERROR_STREAM(r.error_string);
      ROS_ERROR_STREAM("Motion '" << goal->motion_name << "' could not be played.");
      gh.setRejected(r);
      return;
    }
    gh.setAccepted();
    al_goals_[goal_hdl] = gh;
  }

  bool PlayMotionServer::listMotions(play_motion_msgs::ListMotions::Request&  req,
                                     play_motion_msgs::ListMotions::Response& resp)
  {
    try
    {
      MotionNames motions;
      ros::NodeHandle pnh("~");
      getMotionIds(pnh, motions);
      foreach (const std::string& motion, motions)
      {
        play_motion_msgs::MotionInfo info;
        info.name = motion;
        getMotionJoints(pnh, motion, info.joints);
        getMotionDuration(pnh, motion);
        resp.motions.push_back(info);
      }
    }
    catch (const xh::XmlrpcHelperException& e)
    {
      ROS_ERROR_STREAM(e.what());
      return false;
    }
    return true;
  }
}
