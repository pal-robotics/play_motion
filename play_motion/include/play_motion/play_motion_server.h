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

#ifndef PLAYMOTIONSERVER_H
#define PLAYMOTIONSERVER_H

#include <string>
#include <vector>
#include <map>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <actionlib/server/action_server.h>

#include "play_motion/play_motion.h"
#include "play_motion_msgs/PlayMotionAction.h"
#include "play_motion_msgs/ListMotions.h"

namespace play_motion
{
  class PlayMotionServer
  {
  private:
    typedef actionlib::ActionServer<play_motion_msgs::PlayMotionAction> AlServer;
    typedef boost::shared_ptr<PlayMotion> PlayMotionPtr;

  public:
    PlayMotionServer(const ros::NodeHandle& nh, const PlayMotionPtr& pm);
    virtual ~PlayMotionServer();

  private:
    void playMotionCb(const PlayMotion::GoalHandle& goal_hdl);
    void alCancelCb(AlServer::GoalHandle gh);
    void alGoalCb(AlServer::GoalHandle gh);
    bool findGoalId(AlServer::GoalHandle gh, PlayMotion::GoalHandle& goal_id);
    bool listMotions(play_motion_msgs::ListMotions::Request&  req,
                     play_motion_msgs::ListMotions::Response& resp);
    void publishDiagnostics(const ros::TimerEvent &ev) const;

    ros::NodeHandle                                        nh_;
    std::vector<std::string>                               clist_;
    PlayMotionPtr                                          pm_;
    AlServer                                               al_server_;
    std::map<PlayMotion::GoalHandle, AlServer::GoalHandle> al_goals_;
    ros::ServiceServer                                     list_motions_srv_;

    ros::Publisher                                         diagnostic_pub_;
    ros::Timer                                             diagnostic_timer_;
  };
}

#endif
