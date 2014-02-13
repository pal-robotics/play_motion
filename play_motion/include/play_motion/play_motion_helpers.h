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
/** \author Victor Lopez.                     */

#ifndef PLAYMOTIONHELPERS_H
#define PLAYMOTIONHELPERS_H

#include <string>
#include "play_motion/datatypes.h"

namespace ros
{
  class NodeHandle;
}

namespace play_motion
{

  struct MotionInfo
  {
    std::string id;
    std::string name;
    std::string usage;
    std::string description;
    JointNames joints;
    Trajectory traj;
  };

  /**
   * @param nh Nodehandle with the namespace containing the motions
   *           (ie ros::NodeHandle( nh"play_motion"))
   * @throws xh::XmlrpcHelperException if motion_id cannot be found
   */
  void getMotionJoints(const ros::NodeHandle &nh, const std::string& motion_id,
                       JointNames& motion_joints);


  /**
   * @param nh Nodehandle with the namespace containing the motions
   *           (ie ros::NodeHandle( nh"play_motion"))
   * @throws xh::XmlrpcHelperException if motion_id cannot be found
   */
  void getMotionPoints(const ros::NodeHandle &nh, const std::string& motion_id,
                       Trajectory& motion_points);


  /**
   * @brief getMotionDuration gets the total duration of a motion
   * @throws xh::XmlrpcHelperException if motion_id cannot be found
   */
  ros::Duration getMotionDuration(const ros::NodeHandle &nh,
                                  const std::string &motion_id);

  /**
   * @brief getMotions obtain all motion names
   * @param nh Nodehandle with the namespace containing the motions
   *           (ie ros::NodeHandle( nh"play_motion"))
   * @throws xh::XmlrpcHelperException if no motions available
   */
  void getMotionIds(const ros::NodeHandle &nh, MotionNames& motion_ids);


  bool motionExists(const ros::NodeHandle &nh, const std::string &motion_id);

  /**
   * @brief isAlreadyThere checks if the source trajPoint matches the target
   *        trajPoint with a certain tolerance
   *        only the joints in targetJoint will be checked
   * @param tolerance tolerance per joint in radians
   */
  bool isAlreadyThere(const JointNames &targetJoints, const TrajPoint &targetPoint,
                      const JointNames &sourceJoints, const TrajPoint &sourcePoint,
                      double tolerance = 0.15);



  void populateVelocities(const Trajectory& traj_in, Trajectory& traj_out);

  void getMotion(const ros::NodeHandle &nh, const std::string &motion_id,
                 MotionInfo &motionInfo);
}

#endif
