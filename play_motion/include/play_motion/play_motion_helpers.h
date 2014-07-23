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
/** \author Bence Magyar.                     */

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
   * \param nh Nodehandle with the namespace containing the motions
   *           (When omitted, defaults to ros::NodeHandle nh("play_motion"))
   * \throws xh::XmlrpcHelperException if motion_id cannot be found
   */
  void getMotionJoints(const ros::NodeHandle &nh, const std::string& motion_id,
                       JointNames& motion_joints);
  void getMotionJoints(const std::string& motion_id, JointNames& motion_joints);


  /**
   * \param nh Nodehandle with the namespace containing the motions
   *           (When omitted, defaults to ros::NodeHandle nh("play_motion"))
   * \throws xh::XmlrpcHelperException if motion_id cannot be found
   */
  void getMotionPoints(const ros::NodeHandle &nh, const std::string& motion_id,
                       Trajectory& motion_points);
  void getMotionPoints(const std::string& motion_id, Trajectory& motion_points);


  /**
   * \brief getMotionDuration gets the total duration of a motion
   * \throws xh::XmlrpcHelperException if motion_id cannot be found
   */
  ros::Duration getMotionDuration(const ros::NodeHandle &nh,
                                  const std::string &motion_id);

  /**
   * \brief getMotions obtain all motion names
   * \param nh Nodehandle with the namespace containing the motions
   *           (When omitted, defaults to ros::NodeHandle nh("play_motion"))
   * \throws xh::XmlrpcHelperException if no motions available
   */
  void getMotionIds(const ros::NodeHandle &nh, MotionNames& motion_ids);
  void getMotionIds(MotionNames& motion_ids);

  /**
   * \param nh Nodehandle with the namespace containing the motions
   *           (When omitted, defaults to ros::NodeHandle nh("play_motion"))
   * \param motion_id Motion identifier
   * \return True if the motion exists, false otherwise
   */
  bool motionExists(const ros::NodeHandle &nh, const std::string &motion_id);
  bool motionExists(const std::string &motion_id);

  /**
   * \brief isAlreadyThere checks if the source trajPoint matches the target
   *        trajPoint with a certain tolerance
   *        only the joints in targetJoint will be checked
   * \param tolerance tolerance per joint in radians
   */
  bool isAlreadyThere(const JointNames &target_joints, const TrajPoint &target_point,
                      const JointNames &source_joints, const TrajPoint &source_point,
                      double tolerance = 0.15);

  /**
   * \brief Populate joint velocity information of a trajectory waypoint.
   *
   * Joint velocities are computed by numeric differentiation of position information, except in the following cases,
   * where it is set to zero:
   *
   * - \f$p_i = p_{i-1}\f$
   * - \f$p_i < p_{i-1}\f$ and \f$p_i \leq p_{i+1}\f$
   * - \f$p_i > p_{i-1}\f$ and \f$p_i \geq p_{i+1}\f$
   *
   * where \f$p_{i-1}\f$, \f$p_i\f$ and \f$p_{i+1}\f$ are the positions of a joint at the previous, current and next
   * waypoint. This heuristic is well suited for direction reversals and position holding without overshoot, and
   * produces better results than pure numeric differentiation.
   *
   * If the input waypoint already contains a valid velocity specification, it will \e not be overwritten, that is, this
   * method will be a no-op.
   *
   * \param[in]  point_prev Previous trajectory waypoint.
   * \param[in]  point_next Next trajectory waypoint.
   * \param[out] point_curr Trajectory waypoint to which velocity information will be added.
   *
   * \sa populateVelocities(const Trajectory&, Trajectory&)
   */
  void populateVelocities(const TrajPoint& point_prev,
                          const TrajPoint& point_next,
                                TrajPoint& point_curr);

  /**
   * \brief Populate joint velocity information of a trajectory.
   *
   * Joint velocities will be computed for all waypoints not containing a valid velocity specification. Waypoints with
   * an existing velocity specification will not be modified. If the trajectory endpoints don't specify velocites, they
   * will be set to zero.
   *
   * \param[in] traj_in Input trajectory. Some waypoints may have a velocity specification (or not at all).
   * \param[out] traj_out Output trajectory. All waypoints have a velocity specification. Can be the same instance as
   * \c traj_in.
   *
   * \sa populateVelocities(const TrajPoint&, const TrajPoint&, TrajPoint&)
   */
  void populateVelocities(const Trajectory& traj_in, Trajectory& traj_out);

  /**
   * \brief Parse a motion specified in the ROS parameter server into a data structure.
   * \param[in] nh Nodehandle with the namespace containing the motions
   *           (When omitted, defaults to ros::NodeHandle nh("play_motion"))
   * \param[in] motion_id Motion identifier
   * \param[out] motionInfo Data structure containing parsed motion
   * \throws ros::Exception if the motion does not exist or is malformed.
   */
  void getMotion(const ros::NodeHandle &nh, const std::string &motion_id,
                 MotionInfo &motion_info);
  void getMotion(const std::string &motion_id, MotionInfo &motion_info);
}

#endif
