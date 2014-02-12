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
/** \author Victor Lopez. */
#include "play_motion/play_motion_helpers.h"
#include <cassert>

#include <ros/ros.h>
#include <boost/foreach.hpp>
#include "play_motion/xmlrpc_helpers.h"
#define foreach BOOST_FOREACH

namespace play_motion
{

  std::string getParamName(const std::string &motion_id)
  {
    return  "motions/" + motion_id;
  }

  void extractTrajectory(xh::Array &traj_points, Trajectory& motion_points)
  {

    motion_points.clear();
    motion_points.reserve(traj_points.size());
    for (int i = 0; i < traj_points.size(); ++i)
    {
      xh::Struct &name_value = traj_points[i];
      TrajPoint point;
      xh::getStructMember(name_value, "time_from_start", point.time_from_start);

      xh::Array positions;
      xh::getStructMember(name_value, "positions", positions);
      point.positions.resize(positions.size());
      for (int j = 0; j < positions.size(); ++j)
        xh::getArrayItem(positions, j, point.positions[j]);

      if (name_value.hasMember("velocities"))
      {
        xh::Array velocities;
        xh::getStructMember(name_value, "velocities", velocities);
        point.velocities.resize(velocities.size());
        for (int j = 0; j < velocities.size(); ++j)
          xh::getArrayItem(velocities, j, point.velocities[j]);
      }
      motion_points.push_back(point);
    }
  }

  void extractJoints(xh::Array &joint_names, JointNames &motion_joints)
  {
    motion_joints.clear();
    motion_joints.resize(joint_names.size());
    for (int i = 0; i < joint_names.size(); ++i)
      xh::getArrayItem(joint_names, i, motion_joints[i]);
  }

  void getMotionJoints(const ros::NodeHandle &nh, const std::string& motion_id,
                       JointNames& motion_joints)
  {
    MotionInfo info;
    getMotion(nh, motion_id, info);
    motion_joints = info.joints;
  }

  void getMotionPoints(const ros::NodeHandle &nh, const std::string& motion_id,
                       Trajectory& motion_points)
  {
    MotionInfo info;
    getMotion(nh, motion_id, info);
    motion_points = info.traj;
  }

  void getMotionIds(const ros::NodeHandle &nh, MotionNames& motion_ids)
  {
    xh::Struct motions;

    xh::fetchParam(nh, "motions/", motions);
    for (xh::Struct::iterator it = motions.begin(); it != motions.end(); ++it)
    {
      motion_ids.push_back(it->first);
    }
  }

  void populateVelocities(const TrajPoint& point_prev,
                          const TrajPoint& point_next,
                                TrajPoint& point_curr)
  {
    const int num_joints = point_curr.positions.size();
    assert(num_joints == point_prev.positions.size() && num_joints == point_next.positions.size());

    // Do nothing if waypoint contains a valid velocity specification
    if (int(point_curr.velocities.size()) == num_joints) {return;}

    // Initialize joint velocities to zero
    std::vector<double>& vel_out = point_curr.velocities;
    vel_out.resize(num_joints, 0.0);

    // Set individual joint velocities
    for (int i = 0; i < num_joints; ++i)
    {
      const double pos_curr = point_curr.positions[i];
      const double pos_prev = point_prev.positions[i];
      const double pos_next = point_next.positions[i];

      if ( (pos_curr == pos_prev)                        ||
           (pos_curr < pos_prev && pos_curr <= pos_next) ||
           (pos_curr > pos_prev && pos_curr >= pos_next) )
      {
        vel_out[i] = 0.0; // Special cases where zero velocity is enforced
      }
      else
      {
        // General case using numeric differentiation
        const double t_prev = point_curr.time_from_start.toSec() - point_prev.time_from_start.toSec();
        const double t_next = point_next.time_from_start.toSec() - point_curr.time_from_start.toSec();

        const double v_prev = (pos_curr - pos_prev)/t_prev;
        const double v_next = (pos_next - pos_curr)/t_next;

        vel_out[i] = 0.5 * (v_prev + v_next);
      }
    }
  }

  void populateVelocities(const Trajectory& traj_in, Trajectory& traj_out)
  {
    if (traj_in.empty()) {return;}

    const int num_waypoints = traj_in.size();
    const int num_joints    = traj_in.front().positions.size();

    // Initialize first and last points with zero velocity, if unspecified or not properly sized
    TrajPoint& point_first = traj_out.front();
    TrajPoint& point_last  = traj_out.back();

    if (int(point_first.velocities.size()) != num_joints) {point_first.velocities.resize(num_joints, 0.0);}
    if (int(point_last.velocities.size())  != num_joints) {point_last.velocities.resize(num_joints, 0.0);}

    // Populate velocities for remaining points (all but first and last)
    for (int i = 1; i < num_waypoints - 1; ++i) {populateVelocities(traj_in[i - 1], traj_in[i + 1], traj_out[i]);}
  }

  ros::Duration getMotionDuration(const ros::NodeHandle &nh, const std::string &motion_id)
  {
    Trajectory traj;
    getMotionPoints(nh, motion_id, traj);

    return traj.back().time_from_start;
  }

  bool motionExists(const ros::NodeHandle &nh, const std::string &motion_id)
  {
    return nh.hasParam(getParamName(motion_id));
  }

  bool isAlreadyThere(const JointNames &targetJoints, const TrajPoint &targetPoint,
                      const JointNames &sourceJoints, const TrajPoint &sourcePoint,
                      double tolerance)
  {
    if (targetJoints.size() != targetPoint.positions.size())
      throw ros::Exception("targetJoint and targetPoint positions sizes do not match");

    if (sourceJoints.size() != sourcePoint.positions.size())
      throw ros::Exception("sourceJoint and sourcePoint positions sizes do not match");

    for (int tIndex = 0; tIndex < targetJoints.size(); ++tIndex)
    {
      JointNames::const_iterator it = std::find(sourceJoints.begin(), sourceJoints.end(),
                                                targetJoints[tIndex]);
      /// If a joint used in the target is not used in the available in the
      /// source can't guarantee that the points are equivalent
      if (it == sourceJoints.end())
        return false;

      int sIndex = it - sourceJoints.begin();
      if (std::fabs(targetPoint.positions[tIndex] - sourcePoint.positions[sIndex]) > tolerance)
        return false;
    }
    return true;
  }

  void getMotion(const ros::NodeHandle &nh, const std::string &motion_id,
                 MotionInfo &motionInfo)
  {
    motionInfo.id = motion_id;
    xh::Struct param;
    xh::fetchParam(nh, getParamName(motion_id), param);

    extractTrajectory(param["points"], motionInfo.traj);
    extractJoints(param["joints"], motionInfo.joints);
    if (param.hasMember("meta"))
    {
      xh::getStructMember(param["meta"], "description", motionInfo.description);
      xh::getStructMember(param["meta"], "name", motionInfo.name);
      xh::getStructMember(param["meta"], "usage", motionInfo.usage);
    }
    else
    {
      motionInfo.description = "";
      motionInfo.name = "";
      motionInfo.usage = "";
    }
  }


}
