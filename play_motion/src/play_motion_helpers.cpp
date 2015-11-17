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
/** \author Bence Magyar. */
#include "play_motion/play_motion_helpers.h"
#include <cassert>

#include <ros/ros.h>
#include <boost/foreach.hpp>
#include "play_motion/xmlrpc_helpers.h"
#define foreach BOOST_FOREACH

namespace play_motion
{
  ros::NodeHandle getMotionsNodeHandle(const ros::NodeHandle& nh)
  {
    return ros::NodeHandle(nh, "motions");
  }

  void extractTrajectory(xh::Array &traj_points, Trajectory& motion_points)
  {

    motion_points.clear();
    motion_points.reserve(traj_points.size());
    for (int i = 0; i < traj_points.size(); ++i)
    {
      xh::Struct &name_value = traj_points[i];
      TrajPoint point;
      double tfs;
      xh::getStructMember(name_value, "time_from_start", tfs);
      point.time_from_start = ros::Duration(tfs);

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

  void getMotionJoints(const std::string& motion_id, JointNames& motion_joints)
  {
    ros::NodeHandle pm_nh("play_motion");
    getMotionJoints(pm_nh, motion_id, motion_joints);
  }

  void getMotionPoints(const ros::NodeHandle &nh, const std::string& motion_id,
                       Trajectory& motion_points)
  {
    MotionInfo info;
    getMotion(nh, motion_id, info);
    motion_points = info.traj;
  }

  void getMotionPoints(const std::string& motion_id, Trajectory& motion_points)
  {
    ros::NodeHandle pm_nh("play_motion");
    getMotionPoints(pm_nh, motion_id, motion_points);
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

  void getMotionIds(MotionNames& motion_ids)
  {
    ros::NodeHandle pm_nh("play_motion");
    getMotionIds(pm_nh, motion_ids);
  }

  void populateVelocities(const TrajPoint& point_prev,
                          const TrajPoint& point_next,
                                TrajPoint& point_curr)
  {
    const int num_joints = point_curr.positions.size();
    if (num_joints != point_prev.positions.size() || num_joints != point_next.positions.size())
        throw ros::Exception("The positions array of a point of the trajectory does not have the same number of joints as the trajectory joint_names say.");


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

  ros::Duration getMotionDuration(const std::string &motion_id)
  {
      ros::NodeHandle pm_nh("play_motion");
      return getMotionDuration(pm_nh, motion_id);
  }

  bool motionExists(const ros::NodeHandle &nh, const std::string &motion_id)
  {
    try
    {
      ros::NodeHandle motions_nh = getMotionsNodeHandle(nh);
      return motions_nh.hasParam(motion_id + "/joints") && motions_nh.hasParam(motion_id + "/points");
    }
    catch (const ros::InvalidNameException&)
    {
      return false;
    }
  }

  bool motionExists(const std::string &motion_id)
  {
    ros::NodeHandle pm_nh("play_motion");
    return motionExists(pm_nh, motion_id);
  }

  bool isAlreadyThere(const JointNames &target_joints, const TrajPoint &target_point,
                      const JointNames &source_joints, const TrajPoint &source_point,
                      double tolerance)
  {
    if (target_joints.size() != target_point.positions.size())
      throw ros::Exception("targetJoint and targetPoint positions sizes do not match");

    if (source_joints.size() != source_point.positions.size())
      throw ros::Exception("sourceJoint and sourcePoint positions sizes do not match");

    for (int tIndex = 0; tIndex < target_joints.size(); ++tIndex)
    {
      JointNames::const_iterator it = std::find(source_joints.begin(), source_joints.end(),
                                                target_joints[tIndex]);
      /// If a joint used in the target is not used in the available in the
      /// source can't guarantee that the points are equivalent
      if (it == source_joints.end())
        return false;

      int sIndex = it - source_joints.begin();
      if (std::fabs(target_point.positions[tIndex] - source_point.positions[sIndex]) > tolerance)
        return false;
    }
    return true;
  }

  void getMotion(const ros::NodeHandle &nh, const std::string &motion_id,
                 MotionInfo &motion_info)
  {
    if (!motionExists(nh, motion_id))
    {
      const std::string what = "Motion '" + motion_id + "' does not exist or is malformed " +
                               "(namespace " + getMotionsNodeHandle(nh).getNamespace() + ").";
      throw ros::Exception(what);
    }
    motion_info.id = motion_id;
    xh::Struct param;
    xh::fetchParam(getMotionsNodeHandle(nh), motion_id, param);

    extractTrajectory(param["points"], motion_info.traj);
    extractJoints(param["joints"], motion_info.joints);
    if (param.hasMember("meta"))
    {
      xh::getStructMember(param["meta"], "description", motion_info.description);
      xh::getStructMember(param["meta"], "name", motion_info.name);
      xh::getStructMember(param["meta"], "usage", motion_info.usage);
    }
    else
    {
      motion_info.description = "";
      motion_info.name = "";
      motion_info.usage = "";
    }
  }

  void getMotion(const std::string &motion_id, MotionInfo &motion_info)
  {
    ros::NodeHandle pm_nh("play_motion");
    play_motion::getMotion(pm_nh, motion_id, motion_info);
  }

}
