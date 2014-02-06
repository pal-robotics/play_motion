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
#include "play_motion/xmlrpc_helpers.h"
#define foreach BOOST_FOREACH

namespace play_motion
{
  void getMotionJoints(const std::string& motion_name, JointNames& motion_joints)
  {
    ros::NodeHandle nh("~");
    xh::Array joint_names;

    xh::fetchParam(nh, "motions/" + motion_name + "/joints", joint_names);
    motion_joints.clear();
    motion_joints.resize(joint_names.size());
    for (int i = 0; i < joint_names.size(); ++i)
      xh::getArrayItem(joint_names, i, motion_joints[i]);
  }

  void getMotionPoints(const std::string& motion_name, Trajectory& motion_points)
  {
    ros::NodeHandle nh("~");
    xh::Array traj_points;
    xh::fetchParam(nh, "motions/" + motion_name + "/points", traj_points);
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

  void populateVelocities(const Trajectory& traj_in, Trajectory& traj_out)
  {
    if (traj_in.empty()) {return;}

    const int num_waypoints = traj_in.size();
    const int num_joints    = traj_in.front().positions.size();

    // Initialize first and last points with zero velocity, if unspecified or not properly sized:
    TrajPoint& point_first = traj_out.front();
    TrajPoint& point_last  = traj_out.back();

    if (int(point_first.velocities.size()) != num_joints) {point_first.velocities.resize(num_joints, 0.0);}
    if (int(point_last.velocities.size())  != num_joints) {point_last.velocities.resize(num_joints, 0.0);}

    // Iterate over all waypoints except the first and last
    for (int i = 1; i < num_waypoints - 1; ++i)
    {
      std::vector<double>& vel_out = traj_out[i].velocities;
      const TrajPoint& point_curr = traj_in[i];
      const TrajPoint& point_prev = traj_in[i - 1];
      const TrajPoint& point_next = traj_in[i + 1];

      // Do nothing if waypoint contains a velocity specification, otherwise initialize to zero and continue
      if (int(point_curr.velocities.size()) != num_joints) {vel_out.resize(num_joints, 0.0);}
      else {return;} // Waypoint already specifies a velocity vector of the appropriate size

      // Iterate over all joints in a waypoint
      for (int j = 0; j < num_joints; ++j)
      {
        const double pos_curr = point_curr.positions[j];
        const double pos_prev = point_prev.positions[j];
        const double pos_next = point_next.positions[j];

        if ( (pos_curr == pos_prev)                        ||
             (pos_curr < pos_prev && pos_curr <= pos_next) ||
             (pos_curr > pos_prev && pos_curr >= pos_next) )
        {
          // Special case where zero velocity is enforced
          vel_out[j] = 0.0;
        }
        else
        {
          // General case using numeric differentiation
          const double t_prev = point_curr.time_from_start.toSec() - point_prev.time_from_start.toSec();
          const double t_next = point_next.time_from_start.toSec() - point_curr.time_from_start.toSec();

          const double v_prev = (pos_curr - pos_prev)/t_prev;
          const double v_next = (pos_next - pos_curr)/t_next;

          vel_out[j] = 0.5*(v_prev + v_next);
        }
      }
    }
  }
}
