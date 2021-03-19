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
 *     copyright notice, this list otoSecf conditions and the following
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
#include <exception>
#include <unordered_set>

// #include "play_motion/xmlrpc_helpers.h"

#include "rclcpp/node.hpp"

namespace play_motion
{
  /// @note needed
//  ros::NodeHandle getMotionsNodeHandle(const ros::NodeHandle& nh)
//  {
//    return ros::NodeHandle(nh, "motions");
//  }

  /// @note xmlrpc does not make much sense in ros2
//  void extractTrajectory(xh::Array &traj_points, Trajectory& motion_points)
//  {

//    motion_points.clear();
//    motion_points.reserve(traj_points.size());
//    for (int i = 0; i < traj_points.size(); ++i)
//    {
//      xh::Struct &name_value = traj_points[i];
//      TrajPoint point;
//      double tfs;
//      xh::getStructMember(name_value, "time_from_start", tfs);
//      point.time_from_start = rclcpp::Duration(tfs);

//      xh::Array positions;
//      xh::getStructMember(name_value, "positions", positions);
//      point.positions.resize(positions.size());
//      for (int j = 0; j < positions.size(); ++j)
//        xh::getArrayItem(positions, j, point.positions[j]);

//      if (name_value.hasMember("velocities"))
//      {
//        xh::Array velocities;
//        xh::getStructMember(name_value, "velocities", velocities);
//        point.velocities.resize(velocities.size());
//        for (int j = 0; j < velocities.size(); ++j)
//          xh::getArrayItem(velocities, j, point.velocities[j]);
//      }
//      motion_points.push_back(point);
//    }
//  }

  /// @note xmlrpc does not make much sense in ros2
//  void extractJoints(xh::Array &joint_names, JointNames &motion_joints)
//  {
//    motion_joints.clear();
//    motion_joints.resize(joint_names.size());
//    for (int i = 0; i < joint_names.size(); ++i)
//      xh::getArrayItem(joint_names, i, motion_joints[i]);
//  }

  void getMotionJoints(const rclcpp::Node & node, const std::string& motion_id,
                       JointNames& motion_joints)
  {
    MotionInfo info;
    getMotion(node, motion_id, info);
    motion_joints = info.joints;
  }

//  void getMotionJoints(const std::string& motion_id, JointNames& motion_joints)
//  {
//    ros::NodeHandle pm_nh("play_motion");
//    getMotionJoints(pm_nh, motion_id, motion_joints);
//  }

  void getMotionPoints(const rclcpp::Node & node, const std::string& motion_id,
                       Trajectory& motion_points)
  {
    MotionInfo info;
    getMotion(node, motion_id, info);
    motion_points = info.traj;
  }

//  void getMotionPoints(const std::string& motion_id, Trajectory& motion_points)
//  {
//    ros::NodeHandle pm_nh("play_motion");
//    getMotionPoints(pm_nh, motion_id, motion_points);
//  }

  void getMotionIds(const rclcpp::Node & node, MotionNames& motion_ids)
  {
    auto params = node.list_parameters({"motions"}, 0);

    std::unordered_set<std::string> unique_names;
    for(auto param_name : params.names)
    {
      // std::cout << param_name << std::endl;

      // find the motion name: after 'motions.' and before the next '.'
      auto init_position = std::string("motions.").size();
      const auto motion_name =
          param_name.substr(
            init_position,
            param_name.find_first_of('.', init_position) - init_position);
      unique_names.insert(motion_name);
    }

    for(auto param_name : unique_names)
    {
      // std::cout << "############" << std::endl;
      // std::cout << param_name << std::endl;
    }

    motion_ids.assign(unique_names.begin(), unique_names.end());
  }

//  void getMotionIds(MotionNames& motion_ids)
//  {
//    ros::NodeHandle pm_nh("play_motion");
//    getMotionIds(pm_nh, motion_ids);
//  }

  void populateVelocities(const TrajPoint& point_prev,
                          const TrajPoint& point_next,
                                TrajPoint& point_curr)
  {
    const int num_joints = point_curr.positions.size();
    if (num_joints != point_prev.positions.size() || num_joints != point_next.positions.size())
    {
      throw std::runtime_error("The positions array of a point of the trajectory does not have the same number of joints as the trajectory joint_names say.");
    }


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
        const double t_prev = (rclcpp::Duration(point_curr.time_from_start) - rclcpp::Duration(point_prev.time_from_start)).seconds();
        const double t_next = (rclcpp::Duration(point_next.time_from_start) - rclcpp::Duration(point_curr.time_from_start)).seconds();

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

  rclcpp::Duration getMotionDuration(const rclcpp::Node & node, const std::string &motion_id)
  {
    Trajectory traj;
    getMotionPoints(node, motion_id, traj);

    return traj.back().time_from_start;
  }

//  rclcpp::Duration getMotionDuration(const std::string &motion_id)
//  {
//      ros::NodeHandle pm_nh("play_motion");
//      return getMotionDuration(pm_nh, motion_id);
//  }

  bool motionExists(const rclcpp::Node & node, const std::string & motion_id)
  {
    return node.has_parameter("motions." + motion_id + ".joints") &&
           node.has_parameter("motions." + motion_id + ".positions") &&
           node.has_parameter("motions." + motion_id + ".times_from_start");
  }

//  bool motionExists(const std::string &motion_id)
//  {
//    ros::NodeHandle pm_nh("play_motion");
//    return motionExists(pm_nh, motion_id);
//  }

  bool isAlreadyThere(
    const JointNames & target_joints, const TrajPoint & target_point,
    const JointNames & source_joints, const TrajPoint & source_point,
    double tolerance)
  {
    if (target_joints.size() != target_point.positions.size()) {
      throw std::runtime_error("targetJoint and targetPoint positions sizes do not match");
    }

    if (source_joints.size() != source_point.positions.size()) {
      throw std::runtime_error("sourceJoint and sourcePoint positions sizes do not match");
    }

    for (auto i = 0ul; i < target_joints.size(); ++i) {
      JointNames::const_iterator it = std::find(
        source_joints.begin(),
        source_joints.end(), target_joints[i]);

      // if a joint used in the target is not used in the source
      // then can't guarantee that the points are equivalent
      if (it == source_joints.end()) {
        return false;
      }

      auto p_idx = static_cast<unsigned long>(it - source_joints.begin());
      if (std::fabs(target_point.positions[i] - source_point.positions[p_idx]) > tolerance) {
        return false;
      }
    }
    return true;
  }

  void getMotion(const rclcpp::Node & node, const std::string & motion_id, MotionInfo & motion_info)
  {
    if (!motionExists(node, motion_id)) {
      const std::string what = "Motion '" + motion_id + "' does not exist or is malformed " +
        "(namespace " + node.get_namespace() + ").";
      throw std::runtime_error(what);
    }

    const auto joints =
      node.get_parameter("motions." + motion_id + ".joints").as_string_array();
    const auto positions =
      node.get_parameter("motions." + motion_id + ".positions").as_double_array();
    const auto times =
      node.get_parameter("motions." + motion_id + ".times_from_start").as_double_array();

    // check trajectory is well formed
    const auto num_joints = joints.size();
    const auto num_positions = positions.size();
    const auto num_times = times.size();

    if (num_positions % num_joints != 0) {
      const std::string what = "Motion '" + motion_id + "' number of positions (" + std::to_string(
        num_positions) + ") is not a multiple of the number of joints (" +
        std::to_string(num_joints) + ")";
      throw std::runtime_error(what);
    }

    const auto num_points = num_positions / num_joints;
    if (num_points != num_times) {
      const std::string what = "Motion '" + motion_id + "' number of times_from_start (" +
        std::to_string(num_times) + ") does not match the number of points (" +
        std::to_string(num_points) + ")";
      throw std::runtime_error(what);
    }

    motion_info.id = motion_id;
    motion_info.joints = joints;

    // extract the trajectory
    motion_info.traj.clear();
    for (auto i = 0ul; i < num_points; ++i) {
      TrajPoint traj_point;

      traj_point.positions.insert(
        traj_point.positions.end(),
        positions.begin() + static_cast<long>(i * num_joints),
        positions.begin() + static_cast<long>((i + 1) * num_joints));

      /// @todo add velocities, effort, etc. too?

      traj_point.time_from_start = rclcpp::Duration::from_seconds(times[i]);

      motion_info.traj.emplace_back(traj_point);
    }

    // extract meta parameters
    auto get_meta_info = [&](const std::string & param_name, std::string & dst)
      {
        const std::string full_name = "motions." + motion_id + ".meta." + param_name;
        if (node.has_parameter(full_name)) {
          dst = node.get_parameter(full_name).as_string();
        } else {
          dst = "";
        }
      };

    get_meta_info("name", motion_info.name);
    get_meta_info("usage", motion_info.usage);
    get_meta_info("description", motion_info.description);
  }

//  void getMotion(const std::string &motion_id, MotionInfo &motion_info)
//  {
//    ros::NodeHandle pm_nh("play_motion");
//    play_motion::getMotion(pm_nh, motion_id, motion_info);
//  }

}
