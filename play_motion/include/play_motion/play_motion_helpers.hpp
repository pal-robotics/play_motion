// Copyright 2021 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/** \author Adolfo Rodriguez Tsouroukdissian. */
/** \author Paul Mathieu.                     */
/** \author Victor Lopez.                     */
/** \author Bence Magyar.                     */

#ifndef PLAYMOTIONHELPERS_H
#define PLAYMOTIONHELPERS_H

#include <string>

#include "play_motion/datatypes.hpp"

#include "rclcpp/duration.hpp"
#include "rclcpp/node.hpp"

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
void getMotionJoints(
  const rclcpp::Node * node, const std::string & motion_id,
  JointNames & motion_joints);

// void getMotionJoints(const std::string& motion_id, JointNames& motion_joints);


/**
 * \param nh Nodehandle with the namespace containing the motions
 *           (When omitted, defaults to ros::NodeHandle nh("play_motion"))
 * \throws xh::XmlrpcHelperException if motion_id cannot be found
 */
void getMotionPoints(
  const rclcpp::Node * node, const std::string & motion_id,
  Trajectory & motion_points);

// void getMotionPoints(const std::string& motion_id, Trajectory& motion_points);


/**
 * \brief getMotionDuration gets the total duration of a motion
 * \throws xh::XmlrpcHelperException if motion_id cannot be found
 */
rclcpp::Duration getMotionDuration(
  const rclcpp::Node * node,
  const std::string & motion_id);

/**
 * \brief getMotions obtain all motion names
 * \param nh Nodehandle with the namespace containing the motions
 *           (When omitted, defaults to ros::NodeHandle nh("play_motion"))
 * \throws xh::XmlrpcHelperException if no motions available
 */
void getMotionIds(const rclcpp::Node * node, MotionNames & motion_ids);

// void getMotionIds(MotionNames& motion_ids);

/**
 * \param nh Nodehandle with the namespace containing the motions
 *           (When omitted, defaults to ros::NodeHandle nh("play_motion"))
 * \param motion_id Motion identifier
 * \return True if the motion exists, false otherwise
 */
bool motionExists(const rclcpp::Node * node, const std::string & motion_id);

// bool motionExists(const std::string &motion_id);

/**
 * \brief isAlreadyThere checks if the source trajPoint matches the target
 *        trajPoint with a certain tolerance
 *        only the joints in targetJoint will be checked
 * \param tolerance tolerance per joint in radians
 */
bool isAlreadyThere(
  const JointNames & target_joints, const TrajPoint & target_point,
  const JointNames & source_joints, const TrajPoint & source_point,
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
void populateVelocities(
  const TrajPoint & point_prev,
  const TrajPoint & point_next,
  TrajPoint & point_curr);

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
void populateVelocities(const Trajectory & traj_in, Trajectory & traj_out);

/**
 * \brief Parse a motion specified in the ROS parameter server into a data structure.
 * \param[in] nh Nodehandle with the namespace containing the motions
 *           (When omitted, defaults to ros::NodeHandle nh("play_motion"))
 * \param[in] motion_id Motion identifier
 * \param[out] motionInfo Data structure containing parsed motion
 * \throws ros::Exception if the motion does not exist or is malformed.
 */
void getMotion(
  const rclcpp::Node * node, const std::string & motion_id,
  MotionInfo & motion_info);

// void getMotion(const std::string &motion_id, MotionInfo &motion_info);
}

#endif
