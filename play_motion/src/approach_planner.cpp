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

#include <algorithm>
#include <cmath>
#include <string>

#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group.h>

#include <play_motion/approach_planner.h>
#include <play_motion/xmlrpc_helpers.h>

#define foreach BOOST_FOREACH
using std::string;
using std::vector;

namespace
{

typedef moveit::planning_interface::MoveGroup MoveGroup;
typedef boost::shared_ptr<MoveGroup> MoveGroupPtr;

/// \return Comma-separated list of planning groups.
string planningGroupsStr(const std::vector<MoveGroupPtr>& move_groups)
{
  string ret;
  foreach(MoveGroupPtr group, move_groups) {ret += group->getName() + ", ";}
  if (!ret.empty()) {ret.erase(ret.size() - 2);} // Remove last ", "
  return ret;
}

/// \return Comma-separated list of joints
string keysStr(const std::map<string, double>& in)
{
  string ret;
  typedef std::map<string, double> MapType;
  foreach(MapType::value_type val, in) {ret += val.first + ", ";}
  if (!ret.empty()) {ret.erase(ret.size() - 2);} // Remove last ", "
  return ret;
}

} // namespace

namespace play_motion
{

ApproachPlanner::ApproachPlanner(const ros::NodeHandle& nh)
  : joint_tol_(1e-3)
{
  ros::NodeHandle ap_nh(nh, "approach_planner");

  const string JOINT_TOL_STR          = "joint_tolerance";
  const string PLANNING_GROUPS_STR    = "planning_groups";
  const string NO_PLANNING_JOINTS_STR = "exclude_from_planning_joints";

  // Joint tolerance
  const bool joint_tol_ok = ap_nh.getParam(JOINT_TOL_STR, joint_tol_);
  if (joint_tol_ok) {ROS_DEBUG_STREAM("Using joint tolerance of " << joint_tol_);}
  else              {ROS_DEBUG_STREAM("Joint tolerance not specified. Using default value of " << joint_tol_);}

  // Joints excluded from motion planning
  using namespace XmlRpc;
  XmlRpcValue xml_no_plan_joints;
  ap_nh.getParam(NO_PLANNING_JOINTS_STR, xml_no_plan_joints);
  if (xml_no_plan_joints.getType() != XmlRpcValue::TypeArray)
  {
    const string what = "The '" + NO_PLANNING_JOINTS_STR + "' parameter is not an array (namespace: " +
                             ap_nh.getNamespace() + ").";
    throw ros::Exception(what);
  }
  no_plan_joints_.resize(xml_no_plan_joints.size());
  try
  {
    for (int i = 0; i < xml_no_plan_joints.size(); ++i) {xh::getArrayItem(xml_no_plan_joints, i, no_plan_joints_[i]);}
  }
  catch(const xh::XmlrpcHelperException& ex) {throw ros::Exception(ex.what());}

  // Planning group names
  using namespace XmlRpc;
  XmlRpcValue xml_planning_groups;
  if (!ap_nh.getParam(PLANNING_GROUPS_STR, xml_planning_groups))
  {
    const string what = "Unspecified planning groups for computing approach trajectories. Please set the '" +
                               PLANNING_GROUPS_STR + "' parameter (namespace: " + ap_nh.getNamespace() +  ").";
    throw ros::Exception(what);
  }
  if (xml_planning_groups.getType() != XmlRpcValue::TypeArray)
  {
    const string what = "The '" + PLANNING_GROUPS_STR + "' parameter is not an array (namespace: " +
                             ap_nh.getNamespace() + ").";
    throw ros::Exception(what);
  }
  vector<string> planning_groups(xml_planning_groups.size());
  try
  {
    for (int i = 0; i < xml_planning_groups.size(); ++i) {xh::getArrayItem(xml_planning_groups, i, planning_groups[i]);}
  }
  catch(const xh::XmlrpcHelperException& ex) {throw ros::Exception(ex.what());}

  spinner_.reset(new ros::AsyncSpinner(1)); // Async spinner is required by the move_group_interface
  spinner_->start();

  // Create move_group clients for each planning group
  for (int i = 0; i < planning_groups.size(); ++i)
  {
    MoveGroupPtr ptr(new MoveGroup(planning_groups[i])); // TODO: Timeout and retry, log feedback
    move_groups_.push_back(ptr);
  }

  for (int i = 0; i < planning_groups.size(); ++i)
  {
    ROS_ERROR_STREAM(planning_groups[i]);
  }
  ROS_ERROR(" ");
  for (int i = 0; i < no_plan_joints_.size(); ++i) {ROS_ERROR_STREAM(no_plan_joints_[i]);}

}

// TODO: Work directly with JointStates and JointTrajector messages?
bool ApproachPlanner::prependApproach(const vector<string>&    joint_names,
                                      const vector<double>&    current_pos,
                                      const vector<TrajPoint>& traj_in,
                                            vector<TrajPoint>& traj_out)
{
  // Empty trajectory. Nothing to do
  if (traj_in.empty())
  {
    ROS_DEBUG("Approach motion not needed: Input trajectory is empty.");
    traj_out = traj_in;
    return true;
  }

  const int joint_dim = traj_in.front().positions.size();

  // Preconditions
  if (joint_dim != joint_names.size())
  {
    ROS_ERROR("Can't compute approach trajectory: Size mismatch between joint names and input trajectory.");
    return false;
  }
  if (joint_dim != current_pos.size())
  {
    ROS_ERROR("Can't compute approach trajectory: Size mismatch between current joint positions and input trajectory.");
    return false;
  }

  // Set planning goal state. Ignore joints excluded from planning and joints already at the goal position
  JointGoal joint_goal;
  for (int i = 0; i < joint_dim; ++i)
  {
    if (isPlanningJoint(joint_names[i]) && std::abs(current_pos[i] - traj_in.front().positions[i]) > joint_tol_)
    {
      joint_goal[joint_names[i]] = traj_in.front().positions[i];
    }
  }

  // Optimization: no planning is required
  if (joint_goal.empty())
  {
    ROS_DEBUG("Approach motion not needed.");
    traj_out = traj_in;
    return true;
  }

  // Compute approach trajectory
  vector<MoveGroupPtr> valid_move_groups = getValidMoveGroups(joint_goal);
  trajectory_msgs::JointTrajectory approach;
  bool approach_ok = false;
  foreach(MoveGroupPtr move_group, valid_move_groups)
  {
    approach_ok = computeApproach(joint_goal, move_group, approach);
    if (approach_ok) {break;}
  }

  if (!approach_ok)
  {
    ROS_ERROR_STREAM("Failed to compute approach trajectory with planning groups: [" <<
                     planningGroupsStr(valid_move_groups) << "].");
    return false;
  }

  // Initialize output trajectory with approach
  foreach(const TrajPoint& point_appr, approach.points)
  {
    TrajPoint point;
    point.positions.resize(joint_dim, 0.0);
    if (!point_appr.velocities.empty())    {point.velocities.resize(joint_dim, 0.0);}
    if (!point_appr.accelerations.empty()) {point.accelerations.resize(joint_dim, 0.0);}
    point.time_from_start = point_appr.time_from_start;

    for (unsigned int i = 0; i < joint_dim; ++i)
    {
      const vector<string>& plan_joints = approach.joint_names;
      vector<string>::const_iterator approach_joints_it = find(plan_joints.begin(), plan_joints.end(), joint_names[i]);
      if (approach_joints_it != plan_joints.end())
      {
        // Joint is part of the planned approach
        const unsigned int approach_id = std::distance(plan_joints.begin(), approach_joints_it);
        point.positions[i] = point_appr.positions[approach_id];
        if (!point_appr.velocities.empty())    {point.velocities[i]    = point_appr.velocities[approach_id];}
        if (!point_appr.accelerations.empty()) {point.accelerations[i] = point_appr.accelerations[approach_id];}
      }
      else
      {
        // Joint is not part of the planning group, and hence not contained in the approach plan
        // Default to linear interpolation TODO: Use spline interpolator
        const double t_min = 0.0;
        const double t_max = approach.points.back().time_from_start.toSec();
        const double t     = point_appr.time_from_start.toSec();

        const double p_min = current_pos[i];
        const double p_max = traj_in.front().positions[i];

        const double vel = (p_max - p_min) / (t_max - t_min);

        point.positions[i] = p_min + vel * t;
        if (!point_appr.velocities.empty())    {point.velocities[i]    = vel;}
        if (!point_appr.accelerations.empty()) {point.accelerations[i] = 0.0;}
      }

      // TODO: What happens to joints in the motion plan but _not_ in the input motion?!
    }

    traj_out.push_back(point);
  }

  // If input trajectory is a single point, the approach trajectory is all there is to execute...
  if (1 == traj_in.size()) {return true;}

  // ...otherwise, append input_trajectory after approach:

  // Time offset to apply to input trajectory (approach duration)
  const ros::Duration offset = traj_out.back().time_from_start;

  // Remove duplicate waypoint: Position of last approach point coincides with the input's first point
  traj_out.pop_back();

  // Append input trajectory to approach
  foreach(const TrajPoint& point, traj_in)
  {
    traj_out.push_back(point);
    traj_out.back().time_from_start += offset;
  }

  return true;
}

vector<ApproachPlanner::MoveGroupPtr> ApproachPlanner::getValidMoveGroups(const JointGoal& joint_goal)
{
  vector<MoveGroupPtr> valid_groups;

  foreach(MoveGroupPtr group, move_groups_)
  {
    const vector<string>& group_joints = group->getJoints();
    bool valid_group = true;

    foreach(const JointGoal::value_type& data, joint_goal)
    {
      if (std::find(group_joints.begin(), group_joints.end(), data.first) == group_joints.end())
      {
        valid_group = false;
        break;
      }
    }
    if (valid_group)
    {
      valid_groups.push_back(group);
    }
  }

  if (valid_groups.empty())
  {
    ROS_ERROR_STREAM("Can't compute approach trajectory. There are no planning groups that span the requested joints: ["
                     << keysStr(joint_goal) << "].");
  }
  else
  {
    ROS_ERROR_STREAM("Approach trajectory can be computed by the following groups: "
                     << planningGroupsStr(valid_groups) << "."); // TODO: Make debug
  }

  return valid_groups;
}

bool ApproachPlanner::computeApproach(const JointGoal&                 joint_goal,
                                     MoveGroupPtr                      move_group,
                                     trajectory_msgs::JointTrajectory& traj)
{
  move_group->setStartStateToCurrentState();
  const bool set_goal_ok = move_group->setJointValueTarget(joint_goal);
  if (!set_goal_ok)
  {
    ROS_ERROR_STREAM("Failed to set motion planning problem goal for group '" << move_group->getName() << "'.");
    return false;
  }
  move_group_interface::MoveGroup::Plan plan;
  const bool planning_ok = move_group->plan(plan);
  if (!planning_ok)
  {
    ROS_DEBUG_STREAM("Could not compute approach trajectory with planning group '" << move_group->getName() << "'.");
    return false;
  }
  if (plan.trajectory_.joint_trajectory.points.empty())
  {
    ROS_ERROR_STREAM("Unexpected error: Approach trajectory computed by group '" << move_group->getName() <<
                     "' is empty.");
    return false;
  }

  traj = plan.trajectory_.joint_trajectory;
  ROS_ERROR_STREAM("Successfully computed approach with planning group '" << move_group->getName() << "'."); // TODO: Make DEBUG
  return true;
}

bool ApproachPlanner::isPlanningJoint(const string& joint_name) const
{
  return std::find(no_plan_joints_.begin(), no_plan_joints_.end(), joint_name) == no_plan_joints_.end();
}

} // namesapce
