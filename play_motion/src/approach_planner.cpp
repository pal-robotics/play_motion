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
#include <cassert>
#include <cmath>
#include <sstream>
#include <string>

#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <play_motion/approach_planner.h>
#include <play_motion/xmlrpc_helpers.h>

#define foreach BOOST_FOREACH
using std::string;
using std::vector;

namespace
{

/// \return Comma-separated list of container elements.
template <class T>
string enumerateElementsStr(const T& val)
{
  std::stringstream ss;
  std::copy(val.begin(), val.end(), std::ostream_iterator<typename T::value_type>(ss, ", "));
  string ret = ss.str();
  if (!ret.empty()) {ret.erase(ret.size() - 2);} // Remove last ", "
  return ret;
}

typedef moveit::planning_interface::MoveGroupInterface MoveGroupInterface;
typedef boost::shared_ptr<MoveGroupInterface> MoveGroupInterfacePtr;

/// \return Comma-separated list of planning groups.
string enumeratePlanningGroups(const std::vector<MoveGroupInterfacePtr>& move_groups)
{
  string ret;
  foreach(MoveGroupInterfacePtr group, move_groups) {ret += group->getName() + ", ";}
  if (!ret.empty()) {ret.erase(ret.size() - 2);} // Remove last ", "
  return ret;
}

} // namespace

namespace play_motion
{

ApproachPlanner::PlanningData::PlanningData(MoveGroupInterfacePtr move_group_ptr)
  : move_group(move_group_ptr),
    sorted_joint_names(move_group_ptr->getActiveJoints())
{
  std::sort(sorted_joint_names.begin(), sorted_joint_names.end());
}

ApproachPlanner::ApproachPlanner(const ros::NodeHandle& nh)
  : joint_tol_(1e-3),
    skip_planning_vel_(0.5),
    skip_planning_min_dur_(0.0),
    planning_disabled_(false)
{
  ros::NodeHandle ap_nh(nh, "approach_planner");

  const string JOINT_TOL_STR             = "joint_tolerance";
  const string PLANNING_GROUPS_STR       = "planning_groups";
  const string NO_PLANNING_JOINTS_STR    = "exclude_from_planning_joints";
  const string SKIP_PLANNING_VEL_STR     = "skip_planning_approach_vel";
  const string SKIP_PLANNING_MIN_DUR_STR = "skip_planning_approach_min_dur";

  // Velocity used in non-planned approaches
  const bool skip_planning_vel_ok = ap_nh.getParam(SKIP_PLANNING_VEL_STR, skip_planning_vel_);
  if (skip_planning_vel_ok) {ROS_DEBUG_STREAM("Using a max velocity of " << skip_planning_vel_ <<
                                              " for unplanned approaches.");}
  else                      {ROS_DEBUG_STREAM("Max velocity for unplanned approaches not specified. " <<
                                              "Using default value of " << skip_planning_vel_);}

  // Minimum duration used in non-planned approaches
  const bool skip_planning_min_dur_ok = ap_nh.getParam(SKIP_PLANNING_MIN_DUR_STR, skip_planning_min_dur_);
  if (skip_planning_min_dur_ok) {ROS_DEBUG_STREAM("Using a min duration of " << skip_planning_min_dur_ <<
                                              " for unplanned approaches.");}
  else                      {ROS_DEBUG_STREAM("Min duration for unplanned approaches not specified. " <<
                                              "Using default value of " << skip_planning_min_dur_);}

  // Initialize motion planning capability, unless explicitly disabled
  nh.getParam("disable_motion_planning", planning_disabled_);
  if (planning_disabled_)
  {
    ROS_WARN_STREAM("Motion planning capability disabled. Goals requesting planning (the default) will be rejected.\n"
                    << "To disable planning in goal requests set 'skip_planning=true'");
    return; // Skip initialization of planning-related members
  }

  // Joint tolerance
  const bool joint_tol_ok = ap_nh.getParam(JOINT_TOL_STR, joint_tol_);
  if (joint_tol_ok) {ROS_DEBUG_STREAM("Using joint tolerance of " << joint_tol_);}
  else              {ROS_DEBUG_STREAM("Joint tolerance not specified. Using default value of " << joint_tol_);}

  // Joints excluded from motion planning
  using namespace XmlRpc;
  XmlRpcValue xml_no_plan_joints;
  const bool xml_no_plan_joints_ok = ap_nh.getParam(NO_PLANNING_JOINTS_STR, xml_no_plan_joints);
  if (xml_no_plan_joints_ok)
  {
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
  }

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

  // Move group instances require their own spinner thread. To isolate this asynchronous spinner from the rest of the
  // node, it is set up in a node handle with a custom callback queue
  ros::NodeHandle as_nh;
  cb_queue_.reset(new ros::CallbackQueue());
  as_nh.setCallbackQueue(cb_queue_.get());
  spinner_.reset(new ros::AsyncSpinner(1, cb_queue_.get()));
  spinner_->start();

  // Populate planning data
  foreach (const string& planning_group, planning_groups)
  {
    MoveGroupInterface::Options opts(planning_group);
    opts.node_handle_ = as_nh;
    MoveGroupInterfacePtr move_group(new MoveGroupInterface(opts)); // TODO: Timeout and retry, log feedback. Throw on failure
    planning_data_.push_back(PlanningData(move_group));
  }
}

// TODO: Work directly with JointStates and JointTrajector messages?
bool ApproachPlanner::prependApproach(const JointNames&        joint_names,
                                      const vector<double>&    current_pos,
                                      bool                     skip_planning,
                                      const vector<TrajPoint>& traj_in,
                                            vector<TrajPoint>& traj_out)
{
  // TODO: Instead of returning false, raise exceptions, so error message can be forwarded to goal result

  // Empty trajectory. Nothing to do
  if (traj_in.empty())
  {
    ROS_DEBUG("Approach motion not needed: Input trajectory is empty.");
    traj_out = traj_in;
    return true;
  }

  const unsigned int joint_dim = traj_in.front().positions.size();

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
  if (!skip_planning && planning_disabled_) // Reject goal if plannign is disabled, but goal requests it
  {
    ROS_ERROR("Motion planning capability disabled. To disable planning in goal requests, set 'skip_planning=true'");
    return false;
  }

  if (skip_planning)
  {
    // Skip motion planning altogether
    traj_out = traj_in;

    // If the first waypoint specifies zero time from start, set a duration that does not exceed a specified
    // max avg velocity
    if (traj_out.front().time_from_start.isZero())
    {
      const double reach_time = noPlanningReachTime(current_pos, traj_out.front().positions);
      foreach(TrajPoint& point, traj_out) {point.time_from_start += ros::Duration(reach_time);}
    }
  }
  else
  {
    // Compute approach trajectory using motion planning
    trajectory_msgs::JointTrajectory approach;
    const bool approach_ok = computeApproach(joint_names,
                                             current_pos,
                                             traj_in.front().positions,
                                             approach);

    // If the first waypoint specifies non-zero time from start and if it is more than the
    // trajectory time then try to scale up the way points to fit the requested trajectory
    // time
    // https://github.com/ros-planning/moveit_ros/issues/368#issuecomment-29717359
    if (!approach.points.empty() && !traj_in.front().time_from_start.isZero())
    {
      const double min_time = traj_in.front().time_from_start.toSec();
      const double factor = min_time / approach.points.back().time_from_start.toSec();
      // Only scale the trajectory if it is faster than expected
      if (factor > 1.0)
      {
        for (trajectory_msgs::JointTrajectoryPoint& waypoint : approach.points)
        {
          waypoint.time_from_start.fromSec(waypoint.time_from_start.toSec() * factor);
          for (size_t i = 0; i < waypoint.velocities.size(); i++)
          {
            waypoint.velocities[i] /= factor;
            waypoint.accelerations[i] /= factor * factor;
          }
        }
      }
    }

    if (!approach_ok) {return false;}

    // No approach is required
    if (approach.points.empty())
    {
      traj_out = traj_in;
      ROS_INFO("Approach motion not needed.");
    }
    else
    {
      // Combine approach and input motion trajectories
      combineTrajectories(joint_names,
                          current_pos,
                          traj_in,
                          approach,
                          traj_out);
    }
  }

  // Deal with first waypoints specifying zero time from start. Two cases can happen:
  // 1. If at least one joint is not at its destination, compute an appropriate reach time
  const double eps_time = 1e-3; // NOTE: Magic number
  if (traj_out.front().time_from_start.isZero())
  {
    const double reach_time = noPlanningReachTime(current_pos, traj_out.front().positions);
    if (reach_time > eps_time)
    {
      foreach(TrajPoint& point, traj_out) {point.time_from_start += ros::Duration(reach_time);}
    }
  }
  // 2 . First waypoint corresponds to current state: Make the first time_from_start a small nonzero value.
  // Rationale: Sending a waypoint with zero time from start will make the controllers complain with a warning, and
  // rightly so, because in general it's impossible to reach a point in zero time.
  // This avoids unsavory warnings that might confuse users.
  if (traj_out.front().time_from_start.isZero()) // If still zero it's because previous step yield zero time
  {
    traj_out.front().time_from_start = ros::Duration(eps_time);
  }

  return true;
}

bool ApproachPlanner::needsApproach(const std::vector<double>& current_pos,
                                    const std::vector<double>& goal_pos)
{
  assert(current_pos.size() == goal_pos.size());
  for (unsigned int i = 0; i < current_pos.size(); ++i)
  {
    if (std::abs(current_pos[i] - goal_pos[i]) > joint_tol_) return true;
  }
  return false;
}

bool ApproachPlanner::computeApproach(const vector<string>&             joint_names,
                                      const vector<double>&             current_pos,
                                      const vector<double>&             goal_pos,
                                      trajectory_msgs::JointTrajectory& traj)
{
  traj.joint_names.clear();
  traj.points.clear();

  // Maximum set of joints that a planning group can have. Corresponds to the original motion joints minus the joints
  // excluded from planning. Planning groups eligible to compute the approach can't contain joints outside this set.
  JointNames max_planning_group;

  // Joint positions associated to the maximum set
  vector<double> max_planning_values;

  // Minimum set of joints that a planning group can have. Corresponds to the maximum set minus the joints that are
  // already at their goal configuration. If this set is empty, no approach is required, i.e. all motion joints are
  // either excluded from planning or already at the goal.
  JointNames min_planning_group;

  for (unsigned int i = 0; i < joint_names.size(); ++i)
  {
    if (isPlanningJoint(joint_names[i]))
    {
      max_planning_group.push_back(joint_names[i]);
      max_planning_values.push_back(goal_pos[i]);
      if (std::abs(current_pos[i] - goal_pos[i]) > joint_tol_) {min_planning_group.push_back(joint_names[i]);}
    }
  }

  // No planning is required, return empty trajectory
  if (min_planning_group.empty()) {return true;}

  // Find planning groups that are eligible for computing this particular approach trajectory
  vector<MoveGroupInterfacePtr> valid_move_groups = getValidMoveGroups(min_planning_group, max_planning_group);
  if (valid_move_groups.empty())
  {
    ROS_ERROR_STREAM("Can't compute approach trajectory. There are no planning groups that span at least these joints:"
                     << "\n[" << enumerateElementsStr(min_planning_group) << "]\n" << "and at most these joints:"
                     << "\n[" << enumerateElementsStr(max_planning_group) << "].");
    return false;
  }
  else
  {
    ROS_INFO_STREAM("Approach motion can be computed by the following groups: "
                     << enumeratePlanningGroups(valid_move_groups) << ".");
  }

  // Call motion planners
  bool approach_ok = false;
  foreach(MoveGroupInterfacePtr move_group, valid_move_groups)
  {
    approach_ok = planApproach(max_planning_group, max_planning_values, move_group, traj);
    if (approach_ok) {break;}
  }

  if (!approach_ok)
  {
    ROS_ERROR_STREAM("Failed to compute approach trajectory with planning groups: [" <<
                     enumeratePlanningGroups(valid_move_groups) << "].");
    return false;
  }

  return true;
}

bool ApproachPlanner::planApproach(const JointNames&                 joint_names,
                                   const std::vector<double>&        joint_values,
                                   MoveGroupInterfacePtr             move_group,
                                   trajectory_msgs::JointTrajectory& traj)
{
  move_group->setStartStateToCurrentState();
  for (unsigned int i = 0; i < joint_names.size(); ++i)
  {
    const bool set_goal_ok = move_group->setJointValueTarget(joint_names[i], joint_values[i]);
    if (!set_goal_ok)
    {
      ROS_ERROR_STREAM("Failed attempt to set planning goal for joint '" << joint_names[i] << "' on group '" <<
                       move_group->getName() << "'.");
      return false;
    }
  }
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const moveit::planning_interface::MoveItErrorCode planning_ok = move_group->plan(plan);
  if (!(planning_ok == moveit::planning_interface::MoveItErrorCode::SUCCESS))
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
  ROS_INFO_STREAM("Successfully computed approach with planning group '" << move_group->getName() << "'.");
  return true;
}

void ApproachPlanner::combineTrajectories(const JointNames&                  joint_names,
                                          const std::vector<double>&         current_pos,
                                          const std::vector<TrajPoint>&      traj_in,
                                          trajectory_msgs::JointTrajectory&  approach,
                                          std::vector<TrajPoint>&            traj_out)
{
  const unsigned int joint_dim = traj_in.front().positions.size();

  foreach(const TrajPoint& point_appr, approach.points)
  {
    const bool has_velocities    = !point_appr.velocities.empty();
    const bool has_accelerations = !point_appr.accelerations.empty();
    TrajPoint point;
    point.positions.resize(joint_dim, 0.0);
    if (has_velocities)    {point.velocities.resize(joint_dim, 0.0);}
    if (has_accelerations) {point.accelerations.resize(joint_dim, 0.0);}
    point.time_from_start = point_appr.time_from_start;

    for (unsigned int i = 0; i < joint_dim; ++i)
    {
      const JointNames& plan_joints = approach.joint_names;
      JointNames::const_iterator approach_joints_it = find(plan_joints.begin(), plan_joints.end(), joint_names[i]);
      if (approach_joints_it != plan_joints.end())
      {
        // Joint is part of the planned approach
        const unsigned int approach_id = std::distance(plan_joints.begin(), approach_joints_it);
        point.positions[i] = point_appr.positions[approach_id];
        if (has_velocities)    {point.velocities[i]    = point_appr.velocities[approach_id];}
        if (has_accelerations) {point.accelerations[i] = point_appr.accelerations[approach_id];}
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
        if (has_velocities)    {point.velocities[i]    = vel;}
        if (has_accelerations) {point.accelerations[i] = 0.0;}
      }
    }

    traj_out.push_back(point);
  }

  // If input trajectory is a single point, the approach trajectory is all there is to execute...
  if (1 == traj_in.size()) {return;}

  // ...otherwise, append input_trajectory after approach:

  // Time offset to apply to input trajectory (approach duration)
  const ros::Duration offset = traj_out.back().time_from_start - traj_in.front().time_from_start;

  // Remove duplicate waypoint: Position of last approach point coincides with the input's first point
  traj_out.pop_back();

  // Append input trajectory to approach
  foreach(const TrajPoint& point, traj_in)
  {
    traj_out.push_back(point);
    traj_out.back().time_from_start += offset;
  }
}

vector<ApproachPlanner::MoveGroupInterfacePtr> ApproachPlanner::getValidMoveGroups(const JointNames& min_group,
                                                                          const JointNames& max_group)
{
  vector<MoveGroupInterfacePtr> valid_groups;

  // Create sorted ranges of min/max planning groups
  JointNames min_group_s = min_group;
  JointNames max_group_s = max_group;
  std::sort(min_group_s.begin(), min_group_s.end());
  std::sort(max_group_s.begin(), max_group_s.end());

  foreach(const PlanningData& data, planning_data_)
  {
    const JointNames& group_s = data.sorted_joint_names;

    // A valid planning group is one that has the minimum group as a subset, and is a subset of the maximum group
    if (std::includes(group_s.begin(), group_s.end(), min_group_s.begin(), min_group_s.end()) &&
        std::includes(max_group_s.begin(), max_group_s.end(), group_s.begin(), group_s.end()))
    {
      valid_groups.push_back(data.move_group);
    }
  }
  return valid_groups;
}

bool ApproachPlanner::isPlanningJoint(const string& joint_name) const
{
  return std::find(no_plan_joints_.begin(), no_plan_joints_.end(), joint_name) == no_plan_joints_.end();
}

double ApproachPlanner::noPlanningReachTime(const std::vector<double>& curr_pos,
                                            const std::vector<double>& goal_pos)
{
  double dmax = 0.0; // Maximum joint displacement
  for (unsigned int i = 0; i < curr_pos.size(); ++i)
  {
    const double d = std::abs(goal_pos[i] - curr_pos[i]);
    if (d > dmax)
      dmax = d;
  }
  return std::max(dmax / skip_planning_vel_, skip_planning_min_dur_);
}

} // namesapce
