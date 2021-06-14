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

#ifndef PLAY_MOTION_APPROACH_PLANNER_H
#define PLAY_MOTION_APPROACH_PLANNER_H

#include <map>
#include <memory>
#include <vector>

#include "play_motion/datatypes.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/logger.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include "moveit/move_group_interface/move_group_interface.h"

namespace play_motion
{
/// TODO
class ApproachPlanner
{
public:
  ApproachPlanner(const rclcpp::Node::SharedPtr & node);

  /// TODO
  bool prependApproach(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & current_pos,
    bool skip_planning,
    const std::vector<TrajPoint> & traj_in,
    std::vector<TrajPoint> & traj_out);

  /// TODO
  bool needsApproach(
    const std::vector<double> & current_pos,
    const std::vector<double> & goal_pos);

private:
  using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
  using MoveGroupInterfacePtr = moveit::planning_interface::MoveGroupInterfacePtr;
  // typedef std::shared_ptr<ros::AsyncSpinner> AsyncSpinnerPtr;
  // typedef std::shared_ptr<ros::CallbackQueue> CallbackQueuePtr;
  typedef std::vector<std::string> JointNames;
  typedef std::map<std::string, double> JointGoal;

  struct PlanningData
  {
    PlanningData(MoveGroupInterfacePtr move_group_ptr);
    MoveGroupInterfacePtr move_group;
    JointNames sorted_joint_names;
  };

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  std::vector<PlanningData> planning_data_;
  std::vector<std::string> no_plan_joints_;
  double joint_tol_;   ///< Absolute tolerance used to determine if two joint positions are approximately equal.
  double skip_planning_vel_;   ///< Maximum average velocity that any joint can have in a non-planned approach.
  double skip_planning_min_dur_;   ///< Minimum duration that a non-planned approach can have
  // CallbackQueuePtr cb_queue_;
  // AsyncSpinnerPtr spinner_;
  bool planning_disabled_;

  /// TODO
  bool computeApproach(
    const JointNames & joint_names,
    const std::vector<double> & current_pos,
    const std::vector<double> & goal_pos,
    trajectory_msgs::msg::JointTrajectory & traj);

  /// TODO
  bool planApproach(
    const JointNames & joint_names,
    const std::vector<double> & joint_values,
    MoveGroupInterfacePtr move_group,
    trajectory_msgs::msg::JointTrajectory & traj);
  /// TODO
  void combineTrajectories(
    const JointNames & joint_names,
    const std::vector<double> & current_pos,
    const std::vector<TrajPoint> & traj_in,
    trajectory_msgs::msg::JointTrajectory & approach,
    std::vector<TrajPoint> & traj_out);

  /// TODO
  std::vector<MoveGroupInterfacePtr> getValidMoveGroups(
    const JointNames & min_group,
    const JointNames & max_group);

  /// TODO
  bool isPlanningJoint(const std::string & joint_name) const;

  /// TODO
  double noPlanningReachTime(
    const std::vector<double> & curr_pos,
    const std::vector<double> & goal_pos);
};

} // namespace

#endif
