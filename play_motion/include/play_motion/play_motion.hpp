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

#ifndef REACHPOSE_H
#define REACHPOSE_H

#include <exception>
#include <functional>
#include <list>
#include <memory>
#include <map>
#include <string>

#include "play_motion/datatypes.hpp"
#include "play_motion/controller_updater.hpp"
#include "play_motion_msgs/action/play_motion.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include "std_srvs/srv/trigger.hpp"

namespace play_motion
{
using PlayMotionResult = play_motion_msgs::action::PlayMotion::Result;

class MoveJointGroup;
class ApproachPlanner;

class PlayMotionException : public std::runtime_error
{
public:
  PlayMotionException(const std::string & error_msg, int error_code = PlayMotionResult::OTHER_ERROR)
  : std::runtime_error(error_msg), error_code_(error_code)
  {}

  int error_code() const
  {return error_code_;}

private:
  int error_code_;
};

/** Move robot joints to a given pose.
 * Poses are specified in the parameter server, and are identified by name.
 */
class PlayMotion : public rclcpp::Node
{
public:
  class Goal;
  using GoalHandle = std::shared_ptr<Goal>;

private:
  using ApproachPlannerPtr = std::shared_ptr<ApproachPlanner>;
  using ControllerUpdaterPtr = std::shared_ptr<ControllerUpdater>;
  using MoveJointGroupPtr = std::shared_ptr<MoveJointGroup>;
  using ControllerList = std::list<MoveJointGroupPtr>;
  using Callback = std::function<void (const GoalHandle &)>;
  using IsReadyService = std_srvs::srv::Trigger;

public:
  class Goal
  {
    friend class PlayMotion;

public:
    int error_code;
    std::string error_string;
    int active_controllers;
    Callback cb;
    ControllerList controllers;
    bool canceled;

    ~Goal();
    void cancel();
    void addController(const MoveJointGroupPtr & ctrl);

private:
    Goal(const Callback & cbk);
  };

  PlayMotion();

  void init();

  /// \brief Send motion goal request
  /// \param motion_name Name of motion to execute.
  /// \param skip_planning Skip motion planning for computing the approach trajectory.
  /// \param[out] goal_id contains the goal ID if function returns true
  bool run(
    const std::string & motion_name,
    bool skip_planning,
    GoalHandle & gh,
    const Callback & cb);

  void is_ready(
    const IsReadyService::Request::SharedPtr request,
    IsReadyService::Response::SharedPtr response);

private:
  void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg);

  bool getGroupTraj(
    MoveJointGroupPtr move_joint_group,
    const JointNames & motion_joints,
    const Trajectory & motion_points, Trajectory & traj_group);
  void getMotionJoints(const std::string & motion_name, JointNames & motion_joints);
  void getMotionPoints(const std::string & motion_name, Trajectory & motion_points);

  /// \brief Populate a list of controllers that span the motion joints.
  ///
  /// In the general case, the controllers will span more than the motion joints, but never less.
  /// This method also validates that the controllers are not busy executing another goal.
  /// \param motion_joints List of motion joints.
  /// \return A list of controllers that span (at least) all the motion joints.
  /// \throws PMException if no controllers spanning the motion joints were found, or if some of them are busy.
  ControllerList getMotionControllers(const JointNames & motion_joints);
  void updateControllersCb(
    const ControllerUpdater::ControllerStates & states,
    const ControllerUpdater::ControllerJoints & joints);

  ControllerList move_joint_groups_;
  std::map<std::string, double> joint_states_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  ControllerUpdaterPtr ctrlr_updater_;
  ApproachPlannerPtr approach_planner_;
  rclcpp::Service<IsReadyService>::SharedPtr is_ready_srv_;
};
}

#endif
