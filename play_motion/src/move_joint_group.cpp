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

#include <functional>

#include "play_motion_msgs/action/play_motion.hpp"
#include "play_motion/move_joint_group.hpp"

#include "rclcpp_action/create_client.hpp"
#include "rclcpp/logging.hpp"

#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

namespace play_motion
{
MoveJointGroup::MoveJointGroup(
  const rclcpp::Node::SharedPtr & node,
  const std::string & controller_name, const JointNames & joint_names)
: busy_(false),
  node_(node),
  logger_(node->get_logger().get_child("move_joint_group")),
  controller_name_(controller_name),
  joint_names_(joint_names),
  client_(),
  last_result_code_(rclcpp_action::ResultCode::UNKNOWN)
{
  client_ = rclcpp_action::create_client<FollowJointTrajectory>(
    node_,
    controller_name_ +
    "/follow_joint_trajectory");
}

void MoveJointGroup::resultCallback(const GoalHandleFollowJointTrajectory::WrappedResult & result)
{
  busy_ = false;
  last_result_code_ = result.code;
  active_cb_(result.result->error_code);
  active_cb_ = nullptr;
}

bool MoveJointGroup::isIdle() const
{
  return !busy_;
}

void MoveJointGroup::cancel()
{
  busy_ = false;
  auto cancel_future = client_->async_cancel_all_goals();
  cancel_future.wait();
}

void MoveJointGroup::abort()
{
  if (busy_) {
    auto cancel_future = client_->async_cancel_all_goals();
    cancel_future.wait();
  }
  if (active_cb_) {
    active_cb_(play_motion_msgs::action::PlayMotion_Result::OTHER_ERROR);
  }
}

void MoveJointGroup::setCallback(const Callback & cb)
{
  active_cb_ = cb;
}

const JointNames & MoveJointGroup::getJointNames() const
{
  return joint_names_;
}

int8_t MoveJointGroup::getState()
{
  return static_cast<int8_t>(last_result_code_);
}

const std::string & MoveJointGroup::getName() const
{
  return controller_name_;
}

bool MoveJointGroup::isControllingJoint(const std::string & joint_name)
{
  if (!client_->wait_for_action_server()) {
    RCLCPP_ERROR_STREAM(logger_, "action server not available");
    return false;
  }

  for (const std::string & jn : joint_names_) {
    if (joint_name == jn) {
      return true;
    }
  }

  return false;
}

bool MoveJointGroup::sendGoal(const std::vector<TrajPoint> & traj)
{
  if (!client_->wait_for_action_server()) {
    RCLCPP_ERROR_STREAM(logger_, "action server not available");
    return false;
  }

  RCLCPP_INFO_STREAM(logger_, "Sending trajectory goal to " << controller_name_ << ".");

  ActionGoal goal;
  goal.trajectory.joint_names = joint_names_;
  goal.trajectory.points.reserve(traj.size());

  for (const TrajPoint & p : traj) {
    if (p.positions.size() != joint_names_.size()) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Pose size mismatch. Expected: " << joint_names_.size() << ", got: " <<
          p.positions.size() << ".");
      return false;
    }
    trajectory_msgs::msg::JointTrajectoryPoint point;

    point.positions = p.positions;               // Reach these joint positions...
    point.velocities = p.velocities;             // ...with a given velocity (may be unset)
    point.accelerations = p.accelerations;       // ...and acceleration      (may be unset)

    point.time_from_start = p.time_from_start;   // ...in this time

    goal.trajectory.points.push_back(point);
  }

  using namespace std::placeholders;

  // set up goal options
  auto goal_options = ActionClient::SendGoalOptions();
  goal_options.result_callback =
    std::bind(&MoveJointGroup::resultCallback, this, _1);

  /// @todo replace this busy_ with the state of the goal_future?
  busy_ = true;

  last_result_code_ = rclcpp_action::ResultCode::UNKNOWN;
  goal_future_ = client_->async_send_goal(goal, goal_options);

  return true;
}
}
