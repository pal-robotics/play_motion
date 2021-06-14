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

/** \author Paul Mathieu. */

#include "play_motion/play_motion.hpp"

#include <cassert>
#include <functional>
#include <list>
#include <memory>
#include <stdexcept>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "play_motion/approach_planner.hpp"
#include "play_motion/move_joint_group.hpp"
#include "play_motion/play_motion_helpers.hpp"

#include "rclcpp/logging.hpp"
#include "rclcpp/node_options.hpp"


namespace
{
typedef play_motion::PlayMotion::GoalHandle GoalHandle;
using MoveJointGroupPtr = std::shared_ptr<play_motion::MoveJointGroup>;
using MoveJointGroupWeakPtr = std::weak_ptr<play_motion::MoveJointGroup>;
using ControllerList = std::list<MoveJointGroupPtr>;
using play_motion::PlayMotionResult;
using rclcpp_action::GoalStatus;

void generateErrorCode(GoalHandle goal_hdl, int error_code, int8_t /*ctrl_state*/)
{
  using FollowJointTrajectoryResult = control_msgs::action::FollowJointTrajectory::Result;

  switch (error_code) {
    case FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED:
      goal_hdl->error_code = PlayMotionResult::TRAJECTORY_ERROR;
      break;
    case FollowJointTrajectoryResult::GOAL_TOLERANCE_VIOLATED:
      goal_hdl->error_code = PlayMotionResult::GOAL_NOT_REACHED;
      break;
    default:
      std::ostringstream os;
      goal_hdl->error_code = PlayMotionResult::OTHER_ERROR;
      os << "Got error code " << error_code << ", motion aborted.";
      goal_hdl->error_string = os.str();
  }
  //TODO: add handling for controller state
}

void controllerCb(
  rclcpp::Logger & logger, int error_code, GoalHandle goal_hdl,
  const MoveJointGroupWeakPtr & weak_ctrl)
{
  // If we don't use a weak ptr, using a boost::bind on controllerCb keeps a copy
  // of the shared ptr and keeps it alive.
  if (weak_ctrl.expired()) {
    RCLCPP_ERROR_STREAM(logger, "Got callback on expired MoveJointGroup, ignoring it");
    return;
  }
  MoveJointGroupPtr ctrl = weak_ctrl.lock();
  ControllerList::iterator it = std::find(
    goal_hdl->controllers.begin(),
    goal_hdl->controllers.end(), ctrl);
  if (it == goal_hdl->controllers.end()) {
    RCLCPP_ERROR_STREAM(
      logger,
      "Something is wrong in the controller callback handling. " <<
        ctrl->getName() << " called a goal callback while no "
        "motion goal was alive for it.");
    return;
  }
  goal_hdl->controllers.erase(it);

  RCLCPP_DEBUG_STREAM(
    logger,
    "Return from joint group " << ctrl->getName() << ", " <<
      goal_hdl->controllers.size() << " active controllers, "
      "error: " << error_code);

  if (goal_hdl->canceled) {
    RCLCPP_INFO_STREAM(logger, "The Goal was canceled, not calling Motion callback.");
    return;
  }

  goal_hdl->error_code = PlayMotionResult::SUCCEEDED;
  if (error_code != 0 || ctrl->getState() != GoalStatus::STATUS_SUCCEEDED) {
    RCLCPP_ERROR_STREAM(logger, "Controller " << ctrl->getName() << " aborted.");
    goal_hdl->cancel();
    generateErrorCode(goal_hdl, error_code, ctrl->getState());
    goal_hdl->cb(goal_hdl);
    return;
  }

  if (goal_hdl->controllers.empty()) {
    goal_hdl->cb(goal_hdl);
  }
}

template<class T>
bool hasNonNullIntersection(const std::vector<T> & v1, const std::vector<T> & v2)
{
  for (const T & e1 : v1) {
    for (const T & e2 : v2) {
      if (e1 == e2) {
        return true;
      }
    }
  }
  return false;
}
} // unnamed namespace

namespace play_motion
{
using namespace std::placeholders;

rclcpp::NodeOptions get_pm_node_options()
{
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  return node_options;
}

PlayMotion::PlayMotion()
: rclcpp::Node("play_motion", get_pm_node_options()),
  ctrlr_updater_(nullptr),
  approach_planner_(nullptr),
  is_ready_srv_(nullptr)
{
}

void PlayMotion::init()
{
  ctrlr_updater_ = std::make_shared<ControllerUpdater>(shared_from_this());
  approach_planner_ = std::make_shared<ApproachPlanner>(shared_from_this());

  ctrlr_updater_->registerUpdateCb(std::bind(&PlayMotion::updateControllersCb, this, _1, _2));

  joint_states_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&PlayMotion::jointStateCb, this, _1));

  is_ready_srv_ =
    create_service<IsReadyService>(
    std::string(get_name()) + "/is_ready",
    std::bind(&PlayMotion::is_ready, this, _1, _2));
}

PlayMotion::Goal::Goal(const Callback & cbk)
: error_code(0),
  active_controllers(0),
  cb(cbk),
  canceled(false)
{}

void PlayMotion::Goal::cancel()
{
  canceled = true;
  for (MoveJointGroupPtr mjg : controllers) {
    mjg->cancel();
  }
}

void PlayMotion::Goal::addController(const MoveJointGroupPtr & ctrl)
{
  controllers.push_back(ctrl);
}

PlayMotion::Goal::~Goal()
{
  cancel();
}

void PlayMotion::updateControllersCb(
  const ControllerUpdater::ControllerStates & states,
  const ControllerUpdater::ControllerJoints & joints)
{
  typedef std::pair<std::string, ControllerUpdater::ControllerState> ctrlr_state_pair_t;

  RCLCPP_INFO_STREAM(get_logger(), "Controllers have changed, cancelling all active goals");
  for (MoveJointGroupPtr mjg : move_joint_groups_) {
    // Deleting the groups isn't enough, because they are referenced by
    // the goalhandles. They will only be destroyed when the action ends,
    // which will crash because you cannot destroy actionclient from within a callback
    // We must abort, so the goalhandle is destroyed
    mjg->abort();
  }
  move_joint_groups_.clear();
  for (const auto & p : states) {
    if (p.second != ControllerUpdater::RUNNING) {
      continue;
    }
    move_joint_groups_.push_back(
      MoveJointGroupPtr(
        new MoveJointGroup(
          shared_from_this(),
          p.first,
          joints.at(p.first))));
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Controller '" << p.first << "' with " << joints.at(
        p.first).size() << " joints.");
  }
}

void PlayMotion::jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  joint_states_.clear();
  for (uint32_t i = 0; i < msg->name.size(); ++i) {
    joint_states_[msg->name[i]] = msg->position[i];
  }
}

bool PlayMotion::getGroupTraj(
  MoveJointGroupPtr move_joint_group,
  const JointNames & motion_joints,
  const Trajectory & motion_points, Trajectory & traj_group)
{
  JointNames group_joint_names = move_joint_group->getJointNames();
  std::vector<double> joint_states;
  std::map<std::string, int> joint_index;

  traj_group.clear();
  traj_group.reserve(motion_points.size());

  for (const std::string & jn : group_joint_names) {
    // store the index of this joint in the given motion
    int index =
      static_cast<int>(std::find(
        motion_joints.begin(), motion_joints.end(),
        jn) - motion_joints.begin());
    joint_index[jn] = index;

    // retrieve joint state,  we should have it from the joint_states subscriber
    if (joint_states_.find(jn) == joint_states_.end()) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Could not get current position of joint \'" << jn << "\'.");
      return false;
    }
    joint_states.push_back(joint_states_[jn]);
  }

  for (const TrajPoint & p : motion_points) {
    bool has_velocities = !p.velocities.empty();
    bool has_accelerations = !p.accelerations.empty();
    TrajPoint point;
    point.positions.resize(group_joint_names.size());
    if (has_velocities) {
      point.velocities.resize(group_joint_names.size(), 0);
    }
    if (has_accelerations) {
      point.accelerations.resize(group_joint_names.size(), 0);
    }
    point.time_from_start = p.time_from_start;

    for (std::size_t i = 0; i < group_joint_names.size(); ++i) {
      // first assignment, overriden by given motion if joint specified
      point.positions[i] = joint_states[i];

      size_t index = static_cast<size_t>(joint_index[group_joint_names[i]]);
      if (index < motion_joints.size()) {
        point.positions[i] = p.positions[index];
        if (has_velocities) {
          point.velocities[i] = p.velocities[index];
        }
        if (has_accelerations) {
          point.accelerations[i] = p.accelerations[index];
        }
      }
    }
    traj_group.push_back(point);
  }
  return true;
}

void PlayMotion::getMotionJoints(const std::string & motion_name, JointNames & motion_joints)
{
  try {
    play_motion::getMotionJoints(this, motion_name, motion_joints);
  } catch (const std::runtime_error & e) {
    std::ostringstream error_msg;
    error_msg << "could not parse motion '" << motion_name << "': " << e.what();
    throw PlayMotionException(error_msg.str(), PlayMotionResult::MOTION_NOT_FOUND);
  }
}

void PlayMotion::getMotionPoints(const std::string & motion_name, Trajectory & motion_points)
{
  try {
    ::play_motion::getMotionPoints(this, motion_name, motion_points);
  } catch (const std::runtime_error & e) {
    std::ostringstream error_msg;
    error_msg << "could not parse motion '" << motion_name << "': " << e.what();
    throw PlayMotionException(error_msg.str(), PlayMotionResult::MOTION_NOT_FOUND);
  }
}

ControllerList PlayMotion::getMotionControllers(const JointNames & motion_joints)
{
  // Populate list of controllers containing at least one motion joint,...
  ControllerList ctrlr_list;
  for (MoveJointGroupPtr move_joint_group : move_joint_groups_) {
    if (hasNonNullIntersection(motion_joints, move_joint_group->getJointNames())) {
      ctrlr_list.push_back(move_joint_group);
    }
  }

  // ...check that all motion joints are contained in this list...
  for (const std::string & jn : motion_joints) {
    for (MoveJointGroupPtr ctrlr : ctrlr_list) {
      if (ctrlr->isControllingJoint(jn)) {
        goto next_joint;
      }
    }

    throw PlayMotionException(
            "No controller was found for joint '" + jn + "'",
            PlayMotionResult::MISSING_CONTROLLER);
next_joint:;
  }

  // ...and that no controller in the list is busy executing another goal
  for (MoveJointGroupPtr move_joint_group : ctrlr_list) {
    if (!move_joint_group->isIdle()) {
      throw PlayMotionException(
              "Controller '" + move_joint_group->getName() + "' is busy",
              PlayMotionResult::CONTROLLER_BUSY);
    }
  }

  return ctrlr_list;
}

bool PlayMotion::run(
  const std::string & motion_name,
  bool skip_planning,
  GoalHandle & goal_hdl,
  const Callback & cb)
{
  JointNames motion_joints;
  Trajectory motion_points;
  std::map<MoveJointGroupPtr, Trajectory> joint_group_traj;

  goal_hdl = GoalHandle(new Goal(cb));

  try {
    ControllerList groups;

    try {
      getMotionJoints(motion_name, motion_joints);
      groups = getMotionControllers(motion_joints);       // Checks many preconditions
      getMotionPoints(motion_name, motion_points);
    } catch (const PlayMotionException & e) {
      throw e;
    } catch (const std::runtime_error & e) {
      throw PlayMotionException(e.what(), PlayMotionResult::OTHER_ERROR);
    }

    std::vector<double> curr_pos;   // Current position of motion joints
    for (const std::string & motion_joint : motion_joints) {
      curr_pos.push_back(joint_states_[motion_joint]);       // TODO: What if motion joint does not exist?
    }

    // Approach trajectory
    Trajectory motion_points_safe;
    if (!approach_planner_->prependApproach(
        motion_joints, curr_pos,
        skip_planning,
        motion_points, motion_points_safe))
    {
      throw PlayMotionException("Approach motion planning failed", PlayMotionResult::NO_PLAN_FOUND);  // TODO: Expose descriptive error string from approach_planner
    }

    // TODO: Resample and validate output trajectory
    try {
      populateVelocities(motion_points_safe, motion_points_safe);
    } catch (const std::runtime_error & e) {
      throw PlayMotionException(e.what(), PlayMotionResult::OTHER_ERROR);
    }

    // Seed target pose with current joint state
    for (MoveJointGroupPtr move_joint_group : groups) {
      if (!getGroupTraj(
          move_joint_group, motion_joints, motion_points_safe,
          joint_group_traj[move_joint_group]))
      {
        throw PlayMotionException(
                "Missing joint state for joint in controller '" +
                move_joint_group->getName() + "'");
      }
    }

    if (joint_group_traj.empty()) {
      throw PlayMotionException("Nothing to send to controllers");
    }

    // Send pose commands
    for (const auto & p : joint_group_traj) {
      goal_hdl->addController(p.first);
      p.first->setCallback(
        std::bind(
          controllerCb, get_logger(), _1, goal_hdl,
          MoveJointGroupWeakPtr(p.first)));
      if (!p.first->sendGoal(p.second)) {
        throw PlayMotionException(
                "Controller '" + p.first->getName() + "' did not accept trajectory, "
                "canceling everything");
      }
    }
  } catch (const PlayMotionException & e) {
    goal_hdl->error_string = e.what();
    goal_hdl->error_code = e.error_code();
    return false;
  }
  return true;
}

void PlayMotion::is_ready(
  const IsReadyService::Request::SharedPtr /*request*/,
  IsReadyService::Response::SharedPtr response)
{
  /// @warning care concurrency?
  response->success = !move_joint_groups_.empty();
}

}
