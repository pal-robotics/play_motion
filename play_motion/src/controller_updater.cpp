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

#include <chrono>

#include "play_motion/controller_updater.hpp"

#include "controller_manager_msgs/srv/list_controllers.hpp"

#include "rclcpp/executors.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/rate.hpp"

using namespace std::chrono_literals;

namespace play_motion
{
ControllerUpdater::ControllerUpdater(const rclcpp::Node::SharedPtr & node)
: node_(node),
  logger_(node->get_logger().get_child("controller_updater"))
{
  cm_client_ = node_->create_client<ListControllers>("/controller_manager/list_controllers");
  main_thread_ = std::thread(&ControllerUpdater::mainLoop, this);
}

ControllerUpdater::~ControllerUpdater()
{}

static bool isJointTrajectoryController(const std::string & name)
{
  std::string tofind = "JointTrajectoryController";   //XXX: magic value
  size_t pos = name.find(tofind);
  if (pos == std::string::npos) {
    return false;
  }
  if (pos != name.length() - tofind.length()) {
    return false;
  }

  return true;
}

void ControllerUpdater::mainLoop()
{
  rclcpp::Rate r(1s);   //XXX: magic value
  while (rclcpp::ok()) {
    r.sleep();

    if (!update_cb_) {
      continue;
    }

    if (!cm_client_->wait_for_service(5s)) {
      RCLCPP_WARN(logger_, "List controllers service is not ready");
      continue;
    }

    auto request = std::make_shared<ListControllers::Request>();
    auto result = cm_client_->async_send_request(request);
//      if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
//        RCLCPP_ERROR(logger_, "Failed to call list controllers service");
//      }

    result.wait();
    if (!result.get()) {
      RCLCPP_ERROR(logger_, "Failed to call list controllers service");
      return;
    }

    ControllerStates states;
    ControllerJoints joints;
    using cstate_t = controller_manager_msgs::msg::ControllerState;

    for (const cstate_t & cs : result.get()->controller) {
      if (!isJointTrajectoryController(cs.type)) {
        continue;
      }
      states[cs.name] = (cs.state == "active" ? RUNNING : STOPPED);

      /// @note claimed_interfaces are expressed like "[joint_name]/[interface_name]",
      /// we need to extract the joint names here
      auto get_joint_names = [](const std::vector<std::string> & claimed_interfaces)
        {
          JointNames joint_names;
          for (const auto & interface : claimed_interfaces) {
            auto dash = interface.find_first_of("/");
            joint_names.emplace_back(interface.substr(0, dash));
          }
          return joint_names;
        };
      joints[cs.name] = get_joint_names(cs.claimed_interfaces);
    }

    if (states == last_cstates_) {
      continue;
    }

    RCLCPP_INFO(
      logger_,
      "The set of running joint trajectory controllers has changed, updating it");

    /// @todo this must be a oneshot
    // update_timer_ = nh_.createTimer(ros::Duration(0), boost::bind(update_cb_, states, joints), true);
    // update_timer_ = rclcpp::create_timer(node_, node_->get_clock(), 0s, std::bind(update_cb_, states, joints));

    /// @note ros2: any problem with doing this synchronously instead?
    update_cb_(states, joints);

    last_cstates_ = states;
  }
}
}
