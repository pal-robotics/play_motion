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

#include <chrono>

#include "play_motion/controller_updater.h"

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

  static bool isJointTrajectoryController(const std::string& name)
  {
    std::string tofind = "JointTrajectoryController"; //XXX: magic value
    size_t pos = name.find(tofind);
    if (pos == std::string::npos)
      return false;
    if (pos != name.length() - tofind.length())
      return false;

    return true;
  }

  void ControllerUpdater::mainLoop()
  {
    rclcpp::Rate r(1s); //XXX: magic value
    while(rclcpp::ok())
    {
      r.sleep();

      if (!update_cb_)
        continue;

      if(!cm_client_->wait_for_service(5s)) {
        RCLCPP_WARN(logger_, "List controllers service is not ready");
        continue;
      }

      auto request = std::make_shared<ListControllers::Request>();
      auto result = cm_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node_, result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(logger_, "Failed to call list controllers service");
      }

      ControllerStates states;
      ControllerJoints joints;
      using cstate_t = controller_manager_msgs::msg::ControllerState;

      for (const cstate_t & cs : result.get()->controller)
      {
        if (!isJointTrajectoryController(cs.type))
          continue;
        states[cs.name] = (cs.state == "running" ? RUNNING : STOPPED);

        /// @todo add this to the controller_manager
        // joints[cs.name] = cs.claimed_resources[0].resources;
      }

      if (states == last_cstates_)
        continue;

      RCLCPP_INFO(logger_, "The set of running joint trajectory controllers has changed, updating it");

      /// @todo this must be a oneshot
      // update_timer_ = nh_.createTimer(ros::Duration(0), boost::bind(update_cb_, states, joints), true);
      // update_timer_ = rclcpp::create_timer(node_, node_->get_clock(), 0s, std::bind(update_cb_, states, joints));

      last_cstates_ = states;
    }
  }
}
