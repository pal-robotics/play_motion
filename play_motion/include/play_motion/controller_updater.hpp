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

#ifndef CONSTROLLERUPDATER_H
#define CONSTROLLERUPDATER_H

#include <functional>
#include <map>
#include <string>
#include <thread>

#include "controller_manager_msgs/srv/list_controllers.hpp"

#include "play_motion/datatypes.hpp"

#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/timer.hpp"

namespace play_motion
{
/** Keeps track of controller statuses by polling the controller manager.
* The service call happens in a separate thread to not disrupt the main code.
*/
class ControllerUpdater
{
public:
  enum ControllerState
  {
    RUNNING,
    STOPPED
  };

  using ControllerStates = std::map<std::string, ControllerState>;
  using ControllerJoints = std::map<std::string, JointNames>;

private:
  using Callback = std::function<void (const ControllerStates & states,
      const ControllerJoints & joints)>;
  using ListControllers = controller_manager_msgs::srv::ListControllers;

public:
  ControllerUpdater(const rclcpp::Node::SharedPtr & node);
  virtual ~ControllerUpdater();

  void registerUpdateCb(const Callback & cb) {update_cb_ = cb;}

private:
  void mainLoop();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  std::thread main_thread_;
  Callback update_cb_;
  rclcpp::Client<ListControllers>::SharedPtr cm_client_;
  ControllerStates last_cstates_;
};

}
#endif
