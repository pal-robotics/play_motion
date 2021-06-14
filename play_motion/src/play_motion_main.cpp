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

#include <memory>

#include "play_motion/play_motion_server.hpp"
#include "play_motion/play_motion.hpp"

#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto play_motion = std::make_shared<play_motion::PlayMotion>();
  play_motion->init();

  play_motion::PlayMotionServer pms(play_motion);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(play_motion);
  executor.spin();
}
