///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2013, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/// \author Víctor Lopez

#include <exception>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "gtest/gtest.h"

#include "play_motion/play_motion_helpers.h"

#include "rcl/allocator.h"

#include "rcl_yaml_param_parser/parser.h"

#include "rclcpp/duration.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/parameter_map.hpp"

class PlayMotionHelpersTest : public ::testing::Test
{
public:
  void SetUp()
  {
    node_ = std::make_unique<rclcpp::Node>(
      "play_motion",
      rclcpp::NodeOptions().allow_undeclared_parameters(true));

    load_parameters();
  }

  void TearDown()
  {
    node_.reset(nullptr);
  }

  bool load_parameters()
  {
    const auto pkg_path = ament_index_cpp::get_package_share_directory("play_motion");
    const std::string file_path = pkg_path + "/test/play_motion_helpers_poses.yaml";

    auto allocator = rcl_get_default_allocator();
    auto params = rcl_yaml_node_struct_init(allocator);
    if (rcl_parse_yaml_file(file_path.c_str(), params)) {
      auto param_map = rclcpp::parameter_map_from(params);
      node_->set_parameters(param_map["/play_motion"]);
      return true;
    } else {
      return false;
    }
  }

  const rclcpp::Node * node()
  {
    return node_.get();
  }

private:
  rclcpp::Node::UniquePtr node_;
};

TEST_F(PlayMotionHelpersTest, GetMotionsIdsTest)
{
  play_motion::MotionNames motion_names;
  play_motion::getMotionIds(node(), motion_names);
  EXPECT_EQ(4, motion_names.size());
  EXPECT_NE(motion_names.end(), std::find(motion_names.begin(), motion_names.end(), "arms_t"));
  EXPECT_NE(motion_names.end(), std::find(motion_names.begin(), motion_names.end(), "bow"));
  EXPECT_NE(
    motion_names.end(),
    std::find(motion_names.begin(), motion_names.end(), "five_joint_motion"));
  EXPECT_NE(
    motion_names.end(),
    std::find(motion_names.begin(), motion_names.end(), "three_point_motion"));
}

TEST_F(PlayMotionHelpersTest, MotionExistsTest)
{
  ASSERT_TRUE(play_motion::motionExists(node(), "arms_t"));
  ASSERT_TRUE(play_motion::motionExists(node(), "bow"));
  ASSERT_TRUE(play_motion::motionExists(node(), "five_joint_motion"));
  ASSERT_TRUE(play_motion::motionExists(node(), "three_point_motion"));
  ASSERT_FALSE(play_motion::motionExists(node(), "no_motion"));
}

TEST_F(PlayMotionHelpersTest, GetMotionJointsTest)
{
  play_motion::JointNames arms_t_joints, bow_joints, five_joints, three_joints, no_joints;
  ASSERT_NO_THROW(play_motion::getMotionJoints(node(), "arms_t", arms_t_joints));
  ASSERT_NO_THROW(play_motion::getMotionJoints(node(), "bow", bow_joints));
  ASSERT_THROW(
    play_motion::getMotionJoints(
      node(), "five_joint_motion", five_joints), std::runtime_error);
  ASSERT_NO_THROW(play_motion::getMotionJoints(node(), "three_point_motion", three_joints));
  ASSERT_THROW(play_motion::getMotionJoints(node(), "no_motion", no_joints), std::runtime_error);
  ASSERT_EQ(arms_t_joints.size(), 18);
  ASSERT_EQ(bow_joints.size(), 18);
  ASSERT_EQ(three_joints.size(), 18);
}

TEST_F(PlayMotionHelpersTest, GetMotionPointsTest)
{
  play_motion::Trajectory arms_t_traj, bow_traj, five_traj, three_traj, no_traj;
  ASSERT_NO_THROW(play_motion::getMotionPoints(node(), "arms_t", arms_t_traj));
  ASSERT_NO_THROW(play_motion::getMotionPoints(node(), "bow", bow_traj));
  ASSERT_THROW(
    play_motion::getMotionPoints(
      node(), "five_joint_motion", five_traj), std::runtime_error);
  ASSERT_NO_THROW(play_motion::getMotionPoints(node(), "three_point_motion", three_traj));
  ASSERT_THROW(play_motion::getMotionPoints(node(), "no_motion", no_traj), std::runtime_error);
  ASSERT_EQ(arms_t_traj.size(), 1);
  ASSERT_EQ(bow_traj.size(), 2);
  ASSERT_EQ(three_traj.size(), 3);
}

TEST_F(PlayMotionHelpersTest, GetMotionDurationTest)
{
  std::unique_ptr<rclcpp::Duration> duration;

  ASSERT_NO_THROW(
    duration = std::make_unique<rclcpp::Duration>(
      play_motion::getMotionDuration(node(), "arms_t")));
  EXPECT_NEAR(duration->seconds(), 0.0, 0.01);

  ASSERT_NO_THROW(
    duration = std::make_unique<rclcpp::Duration>(
      play_motion::getMotionDuration(node(), "bow")));
  EXPECT_NEAR(duration->seconds(), 6.5, 0.01);

  ASSERT_THROW(
    duration = std::make_unique<rclcpp::Duration>(
      play_motion::getMotionDuration(node(), "five_joint_motion")), std::runtime_error);

  ASSERT_NO_THROW(
    duration = std::make_unique<rclcpp::Duration>(
      play_motion::getMotionDuration(node(), "three_point_motion")));
  EXPECT_NEAR(duration->seconds(), 12.799, 0.01);

  ASSERT_THROW(
    duration = std::make_unique<rclcpp::Duration>(
      play_motion::getMotionDuration(node(), "no_motion")), std::runtime_error);
}

TEST_F(PlayMotionHelpersTest, IsAlreadyThereTest)
{
  play_motion::JointNames source_joints;
  play_motion::Trajectory source_traj;
  play_motion::getMotionJoints(node(), "bow", source_joints);
  play_motion::getMotionPoints(node(), "bow", source_traj);

  // same position
  EXPECT_TRUE(
    play_motion::isAlreadyThere(
      source_joints, source_traj[0],
      source_joints, source_traj[0]));

  // different position
  EXPECT_FALSE(
    play_motion::isAlreadyThere(
      source_joints, source_traj[0],
      source_joints, source_traj[1]));

  // different position but with 360º tolerance
  EXPECT_TRUE(
    play_motion::isAlreadyThere(
      source_joints, source_traj[0],
      source_joints, source_traj[1], M_2_PI));

  play_motion::JointNames different_joints;
  play_motion::getMotionJoints(node(), "bow", different_joints);
  different_joints[0] = "made_up_joint";

  // same position but different joint names
  EXPECT_FALSE(
    play_motion::isAlreadyThere(
      different_joints, source_traj[0],
      source_joints, source_traj[0]));

  different_joints.clear();
  EXPECT_THROW(
    play_motion::isAlreadyThere(
      different_joints, source_traj[0],
      source_joints, source_traj[0]), std::runtime_error);
}

TEST_F(PlayMotionHelpersTest, GetMotionTest)
{
  play_motion::MotionInfo info;

  ASSERT_THROW(play_motion::getMotion(node(), "", info), std::runtime_error);
  ASSERT_THROW(play_motion::getMotion(node(), "bad_name", info), std::runtime_error);

  ASSERT_NO_THROW(play_motion::getMotion(node(), "arms_t", info));
  EXPECT_EQ(info.id, "arms_t");
  EXPECT_EQ(info.name, "Arms T");
  EXPECT_EQ(info.usage, "posture");
  EXPECT_EQ(info.description, "Both arms set straight pointing sideways at 45 degrees.");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);

  int ret = RUN_ALL_TESTS();
  return ret;
}
