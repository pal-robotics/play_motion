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

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <ros/time.h>

#include "play_motion/play_motion_helpers.h"

TEST(PlayMotionHelpersTest, getMotions)
{
  play_motion::MotionNames motion_names;
  ros::NodeHandle nh("play_motion");
  play_motion::getMotionIds(nh, motion_names);
  EXPECT_EQ(4, motion_names.size());
  EXPECT_NE(motion_names.end(), std::find(motion_names.begin(), motion_names.end(), "arms_t"));
  EXPECT_NE(motion_names.end(), std::find(motion_names.begin(), motion_names.end(), "bow"));
  EXPECT_NE(motion_names.end(), std::find(motion_names.begin(), motion_names.end(), "five_joint_motion"));
  EXPECT_NE(motion_names.end(), std::find(motion_names.begin(), motion_names.end(), "three_point_motion"));
}

TEST(PlayMotionHelpersTest, getMotionJoints)
{
  ros::NodeHandle nh("play_motion");
  play_motion::JointNames names;
  play_motion::getMotionJoints(nh, "bow", names);
  EXPECT_EQ(18, names.size());
  play_motion::getMotionJoints(nh, "five_joint_motion", names);
  EXPECT_EQ(5, names.size());
}


TEST(PlayMotionHelpersTest, getMotionPoints)
{
  ros::NodeHandle nh("play_motion");
  play_motion::Trajectory traj;
  play_motion::getMotionPoints(nh, "arms_t", traj);
  EXPECT_EQ(1, traj.size());
  traj.clear();
  play_motion::getMotionPoints(nh, "three_point_motion", traj);
  EXPECT_EQ(3, traj.size());
}

TEST(PlayMotionHelpersTest, getMotionDuration)
{
  ros::NodeHandle nh("play_motion");
  ros::Duration d = play_motion::getMotionDuration(nh, "arms_t");
  EXPECT_NEAR(0.0, d.toSec(), 0.01);
  d = play_motion::getMotionDuration(nh, "bow");
  EXPECT_NEAR(6.5, d.toSec(), 0.01);
}

TEST(PlayMotionHelpersTest, isAlreadyThere)
{
  ros::NodeHandle nh("play_motion");

  play_motion::JointNames sourceJoints;
  play_motion::Trajectory sourceTraj;
  play_motion::getMotionJoints(nh, "bow", sourceJoints);
  play_motion::getMotionPoints(nh, "bow", sourceTraj);


  /// Same position
  EXPECT_TRUE(play_motion::isAlreadyThere(sourceJoints, sourceTraj[0],
              sourceJoints, sourceTraj[0]));

  /// Different position
  EXPECT_FALSE(play_motion::isAlreadyThere(sourceJoints, sourceTraj[0],
               sourceJoints, sourceTraj[1]));

  /// Different position but with  360º tolerance
  EXPECT_TRUE(play_motion::isAlreadyThere(sourceJoints, sourceTraj[0],
              sourceJoints, sourceTraj[1], M_2_PI));

  play_motion::JointNames differentJoints;
  play_motion::getMotionJoints(nh, "bow", differentJoints);
  differentJoints[0] = "made_up_joint";
  /// Same position but different joint names
  EXPECT_FALSE(play_motion::isAlreadyThere(differentJoints, sourceTraj[0],
               sourceJoints, sourceTraj[0]));


  differentJoints.clear();
  EXPECT_THROW(play_motion::isAlreadyThere(differentJoints, sourceTraj[0],
               sourceJoints, sourceTraj[0]), ros::Exception);
}

TEST(PlayMotionHelpersTest, getMotionOk)
{
  ros::NodeHandle nh("play_motion");
  play_motion::MotionInfo info;
  play_motion::getMotion(nh, "arms_t", info);
  EXPECT_EQ("arms_t", info.id);
  EXPECT_EQ("Arms T", info.name);
  EXPECT_EQ("posture", info.usage);
  EXPECT_EQ("Both arms set straight pointing sideways at 45 degrees.", info.description);
}

TEST(PlayMotionHelpersTest, getMotionKo)
{
  ros::NodeHandle nh("play_motion");
  play_motion::MotionInfo info;
  EXPECT_THROW(play_motion::getMotion(nh, "",          info), ros::Exception);
  EXPECT_THROW(play_motion::getMotion(nh, "bad_name",  info), ros::Exception);
  EXPECT_THROW(play_motion::getMotion(nh, "~bad_name", info), ros::Exception);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "play_motion_helpers_test");

  int ret = RUN_ALL_TESTS();
  return ret;
}

