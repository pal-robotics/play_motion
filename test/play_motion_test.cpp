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

/// \author Paul Mathieu

#include <gtest/gtest.h>

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <ros/time.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/JointState.h>

#include "play_motion/PlayMotionAction.h"

class PlayMotionTestClient
{
  typedef actionlib::SimpleActionClient<play_motion::PlayMotionAction> ActionClient;
  typedef boost::shared_ptr<ActionClient> ActionClientPtr;
  typedef play_motion::PlayMotionGoal ActionGoal;
  typedef actionlib::SimpleClientGoalState ActionGoalState;
  typedef boost::shared_ptr<ActionGoalState> ActionGoalStatePtr;

public:
  PlayMotionTestClient()
  {
    ac_.reset(new ActionClient("/play_motion"));
    js_sub_ = nh_.subscribe("/joint_states", 10, &PlayMotionTestClient::jsCb, this);
  }

  int playMotion(const std::string& motion_name, double reach_time)
  {
    ActionGoal goal;
    goal.motion_name = motion_name;
    goal.reach_time  = ros::Duration(reach_time);

    ac_->waitForServer();
    gs_.reset(new ActionGoalState(ac_->sendGoalAndWait(goal)));
    ret_ = ac_->getResult()->error_code;
    return ret_;
  }

  double getJointPos(const std::string& joint_name)
  {
    int i;
    for (i = 0; i < js_.name.size(); ++i)
    {
      if (js_.name[i] == joint_name)
        return js_.position[i];
    }

    return std::numeric_limits<double>::quiet_NaN();
  }

  int getLastErrorCode()
  {
    return ret_;
  }

  ActionGoalState getLastGoalState()
  {
    return *gs_;
  }

protected:
  void jsCb(const sensor_msgs::JointStatePtr& js)
  {
    js_ = *js;
  }

private:
  int ret_;
  ActionGoalStatePtr gs_;
  ros::NodeHandle nh_;
  ActionClientPtr ac_;
  sensor_msgs::JointState js_;
  ros::Subscriber js_sub_;
};

typedef actionlib::SimpleClientGoalState GS;


TEST(PlayMotionTest, basicReachPose)
{
  PlayMotionTestClient pmtc;

  int error_code = pmtc.playMotion("pose1", 1.0);
  EXPECT_EQ(error_code, 0); // successful

  GS goal_state = pmtc.getLastGoalState();
  EXPECT_EQ(goal_state.state_, GS::SUCCEEDED);

  double final_pos = pmtc.getJointPos("joint1");
  EXPECT_NEAR(final_pos, 1.8, 0.01);
}

TEST(PlayMotionTest, rejectSecondGoal)
{
  PlayMotionTestClient pmtc1;
  PlayMotionTestClient pmtc2;

  boost::thread t(boost::bind(&PlayMotionTestClient::playMotion, &pmtc1, "home", 1.0));
  ros::Duration(0.3).sleep();

  int error_code = pmtc2.playMotion("home", 1.0);
  EXPECT_EQ(error_code, 3); // controller busy

  GS goal_state = pmtc2.getLastGoalState();
  EXPECT_EQ(goal_state.state_, GS::REJECTED);

  t.join();
  error_code = pmtc1.getLastErrorCode();
  EXPECT_EQ(error_code, 0);

  goal_state = pmtc1.getLastGoalState();
  EXPECT_EQ(goal_state.state_, GS::SUCCEEDED);
}

TEST(PlayMotionTest, badMotionName)
{
  PlayMotionTestClient pmtc;

  int error_code = pmtc.playMotion("inexistant_motion", 1.0);
  EXPECT_EQ(error_code, 1); // successful

  GS goal_state = pmtc.getLastGoalState();
  EXPECT_EQ(goal_state.state_, GS::REJECTED);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "play_motion_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Duration(2.0).sleep(); // wait a bit for the controllers to start
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}

