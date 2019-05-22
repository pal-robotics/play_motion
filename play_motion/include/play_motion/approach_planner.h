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

/** \author Adolfo Rodriguez Tsouroukdissian. */

#ifndef PLAY_MOTION_APPROACH_PLANNER_H
#define PLAY_MOTION_APPROACH_PLANNER_H

#include <map>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <ros/message_forward.h>

#include <play_motion/datatypes.h>

namespace ros
{
  class NodeHandle;
  class AsyncSpinner;
  class CallbackQueue;
}

namespace trajectory_msgs
{
  ROS_DECLARE_MESSAGE(JointTrajectory);
}

namespace moveit
{
  namespace planning_interface
  {
    class MoveGroupInterface;
  }
}

namespace play_motion
{
  /// TODO
  class ApproachPlanner
  {
  public:

    ApproachPlanner(const ros::NodeHandle& nh);

    /// TODO
    bool prependApproach(const std::vector<std::string>& joint_names,
                         const std::vector<double>&      current_pos,
                         bool                            skip_planning,
                         const std::vector<TrajPoint>&   traj_in,
                               std::vector<TrajPoint>&   traj_out);

    /// TODO
    bool needsApproach(const std::vector<double>& current_pos,
                       const std::vector<double>& goal_pos);

  private:
    typedef moveit::planning_interface::MoveGroupInterface MoveGroupInterface;
    typedef boost::shared_ptr<MoveGroupInterface> MoveGroupInterfacePtr;
    typedef boost::shared_ptr<ros::AsyncSpinner> AsyncSpinnerPtr;
    typedef boost::shared_ptr<ros::CallbackQueue> CallbackQueuePtr;
    typedef std::vector<std::string> JointNames;
    typedef std::map<std::string, double> JointGoal;

    struct PlanningData
    {
      PlanningData(MoveGroupInterfacePtr move_group_ptr);
      MoveGroupInterfacePtr move_group;
      JointNames   sorted_joint_names;
    };

    std::vector<PlanningData> planning_data_;
    std::vector<std::string> no_plan_joints_;
    double joint_tol_; ///< Absolute tolerance used to determine if two joint positions are approximately equal.
    double skip_planning_vel_; ///< Maximum average velocity that any joint can have in a non-planned approach.
    double skip_planning_min_dur_; ///< Minimum duration that a non-planned approach can have
    CallbackQueuePtr cb_queue_;
    AsyncSpinnerPtr spinner_;
    bool planning_disabled_;

    /// TODO
    bool computeApproach(const JointNames&                 joint_names,
                         const std::vector<double>&        current_pos,
                         const std::vector<double>&        goal_pos,
                         trajectory_msgs::JointTrajectory& traj);

    /// TODO
    bool planApproach(const JointNames&                 joint_names,
                      const std::vector<double>&        joint_values,
                      MoveGroupInterfacePtr             move_group,
                      trajectory_msgs::JointTrajectory& traj);
    /// TODO
    void combineTrajectories(const JointNames&                  joint_names,
                             const std::vector<double>&         current_pos,
                             const std::vector<TrajPoint>&      traj_in,
                             trajectory_msgs::JointTrajectory&  approach,
                             std::vector<TrajPoint>&            traj_out);

    /// TODO
    std::vector<MoveGroupInterfacePtr> getValidMoveGroups(const JointNames& min_group,
                                                 const JointNames& max_group);

    /// TODO
    bool isPlanningJoint(const std::string& joint_name) const;

    /// TODO
    double noPlanningReachTime(const std::vector<double>& curr_pos,
                               const std::vector<double>& goal_pos);
  };

} // namespace

#endif
