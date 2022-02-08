#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""


Tests move_joint
"""

import rospy
# import sys
import unittest
import rosunit
import rostest
import actionlib
from uuid import uuid4
from play_motion_msgs.msg import PlayMotionAction, PlayMotionResult, PlayMotionFeedback
import play_motion


PKG = 'play_motion'
TEST = 'move_joint_test'
PLAY_MOTION_ACTION = "play_motion"



class MockActionServer(object):

    def __init__(self, name, action):
        self._action_name = name
        self._server = actionlib.SimpleActionServer(name,
                                                    action,
                                                    execute_cb=self.execute_cb,
                                                    auto_start=False)
        # self._server.start()
        self._last_goal = None
        self._result = PlayMotionResult()
        self._isactive = False

        self.succeed = True

        rospy.loginfo("Starting server for action: " + self._action_name)

    def __del__(self):
        pass

    def execute_cb(self, goal):

        # if !PLAY_MOTION_SERVER._server.
        rospy.loginfo("Goal Received in execute_play_motion\n" + str(goal))

        if self.succeed:
            self._result.error_code = PlayMotionResult.SUCCEEDED
            self._server.set_succeeded(self._result)
        else:
            self._result.error_code = PlayMotionResult.OTHER_ERROR
            self._server.set_aborted(self._result)

    def start_server(self):
        if not self._isactive:
            self._server.start()
            self._isactive = True



    def end_server(self):
        self._isactive = False
        rospy.signal_shutdown("Test end")


class TestMoveJoint(unittest.TestCase):
    def __init__(self, *args):
        super(TestMoveJoint, self).__init__(*args)

        self._result = PlayMotionResult()
        self._mock_args = play_motion.parse_args(['arm_2_joint', '-0.1', '5'])


    def test_server_success(self):
        PLAY_MOTION_SERVER.succeed = True
        PLAY_MOTION_SERVER.start_server()

        try:
            play_motion.execute_motion(self._mock_args)
            rospy.loginfo(f"Finished execute motion with motion_succes = \
             {PLAY_MOTION_SERVER.succeed}")

            self.assertTrue(True)
        except SystemExit:
            rospy.loginfo(f"Systemerror execute motion with motion_succes = \
             {PLAY_MOTION_SERVER.succeed}")

            self.assertTrue(False)

    def test_server_aborted(self):
        PLAY_MOTION_SERVER.succeed = False
        # play_motion_server = MockActionServer(PLAY_MOTION_ACTION,
        #                           PlayMotionAction, motion_succes)
        PLAY_MOTION_SERVER.start_server()
        try:
            play_motion.execute_motion(self._mock_args)
            rospy.loginfo(f"Finished execute motion with motion_succes = \
                {PLAY_MOTION_SERVER.succeed}")
            self.assertTrue(False)
        except SystemExit:
            rospy.loginfo(f"Systemerror execute motion with motion_succes = \
                            {PLAY_MOTION_SERVER.succeed}")

            self.assertTrue(True)

    def test_clear_params(self):
        """get parameters"""
        # Set motiondata
        motion_namespace = 'motion_ns_test'
        motion_data = play_motion.MotionData()
        motion_data.motion_name = 'arm_joint_2' + '_' + uuid4().hex
        motion_data.joint_name = 'arm_joint_2'
        motion_data.position = '-0.1'
        motion_data.duration = '1'

        # Load motion in parameterspace
        play_motion.load_motion(motion_namespace, motion_data)
        motion_param = motion_namespace + '/' + motion_data.motion_name
        # Check if paramspace is correct
        self.assertEqual(motion_data.joint_name,
                         rospy.get_param(motion_param + '/joints')[0])
        self.assertEqual(motion_data.position,
                         rospy.get_param(motion_param + '/points')
                         [0]['positions'][0])
        self.assertEqual(motion_data.duration,
                         rospy.get_param(motion_param + '/points')
                         [0]['time_from_start'][0])
        # Unload paramspace
        play_motion.unload_motion(motion_namespace, motion_data.motion_name)
        # Check if unload is successful
        self.assertFalse(rospy.has_param(motion_param))


PLAY_MOTION_SERVER = MockActionServer(PLAY_MOTION_ACTION,
                                      PlayMotionAction)

def main():
    # play_motion_server = MockActionServer(
    #         PLAY_MOTION_ACTION, PlayMotionAction)
    # rosunit.unitrun(PKG, TEST, TestMoveJoint)
    rospy.init_node("move_joint_test")
    rostest.unitrun(PKG, TEST, TestMoveJoint)
    rospy.spin()
    rospy.signal_shutdown("Test finished")


if __name__ == '__main__':
    main()
