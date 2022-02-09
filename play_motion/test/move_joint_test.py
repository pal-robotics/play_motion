#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""


Tests move_joint
"""

import rospy
import sys
import unittest
import rostest
import actionlib
from uuid import uuid4
from play_motion_msgs.msg import PlayMotionAction, PlayMotionResult
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
        self._server.start()
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


class TestMoveJointSucceed(unittest.TestCase):
    def __init__(self, *args):
        super(TestMoveJointSucceed, self).__init__(*args)
        self._result = PlayMotionResult()
        self._mock_args = play_motion.parse_args(['arm_2_joint', '-0.1', '5'])

    def test_server_successtest(self):

        play_motion.execute_motion(self._mock_args)

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


class TestMoveJointAbort(unittest.TestCase):
    def __init__(self, *args):
        super(TestMoveJointAbort, self).__init__(*args)
        self._result = PlayMotionResult()
        self._mock_args = play_motion.parse_args(['arm_2_joint', '-0.1', '5'])

    def test_server_abort(self):
            
        self.assertRaises(SystemExit, play_motion.execute_motion, self._mock_args)

def main(argv):
    rospy.init_node("move_joint_test")
    # rostest.unitrun(PKG, TEST, TestMoveJoint.)
    play_motion_server = MockActionServer(PLAY_MOTION_ACTION,
                                          PlayMotionAction)
    if argv[0] == 'success':
        play_motion_server.succeed = True
        rostest.rosrun(PKG, TEST, TestMoveJointSucceed)
    elif argv[0] == 'abort':
        play_motion_server.succeed = False
        rostest.rosrun(PKG, TEST, TestMoveJointAbort)
    else:
        rospy.loginfo("Input argument is incorrect, should be 'abort' or 'success'.")

    rospy.signal_shutdown("Test finished")


if __name__ == '__main__':
    # import rostest

    main(sys.argv[1:2])
