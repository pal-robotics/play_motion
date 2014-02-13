#!/usr/bin/env python

import rospy
import sensor_msgs.msg
import play_motion_msgs.srv as PMS

class IsAlreadyThere:
    def __init__(self):
        self.srv = rospy.Service("play_motion/is_already_there",
                                 PMS.IsAlreadyThere,
                                 self.is_already_there)
        self.joint_sub = rospy.Subscriber("joint_states",
                                          sensor_msgs.msg.JointState,
                                          self.joint_state_cb)
        self.joint_states = {}

    def joint_state_cb(self, data):
        for i in range(len(data.name)):
            self.joint_states[data.name[i]] = data.position[i]

    def is_already_there(self, data):
        param = "play_motion/motions/{}".format(data.motion_name)
        if not rospy.has_param(param):
            rospy.logdebug("could not find motion {}".format(data.motion_name))
            return False
        motion = rospy.get_param(param)
        for i in range(len(motion['joints'])):
            joint_name = motion['joints'][i]
            if joint_name not in self.joint_states:
                rospy.logdebug("could not find state of joint {}".format(joint_name))
                return False
            if abs(motion['points'][0]['positions'][i]
                   - self.joint_states[joint_name]) > data.tolerance:
                rospy.logdebug("tolerance exceeded fot joint {}, {} vs {}"
                              .format(joint_name,
                                      motion['points'][0]['positions'][i],
                                      self.joint_states[joint_name]))
                return False
        return True


if __name__ == "__main__":
    rospy.init_node("is_already_there")
    iat = IsAlreadyThere()
    rospy.spin()
