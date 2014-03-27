#!/usr/bin/env python

from __future__ import print_function
import argparse
import sys
import signal
from time import time, sleep
from uuid import uuid4
import rospy
from actionlib import SimpleActionClient, GoalStatus
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal, PlayMotionResult

# pretty - A miniature library that provides a Python print and stdout
# wrapper that makes colored terminal text easier to use (eg. without
# having to mess around with ANSI escape sequences). This code is public
# domain - there is no license except that you must leave this header.
#
# Copyright (C) 2008 Brian Nez <thedude at bri1 dot com>
#
# With modifications
# (C) 2013 Paul M <pmathieu@willowgarage.com>
# (C) 2014 Adolfo Rodriguez Tsouroukdissian <adolfo.rodriguez@pal-robotics.com>

codeCodes = {
    'black':    '0;30', 'bright gray':  '0;37',
    'blue':     '0;34', 'white':        '1;37',
    'green':    '0;32', 'bright blue':  '1;34',
    'cyan':     '0;36', 'bright green': '1;32',
    'red':      '0;31', 'bright cyan':  '1;36',
    'purple':   '0;35', 'bright red':   '1;31',
    'yellow':   '0;33', 'bright purple':'1;35',
    'dark gray':'1;30', 'bright yellow':'1;33',
    'normal':   '0'
}

def printc(text, color, file = sys.stdout):
    """Print in color."""
    if ((file == sys.stdout and sys.stdout.isatty()) or
        (file == sys.stderr and sys.stderr.isatty())):
        print("\033["+codeCodes[color]+"m"+text+"\033[0m", file = file)
    else:
        print(text)

def bold(msg):      return "\033[0;1m"+msg+"\033[0;22m"
def print_err(msg): printc(msg, 'red', file = sys.stderr)
def print_ok(msg):  printc(msg, 'green')

class MotionData:
    def __init__(self):
        self.motion_name = None
        self.joint_name  = None
        self.position    = None
        self.duration    = None

class MoveJointException(Exception):
    pass

# Globals
motion_data = MotionData()

def parse_args(args=None):
    parser = argparse.ArgumentParser(description='Move individual joints.')
    parser.add_argument('name',
                        help='Name of the joint to move.')
    parser.add_argument('position', type=float,
                        help='Desired joint position in radians/meters.')
    parser.add_argument('duration', type=float, default=0.0, nargs='?',
                        help='Motion duration in seconds.')
    return parser.parse_args(args=args)

def wait_for_clock():
    timeout = time() + 2.0 # Wall-clock time
    while not rospy.is_shutdown() and rospy.Time.now().is_zero():
        if time() > timeout:
            raise MoveJointException('Timed out waiting for a valid time. Is the /clock topic being published?.')
        sleep(0.1) # Wall-clock sleep

def play_motion_client(action_ns):
    client = SimpleActionClient(action_ns, PlayMotionAction)
    if not client.wait_for_server(rospy.Duration(1)):
        raise MoveJointException("No play_motion server running (namespace '{}')".format(action_ns))
    return client

def load_motion(motion_namespace, motion_data):
    motion_param = motion_namespace + '/' + motion_data.motion_name
    rospy.set_param(motion_param + '/joints', [motion_data.joint_name])
    rospy.set_param(motion_param + '/points', [{'positions': [motion_data.position],
                                                'time_from_start': motion_data.duration}])

def unload_motion(motion_namespace, motion_name):
    motion_param = motion_namespace + '/' + motion_name
    if rospy.has_param(motion_param):
        rospy.delete_param(motion_param)

def active_cb():
    msg = 'Moving joint ' + bold(motion_data.joint_name) + ' to position ' + bold(str(motion_data.position))
    if motion_data.duration > 0.0:
        msg += ' in ' + bold(str(motion_data.duration)) + 's'
    print(msg)

def move_joint():
    global motion_data

    args = parse_args(rospy.myargv()[1:])
    rospy.init_node('move_joint', anonymous=True)

    motion_data.motion_name = args.name + '_' + uuid4().hex
    motion_data.joint_name  = args.name
    motion_data.position    = args.position
    motion_data.duration    = args.duration

    play_motion_ns = 'play_motion' # TODO: How to resolve names?
    motion_ns      = play_motion_ns + '/motions'

    try:
        wait_for_clock()
        client = play_motion_client(play_motion_ns)
    except MoveJointException, e:
        print_err(str(e))

    def cancel(signum, frame):
      client.cancel_goal()
      rospy.signal_shutdown("goal canceled")
    signal.signal(signal.SIGINT, cancel)

    load_motion(motion_ns, motion_data)
    goal = PlayMotionGoal(motion_name = motion_data.motion_name, skip_planning = True)
    client.send_goal(goal, None, active_cb)
    client.wait_for_result()

    unload_motion(motion_ns, motion_data.motion_name)
    al_res = client.get_state()
    pm_res = client.get_result()
    if al_res != GoalStatus.SUCCEEDED or pm_res.error_code != PlayMotionResult.SUCCEEDED:
        print_err("Execution failed with status {}. {}".format(GoalStatus.to_string(al_res), pm_res.error_string))
        sys.exit(1)

if __name__ == "__main__":
    move_joint()
