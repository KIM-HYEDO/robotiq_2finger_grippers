#!/usr/bin/env python3
"""Tutorial 04-1: check the connection to a real gripper without moving it.

Waits for the action server and for one /joint_states message, then prints the current finger
distance. Nothing is commanded, so it is safe to run at any time.

Parameters:
    ~stroke: Gripper stroke [m] used to convert the joint angle to a finger distance
             (default 0.085). The joint range is 0.8 rad for the 2F-85 and 0.7 rad for the 2F-140.

Usage:
    rosrun robotiq_2f_gripper_examples check_connection.py
"""

import actionlib
import rospy
from sensor_msgs.msg import JointState

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperAction

ACTION_NAME = 'command_robotiq_action'
JOINT_NAME = 'finger_joint'


if __name__ == '__main__':
    rospy.init_node('example_check_connection')
    stroke = rospy.get_param('~stroke', 0.085)
    max_joint = 0.7 if stroke == 0.140 else 0.8  # [rad] Joint angle when the gripper is fully closed

    client = actionlib.SimpleActionClient(ACTION_NAME, CommandRobotiqGripperAction)
    if not client.wait_for_server(rospy.Duration(15.0)):
        rospy.logerr('Action server "%s" not found. Is real_gripper.launch running?', ACTION_NAME)
        raise SystemExit(1)
    rospy.loginfo('Action server found')

    try:
        msg = rospy.wait_for_message('/joint_states', JointState, timeout=5.0)
    except rospy.ROSException:
        rospy.logerr('No /joint_states received. Check the cable and the comport argument.')
        raise SystemExit(1)

    angle = msg.position[msg.name.index(JOINT_NAME)]
    width = stroke * (1.0 - angle / max_joint)  # Inverse of the driver's width-to-angle mapping
    rospy.loginfo('Connection OK. Joint angle %.3f rad, finger distance %.1f mm', angle, width * 1000.0)
