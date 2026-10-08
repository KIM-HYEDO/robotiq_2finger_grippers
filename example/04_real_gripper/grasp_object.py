#!/usr/bin/env python3
"""Tutorial 04-2: grasp an object and detect it.

Closes slowly with a low force. If the fingers stop on an object, the gripper reports
`obj_detected` and the final position is the object width. Then the gripper opens again.
Object detection only works on a real gripper: the simulation never reports it.

Parameters:
    ~speed: Closing speed [m/s] (default 0.03, valid 0.013 - 0.1)
    ~force: Grip force [%] (default 20, valid 0 - 100). Start low for fragile objects.

Usage:
    rosrun robotiq_2f_gripper_examples grasp_object.py _force:=20
"""

import actionlib
import rospy

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperAction, CommandRobotiqGripperGoal

ACTION_NAME = 'command_robotiq_action'
STROKE = 0.085  # [m]


def move(client, position, speed, force):
    goal = CommandRobotiqGripperGoal()
    goal.position = position
    goal.speed = speed
    goal.force = force
    client.send_goal(goal)
    client.wait_for_result()
    return client.get_result()


if __name__ == '__main__':
    rospy.init_node('example_grasp_object')
    speed = rospy.get_param('~speed', 0.03)
    force = rospy.get_param('~force', 20.0)

    client = actionlib.SimpleActionClient(ACTION_NAME, CommandRobotiqGripperAction)
    client.wait_for_server()

    rospy.loginfo('Opening')
    move(client, STROKE, speed, force)

    rospy.loginfo('Place an object between the fingers. Closing in 3 seconds...')
    rospy.sleep(3.0)

    result = move(client, 0.0, speed, force)  # Target 0: the fingers stop early if they touch an object
    if result.obj_detected:
        rospy.loginfo('Object grasped. Object width: %.1f mm', result.position * 1000.0)
    else:
        rospy.logwarn('No object detected, the fingers closed completely (%.1f mm)', result.position * 1000.0)

    rospy.sleep(2.0)
    rospy.loginfo('Releasing')
    move(client, STROKE, speed, force)
