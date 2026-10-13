#!/usr/bin/env python
"""--------------------------------------------------------------------
This Node/Script creates an instance of a `SimpleActionServer` dedicated to receive, 
process and execute user commands for the Robotiq 2 finger adaptive grippers.

The action type is defined in the `robotiq_2f_gripper_msgs` as `CommandRobotiqGripper.action` and
the default action name is `/command_robotiq_action` but you can namespace it for multiple grippers
control.

See the `robotiq_action_server.launch` file on this package for an example on how to call this node.

Parameters:
    comport: USB Communication port to which the gripper is connected to (not needed in `sim` mode).  
    baud: Baudrate of communication with gripper (not needed in `sim` mode).
    stroke: Maximum distance in meters, between the gripper fingers (Only 0,085 and 0.140 are currently supported)
    joint_name: Name of the URDF gripper actuated joints to publish on the `/joint_state` topic 
    sim: Boolean indicating whether to use a simulated gripper or try to connect to a real one.
    rate: Frequency in Herz to update the gripper variables (command, joint_state, gripper_state)
    trajectory_force: Force [0-100] for FollowJointTrajectory goals without effort, e.g. from MoveIt (default 50)
    trajectory_speed: Speed [m/s] for FollowJointTrajectory goals without a duration (default 0.1)

@author: Daniel Felipe Ordonez Apraez
@email: daniels.ordonez@gmail.com
--------------------------------------------------------------------"""
import rospy
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver, Robotiq2FingerSimulatedGripperDriver 
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryResult, FollowJointTrajectoryFeedback, FollowJointTrajectoryGoal
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal

GOAL_DETECTION_THRESHOLD = 0.003 # [m] Max deviation from target goal to consider as goal "reached"
MIN_SPEED = 0.013 # [m/s] Speed limits of the gripper, same as in the driver
MAX_SPEED = 0.1   # [m/s]

class CommandGripperActionServer(object):

    def __init__(self, namespace, action_name, driver):
        self._action_name = namespace + action_name
        self._action_server = actionlib.SimpleActionServer(self._action_name, 
                                                            CommandRobotiqGripperAction, 
                                                            execute_cb=self.execute_cb, 
                                                            auto_start = False)
        self._joint_trajectory_action_server = actionlib.SimpleActionServer(namespace + "robotiq_controller/follow_joint_trajectory", 
                                                            FollowJointTrajectoryAction, 
                                                            execute_cb=self.execute_joint_trajectory_cb,
                                                            auto_start = False)
        self._driver = driver       # Get handle to the Gripper Control Server
        
        # Wait until gripper driver is ready to take commands.
        watchdog = rospy.Timer(rospy.Duration(15.0), self._connection_timeout, oneshot=True)
        while not rospy.is_shutdown() and not self._driver.is_ready:
            rospy.sleep(0.5)
            rospy.logwarn_throttle(5, self._action_name + ": Waiting for gripper to be ready...")
        
        watchdog.shutdown() 
        if not rospy.is_shutdown():
            self._processing_goal = False
            self._is_stalled = False

            self._action_server.start()
            self._joint_trajectory_action_server.start()
            rospy.loginfo("Robotiq server started")
    
    def _connection_timeout(self, event):
        rospy.logfatal("Gripper on port %s seems not to respond" % (self._driver._comport))
        rospy.signal_shutdown("Gripper on port %s seems not to respond" % (self._driver._comport))
    
    def execute_cb(self, goal_command):
      rospy.logdebug( (self._action_name + ": New goal received Pos:%.3f Speed: %.3f Force: %.3f Force-Stop: %r") % (goal_command.position, goal_command.speed, goal_command.force, goal_command.stop) )
      # Send incoming command to gripper driver
      self._driver.update_gripper_command(goal_command)
      # Wait until command is received by the gripper 
      rospy.sleep(rospy.Duration(0.1))
      # Set Action Server as active === processing goal...
      self._processing_goal = True        
      
      feedback = CommandRobotiqGripperFeedback()
      result = CommandRobotiqGripperResult()

      # Set timeout timer 
      watchdog = rospy.Timer(rospy.Duration(5.0), self._execution_timeout, oneshot=True)

      # Wait until goal is achieved and provide feedback
      rate = rospy.Rate( rospy.get_param('~rate', 30) )

      while not rospy.is_shutdown() and self._processing_goal and not self._is_stalled:             # While moving and not stalled provide feedback and check for result
          feedback = self._driver.get_current_gripper_status()
          self._action_server.publish_feedback( feedback )
          rospy.logdebug("Error = %.5f Requested position = %.3f Current position = %.3f" % (abs(feedback.requested_position - feedback.position), feedback.requested_position, feedback.position))
          # Check for completition of action 
          if( feedback.fault_status != 0 and not self._is_stalled):               # Check for errors
              rospy.logerr(self._action_name + ": fault status (gFLT) is: %d", feedback.fault_status)
              self._is_stalled = True
              self._action_server.set_aborted( feedback , (self._action_name + ": fault status (gFLT) is: %d" % feedback.fault_status))
              break
          if( abs(feedback.requested_position - feedback.position) < GOAL_DETECTION_THRESHOLD or feedback.obj_detected):    # Check if position has been reached 
              watchdog.shutdown()                         # Stop timeout watchdog.
              self._processing_goal = False 
              self._is_stalled = False              
          rate.sleep()
      
      result = feedback                                   # Message declarations are the same 
      # Send result 
      if not self._is_stalled:
          rospy.logdebug(self._action_name + ": goal reached or object detected Pos: %.3f PosRequested: %.3f ObjectDetected: %r" % (goal_command.position, feedback.requested_position, feedback.obj_detected) )
          self._action_server.set_succeeded(result)  
      else:
          rospy.logerr(self._action_name + ": goal aborted Pos: %.3f PosRequested: %.3f ObjectDetected: %r" % (goal_command.position, feedback.requested_position, feedback.obj_detected) )
          self._action_server.set_aborted(result)  

      self._processing_goal = False 
      self._is_stalled = False 
    
    def execute_joint_trajectory_cb(self, goal):
      """
      FollowJointTrajectory interface used by MoveIt. Only the last trajectory point is sent to the gripper:
      its position is the target, the speed is chosen so that the gripper arrives in the planned time
      (MoveIt velocity scaling is therefore respected) and the force is the point's effort if given,
      otherwise the `~trajectory_force` parameter.
      """
      rospy.loginfo("Trajectory received with %d points", len(goal.trajectory.points))
      feedback = FollowJointTrajectoryFeedback()
      result = FollowJointTrajectoryResult()

      # Check trajectory joint names
      joint_names = goal.trajectory.joint_names
      if len(joint_names) != 1 or joint_names[0] != self._driver._joint_name:
        msg = "Joint trajectory joints %s do not match gripper joint %s" % (list(joint_names), self._driver._joint_name)
        rospy.logerr(msg)
        result.error_code = result.INVALID_JOINTS
        result.error_string = msg
        self._joint_trajectory_action_server.set_aborted(result)
        return
      # Check trajectory points
      if len(goal.trajectory.points) == 0:
        msg = "Ignoring empty trajectory"
        rospy.logerr(msg)
        result.error_code = result.INVALID_GOAL
        result.error_string = msg
        self._joint_trajectory_action_server.set_aborted(result)
        return
      goal_trajectory_point = goal.trajectory.points[-1]
      if len(goal_trajectory_point.positions) != 1:
        msg = "Invalid joint position on trajectory point"
        rospy.logerr(msg)
        result.error_code = result.INVALID_GOAL
        result.error_string = msg
        self._joint_trajectory_action_server.set_aborted(result)
        return

      # Target [m], speed [m/s] to arrive in the planned time, force [%]
      goal_command = CommandRobotiqGripperGoal()
      goal_command.position = self._driver.from_radians_to_distance(goal_trajectory_point.positions[0])
      distance = abs(goal_command.position - self._driver.get_current_gripper_status().position)
      duration = goal_trajectory_point.time_from_start.to_sec()
      if duration > 0.0:
        goal_command.speed = max(MIN_SPEED, min(MAX_SPEED, distance / duration))
      else:
        goal_command.speed = rospy.get_param('~trajectory_speed', MAX_SPEED)
      if len(goal_trajectory_point.effort) > 0:
        goal_command.force = goal_trajectory_point.effort[0]
      else:
        goal_command.force = rospy.get_param('~trajectory_force', 50.0)
      # Time the gripper may take: the expected time at the commanded speed plus a margin
      timeout = 1.5 * distance / max(goal_command.speed, MIN_SPEED) + 1.0
      # GOAL_DETECTION_THRESHOLD is in meters, the feedback error below is in radians
      goal_threshold = abs(self._driver.from_distance_to_radians(0.0) - self._driver.from_distance_to_radians(GOAL_DETECTION_THRESHOLD))
      rospy.logdebug("Gripper target %.3f m, speed %.3f m/s, force %.0f, timeout %.1f s", goal_command.position, goal_command.speed, goal_command.force, timeout)

      # Send incoming command to gripper driver
      self._driver.update_gripper_command(goal_command)
      self._processing_goal = True
      self._is_stalled = False
      watchdog = rospy.Timer(rospy.Duration(timeout), self._execution_timeout, oneshot=True)

      feedback.joint_names = goal.trajectory.joint_names
      feedback.desired.positions = [goal_trajectory_point.positions[0]]
      rate = rospy.Rate(rospy.get_param('~rate', 30))

      while not rospy.is_shutdown() and self._processing_goal and not self._is_stalled:
        # Preempted (new goal or cancel from MoveIt): stop the gripper where it is
        if self._joint_trajectory_action_server.is_preempt_requested():
          watchdog.shutdown()
          goal_command.stop = True
          goal_command.position = self._driver.get_current_gripper_status().position
          self._driver.update_gripper_command(goal_command)
          self._processing_goal = False
          self._joint_trajectory_action_server.set_preempted(result, "Trajectory preempted, gripper stopped")
          return

        current_status = self._driver.get_current_gripper_status()
        feedback.actual.positions = [self._driver.get_current_joint_position()]
        error = abs(feedback.actual.positions[0] - feedback.desired.positions[0])
        feedback.error.positions = [error]
        self._joint_trajectory_action_server.publish_feedback(feedback)

        # Check for errors
        if current_status.fault_status != 0:
          watchdog.shutdown()
          self._processing_goal = False
          msg = "Gripper fault status (gFLT): %d" % current_status.fault_status
          rospy.logerr(msg)
          result.error_code = result.GOAL_TOLERANCE_VIOLATED
          result.error_string = msg
          self._joint_trajectory_action_server.set_aborted(result)
          return
        # Check if object was detected
        if current_status.obj_detected:
          watchdog.shutdown()
          self._processing_goal = False
          result.error_code = result.SUCCESSFUL
          result.error_string = "Object detected/grasped"
          self._joint_trajectory_action_server.set_succeeded(result)
          return
        # Check if the target was reached
        if error < goal_threshold:
          break
        rate.sleep()

      # Let the gripper settle on the target. Bounded, so a position slightly off is still a success.
      settle_deadline = rospy.get_time() + 0.5
      while (not rospy.is_shutdown() and not self._is_stalled and rospy.get_time() < settle_deadline
             and self._driver.get_current_gripper_status().is_moving):
        rate.sleep()

      watchdog.shutdown()
      if rospy.is_shutdown():
        return
      if self._is_stalled:
        msg = "Gripper did not reach %.3f m within %.1f s" % (goal_command.position, timeout)
        rospy.logerr(msg)
        result.error_code = result.GOAL_TOLERANCE_VIOLATED
        result.error_string = msg
        self._joint_trajectory_action_server.set_aborted(result)
      else:
        rospy.logdebug(self._action_name + ": goal reached")
        result.error_code = result.SUCCESSFUL
        result.error_string = "Goal reached"
        self._joint_trajectory_action_server.set_succeeded(result)

      self._processing_goal = False
      self._is_stalled = False

    def _execution_timeout(self, event):
        rospy.logerr("%s: Achieving goal is taking too long, dropping current goal", self._action_name)
        self._is_stalled = True
        self._processing_goal = False

if __name__ == "__main__":

    rospy.init_node('robotiq_2f_action_server')

    # Get Node parameters
    comport = rospy.get_param('~comport','/dev/ttyUSB0')
    baud = rospy.get_param('~baud','115200')
    stroke = rospy.get_param('~stroke', 0.085)                # Default stroke is 85mm (Small C / 2 finger adaptive gripper model)
    joint_name = rospy.get_param('~joint_name', 'finger_joint')    
    sim = rospy.get_param('~sim', False)    

    # Create instance of Robotiq Gripper Driver
    if sim: # Use simulated gripper
        gripper_driver = Robotiq2FingerSimulatedGripperDriver( stroke=stroke, joint_name=joint_name)    
    else:   # Try to connect to a real gripper 
        gripper_driver = Robotiq2FingerGripperDriver( comport=comport, baud=baud, stroke=stroke, joint_name=joint_name)
    # Start action server 
    server = CommandGripperActionServer(rospy.get_namespace(), 'command_robotiq_action', gripper_driver)
    
    # Send and Request data from gripper and update joint state every `r`[Hz]
    r = rospy.Rate(rospy.get_param('~rate', 50 if not sim else 20))
    while not rospy.is_shutdown():
        gripper_driver.update_driver()
        r.sleep()

    rospy.spin()
    
    
