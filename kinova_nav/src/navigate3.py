#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot

# To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run : 
#rosrun kinova_nav navigate.py __ns:="my_gen3"

## THE kortex_driver.launch NEEDS TO BE MODIFIED!!
# arg name="default_goal_time_tolerance" default="0"/> <!--seconds-->

import sys
import time
import os
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, UInt8
from math import pi, cos, sin

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient

from kortex_api.autogen.messages import Base_pb2, Common_pb2, ControlConfig_pb2

class KinovaNav(object):
  """KinovaNav"""
  def __init__(self):
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Initialize the node
    super(KinovaNav, self).__init__()
    rospy.init_node("kinova_nav",  argv=sys.argv)
    sys.argv = rospy.myargv(argv=sys.argv)
    self.pour_drink_sub = rospy.Subscriber("/pour_drink", UInt8, self.callback_pour, queue_size=1)
    self.beer_position_sub = rospy.Subscriber("/beer_position", Pose, self.callback_beer_position, queue_size=1)

    self.drink_poured_pub = rospy.Publisher('/drink_poured', Bool, queue_size=1)
    self.arm_in_position_pub = rospy.Publisher("/arm_in_position", UInt8, queue_size=1)

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        self.base = BaseClient(router)

  def convert_camera(self, msg):
    effector= self.arm_group.get_current_pose().pose
    
    v1 = Pose()
    v1.position.x = msg.position.z
    v1.position.y = -msg.position.x
    v1.position.z = msg.position.y
    
    v2 = Pose()
    v2.position.x = -0.076
    v2.position.y = 0.025
    v2.position.z = 0.07
    
    pose_msg = Pose()
    pose_msg.position.x = v1.position.x + v2.position.x + effector.position.x
    pose_msg.position.y = v1.position.y + v2.position.y + effector.position.y
    pose_msg.position.z = v1.position.z + v2.position.z + effector.position.z
    print("translated_bottle x: {}".format(pose_msg.position.x))
    print("translated_bottle y: {}".format(pose_msg.position.y))
    print("translated_bottle z: {}".format(pose_msg.position.z))

    return pose_msg
  
  def callback_beer_position(self, msg):

    pose_msg = self.convert_camera(msg)

    time.sleep(2)
      
    self.drink_poured_pub.publish(True)

  def callback_pour(self, msg):
    self.pour_drink_sub.unregister()
    print("navigating to section " + str(msg.data))

    self.arm_in_position_pub.publish(msg)
    print("published to george")

  def example_move_to_home_position(self,base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Home":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        sys.exit(0)

    base.ExecuteActionFromReference(action_handle)
    time.sleep(10) # Leave time to action to complete

def example_angular_action_movement(self,base):
    
    print("Starting angular action movement ...")
    action = Base_pb2.Action()
    action.name = "Example angular action movement"
    action.application_data = ""

    actuator_count = base.GetActuatorCount()

    # Place arm straight up
    for joint_id in range(actuator_count.count):
        joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = joint_id
        joint_angle.value = 0

    print("Executing action")
    base.ExecuteAction(action)

    print("Waiting 10 seconds for movement to finish ...")
    time.sleep(10)

    print("Angular movement completed")


def cartesian_movement(self,base):
    
    print("Starting movement to pick up bottle")
    action = Base_pb2.Action()
    action.name = "Picking up Bottle"
    action.application_data = ""

    cartesian_pose = action.reach_pose.target_pose
    cartesian_pose.x = 0.80     # (meters)
    cartesian_pose.y = 0        # (meters)
    cartesian_pose.z = 0.36     # (meters)
    cartesian_pose.theta_x = 10 # (degrees)
    cartesian_pose.theta_y = 90 # (degrees)
    cartesian_pose.theta_z = 10 # (degrees)

    print("Executing action")
    base.ExecuteAction(action)

    print("Cartesian movement completed")


def example_angular_trajectory_movement(self,base):
    
    constrained_joint_angles = Base_pb2.ConstrainedJointAngles()

    actuator_count = base.GetActuatorCount()

    # Place arm straight up
    for joint_id in range(actuator_count.count):
        joint_angle = constrained_joint_angles.joint_angles.joint_angles.add()
        joint_angle.joint_identifier = joint_id
        joint_angle.value = 0

    print("Reaching joint angles...")
    base.PlayJointTrajectory(constrained_joint_angles)

    print("Waiting 10 seconds for movement to finish ...")
    time.sleep(10)

    print("Joint angles reached")

def go_to_bottle(self,base):
    
    constrained_pose = Base_pb2.ConstrainedPose()

    cartesian_pose = constrained_pose.target_pose
    cartesian_pose.x = 0.76     # (meters)
    cartesian_pose.y = 0.12     # (meters)
    cartesian_pose.z = 0.08     # (meters)
    cartesian_pose.theta_x = 90 # (degrees)
    cartesian_pose.theta_y = 0 # (degrees)
    cartesian_pose.theta_z = 90 # (degrees)

    print("Reaching cartesian pose...")
    base.PlayCartesianTrajectory(constrained_pose)

    print("Waiting 10 seconds for movement to finish ...")
    time.sleep(10)

def pick_up_bottle(self,base):

    #  # Create the GripperCommand we will send
    # gripper_command = Base_pb2.GripperCommand()
    # finger = gripper_command.gripper.finger.add()

    # # Close the gripper with position increments
    # print("Performing gripper test in position...")
    # gripper_command.mode = Base_pb2.GRIPPER_POSITION
    # position = 0.00
    # finger.finger_identifier = 1
    # while position < 1.0:
    #     finger.value = position
    #     base.SendGripperCommand(gripper_command)
    #     position += 0.1
    #     time.sleep(1)

    # # Create the GripperCommand we will send
    # gripper_command = Base_pb2.GripperCommand()
    # finger = gripper_command.gripper.finger.add()

    # # Close the gripper with position increments
    # print("Performing gripper test in position...")
    # gripper_command.mode = Base_pb2.GRIPPER_POSITION
    # finger.finger_identifier = 1
    
    # finger.value = 0
    # base.SendGripperCommand(gripper_command)
    # time.sleep(1)

    # finger.value = 0.2
    # base.SendGripperCommand(gripper_command)
    # time.sleep(1)

    cartesian_pose = ControlConfig_pb2.Position()
    print(cartesian_pose)
    cartesian_pose     # (meters)
    
    print(cartesian_pose)

    print("Reaching cartesian pose...")
    base.PlayCartesianTrajectory(cartesian_pose)

    print("Waiting 10 seconds for movement to finish ...")
    time.sleep(5)


def ExampleSendGripperCommands(self):

    # Create the GripperCommand we will send
    gripper_command = Base_pb2.GripperCommand()
    finger = gripper_command.gripper.finger.add()

    # Close the gripper with position increments
    print("Performing gripper test in position...")
    gripper_command.mode = Base_pb2.GRIPPER_POSITION
    position = 0.00
    finger.finger_identifier = 1
    while position < 1.0:
        finger.value = position
        self.base.SendGripperCommand(gripper_command)
        position += 0.1
        time.sleep(1)

    # Set speed to open gripper
    print ("Opening gripper using speed command...")
    gripper_command.mode = Base_pb2.GRIPPER_SPEED
    finger.value = 0.1
    self.base.SendGripperCommand(gripper_command)
    gripper_request = Base_pb2.GripperRequest()

    # Wait for reported position to be opened
    gripper_request.mode = Base_pb2.GRIPPER_POSITION
    while True:
        gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
        if len (gripper_measure.finger):
            print("Current position is : {0}".format(gripper_measure.finger[0].value))
            if gripper_measure.finger[0].value < 0.01:
                break
        else: # Else, no finger present in answer, end loop
            break

    # Set speed to close gripper
    print ("Closing gripper using speed command...")
    gripper_command.mode = Base_pb2.GRIPPER_SPEED
    finger.value = -0.1
    self.base.SendGripperCommand(gripper_command)

    # Wait for reported speed to be 0
    gripper_request.mode = Base_pb2.GRIPPER_SPEED
    while True:
        gripper_measure = self.base.GetMeasuredGripperMovement(gripper_request)
        if len (gripper_measure.finger):
            print("Current speed is : {0}".format(gripper_measure.finger[0].value))
            if gripper_measure.finger[0].value == 0.0:
                break
        else: # Else, no finger present in answer, end loop
            break

def main():
  example = KinovaNav()
  example.example_move_to_home_position(example.base)

if __name__ == '__main__':
  main()