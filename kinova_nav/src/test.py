#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 1018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import os
import time

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient

from kortex_api.autogen.messages import Base_pb2, Common_pb2, ControlConfig_pb2
 
def example_move_to_home_position(base):
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

def example_angular_action_movement(base):
    
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


def cartesian_movement(base):
    
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


def example_angular_trajectory_movement(base):
    
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

def go_to_bottle(base):
    
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

def pick_up_bottle(base):

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
    
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)

        # # Example core
        example_move_to_home_position(base)
        # go_to_bottle(base)
        # pick_up_bottle(base)
        

if __name__ == "__main__":
    main()
