#!/usr/bin/env python3

import sys
import os
import time
import rospy
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, UInt8
from math import pi, cos, sin
# Import the utilities helper module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import utilities

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient

from kortex_api.autogen.messages import Base_pb2, Common_pb2, ControlConfig_pb2

class KinovaNav(object):
    """KinovaNav"""
    def __init__(self):

        # Parse arguments
        self.args = utilities.parseConnectionArguments()

        rospy.init_node("kinova_nav")
        self.pour_drink_sub = rospy.Subscriber("/pour_drink", UInt8, self.callback_pour, queue_size=1)
        self.beer_position_sub = rospy.Subscriber("/beer_position", Pose, self.callback_beer_position, queue_size=1)

        self.drink_poured_pub = rospy.Publisher('/drink_poured', Bool, queue_size=1)
        self.arm_in_position_pub = rospy.Publisher("/arm_in_position", UInt8, queue_size=1)

 
    def example_move_to_home_position(self):
        # Make sure the arm is in Single Level Servoing mode
        base_servo_mode = Base_pb2.ServoingModeInformation()
        base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
        self.base.SetServoingMode(base_servo_mode)
        
        # Move arm to ready position
        print("Moving the arm to a safe position")
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = self.base.ReadAllActions(action_type)
        action_handle = None
        for action in action_list.action_list:
            if action.name == "Home":
                action_handle = action.handle

        if action_handle == None:
            print("Can't reach safe position. Exiting")
            sys.exit(0)

        self.base.ExecuteActionFromReference(action_handle)
        time.sleep(10) # Leave time to action to complete

    def move_to_joints(self, a_0,a_1,a_2,a_3,a_4,a_5,a_6):
        
        print("Starting angular action movement ...")
        action = Base_pb2.Action()
        action.name = "Example angular action movement"
        action.application_data = ""

        actuator_count = self.base.GetActuatorCount()

        joint_angles = {0:a_0,1:a_1,2:a_2,3:a_3,4:a_4,5:a_5,6:a_6}

        # Place arm straight up
        for joint_id in range(actuator_count.count):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = joint_id
            joint_angle.value = joint_angles[joint_id]

        print("Executing action")
        self.base.ExecuteAction(action)

        print("Waiting 10 seconds for movement to finish ...")
        time.sleep(10)

        print("Angular movement completed")


    def reach_position(self, x, y, z, angle_x, angle_y, angle_z,timeout=20):
        
        print("Starting movement to pick up bottle")
        action = Base_pb2.Action()
        action.name = "Picking up Bottle"
        action.application_data = ""

        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x = x
        cartesian_pose.y = y
        cartesian_pose.z = z
        cartesian_pose.theta_x = angle_x
        cartesian_pose.theta_y = angle_y
        cartesian_pose.theta_z = angle_z

        print(cartesian_pose)

        print("Executing action")
        self.base.ExecuteAction(action)

        time.sleep(timeout)

        print("Cartesian movement completed")

     # you should use this function instead of reach_position most of the time
    def go_position(self, x, y, z, yaw, upside_down = False, timeout=20):
        if(upside_down):
            self.reach_position(x, y, z, -90, 0, yaw-180, timeout)
        else:
            self.reach_position(x, y, z, 90, 0, yaw, timeout)


    def go_upside_down(self):
        
        print("Starting angular action movement ...")
        action = Base_pb2.Action()
        action.name = "Example angular action movement"
        action.application_data = ""

        actuator_count = self.base.GetActuatorCount()
        print(self.base.GetMeasuredJointAngles())

        # Place arm straight up
        for joint_id in range(actuator_count.count):
            print(joint_id)
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = joint_id
            joint_angle.value = 0

        print("Executing action")
        self.base.ExecuteAction(action)

        print("Waiting 10 seconds for movement to finish ...")
        time.sleep(10)

        print("Angular movement completed")

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


    def reach_gripper_position(self, x):

        # Create the GripperCommand we will send
        gripper_command = Base_pb2.GripperCommand()
        finger = gripper_command.gripper.finger.add()

        # Close the gripper with position increments
        print("Performing gripper test in position...")
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger.finger_identifier = 1

        finger.value = x
        self.base.SendGripperCommand(gripper_command)
        time.sleep(5)

    def section_position(self):
        x = 0.514
        z = 0.193
        camera_offset = 0.04
        if (self.table_num == 1):
            y = -0.21 
        elif (self.table_num == 2):
            y = -0.07
        elif (self.table_num == 3):
            y = 0.07
        else:
            y = 0.21

        v1 = Pose()
        v1.position.x = x
        v1.position.y = y - camera_offset
        v1.position.z = z

        return v1

    def move_to_section(self):
        # go to the correct section
        section_pose = self.section_position()
        print(section_pose)
        self.go_position(section_pose.position.x, section_pose.position.y, section_pose.position.z, 90, timeout=10)

    def convert_camera(self, msg):

        current_position = self.section_position()
        
        v1 = Pose()
        v1.position.x = msg.position.z
        v1.position.y = -msg.position.x
        v1.position.z = msg.position.y
        
        v2 = Pose()
        v2.position.x = -0.19
        v2.position.y = 0.04
        v2.position.z = -0.07
        
        pose_msg = Pose()
        pose_msg.position.x = v1.position.x + v2.position.x + current_position.position.x
        pose_msg.position.y = v1.position.y + v2.position.y + current_position.position.y
        pose_msg.position.z = v1.position.z + v2.position.z + current_position.position.z
        print("translated_bottle x: {}".format(pose_msg.position.x))
        print("translated_bottle y: {}".format(pose_msg.position.y))
        print("translated_bottle z: {}".format(pose_msg.position.z))

        return pose_msg

    def get_bottle_height(self):
        if (self.table_num == '1'): # hop house
            height = 0.2
        elif (self.table_num == '2'): # san miguel
            height = 0.225
        elif (self.table_num == '3'): # birra moretti
            height = 0.22
        else:                       # becks
            height = 0.215

        return height

    def open_bottle(self):
        height = self.get_bottle_height()

        reference = 0.215


        self.move_to_joints(331.74,108.09,201.49,306.35,143.18,115.53,96.57)

        # reach open position
        self.reach_position(0.162, 0.164, -0.09+reference-height, -90, -180, 93, timeout=10)

        self.reach_position(0.162, 0.164+0.010, -0.09+0.005+reference-height, -90, -180, 93, timeout=10)
        self.reach_position(0.162, 0.164+0.010, -0.09+0.003+reference-height, -90, -178, 93, timeout=10)
        self.reach_gripper_position(1)
        self.reach_position(0.162, 0.164+0.010, -0.09+0.003-0.05+reference-height, -90, -175, 93, timeout=10)
        self.reach_position(0.162+0.10, 0.164+0.010, -0.09+0.003-0.05+reference-height, -90, -175, 93, timeout=10)

        self.example_move_to_home_position()

    def opening_demo(self):
        # go towards the bottle
        self.go_position(pose_msg.position.x - 0.1 + 0.02, pose_msg.position.y, 0.09, 90, timeout=10)
        self.reach_gripper_position(0)
        self.go_position(pose_msg.position.x + 0.02, pose_msg.position.y, 0.09, 90, timeout=10)
        self.reach_gripper_position(1)
        self.go_position(pose_msg.position.x + 0.02, pose_msg.position.y, 0.40, 90, timeout=5)

        # self.move_to_section()

        #  pouring
        self.example_move_to_home_position()
        # self.reach_gripper_position(1)
        # self.reach_position(0.333, -0.471, 0.434, 90,0,35.1, timeout=10)
        # self.reach_position(-0.044, -0.575, 0.434, -90,180,175.5, timeout=10)
        # self.reach_position(-0.467, -0.338, 0.434, -90,180,125.7, timeout=10)
        # self.reach_position(-0.583, -0.416, 0.15, -90,180,90, timeout=10)
        self.move_to_joints(88.38,93.75,252.25,268.96,267.93,17.62,87.91)
        self.reach_position(-0.578, -0.416, 0.068, -90,180,90, timeout=10)
        self.reach_gripper_position(0)
        self.reach_position(-0.578+0.1, -0.416, 0.068, -90,180,90, timeout=10)
        self.example_move_to_home_position()

    def callback_beer_position(self, msg):

        print('recieved from goerge')

        pose_msg = self.convert_camera(msg)

        # Create connection to the device and get the router
        with utilities.DeviceConnection.createTcpConnection(self.args) as router:
            self.base = BaseClient(router)
           

            # go towards the bottle
            self.go_position(pose_msg.position.x - 0.1 + 0.02, pose_msg.position.y, 0.09, 90, timeout=10)
            self.reach_gripper_position(0)
            self.go_position(pose_msg.position.x + 0.02, pose_msg.position.y, 0.09, 90, timeout=10)
            self.reach_gripper_position(1)
            self.go_position(pose_msg.position.x + 0.02, pose_msg.position.y, 0.40, 90, timeout=5)

            # self.move_to_section()

            #  pouring
            self.example_move_to_home_position()

            self.open_bottle()

            self.example_move_to_home_position()

            self.pour_bottle()
            # self.reach_gripper_position(1)
            # self.reach_position(0.333, -0.471, 0.434, 90,0,35.1, timeout=10)
            # self.reach_position(-0.044, -0.575, 0.434, -90,180,175.5, timeout=10)
            # self.reach_position(-0.467, -0.338, 0.434, -90,180,125.7, timeout=10)
            # self.reach_position(-0.583, -0.416, 0.15, -90,180,90, timeout=10)
            # self.move_to_joints(88.38,93.75,252.25,268.96,267.93,17.62,87.91)
            # self.reach_position(-0.578, -0.416, 0.068, -90,180,90, timeout=10)
            # self.reach_gripper_position(0)
            # self.reach_position(-0.578+0.1, -0.416, 0.068, -90,180,90, timeout=10)
            # self.example_move_to_home_position()
            # self.reach_position(-0.583, -0.416, 0.17, -90,180,90, timeout=10)
            # self.reach_position(-0.583, -0.416, 0.17, -90,-107.5,90, timeout=20)
            # self.reach_position(-0.583, -0.38, 0.17, -90,-95,90, timeout=10)
            # self.reach_position(-0.583, -0.36, 0.17, -90,-90,90, timeout=10)
            # self.reach_position(-0.583, -0.416, 0.17, -90,180,90, timeout=20)


            # self.example_move_to_home_position()

            # # go towards the opener
            # self.reach_position(0.314, 0.148, -0.145, -90, -180, 0, timeout=20)
            # self.reach_position(0.199, 0.156, -0.132, -90, -180, 90, timeout=20)

            # # # san miguel open position
            # self.open_bottle()

            # # move to bottle opening
            # self.example_move_to_home_position()
            # self.reach_position(0.333, -0.471, 0.434, 90,0,35.1, timeout=10)
            # self.reach_position(-0.044, -0.575, 0.434, -90,180,175.5, timeout=10)
            # self.reach_position(-0.467, -0.338, 0.434, -90,180,125.7, timeout=10)
            # # self.reach_position(-0.575, -0.001, 0.434, -90,180,90, timeout=10)
            # # self.reach_position(-0.575, -0.001, 0.07, -90,180,90, timeout=10)
            # # self.reach_gripper_position(0)


        # self.move_by_x_y_z(z=-0.1)

        # self.open_bottle()
        
        # print("--- %s seconds ---" % (time.time() - self.start_time))

      # pose = self.scene.get_object_poses(['opener_tip'])['opener_tip']
      # self.reach_position(pose.position.x,pose.position.y, pose.position.z-0.4, pi/2,0, -pi/2)
      # self.reach_position(pose.position.x,pose.position.y, pose.position.z-0.1, pi/2,0, -pi/2)
    # self.reach_position(0.55,0, 0.09,pi/2,0, pi/2)
    # time.sleep(2)
      # self.reach_position(0.55,0, 0.09,pi/2,0, pi/2)
      
        self.drink_poured_pub.publish(True)
    # except:
    #  self.drink_poured_pub.publish(False)

    def pour_bottle(self):
        height = self.get_bottle_height()
        reference = 0.245
        self.move_to_joints(88.38,93.75,252.25,268.96,267.93,17.62,87.91)
        self.reach_position(-0.583, -0.416+reference-height, 0.17, -90,180,90, timeout=10)
        self.reach_position(-0.583, -0.41+reference-height, 0.17, -90,-115,90, timeout=20)
        self.reach_position(-0.583, -0.38+reference-height, 0.17, -90,-95,90, timeout=5)
        self.reach_position(-0.583, -0.36+reference-height, 0.17, -90,-90,90, timeout=5)
        self.reach_position(-0.583, -0.35+reference-height, 0.17, -90,-88,90, timeout=10)
        self.reach_position(-0.583, -0.416+reference-height, 0.17, -90,180,90, timeout=20)


    def callback_pour(self, msg):
        self.pour_drink_sub.unregister()
        print("navigating to section " + str(msg.data))

        # Create connection to the device and get the router
        with utilities.DeviceConnection.createTcpConnection(self.args) as router:
            # Create required services
            self.base = BaseClient(router)
            self.table_num = msg.data
            # go towards the opener
            self.example_move_to_home_position()
            # self.open_bottle()
            # self.reach_position(0.314, 0.148, -0.145, -90, -180, 0, timeout=20)
            # self.move_to_joints(331.74,108.09,201.49,306.35,143.18,115.53,96.57)
            # self.reach_position(0.199, 0.156, -0.132, -90, -180, 90, timeout=20)
            # self.open_bottle()
            # self.example_move_to_home_position()
            # self.reach_gripper_position(0)
            # self.go_position(0.752, 0.089, 0.09, 90, timeout=15)
            # self.reach_gripper_position(1)
            # self.example_move_to_home_position()

            

            self.move_to_section()

            # self.pour_bottle()

            # if (self.table_num == '1'): # hop house
            #     height = 0.2
            # elif (self.table_num == '2'): # san miguel
            #     height = 0.225
            # elif (self.table_num == '3'): # birra moretti
            #     height = 0.22
            # else:                       # becks
            #     height = 0.215

            # reference = 0.215

            # self.example_move_to_home_position()

            # self.example_angular_action_movement()


            # self.example_move_to_home_position()

            # self.reach_gripper_position(0)
            # self.go_position(0.752, 0.089, 0.09, 90, timeout=15)
            # self.reach_gripper_position(1)

            # self.example_move_to_home_position()


            # # move to bottle opening
            # self.example_move_to_home_position()
            # self.reach_position(0.333, -0.471, 0.434, 90,0,35.1, timeout=10)
            # self.reach_position(-0.044, -0.575, 0.434, -90,180,175.5, timeout=10)
            # self.reach_position(-0.467, -0.338, 0.434, -90,180,125.7, timeout=10)

            

            
            # self.example_move_to_home_position()

            #  bottle opening
            # self.example_move_to_home_position()
            # self.reach_position(0.333, -0.471, 0.434, 90,0,35.1, timeout=10)
            # self.reach_position(-0.044, -0.575, 0.434, -90,180,175.5, timeout=10)
            # self.reach_position(-0.467, -0.338, 0.434, -90,180,125.7, timeout=10)

            # self.reach_position(0.314, 0.148, -0.145, -90, -180, 0, timeout=20)
            # self.reach_position(0.199, 0.156, -0.132, -90, -180, 90, timeout=20)

            # go towards the opener
            # self.reach_position(0.314, 0.148, -0.145, -90, -180, 0, timeout=20)
            # self.reach_position(0.199, 0.156, -0.132, -90, -180, 90, timeout=20)
            # self.open_bottle()


            #  pouring
            # self.example_move_to_home_position()
            # self.reach_gripper_position(1)
            # self.reach_position(0.333, -0.471, 0.434, 90,0,35.1, timeout=10)
            # self.reach_position(-0.044, -0.575, 0.434, -90,180,175.5, timeout=10)
            # self.reach_position(-0.467, -0.338, 0.434, -90,180,125.7, timeout=10)
            # self.reach_position(-0.583, -0.416, 0.15, -90,180,90, timeout=10)



            # self.reach_position(0.752, 0.089, 0.184, 90,-75,90, timeout=10)
            # self.reach_position(0.752, 0.089, 0.184, 90,-85,90, timeout=15)
            # self.reach_position(0.752, 0.079, 0.184, 90,-85,90, timeout=10)
            # self.reach_position(0.752, 0.079, 0.184, 90,-90,90, timeout=10)
            # self.reach_position(0.752, 0.065, 0.184, 90,-90,90, timeout=10)
            # self.reach_position(0.752, 0.065, 0.184, 90,-93,90, timeout=10)
            # self.reach_position(0.752, 0.065, 0.184, 90,0,90, timeout=10)

            # self.reach_position(0.314, 0.148, -0.145, -90, -180, 0, timeout=20)
            # self.reach_position(0.199, 0.156, -0.132, -90, -180, 90, timeout=20)

            # san miguel open position
            # self.reach_position(0.153, 0.164, -0.105, -90, -180, 90, timeout=20)

            # self.reach_position(0.153, 0.164+0.008, -0.105+0.005, -90, -180, 90, timeout=20)
            # self.reach_position(0.153, 0.164+0.008, -0.105+0.005, -90, -173, 90, timeout=20)
            # self.reach_position(0.153, 0.164+0.008, -0.105+0.005-0.05, -90, -173, 90, timeout=20)
            # self.reach_position(0.153, 0.164+0.01, -0.105+0.002-0.05, -90, -173, 90, timeout=20)
            
            # san miguel open position
            # self.reach_position(0.153, 0.164, -0.105, -90, -180, 90, timeout=20)

            # self.reach_position(0.153, 0.164+0.013, -0.105+0.002, -90, -180, 90, timeout=20)
            # self.reach_position(0.153, 0.164+0.01, -0.105+0.002, -90, -173, 90, timeout=20)
            # self.reach_position(0.153, 0.164+0.01, -0.105+0.002-0.05, -90, -173, 90, timeout=20)



            # self.go_position(0.634, -0.20, 0.09, 90, timeout=4)
            # # self.reach_gripper_position(1)
            # self.go_position(0.634, -0.20, 0.3, 90, timeout=4)
            # self.go_position(0.634, -0.20, 0.09, 90, timeout=4)
            # # self.reach_gripper_position(1)
            # self.go_position(0.634, -0.20, 0.3, 90, timeout=4)
            # self.reach_position(-0.475, -0.12, 0.08, -90,-180,90, timeout=20)
            # self.go_position(0.485, -0.294, 0.432, 69.3)
            # self.go_position(0.099, -0.564, 0.432, 3.9)
            # self.reach_position(-0.285, -0.421, 0.286, -90,-180,132.2)
            # self.reach_position(-0.474, -0.139, 0.189, -90,-180,90)
            # self.reach_position(-0.474, -0.139, 0.07, -90,-180,90)2)


            # move to bottle opening
            # self.reach_position(0.333, -0.471, 0.434, 90,0,35.1, timeout=10)
            # self.reach_position(-0.044, -0.575, 0.434, -90,180,175.5, timeout=10)
            # self.reach_position(-0.467, -0.338, 0.434, -90,180,125.7, timeout=10)
            # self.reach_position(-0.575, -0.001, 0.434, -90,180,90, timeout=10)
            # self.reach_position(-0.575, -0.001, 0.07, -90,180,90, timeout=10)
            # self.reach_gripper_position(0)
            # self.go_position(0, -0.52, 0.08, 0, timeout=20)
            # self.example_move_to_home_position()
            # self.go_upside_down()
            # self.go_position(0.49, 0, 0.1, pi/2, upside_down=True)

        self.arm_in_position_pub.publish(msg)
        print("published to george")

def main():
    kinova = KinovaNav()
    rospy.spin()
        

if __name__ == "__main__":
    main()
