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
        self.total_time = 0
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
        time.sleep(5) # Leave time to action to complete

    def move_to_joints(self, a_0,a_1,a_2,a_3,a_4,a_5,a_6, timeout=10):
        
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
            # print(joint_angle.value)
            joint_angle.value = joint_angles[joint_id]
            # print(joint_angle.value)
        print("Executing action")
        self.base.ExecuteAction(action)

        print("Waiting 10 seconds for movement to finish ...")
        time.sleep(timeout)
        # print(self.base.GetMeasuredJointAngles())

        print("Angular movement completed")

    def nequiv_coord(self, x, y):
        if (x > y):
            if (x-y <= 0.01):
                return False
        else :
            if (y-x <= 0.01):
                return False
        return True 

    def nequiv_angle(self, x, y):
        if (x > y):
            if (x-y <= 0.75):
                return False
        else :
            if (y-x <= 0.75):
                return False
        return True

    def reach_position(self, x, y, z, angle_x=0, angle_y=0, angle_z=0, timeout=20,  log=False):
        
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
        p = self.base.GetMeasuredCartesianPose()
        start=time.time()
        while(time.time() - start < 17 and (self.nequiv_coord(p.x, x) or self.nequiv_coord(p.y, y) or self.nequiv_coord(p.z, z) or self.nequiv_angle(p.theta_x, angle_x) or self.nequiv_angle(p.theta_y, angle_y) or self.nequiv_angle(p.theta_z, angle_z))):
            # time.sleep(5)
            # if(log):
                # print(self.nequiv_coord(p.x, x))
                # print(self.nequiv_coord(p.y, y))
                # print(self.nequiv_coord(p.z, z))
                # print(self.nequiv_angle(p.theta_x, angle_x))
                # print(self.nequiv_angle(p.theta_y, angle_y)) 
                # print(self.nequiv_angle(p.theta_z, angle_z))
                # print("current position")
                # print(p)
                # print("desired position")
                # print(cartesian_pose)
                # print(time.time()-start)
            p = self.base.GetMeasuredCartesianPose()
        if(time.time()-start > 17):
            print("movement timed out")    
        time.sleep(0.5)    

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
        # print(self.base.GetMeasuredJointAngles())

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
        time.sleep(3)

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
        print(self.base.GetMeasuredCartesianPose())
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
        # height = self.get_bottle_height()

        reference = 0.215
        dif = 0.025

        self.move_to_joints(331.74,108.09,201.49,306.35,143.18,115.53,96.57,timeout=8)
        # self.reach_position(0.2, 0.156, -0.132, -90, -179.99, 90)
        # print("open bottle:")
        # print(self.base.GetMeasuredCartesianPose())


        # reach open position
    # self.reach_position(0.162, 0.164, -0.09+reference-height, -90, -180, 93, timeout=10)
        self.reach_position(0.162, 0.164, -0.09-0.025, -90, -180, 93, timeout=10)

        print("4")
        # print(self.base.GetMeasuredJointAngles())
        # self.reach_position(0.162, 0.164+0.010, -0.09+0.005+reference-height, -90, -180, 93, timeout=5)
        self.reach_position(0.165, 0.164+0.007, -0.09-0.025, -90, -180, 93, timeout=5)

        print("5")
        # print(self.base.GetMeasuredJointAngles())
        # self.reach_position(0.162, 0.164+0.010, -0.09+0.003+reference-height, -90, -178, 93, timeout=5)
        self.reach_position(0.165, 0.164+0.007, -0.09-0.03, -90, -160, 93, timeout=1)

        print("6")
        # print(self.base.GetMeasuredJointAngles())
        self.reach_gripper_position(1)
        # self.reach_position(0.162, 0.164+0.010, -0.09+0.003-0.05+reference-height, -90, -175, 93, timeout=5)
        
        self.reach_position(0.165, 0.164, -0.09-0.025, -90, -170, 93, timeout=5)
        self.reach_position(0.165, 0.164+0.007, -0.09-0.025, -90, -170, 93, timeout=5)

        self.reach_position(0.165, 0.164+0.008, -0.09-0.055, -90, -170, 93, timeout=5)
        print("7")
        # print(self.base.GetMeasuredJointAngles())
       
        self.reach_position(0.165+0.05, 0.164+0.008, -0.09-0.055, -90, -175, 93, timeout=5)
       
        # 0.0493
        print("8")
        # print(self.base.GetMeasuredJointAngles())

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
        st = time.time()
        pose_msg = self.convert_camera(msg)

        # Create connection to the device and get the router
        with utilities.DeviceConnection.createTcpConnection(self.args) as router:
            self.base = BaseClient(router)
           

            # go towards the bottle
            self.go_position(pose_msg.position.x - 0.1 + 0.02, pose_msg.position.y, 0.09, 90, timeout=7)
            # print("1")
            # print(self.base.GetMeasuredJointAngles())
            self.reach_gripper_position(0)
            
            self.go_position(pose_msg.position.x + 0.02, pose_msg.position.y, 0.09, 90, timeout=7)
            self.reach_gripper_position(1)
            # print("2")
            # print(self.base.GetMeasuredJointAngles())
            self.go_position(pose_msg.position.x + 0.02, pose_msg.position.y, 0.40, 90, timeout=5)
            # print("3")
            # print(self.base.GetMeasuredJointAngles())
            # self.move_to_section()

            #  pouring

            # move home
            # self.move_to_joints(360, 15, 180, 230, 360, 55, 90)
            # self.example_move_to_home_position()
            self.reach_position(0.577, 0.00136, 0.4336, 90, 0, 90)

            print("going home:")
            print(self.base.GetMeasuredCartesianPose())
            print("selection : {}".format(self.selection))
            if(self.selection != 4):
                self.open_bottle()
                # move home
                self.move_to_joints(360, 15, 180, 230, 360, 55, 90, timeout=7)
                # self.example_move_to_home_position()
                # self.reach_position(0.577, 0.00136, 0.4336, 90, 0, 90)
                self.pour_bottle()
            self.move_to_joints(96.92, 117.56, 280.03, 288.72, 332.69, 345.76, 358.18, timeout=5)
            print("dropping bottle")
            self.reach_gripper_position(0)
            
            
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
        self.total_time += time.time() - st 
        print("total time: {0}", self.total_time)

        self.drink_poured_pub.publish(True)
        self.pour_drink_sub = rospy.Subscriber("/pour_drink", UInt8, self.callback_pour, queue_size=1)
    # except:
    #  self.drink_poured_pub.publish(False)

    def pour_bottle(self):
        # with utilities.DeviceConnection.createTcpConnection(self.args) as router:
            # Create required services
            # self.base = BaseClient(router)
        # height = self.get_bottle_height()
            reference = 0.245
            self.move_to_joints(88.38,93.75,252.25,268.96,267.93,17.62,87.91, timeout=5.5)
            print("pouring:")
        # print(self.base.GetMeasuredCartesianPose())

        # print("pour1")
            self.reach_position(-0.583, -0.416, 0.19, -90,180,90, timeout=10, log=True)
            print("pour2")
            self.reach_position(-0.583, -0.41, 0.19, -90,-115,90, timeout=10, log = True)
            # start pouring
            self.reach_position(-0.583, -0.38, 0.19, -90,-105,90, timeout=5, log=True)
            print("pour3")
            self.reach_position(-0.583, -0.38, 0.19, -90,-95,90, timeout=5, log=True)
            time.sleep(0.5)
            print("pour4")
            self.reach_position(-0.583, -0.365, 0.18, -90,-93,90, timeout=5, log=True)
            print("pour5")
            time.sleep(0.75)
            self.reach_position(-0.583, -0.35, 0.18, -90,-89,90, timeout=5,log=True)
            print("pour7")
            time.sleep(0.75)
            # self.reach_position(-0.583, -0.335, 0.18, -90,-89,90, timeout=5,log=True)
            # print("pour8")
            # time.sleep(2)
            self.reach_position(-0.583, -0.335, 0.27, -90,-88,90, timeout=5,log=True)
            print("pour9")
            time.sleep(0.5)
            self.reach_position(-0.583, -0.3, 0.35, -90,-95,90, timeout=5,log=True)
            # self.reach_position(-0.583, -0.22, 0.40, -90, 180,90, timeout=5,log=True)
            # print(self.base.GetMeasuredJointAngles())
            self.move_to_joints(93.07, 62.45, 271.32, 239.69, 5.02, 33.13, 23.25, timeout=5)
            print("pour10")
            
            # self.reach_position(-0.583, -0.416, 0.50, -90,180,90, timeout=20,log=True)
            # self.move_to_joints(85.81, 61.43, 272.54, 275.02, 356.46, 351.28, 32.18)

    def callback_pour(self, msg):
        self.pour_drink_sub.unregister()
        print("navigating to section " + str(msg.data))
        self.selection=msg.data
        st = time.time()
        # Create connection to the device and get the router
        with utilities.DeviceConnection.createTcpConnection(self.args) as router:
            # Create required services
            self.base = BaseClient(router)
            self.table_num = msg.data
            # go towards the opener

            # move home
            # print("joint speed limits:")
            # print(self.base.GetAllJointsSpeedHardLimitation())      
            # print("joint torque limits:")
            # print(self.base.GetAllJointsTorqueSoftLimitation())      
            # print("joint twist limits:")
            # print(self.base.GetTwistSoftLimitation())      
            # print("joint wrench limits:")
            # print(self.base.GetWrenchSoftLimitation())      
            # print("end of limits")
            # print(self.base.GetMeasuredCartesianPose().x)
            # print(self.base.GetMeasuredJointAngles().joint_angles.value[0])
            self.move_to_joints(360, 15, 180, 230, 360, 55, 90, timeout=7)
            print(self.base.GetMeasuredCartesianPose())

            # print(self.base.GetMeasuredCartesianPose().x)
            # self.example_move_to_home_position()
            # print("let's see the angles after going home")
            # print(self.base.GetMeasuredJointAngles())
            # print("these are the angles")
            # self.move_to_joints(331.74,108.09,201.49,306.35,143.18,115.53,96.57)
            # print("let's see the angles of the angular move")
            # print(self.base.GetMeasuredJointAngles())
            # print("these are the angles")
            
            # print("let's see the angles of the angular move")
            # print(self.base.GetMeasuredJointAngles())
            # print("these are the angles")

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
        self.total_time = time.time() - st
        self.arm_in_position_pub.publish(msg)
        
        print("published to george")

def main():
    kinova = KinovaNav()
    # kinova.pour_bottle()
    rospy.spin()
        

if __name__ == "__main__":
    main()
