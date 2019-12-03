#!/usr/bin/env python

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
import rospy
import tf
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, UInt8
from math import pi, cos, sin
from std_srvs.srv import Empty
import numpy as np

class KinovaNav(object):
  """KinovaNav"""
  def __init__(self):

    # Initialize the node
    super(KinovaNav, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('kinova_nav')
    self.pour_drink_sub = rospy.Subscriber("/pour_drink", UInt8, self.callback_pour, queue_size=1)
    self.beer_position_sub = rospy.Subscriber("/beer_position", Pose, self.callback_beer_position, queue_size=1)
    self.listener = tf.TransformListener()

    self.drink_poured_pub = rospy.Publisher('/drink_poured', Bool, queue_size=1)
    self.arm_in_position_pub = rospy.Publisher("/arm_in_position", UInt8, queue_size=1)

    # Create the MoveItInterface necessary objects
    arm_group_name = "arm"
    self.robot = moveit_commander.RobotCommander("robot_description")
    self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
    self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
    # print self.robot.get_group_names()
    # self.arm_group.set_planning_time(30)
    # self.arm_group.set_planner_id("EST")
    # self.arm_group.set_end_effector_link("gripper_base_link")

    # override planning parameters
    self.arm_group.set_planning_time(15)
    # self.arm_group.set_goal_tolerance(0.005)
    # self.arm_group.set_goal_orientation_tolerance(0.001)
    self.arm_group.set_num_planning_attempts(5)

    self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    gripper_group_name = "gripper"
    self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())
    self.eef_link = self.arm_group.get_end_effector_link()
     # We can get the name of the reference frame for this robot:
    self.planning_frame = self.gripper_group.get_planning_frame()
    print "============ Reference frame: %s" % self.planning_frame

    # We can also print the name of the end-effector link for this group:
    self.eef_link = self.gripper_group.get_end_effector_link()
    print "============ End effector: %s" % self.eef_link

    # We can get a list of all the groups in the robot:
    self.group_names = self.robot.get_group_names()
    print "============ Robot Groups:", self.robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print self.robot.get_current_state()
    print ""

    rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())

  def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=17):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## Ensuring Collision Updates Are Receieved
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## If the Python node dies before publishing a collision object update message, the message
    ## could get lost and the box will not appear. To ensure that the updates are
    ## made, we wait until we see the changes reflected in the
    ## ``get_known_object_names()`` and ``get_known_object_names()`` lists.
    ## For the purpose of this tutorial, we call this function after adding,
    ## removing, attaching or detaching an object in the planning scene. We then wait
    ## until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = self.scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in self.scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    ## END_SUB_TUTORIAL

  # turn gripper in degrees
  def turn_gripper(self, angle, clockwise = True):
    arm_group = self.arm_group

    self.arm_group.set_goal_joint_tolerance(0.001)

    # Set the joint target configuration
    joint_positions = arm_group.get_current_joint_values()
    if(clockwise):
      joint_positions[6] -= angle/180.0*pi
    else:
      joint_positions[6] += angle/180.0*pi
      
    arm_group.set_joint_value_target(joint_positions)
    
    # Plan and execute in one command
    arm_group.go(wait=True)

  def set_scene(self, timeout=17):

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    rospy.sleep(2)
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.pose.orientation.w = 0.707
    box_pose.pose.orientation.x = 0.707
    box_pose.header.frame_id = self.robot.get_planning_frame()
    box_pose.pose.position.x = 0.6
    box_pose.pose.position.y = 0.05
    box_name = "box"
    self.scene.add_mesh(box_name, box_pose, "./beer.stl", size=(0.002,0.002,0.002))
    return self.wait_for_state_update(box_name, box_is_known=True, timeout=timeout)

  def add_table(self, timeout=15):

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = self.robot.get_planning_frame()
    box_name = "table"
    box_pose.pose.position.x = -0.91
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = -0.0325
    self.scene.add_box(box_name, box_pose, size=(2, 2, 0.025))
    return self.wait_for_state_update(box_name, box_is_known=True, timeout=timeout)

  def add_table_edge(self, timeout=15):

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = self.robot.get_planning_frame()
    box_name = "table_edge"
    box_pose.pose.position.x = 0.2
    box_pose.pose.position.y = -0.5 - 0.6
    box_pose.pose.position.z = -0.0325
    self.scene.add_box(box_name, box_pose, size=(0.4, 1, 0.025))
    return self.wait_for_state_update(box_name, box_is_known=True, timeout=timeout)

  def add_section(self, timeout=15):

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = self.robot.get_planning_frame()
    box_name = "section"
    # +str(label)
    box_pose.pose.position.x = 0.9378
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = -0.3105
    self.scene.add_box(box_name, box_pose, size=(0.43, 0.6, 0.813))
    return self.wait_for_state_update(box_name, box_is_known=True, timeout=timeout)

  def add_clamp(self, timeout=7):

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = self.robot.get_planning_frame()
    box_name = "clamp"
    box_pose.pose.position.x = 0.09
    box_pose.pose.position.y = 0.235
    box_pose.pose.position.z = 0.045
    self.scene.add_box(box_name, box_pose, size=(0.14, 0.054, 0.20))
    return self.wait_for_state_update(box_name, box_is_known=True, timeout=timeout)

  def add_base(self, timeout=7):

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = self.robot.get_planning_frame()
    box_name = "base"
    box_pose.pose.position.x = 0
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = -0.01
    self.scene.add_box(box_name, box_pose, size=(0.18, 0.20, 0.02))
    return self.wait_for_state_update(box_name, box_is_known=True, timeout=timeout)

  def add_opener(self, timeout=7):

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = self.robot.get_planning_frame()
    box_name = "opener"
    box_pose.pose.position.x = 0.1525
    box_pose.pose.position.y = 0.2275
    box_pose.pose.position.z = 0.045
    self.scene.add_box(box_name, box_pose, size=(0.025, 0.095, 0.02))
    return self.wait_for_state_update(box_name, box_is_known=True, timeout=timeout)

  def add_opener_tip(self, timeout=7):

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = self.robot.get_planning_frame()
    box_name = "opener_tip"
    box_pose.pose.position.x = 0.1525
    box_pose.pose.position.y = 0.1625
    box_pose.pose.position.z = 0.0525
    self.scene.add_box(box_name, box_pose, size=(0.025, 0.035, 0.005))
    return self.wait_for_state_update(box_name, box_is_known=True, timeout=timeout)

  def add_wall(self, timeout=7):

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = self.robot.get_planning_frame()
    box_name = "wall"
    box_pose.pose.position.x = -0.7
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = 0
    self.scene.add_box(box_name, box_pose, size=(0.2, 2, 2))
    return self.wait_for_state_update(box_name, box_is_known=True, timeout=timeout)

  def attach_bottle(self, timeout=17):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    # box_name = self.box_name
    # robot = self.robot
    # scene = self.scene
    # eef_link = self.eef_link
    # group_names = self.group_names

    ## BEGIN_SUB_TUTORIAL attach_object
    ##
    ## Attaching Objects to the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    ## robot be able to touch them without the planning scene reporting the contact as a
    ## collision. By adding link names to the ``touch_links`` array, we are telling the
    ## planning scene to ignore collisions between those links and the box. For the Panda
    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    ## you should change this value to the name of your end effector group name.
    touch_links = self.robot.get_link_names(group="gripper")
    self.scene.attach_box("gripper_base_link", 'bottle_base', touch_links=touch_links)
    self.scene.attach_box("gripper_base_link", 'bottle_top', touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update("bottle_base", box_is_attached=True, box_is_known=True, timeout=timeout) and self.wait_for_state_update("bottle_top", box_is_attached=True, box_is_known=True, timeout=timeout)

  def add_bottle(self, x, y, timeout=7):

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    print("adding bottle")
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = self.robot.get_planning_frame()
    box_name = "bottle_base"
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y
    # box_pose.pose.position.z = 0.096+0.13/2
    box_pose.pose.position.z = 0.096+0.13/2
    self.scene.add_box(box_name, box_pose, size=(0.040, 0.040, 0.125))

    box_pose2 = geometry_msgs.msg.PoseStamped()
    box_pose2.header.frame_id = self.robot.get_planning_frame()
    box_name2 = "bottle_top"
    box_pose2.pose.position.x = box_pose.pose.position.x
    box_pose2.pose.position.y = box_pose.pose.position.y
    box_pose2.pose.position.z = 0.096 + 0.13 + 0.105 / 2
    self.scene.add_box(box_name2, box_pose2, size=(0.025, 0.025, 0.105))
    return self.wait_for_state_update(box_name, box_is_known=True, timeout=timeout) and self.wait_for_state_update(box_name2, box_is_known=True, timeout=timeout)

  def add_realsense(self, timeout=7):

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## Adding Objects to the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = self.robot.get_planning_frame()
    box_name = "realsense"
    #self.reach_named_position("home")
    box_pose.pose.position.x = 0.38
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = 0.37
    self.scene.add_box(box_name, box_pose, size=(0.03, 0.2, 0.04))

    box_name = "realsense_base"
    box_pose.pose.position.x = 0.42
    box_pose.pose.position.y = 0
    box_pose.pose.position.z = 0.39
    self.scene.add_box(box_name, box_pose, size=(0.11, 0.04, 0.01))

    touch_links = self.robot.get_link_names(group='gripper')
    self.scene.attach_box("gripper_base_link", "realsense", touch_links=touch_links)
    self.scene.attach_box("gripper_base_link", "realsense_base", touch_links=touch_links)

    return self.wait_for_state_update("realsense", box_is_known=True, box_is_attached=True, timeout=timeout) and self.wait_for_state_update("realsense_base", box_is_attached=True, box_is_known=True, timeout=timeout)

  def detach_object(self, bottle_name, timeout=4):
    self.scene.remove_attached_object("gripper_base_link", name=bottle_name)

    return self.wait_for_state_update(bottle_name, box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_object(self, obj_name):

    self.scene.remove_world_object(obj_name)

    return self.wait_for_state_update(obj_name,box_is_attached=False, box_is_known=False, timeout=10)

  def remove_box(self, box_name):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.

    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    self.scene.remove_world_object(box_name)

    ## **Note:** The object must be detached before we can remove it from the world
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

  def reach_named_position(self, target):
    arm_group = self.arm_group
    
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    planned_path1 = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    arm_group.execute(planned_path1, wait=True)

  def get_current_pose(self):
    return self.arm_group.get_current_pose().pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      #import pdb; pdb.set_trace()
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    # Plan and execute
    # rospy.loginfo("Planning and going to the Cartesian Pose")
    arm_group.go(wait=True)

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint("gripper_finger1_joint")
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_joint.move(relative_position * gripper_max_absolute_pos, True)

  # def reach_position2(self, x, y, z, angle_x, angle_y, angle_z):

  #   # (trans,rot) = self.listener.lookupTransform('/gripper_finger1_knuckle_link', '/gripper_finger2_knuckle_link', rospy.Time(0))
  #   # print(trans,rot)
  #   quaternion = tf.transformations.quaternion_from_euler(angle_x,angle_y, angle_z)

  #   actual_pose = geometry_msgs.msg.PoseStamped()
  #   actual_pose.header.frame_id = self.robot.get_planning_frame()
  #   # this works for -pi/2,0,x
  #   # actual_pose.pose.position.x = x + 0.12*sin(angle_z)
  #   # actual_pose.pose.position.y = y - 0.12*cos(angle_z)
  #   # actual_pose.pose.position.z = z
  #   # this works for pi/2,0,x
  #   actual_pose.pose.position.x = x - 0.12*sin(angle_z)
  #   actual_pose.pose.position.y = y + 0.12*cos(angle_z)
  #   actual_pose.pose.position.z = z
  #   # actual_pose.pose.position.x = x + trans[0]
  #   # actual_pose.pose.position.y = y + trans[1]
  #   # actual_pose.pose.position.z = z + trans[2]
  #   actual_pose.pose.orientation.x =  quaternion[0]
  #   actual_pose.pose.orientation.y = quaternion[1]
  #   actual_pose.pose.orientation.z = quaternion[2]
  #   actual_pose.pose.orientation.w = quaternion[3]

  #   rospy.loginfo(actual_pose)

  #   self.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)

  def reach_position(self, x, y, z, angle_x, angle_y, angle_z):
    quaternion = tf.transformations.quaternion_from_euler(angle_x,angle_y, angle_z)

    actual_pose = geometry_msgs.msg.PoseStamped()
    actual_pose.header.frame_id = self.robot.get_planning_frame()
    actual_pose.pose.position.x = x
    actual_pose.pose.position.y = y
    actual_pose.pose.position.z = z
    actual_pose.pose.orientation.x =  quaternion[0]
    actual_pose.pose.orientation.y = quaternion[1]
    actual_pose.pose.orientation.z = quaternion[2]
    actual_pose.pose.orientation.w = quaternion[3]

    # rospy.loginfo(actual_pose)

    self.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)

  # you should use this function instead of reach_position most of the time
  def go_position(self, x, y, z, yaw, upside_down = True):
    if(upside_down):
      self.reach_position(x-0.12*sin(yaw), y+0.12*cos(yaw), z, -pi/2, 0, yaw-pi)
    else:
      self.reach_position(x-0.12*sin(yaw), y+0.12*cos(yaw), z, pi/2, 0, yaw)

  def go_waypoints_path(self, scale=1):
    waypoints = []

    wpose = self.arm_group.get_current_pose().pose

    wpose.position.z += 0.05  # First move up (z)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x -= 0.2  # then move back
    waypoints.append(copy.deepcopy(wpose))

    

    # print(waypoints)

    # # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # # which is why we will specify 0.01 as the eef_step in Cartesian
    # # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = self.arm_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0)         # jump_threshold

    # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    # display_trajectory.trajectory_start = self.robot.get_current_state()
    # display_trajectory.trajectory.append(plan)
    # # Publish
    # self.display_trajectory_publisher.publish(display_trajectory)


    self.arm_group.execute(plan, wait=True)

  # move by x,y,z relative to the gripper center
  def move_by_x_y_z(self, x=0,y=0,z=0):
    wpose = self.arm_group.get_current_pose().pose
    wpose.position.x += x
    wpose.position.y += y
    wpose.position.z += z

    self.reach_cartesian_pose(pose=wpose, tolerance=0.01, constraints=None)

  def open_bottle(self):
    pose = self.scene.get_object_poses(['opener_tip'])['opener_tip']
    self.go_position(pose.position.x,pose.position.y+0.003, pose.position.z-0.145,-pi/2)

    self.remove_object('opener_tip')
    self.remove_object('opener')
    self.remove_object('clamp')

    self.move_by_x_y_z(y=0.015,z=0.01)
    self.turn_gripper(7)
    self.move_by_x_y_z(z=-0.08)

    self.add_clamp()
    self.add_opener()
    self.add_opener_tip()

  def move_to_section(self, table_num):
    # go to the correct section
    if (table_num == '1'):
      self.reach_named_position("home")
    elif (table_num == '2'):
      self.reach_named_position("home")
    elif (table_num == '3'):
      self.reach_named_position("home")
    else:
      self.reach_named_position("home")

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

    self.add_bottle(pose_msg.position.x+0.03, pose_msg.position.y)

    self.reach_gripper_position(0)

    pose = self.scene.get_object_poses(['bottle_base'])['bottle_base']
    print("above bottle")
    # self.go_position(pose.position.x+0.02, pose.position.y, pose.position.z+0.2, pi/2)
    print("bottle level")
    self.go_position(pose.position.x, pose.position.y, pose.position.z+0.02, pi/2)

    # self.go_position(pose.position.x+0.02, pose.position.y, pose.position.z+0.5, pi/2)

    print("grippers")
    self.attach_bottle()
    self.reach_gripper_position(0.7)
    self.move_by_x_y_z(z=-0.1)

    self.open_bottle()

    self.start_time = time.time() - self.start_time
    print("--- %s seconds ---" % (time.time() - self.start_time))

      # pose = self.scene.get_object_poses(['opener_tip'])['opener_tip']
      # self.reach_position(pose.position.x,pose.position.y, pose.position.z-0.4, pi/2,0, -pi/2)
      # self.reach_position(pose.position.x,pose.position.y, pose.position.z-0.1, pi/2,0, -pi/2)
    # self.reach_position(0.55,0, 0.09,pi/2,0, pi/2)
    time.sleep(2)
      # self.reach_position(0.55,0, 0.09,pi/2,0, pi/2)
      
    self.drink_poured_pub.publish(True)
    # except:
    #  self.drink_poured_pub.publish(False)

  def callback_pour(self, msg):
    self.start_time = time.time()
    self.pour_drink_sub.unregister()
    print("navigating to section " + str(msg.data))
    # try:
    self.add_table()
    self.add_clamp()
    self.add_base()
    self.add_opener()
    self.add_opener_tip()
    self.add_section()
    print("section should be added now")
    self.reach_named_position("home")
    self.add_realsense()
    # go to new home position
    self.start_time = time.time()
    self.go_position(0.49, 0, 0.1, pi/2, upside_down=True)

    self.arm_in_position_pub.publish(msg)
    print("published to george")

      # example.reach_position(-0.4,-0.4, 0.4,-pi/2,0, pi/4)

      # pose = self.scene.get_object_poses(['bottle_base'])['bottle_base']
      # self.reach_position(pose.position.x,pose.position.y, pose.position.z+0.3, -pi/2,0, 3*pi/4)
      # self.reach_gripper_position(0.40)
      # self.drink_poured_pub.publish(True)
    # except:
      # self.drink_poured_pub.publish(False)

    self.start_time = time.time() - self.start_time 

def main():
    example = KinovaNav()
    example.add_table()
    example.add_table()
    example.add_clamp()
    example.add_base()
    example.add_opener()
    example.add_opener_tip()
    example.add_section()
    example.add_table_edge()
    example.add_wall()
    example.add_bottle(0.88, 0.05)

    # example.reach_named_position("home")
    
    # example.go_position(0.88,0.05, 0.17,pi/2)
    # example.attach_bottle()
    # example.reach_named_position("home")
    pose = example.scene.get_object_poses(['opener_tip'])['opener_tip']
    example.go_position(pose.position.x,pose.position.y+0.003, pose.position.z-0.16,-pi/2)
    lol = {}
    planners = ["SBL","EST", "LBKPIECE", "BKPIECE","KPIECE","RRT","RRTConnect","RRTstar","TRRT","PRM","PRMstar","FMT","BFMT","PDST","STRIDE","BiTRRT","LBTRRT","BiEST","ProjEST","LazyPRM","LazyPRMstar","SPARS","SPARStwo"]
    for x in planners:
      print("RUNNING:"+ x)
      start_time = time.time()
      example.arm_group.set_planner_id(x)
      for y in range(0,3):
        # example.detach_object('bottle_base')
        # example.detach_object('bottle_top')
        # example.remove_object('bottle_base')
        # example.remove_object('bottle_top')
        # example.add_bottle(0.88, 0.05)
        example.go_position(0.88,0.05, 0.17,pi/2)
        # example.attach_bottle()
        pose = example.scene.get_object_poses(['opener_tip'])['opener_tip']
        example.go_position(pose.position.x,pose.position.y+0.003, pose.position.z-0.16,-pi/2)
      start_time = time.time() - start_time
      lol[x] = start_time

      print(x + ": --- %s seconds ---" % (start_time))

    print(lol)
    # example.open_bottle()
    # example.move_by_x_y_z(z=-0.1)

    # example.reach_named_position("home")

    # example.reach_position2(0.25, 0, 0.1, -pi/2, 0, -pi/2)
    # example.go_position(0.49, 0, 0.1, pi/2, upside_down=True)
    
    # print("section should be added now")
    # example.reach_named_position("home")
    # example.add_realsense()

    # example.open_bottle()

    # example.add_bottle(0.70,0)
    # Uncomment next block if running in integrate system
    # example.callback_pour(UInt8(1))
    # pose_msg = Pose()
    # pose_msg.position.x = 0.85
    # pose_msg.position.y = 0
    # pose_msg.position.z = 0
    # example.callback_beer_position(pose_msg)


    # example.go_position(0.49, 0, 0.1, pi/2, upside_down=True)
    # example.open_bottle()
    # example.reach_named_position("home")
    # example.go_waypoints_path()
    # pose = example.scene.get_object_poses(['opener_tip'])['opener_tip']
    # example.go_position(pose.position.x,pose.position.y+0.003, pose.position.z-0.145,-pi/2)
    # rospy.spin()

def pose_to_q(msg):
  """Convert a C{geometry_msgs/Pose} into position/quaternion np arrays

  @param msg: ROS message to be converted
  @return:,p: position as a np.array,q: quaternion as a numpy array (order = [x,y,z,w])
  """
  q = np.array([msg.orientation.x, msg.orientation.y,
                msg.orientation.z, msg.orientation.w])
  return q

if __name__ == '__main__':
  main()

  # FUNCTIONS HERE MIGHT BE USEFUL LATER PLS DON'T DELETE!!!!

  # def plan_cartesian_path(self, scale=1):
  #   # Copy class variables to local variables to make the web tutorials more clear.
  #   # In practice, you should use the class variables directly unless you have a good
  #   # reason not to.
  #   group = self.arm_group

  #   ## BEGIN_SUB_TUTORIAL plan_cartesian_path
  #   ##
  #   ## Cartesian Paths
  #   ## ^^^^^^^^^^^^^^^
  #   ## You can plan a Cartesian path directly by specifying a list of waypoints
  #   ## for the end-effector to go through:
  #   ##
  #   waypoints = []

  #   wpose = group.get_current_pose().pose
  #   wpose.position.z += 0.2  # First move up (z)
  #   waypoints.append(copy.deepcopy(wpose))

  #   # We want the Cartesian path to be interpolated at a resolution of 1 cm
  #   # which is why we will specify 0.01 as the eef_step in Cartesian
  #   # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
  #   (plan, fraction) = group.compute_cartesian_path(
  #                                      waypoints,   # waypoints to follow
  #                                      0.01,        # eef_step
  #                                      0.0)         # jump_threshold

  #   # Note: We are just planning, not asking move_group to actually move the robot yet:
  #   return plan, fraction

  #   ## END_SUB_TUTORIAL

  # def display_trajectory(self, plan):
  #   # Copy class variables to local variables to make the web tutorials more clear.
  #   # In practice, you should use the class variables directly unless you have a good
  #   # reason not to.
  #   robot = self.robot
  #   display_trajectory_publisher = self.display_trajectory_publisher

  #   ## BEGIN_SUB_TUTORIAL display_trajectory
  #   ##
  #   ## Displaying a Trajectory
  #   ## ^^^^^^^^^^^^^^^^^^^^^^^
  #   ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
  #   ## group.plan() method does this automatically so this is not that useful
  #   ## here (it just displays the same trajectory again):
  #   ##
  #   ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
  #   ## We populate the trajectory_start with our current robot state to copy over
  #   ## any AttachedCollisionObjects and add our plan to the trajectory.
  #   display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  #   display_trajectory.trajectory_start = robot.get_current_state()
  #   display_trajectory.trajectory.append(plan)
  #   # Publish
  #   display_trajectory_publisher.publish(display_trajectory)

  #   ## END_SUB_TUTORIAL

  # def execute_plan(self, plan):
  #   # Copy class variables to local variables to make the web tutorials more clear.
  #   # In practice, you should use the class variables directly unless you have a good
  #   # reason not to.
  #   group = self.arm_group

  #   ## BEGIN_SUB_TUTORIAL execute_plan
  #   ##
  #   ## Executing a Plan
  #   ## ^^^^^^^^^^^^^^^^
  #   ## Use execute if you would like the robot to follow
  #   ## the plan that has already been computed:
  #   group.execute(plan, wait=True)

  #   ## **Note:** The robot's current joint state must be within some tolerance of the
  #   ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
  #   ## END_SUB_TUTORIAL

  # Get actual pose
#     actual_pose = example.get_cartesian_pose()
#     actual_pose.position.y -= 0.3
    
#     # Orientation constraint (we want the end effector to stay the same orientation)
  #     constraints = moveit_msgs.msg.Constraints()
  #     orientation_constraint = moveit_msgs.msg.OrientationConstraint()
  #     orientation_constraint.orientation = actual_pose.orientation
  #     constraints.orientation_constraints.append(orientation_constraint)

  # def tilt(self, radians):
  #   wpose = self.arm_group.get_current_pose().pose
  #   euler = tf.transformations.euler_from_quaternion(pose_to_q(wpose))
  #   quaternion = tf.transformations.quaternion_from_euler(euler[0],euler[1]-radians, euler[2])

  #   wpose.orientation.x =  quaternion[0]
  #   wpose.orientation.y = quaternion[1]
  #   wpose.orientation.z = quaternion[2]
  #   wpose.orientation.w = quaternion[3]
    
  #   self.reach_cartesian_pose(pose=wpose, tolerance=0.01, constraints=None)