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
from math import pi
from std_srvs.srv import Empty
import numpy as np

class ExampleMoveItTrajectories(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(ExampleMoveItTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('example_move_it_trajectories')

    # Create the MoveItInterface necessary objects
    arm_group_name = "arm"
    self.robot = moveit_commander.RobotCommander("robot_description")
    self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
    self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
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

  def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4):
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

  def set_scene(self, timeout=4):

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


  def attach_box(self, timeout=4):
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
    grasping_group = 'end_effector'
    touch_links = self.robot.get_link_names(group=grasping_group)
    scene.attach_box(self.eef_link, 'box', touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

  def detach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## BEGIN_SUB_TUTORIAL detach_object
    ##
    ## Detaching Objects from the Robot
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can also detach and remove the object from the planning scene:
    scene.remove_attached_object(eef_link, name=box_name)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## Removing Objects from the Planning Scene
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## We can remove the box from the world.
    scene.remove_world_object(box_name)

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
    rospy.loginfo("Planning and going to the Cartesian Pose")
    arm_group.go(wait=True)

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint("gripper_finger1_joint")
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_joint.move(relative_position * gripper_max_absolute_pos, True)

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

    rospy.loginfo(actual_pose)

    self.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)

  def pick_up_object(self):
    self.reach_position(0.62, 0, 0.09, pi/2, 0, pi/2)
    self.move_up(0.1)

  def move_up(self, z):
    wpose = self.arm_group.get_current_pose().pose
    wpose.position.z += z

    self.reach_cartesian_pose(pose=wpose, tolerance=0.01, constraints=None)

  def tilt(self, radians):
    wpose = self.arm_group.get_current_pose().pose
    euler = tf.transformations.euler_from_quaternion(pose_to_q(wpose))
    quaternion = tf.transformations.quaternion_from_euler(euler[0],euler[1]-radians, euler[2])

    wpose.orientation.x =  quaternion[0]
    wpose.orientation.y = quaternion[1]
    wpose.orientation.z = quaternion[2]
    wpose.orientation.w = quaternion[3]
    
    self.reach_cartesian_pose(pose=wpose, tolerance=0.01, constraints=None)

  def reach_x_y(self, x, y, angle_x, angle_y, angle_z):
    wpose = self.arm_group.get_current_pose().pose
    self.reach_position(x, y, wpose.position.z, angle_x, angle_y, angle_z)

def main():
    example = ExampleMoveItTrajectories()

    # simulator scene
    # example.set_scene()

    example.reach_named_position("home")

    example.reach_position(0.55,0, 0.09,pi/2,0, pi/2)
    example.reach_gripper_position(0)
    example.reach_position(0.62,0, 0.09,pi/2,0, pi/2)
    example.reach_gripper_position(0.15)
    example.move_up(0.1)
    example.reach_x_y(0,0.62,-pi/2,-pi, 0)
    example.tilt(-pi/3)
    time.sleep(0.5)
    example.tilt(-pi/3)

#   rospy.loginfo("Press any key to start Named Target Vertical sub example")
#   raw_input()
#   example.reach_named_position("vertical")

#   rospy.loginfo("Press any key to start Reach Joint Angles sub example")
#   raw_input()
#   example.reach_joint_angles(tolerance=0.01) #rad

#   rospy.loginfo("Press any key to start Named Target Home sub example")
#   raw_input()
#   example.reach_named_position("home")

#   rospy.loginfo("Press any key to start Reach Cartesian Pose sub example")
#   raw_input()
#   actual_pose = example.get_cartesian_pose()
#   actual_pose.position.x -= 0.1
#   actual_pose.position.y += 0.3
#   actual_pose.position.z += 0.3
#   example.reach_cartesian_pose(pose=actual_pose, tolerance=0.01, constraints=None)
  
#   if True:

#     rospy.loginfo("Press any key to start Reach Cartesian Pose With Constraints sub example")
#     raw_input()
    
#     # Get actual pose
#     actual_pose = example.get_cartesian_pose()
#     actual_pose.position.y -= 0.3
    
#     # Orientation constraint (we want the end effector to stay the same orientation)
  #     constraints = moveit_msgs.msg.Constraints()
  #     orientation_constraint = moveit_msgs.msg.OrientationConstraint()
  #     orientation_constraint.orientation = actual_pose.orientation
  #     constraints.orientation_constraints.append(orientation_constraint)

def pose_to_q(msg):
  """Convert a C{geometry_msgs/Pose} into position/quaternion np arrays

  @param msg: ROS message to be converted
  @return:
    - p: position as a np.array
    - q: quaternion as a numpy array (order = [x,y,z,w])
  """
  q = np.array([msg.orientation.x, msg.orientation.y,
                msg.orientation.z, msg.orientation.w])
  return q

if __name__ == '__main__':
  main()

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
