#!/usr/bin/env python

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
    # self.listener = tf.TransformListener()

    self.drink_poured_pub = rospy.Publisher('/drink_poured', Bool, queue_size=1)
    self.arm_in_position_pub = rospy.Publisher('/arm_in_position', UInt8, queue_size=1)

    # Create the MoveItInterface necessary objects
    arm_group_name = "arm"
    self.robot = moveit_commander.RobotCommander("robot_description")
    self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
    self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())

    # override planning parameters
    self.arm_group.set_planning_time(15)
    self.arm_group.set_goal_tolerance(0.005)
    self.arm_group.set_goal_orientation_tolerance(0.001)
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

  def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4):

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

  def add_collision_box(self,name, x, y, z, size = (1, 1, 1), timeout = 5):
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = self.robot.get_planning_frame()
    box_name = name
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y
    box_pose.pose.position.z = z
    self.scene.add_box(box_name, box_pose, size=size)
    return self.wait_for_state_update(box_name, box_is_known=True, timeout=timeout)

  def add_table(self):
    return self.add_collision_box("table", -0.91, 0, -0.0325, size=(2, 2, 0.025), timeout=15)

  def add_table2(self):
    return self.add_collision_box("table2", 1.3, 0, -0.505, size=(1, 1, 1), timeout=15)

  def add_clamp(self):
    return self.add_collision_box("clamp", 0.09, 0.235, 0.045, size=(0.14, 0.054, 0.20), timeout=4)

  def add_base(self):
    return self.add_collision_box("base", 0, 0, -0.01, size=(0.18, 0.20, 0.02), timeout=4)

  def add_opener(self):
    return self.add_collision_box("opener", 0.16, 0.225, 0.045, size=(0.01, 0.10, 0.02), timeout=4)

  def add_opener_tip(self):
    return self.add_collision_box("opener_tip", 0.16, 0.16, 0.0525, size=(0.01, 0.03, 0.005), timeout=4)

  def attach_bottle(self, bottle_name, timeout=4):

    touch_links = self.robot.get_link_names(group='gripper')
    self.scene.attach_box("gripper_base_link", bottle_name, touch_links=touch_links)
    self.scene.attach_box("gripper_base_link", bottle_name+'_base', touch_links=touch_links)

    return self.wait_for_state_update(bottle_name,box_is_attached=True, box_is_known=False, timeout=timeout) and self.wait_for_state_update(bottle_name+'_base',box_is_attached=True, box_is_known=False, timeout=timeout)
 
  def add_bottle(self, bottle_name, x, y, timeout=4):

    ## The the z postion is constant - on the table
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = self.robot.get_planning_frame()
    box_name = bottle_name
    box_pose.pose.position.x = x
    box_pose.pose.position.y = y
    box_pose.pose.position.z = 0.135/2 - 0.005 #0.5cm below base
    self.scene.add_cylinder(box_name, box_pose, radius = 0.0275, height = 0.135)

    box_pose2 = geometry_msgs.msg.PoseStamped()
    box_pose2.header.frame_id = self.robot.get_planning_frame()
    box_name2 = bottle_name + "_top"
    box_pose2.pose.position.x = box_pose.pose.position.x
    box_pose2.pose.position.y = box_pose.pose.position.y
    box_pose2.pose.position.z = 0.135 - 0.005 + 0.107 / 2
    self.scene.add_cylinder(box_name2, box_pose2, radius =  0.0175, height = 0.107)
    return self.wait_for_state_update(box_name, box_is_known=True, timeout=timeout) and self.wait_for_state_update(box_name2, box_is_known=True, timeout=timeout)

  def detach_object(self, bottle_name, timeout=4):
    self.scene.remove_attached_object("gripper_base_link", name=bottle_name)

    return self.wait_for_state_update(bottle_name, box_is_known=True, box_is_attached=False, timeout=timeout)

  def remove_object(self, obj_name):

    self.scene.remove_world_object(obj_name)

    return self.wait_for_state_update(obj_name,box_is_attached=False, box_is_known=False, timeout=10)

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

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint("gripper_finger1_joint")
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_joint.move(relative_position * gripper_max_absolute_pos, True)

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

  # you should use this function instead of reach_position most of the time
  def go_position(self, x, y, z, yaw, upside_down = False):
    if(upside_down):
      self.reach_position(x+0.12*sin(yaw), y-0.12*cos(yaw), z, -pi/2, 0, yaw)
    else:
      self.reach_position(x-0.12*sin(yaw), y+0.12*cos(yaw), z, pi/2, 0, yaw)

  # move by x,y,z relative to the gripper center
  def move_by_x_y_z(self, x=0,y=0,z=0):
    wpose = self.arm_group.get_current_pose().pose
    wpose.position.x += x
    wpose.position.y += y
    wpose.position.z += z

    self.reach_cartesian_pose(pose=wpose, tolerance=0.01, constraints=None)

  def open_bottle(self):
    pose = self.scene.get_object_poses(['opener_tip'])['opener_tip']
    self.reach_position(pose.position.x,pose.position.y-0.002, pose.position.z-0.175, pi/2,0, -pi/2)

    self.remove_object('opener_tip')
    self.remove_object('opener')
    self.remove_object('clamp')

    self.move_by_x_y_z(y=0.005,z=0.01)
    self.turn_gripper(7)
    self.move_by_x_y_z(z=-0.08)

    self.reach_position(pose.position.x+0.1,pose.position.y, pose.position.z-0.175, pi/2,0, -pi/2)
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

  def callback_pour(self, msg):
    print("navigating to section " + str(msg.data))

    # Add obstacles
    self.add_table()
    self.add_table2()
    self.add_clamp()
    self.add_base()
    self.add_opener()
    self.add_opener_tip()

    self.move_to_section(msg.data)
    
    # Open Gripper
    self.reach_gripper_position(0)
    
    self.arm_in_position_pub.publish(msg)
  
  def callback_beer_position(self, msg):
    print("navigating to position " + str(msg.position))
    # Add bottle and navigate to it
    self.add_bottle("corona1", msg.position.x,msg.position.y)
    self.go_position(msg.position.x, msg.position.y, msg.position.z, pi/2)

    # Pick it up and open it
    self.open_bottle()

    self.drink_poured_pub.publish(True)
    # try:
      
    # except:
    #   self.drink_poured_pub.publish(False)

def main():
    example = KinovaNav()

    # example.add_bottle("corona1", 0.85,0)
    # Uncomment next block if running in integrate system
    example.callback_pour(UInt8(1))
    pose_msg = Pose()
    pose_msg.position.x = 0.85
    pose_msg.position.y = 0
    pose_msg.position.z = 0
    example.callback_beer_position(pose_msg)

    # rospy.spin()

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
