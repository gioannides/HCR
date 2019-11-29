#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import UInt8

class FindPosition(object):
    def __init__(self):
        rospy.init_node("tmp_obj_dt")
        self.pub_position = rospy.Publisher('/beer_position', Pose, queue_size=1)
        rospy.Subscriber('/arm_in_position', UInt8, self.callback, queue_size=1)

    def callback(self, msg):
        print("detected bottle")
        pose_msg = Pose()

        pose_msg.position.x = 0.4
        pose_msg.position.y = 0
        pose_msg.position.z = 0.4

        self.pub_position.publish(pose_msg)

def main():
    example = FindPosition()
    rospy.spin()

if __name__ == '__main__':
  main()
