import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import UInt8


class findPosition(object):
    def __init__(self):
        rospy.init_node("tmp_obj_dt")
        self.pub_position = publish('/beer_position', Pose, queue_size=1)
        rospy.Subscriber = ropsy.('/arm_in_position', UInt8, self.callback, queue_size=1)

    def callback(self, msg):
        pose.msg = Pose()
        pose_msg.orientation.x = 0
        pose_msg.orientation.y = 0
        pose_msg.orientation.z = 0
        pose_msg.orientation.w = 0

        pose_msg.position.x = 1
        pose_msg.position.y = 2
        pose_msg.position.z = 3

        self.pub_position(pose_msg)

def main():
    example = findPosition()
    rospy.spin()

if __name__ == '__main__':
  main()
