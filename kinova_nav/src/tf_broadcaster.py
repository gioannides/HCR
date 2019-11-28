#!/usr/bin/env python  
import rospy
import tf
 
if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        listener.lookupTransform('/base', '/gripper_finger2_knuckle_link', rospy.Time(0))
        br.sendTransform((0, -0.12, 0),
                        (0.0, 0.0, 0.0, 1.0),
                        rospy.Time.now(),
                        "end_effector_link",
                        "gripper_center")
        rate.sleep()