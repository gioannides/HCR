#!/usr/bin/env python
import os
import rospy

import time
import sys
from std_msgs.msg import Bool,String, Float64, UInt8
from cob_perception_msgs.msg import ColorDepthImageArray as FoundFace

class JustTouchpad(object):
    def __init__(self):
        self.pub_customer_welcomed = rospy.Publisher("/customerwelcomed",String, queue_size=1)
        self.pub_to_screen = rospy.Publisher("/term1_displayScreenX",UInt8, queue_size=1)
        self.pub_to_arm = rospy.Publisher("/pour_drink", UInt8, queue_size=1)
        self.pub_to_kinect = rospy.Publisher("/tilt_angle",Float64, queue_size=1)
        self.pub_to_kinect.publish(40)

        self.sub_customer = rospy.Subscriber("/face_detector/face_positions", FoundFace, self.callback_foundcustomer, queue_size=1)
        print("JustTouchpad INIT DONE")
    def callback_foundcustomer(self,msg):
        if(len(msg.head_detections)>0):
            print("customer found")
            self.sub_customer.unregister()
            self.pub_customer_welcomed.publish("Welcomed")
            self.sub_age = rospy.Subscriber("/above18",Bool, self.callback_drink_select)
    def callback_drink_select(self,msg):
        print("ID Found")
        self.pub_to_kinect.publish(40) # move kinect back up
        self.sub_age.unregister() # stop looking for age
        print("Drink Select Called")
        self.pub_to_screen.publish(1) # select drink screen
        self.sub_selected = rospy.Subscriber("/term1_buttonPressed",UInt8, self.callback_sendarm)
    def callback_sendarm(self,msg):
        self.sub_selected.unregister() # ignore new button pressed
        self.pub_to_arm.publish(msg) # tell arm to start pouring
        print("Sent To Arm")
        self.pub_to_screen.publish(2)
        self.sub_ready = rospy.Subscriber("/drink_poured", Bool, self.callback_drink_ready)
    def callback_drink_ready(self,msg):
        self.sub_ready.unregister() # ignore other messages from arm
        self.pub_to_screen.publish(4) # enjoy responsibly
        time.sleep(3)
        print("Drink Ready please collect function called")
        self.pub_to_screen.publish(3) # other drink?
        self.sub_selected = rospy.Subscriber("/term1_buttonPressed",UInt8, self.callback_anotherDrink)
    def callback_anotherDrink(self,msg):
        print("customer responded")
        print(msg)
        self.sub_selected.unregister()
        if(msg.data==3):
            # other drink selected, go back to selection screen
            time.sleep(1)
            self.callback_drink_select("")
        else:
            # restart process
            self.pub_to_screen.publish(4) #change later on with goodbye screen
            time.sleep(3) # wait to avoid detecting same customer
            self.sub_customer = rospy.Subscriber("/face_detector/face_positions", FoundFace, self.callback_foundcustomer, queue_size=1)

if __name__ == "__main__":
    try:
        rospy.init_node("JustTouchpad",  argv=sys.argv)
        sys.argv = rospy.myargv(argv=sys.argv)
        justouchpad = JustTouchpad()
        rospy.spin()
    except rospy.exceptions.ROSInterruptException:
        print("See ya")
    except rospy.ROSException as e:
        if str(e) == "publish() to a closed topic":
            print("See ya")
        else:
            raise e
    except KeyboardInterrupt:
        print("Shutting down")
    print()  # print new line
