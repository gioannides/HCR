#!/usr/bin/env python
import os
import rospy

import time
import sys
from std_msgs.msg import Bool,String, Float64, UInt8
from cob_perception_msgs.msg import ColorDepthImageArray as FoundFace

class JustTouchpad(object):
    def __init__(self):
        # publisher to interaction_complete
        self.pub_interaction_complete = rospy.Publisher("/interaction_complete", Bool, queue_size=1)
        
        # publisher to touchpad
        self.pub_to_screen = rospy.Publisher("/term2_displayScreenX",UInt8, queue_size=1)
        
        # publisher to arm
        self.pub_to_arm = rospy.Publisher("/pour_drink", UInt8, queue_size=1)

        time.sleep(0.1)

        # put selection drink
        self.pub_to_screen.publish(1) # select drink screen

        # subscribed to button pressed
        self.sub_selected = rospy.Subscriber("/term2_buttonPressed", UInt8, self.callback_sendarm)

        # subscribe to NAO in operation
        self.sub_wait = rospy.Subscriber("/customerwelcomed",String, self.callback_wait_nao_done)
        print("JustTouchpad INIT DONE")
    
    def callback_wait_nao_done(self, msg):
        # once nao started, ignore touchpad
        self.sub_selected.unregister()
        print("waiting NAO done")
        self.sub_interaction_complete = rospy.Subscriber("/interaction_complete", Bool, self.callback_interaction_done)

    def callback_interaction_done(self, msg):
        # once NAO done, wait until button pressed
        self.sub_interaction_complete.unregister()
        print("waiting button pressed")
        self.sub_selected = rospy.Subscriber("/term2_buttonPressed", UInt8, self.callback_sendarm)

    def callback_sendarm(self,msg):
        print("sent to arm")
        print(msg.data)
        if (msg.data==0):
            return
        self.sub_selected.unregister() # ignore new button pressed
        self.pub_to_arm.publish(msg) # tell arm to start pouring
        print("Sent To Arm")
        self.pub_to_screen.publish(2) # "drink is being poured"
        self.sub_ready = rospy.Subscriber("/drink_poured", Bool, self.callback_drink_ready)
    def callback_drink_ready(self,msg):
        self.sub_ready.unregister() # ignore other messages from arm
        self.pub_to_screen.publish(4) # enjoy responsibly
        time.sleep(2)
        print("Drink Ready please collect function called")
        self.pub_to_screen.publish(6) # goodbye
        time.sleep(2)
        # subscribe to button pressed
        self.sub_selected = rospy.Subscriber("/term2_buttonPressed",UInt8, self.callback_sendarm)
        time.sleep(1)
        # drink selection screen
        self.pub_to_screen.publish(1)
        # interaction is now complete
        self.pub_interaction_complete.publish(True)

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
