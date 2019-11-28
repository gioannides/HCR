#!/usr/bin/env python


import os
import rospy
import rospkg
import time
import argparse
from std_msgs.msg import Bool,String, Float64, UInt8
from cob_perception_msgs.msg import ColorDepthImageArray as FoundFace
from naoqi import ALProxy
import sys
sys.path.append("/home/human/catkin_ws/src/HCR/nao_scripts/scripts")
from nao2 import hello_ID,drink_select,choice,dance,joke,joke1,joke2,pickup

sys.dont_write_bytecode = True


class NAO(object):
    def __init__(self):
        self.pub_customer_welcomed = rospy.Publisher("/customerwelcomed",String, queue_size=1)
        self.pub_to_screen = rospy.Publisher("/term1_displayScreenX",UInt8, queue_size=1)
        self.pub_to_arm = rospy.Publisher("/pour_drink", UInt8, queue_size=1)
        self.pub_to_kinect = rospy.Publisher("/tilt_angle",Float64, queue_size=1)
        self.parser = argparse.ArgumentParser()
        self.parser.add_argument("--ip", type=str, default= "192.168.43.70",
                            help="Robot ip address")
        self.parser.add_argument("--port", type=int, default=9559,
                            help="Robot port number")

        self.args = self.parser.parse_args()
        robotIP = "192.168.43.70"
        PORT = 9559
        self.tts    = ALProxy("ALTextToSpeech", robotIP, PORT)
        self.motionProxy = ALProxy("ALMotion", robotIP, PORT)
        self.motionProxy.setStiffnesses("Body", 1.0)
        self.postureProxy = ALProxy("ALRobotPosture", robotIP, PORT)

        self.postureProxy.goToPosture("StandInit",0.5)
        # Wake up robot
        self.motionProxy.wakeUp()
        self.pub_to_kinect.publish(40)
        self.pub_to_screen.publish(4)

        self.sub_customer = rospy.Subscriber("/face_detector/face_positions", FoundFace, self.callback_hello, queue_size=1)
        print("NAO INIT DONE")

    def callback_hello(self,msg):
        #print("HELLO CALLED")
        if(len(msg.head_detections) > 0 ):
            self.sub_customer.unregister() # stop looking for new people
            hello_ID(self.tts,self.motionProxy,self.postureProxy) # nao says hello
            self.pub_customer_welcomed.publish("Welcomed") # start ID scan
            self.sub_age = rospy.Subscriber("/above18",Bool, self.callback_drink_select) # wait until ID confirmed

    def callback_drink_select(self,msg):
        self.pub_to_kinect.publish(40) # move kinect back up
        self.sub_age.unregister() # stop looking for age
        print("Drink Select Called")
        self.over18=msg
        self.pub_to_screen.publish(1) # select drink screen
        self.sub_selected = rospy.Subscriber("/term1_buttonPressed",UInt8, self.callback_entertain)
        drink_select(self.tts,self.motionProxy,self.postureProxy,msg) # move Nao to point at touchpad
        print("Drink Select Done")
        

    def callback_entertain(self,msg):
        self.sub_selected.unregister() # ignore new button pressed
        if(msg.data==1):
            self.choice="Stella Artois"
        elif(msg.data==2):
            self.choice="Heineken"
        elif(msg.data==3):
            self.choice="Guiness"
        else:
            self.choice="The Non Alcoholic Option"
        self.pub_to_arm.publish(msg) # tell arm to start pouring
        self.sub_ready = rospy.Subscriber("/drink_poured", Bool, self.callback_drink_ready) # wait until arm is done
        self.pub_to_screen.publish(2)
        choice(self.tts, self.motionProxy, self.postureProxy,self.choice) #drink selection
        #print("Entertain Called")
        #dance(self.tts, self.motionProxy, self.postureProxy)
        #time.sleep(3)
        #joke(self.tts, self.motionProxy, self.postureProxy)
        #time.sleep(3)
        #joke1(self.tts, self.motionProxy, self.postureProxy)
        #time.sleep(3)
        joke2(self.tts, self.motionProxy, self.postureProxy)
        print("Entertain Done")

    def callback_drink_ready(self,msg):
        self.sub_ready.unregister() # ignore other messages from arm
        self.pub_to_screen.publish(5) # drink ready
        pickup(self.tts, self.motionProxy, self.postureProxy,self.choice)
        time.sleep(3)
        print("Drink Ready please collect function called")
        self.pub_to_screen.publish(3) # other drink?
        self.sub_selected = rospy.Subscriber("/term1_buttonPressed",UInt8, self.callback_anotherDrink)

    def callback_anotherDrink(self,msg):
        print("customer responded")
        print(msg)
        if(msg.data==3):
            self.sub_selected.unregister()
            time.sleep(1)
            # other drink selected, go back to selection screen
            self.callback_drink_select(self.over18)
        elif(msg.data==4):
            self.sub_selected.unregister()
            # restart process
            self.pub_to_screen.publish(6) #display goodbye screen
            time.sleep(3) # wait to avoid detecting same customer
            self.sub_customer = rospy.Subscriber("/face_detector/face_positions", FoundFace, self.callback_hello, queue_size=1)

if __name__ == "__main__":

    try:
        rospy.init_node("NAO",  argv=sys.argv)
        sys.argv = rospy.myargv(argv=sys.argv)
        nao = NAO()
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
