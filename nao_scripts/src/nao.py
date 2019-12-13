#!/usr/bin/env python


import os
import rospy
import rospkg
import time
import argparse
import random
from std_msgs.msg import Bool,String, Float64, UInt8
from cob_perception_msgs.msg import ColorDepthImageArray as FoundFace
from naoqi import ALProxy
import sys
sys.path.append("/home/human/catkin_ws/src/HCR/nao_scripts/scripts")
from nao2 import hello_ID,drink_select,choice,dance,joke,joke1,joke2,pickup,goodbye,another,watchArnold

sys.dont_write_bytecode = True


class NAO(object):
    def __init__(self):

        self.pub_interaction_complete = rospy.Publisher("/interaction_complete", Bool, queue_size=1)
        self.pub_customer_welcomed = rospy.Publisher("/customerwelcomed",String, queue_size=1)
        self.pub_to_screen = rospy.Publisher("/term1_displayScreenX",UInt8, queue_size=1)
        self.pub_to_arm = rospy.Publisher("/pour_drink", UInt8, queue_size=1)
        self.pub_to_kinect = rospy.Publisher("/tilt_angle",Float64, queue_size=1)
        self.parser = argparse.ArgumentParser()
        self.parser.add_argument("--ip", type=str, default= "192.168.1.2",
                                    help="Robot ip address")
        self.parser.add_argument("--port", type=int, default=9559,
                            help="Robot port number")

        self.args = self.parser.parse_args()
        robotIP = "192.168.1.2"
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
        self.sub_wait = rospy.Subscriber("/term2_buttonPressed", UInt8, self.callback_wait_touchpad_done)

    def callback_wait_touchpad_done(self,msg):
        self.sub_customer.unregister()
        self.sub_wait.unregister()
        self.sub_interaction_complete = rospy.Subscriber("/interaction_complete", Bool, self.callback_touchpad_done)

    def callback_touchpad_done(self,msg):
        self.sub_interaction_complete.unregister()
        time.sleep(1)
        self.sub_wait = rospy.Subscriber("/term2_buttonPressed", UInt8, self.callback_wait_touchpad_done)
        self.sub_customer = rospy.Subscriber("/face_detector/face_positions", FoundFace, self.callback_hello, queue_size=1)

    # with ID detection
    #--------------------

    # def callback_hello(self,msg):
    #     #print("HELLO CALLED")
    #     if(len(msg.head_detections) > 0 ):
    #         self.sub_customer.unregister() # stop looking for new people
    #         hello_ID(self.tts,self.motionProxy,self.postureProxy) # nao says hello
    #         self.pub_customer_welcomed.publish("Welcomed") # start ID scan
    #         self.sub_age = rospy.Subscriber("/above18",Bool, self.callback_drink_select) # wait until ID confirmed

    # def callback_drink_select(self,msg):
    #     self.pub_to_kinect.publish(40) # move kinect back up
    #     self.sub_age.unregister() # stop looking for age
    #     print("Drink Select Called")
    #     self.over18=msg
    #     if msg.data==True:
    #         self.pub_to_screen.publish(1) # select drink screen
    #     else:
    #         self.pub_to_screen.publish(7) # select drink screen for <18

    #     self.sub_selected = rospy.Subscriber("/term1_buttonPressed",UInt8, self.callback_entertain, callback_args = self.over18)
    #     drink_select(self.tts,self.motionProxy,self.postureProxy,msg) # move Nao to point at touchpad
    #     print("Drink Select Done")


    # without ID detection
    #----------------------

    def callback_hello(self,msg):
        #print("HELLO CALLED")
        if(len(msg.head_detections) > 0 ):
            self.pub_customer_welcomed.publish("Welcomed")
            self.sub_customer.unregister() # stop looking for new people
            hello_ID(self.tts,self.motionProxy,self.postureProxy) # nao says hello
            print("Drink Select Called")
            self.pub_to_screen.publish(1) # select drink screen

            self.over18 = True
            self.sub_selected = rospy.Subscriber("/term1_buttonPressed",UInt8, self.callback_entertain, callback_args = self.over18)
            drink_select(self.tts,self.motionProxy,self.postureProxy,self.over18) # move Nao to point at touchpad
            print("Drink Select Done")

    def callback_drink_select(self,msg):
            self.pub_to_screen.publish(1) # select drink screen

            self.over18 = True
            self.sub_selected = rospy.Subscriber("/term1_buttonPressed",UInt8, self.callback_entertain, callback_args = self.over18)
            drink_select(self.tts,self.motionProxy,self.postureProxy,msg) # move Nao to point at touchpad
            print("Drink Select Done")
    # ------------------------------------------------------------------------------------------

    def callback_entertain(self,msg, over18):
        self.sub_selected.unregister() # ignore new button pressed
        if (over18==False & msg.data==4): #if <18 and don't want non-alcoholic drink
            self.pub_to_screen.publish(6) #publish goodbye
            time.sleep(3) # wait to avoid detecting same customer
            self.sub_customer = rospy.Subscriber("/face_detector/face_positions", FoundFace, self.callback_hello, queue_size=1)
        else:
            if over18:
                if(msg.data==1):
                    self.choice="Camden Hells"
                elif(msg.data==2):
                    self.choice="BREW DOG PUNK I P A"
                elif(msg.data==3):
                    self.choice="BREW DOG ELVIS JUICE "
                else:
                    self.choice="Coca Cola"
            else:
                msg.data = 4        #translate the 'yes' into the correct drink
                self.choice="Coca Cola"

            self.pub_to_arm.publish(msg) # tell arm to start pouring
            self.sub_ready = rospy.Subscriber("/drink_poured", Bool, self.callback_drink_ready) # wait until arm is done
            self.pub_to_screen.publish(2)
            choice(self.tts, self.motionProxy, self.postureProxy,self.choice) #drink selection
            time.sleep(4)

            joke_no = random.randint(1,3)
            print(joke_no)
            if joke_no ==1:
                joke1(self.tts, self.motionProxy, self.postureProxy)
            elif joke_no ==2:
            # time.sleep(2.5)
                joke2(self.tts, self.motionProxy, self.postureProxy)
            # time.sleep(2.5)
            else:
            #print("Entertain Called")
            #dance(self.tts, self.motionProxy, self.postureProxy)
            #time.sleep(3)
                joke(self.tts, self.motionProxy, self.postureProxy)
            #time.sleep(3)

            time.sleep(4)
            watchArnold(self.tts, self.motionProxy, self.postureProxy)

            print("Entertain Done")

    def callback_drink_ready(self,msg):
        self.sub_ready.unregister() # ignore other messages from arm
        self.pub_to_screen.publish(5) # drink ready
        pickup(self.tts, self.motionProxy, self.postureProxy,self.choice)
        time.sleep(3)
        print("Drink Ready please collect function called")
        self.pub_to_screen.publish(3) # other drink?
        another(self.tts, self.motionProxy, self.postureProxy)
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
            goodbye(self.tts, self.motionProxy, self.postureProxy)
            self.pub_interaction_complete.publish(True)
            time.sleep(10) # wait to avoid detecting same customer
            self.pub_to_screen.publish(1)
            self.sub_selected = rospy.Subscriber("/term1_buttonPressed",UInt8, self.callback_entertain, callback_args = self.over18)
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
