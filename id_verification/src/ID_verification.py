#!/usr/bin/env python


from __future__ import print_function, division, absolute_import

import os
import rospy
import rospkg

from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import imutils

import numpy as np

CARD_MAX_AREA= 120000
CARD_MIN_AREA= 25000

class IDVerifier(object):
    def __init__(self):
        self.pub_ID_verify = rospy.Publisher("/IDimageverify", Bool, queue_size=1)
        self.cv_bridge = CvBridge()

        self.lower_threshold, self.upper_threshold = rospy.get_param("~lower_threshold", 50), rospy.get_param("~upper_threshold", 200)

        # Note: I recommend creating the subscriber always last, so that all other variables already exist
        # For images, make sure to set a large buff_size to avoid lags
        self.sub = rospy.Subscriber("/image", Image, self.callback, queue_size=1, buff_size=2**24)
        print('Init done', self.lower_threshold, self.upper_threshold)

    def callback(self, msg):
        # First convert the ROS message to an OpenCV compatible image type
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        print("image recieved")
        # You can now use any OpenCV operation you find to extract "meaning" from the image
        # Here, let's extract edges from the image using the Canny Edge detector
        edges = cv2.Canny(img, self.lower_threshold, self.upper_threshold)

        #find the contours in the edged image, keeping only the largest ones and initialize the screen contour
        cnts=cv2.findContours(edges.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cnts=imutils.grab_contours(cnts)
        cnts=sorted(cnts,key = cv2.contourArea, reverse = True)[:5]
        screenCnt=[]
        #loop over the contours
        for c in cnts:
            size= cv2.contourArea(c)
            peri= cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c,0.02 * peri, True)
            #print("size= {}".format(size))
            #if our approximated contour has four points, then we can assume that we have found our ID
            if len(approx) == 4 and size<=19000 and size >= 9000:
                screenCnt = approx
                print("found card")
                print("size= {}".format(size))
                cv2.drawContours(img, [screenCnt], -1, (0,255,0),2)
                



        # Now, let's convert the OpenCV image back to a ROS message
        ID_img = self.cv_bridge.cv2_to_imgmsg(img)

        # I recommend setting the timestamp consistent with that of the original image
        # This is useful if you want to synchronise images later on
        ID_img.header.stamp = msg.header.stamp

        # Finally, publish the edge image
        try:
            self.pub_ID_image.publish(ID_img)
        except rospy.ROSException as e:
            if str(e) == "publish() to a closed topic":
                print("See ya")
            else:
                raise e
        print('Published image', msg.header.stamp.to_sec(), end='\r')


if __name__ == "__main__":
    try:
        rospy.init_node("ID_verifier")
        edge_detector = IDVerifier()
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
