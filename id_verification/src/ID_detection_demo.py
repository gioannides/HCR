#!/usr/bin/env python


from __future__ import print_function, division, absolute_import

import os
import rospy
import rospkg

from sensor_msgs.msg import Image
from std_msgs.msg import Bool,String,Float64
from cv_bridge import CvBridge
import cv2
import imutils
import time
from transform_lib import four_point_transform

import numpy as np

CARD_MAX_AREA= 120000
CARD_MIN_AREA= 25000

class IDDetector(object):
    def __init__(self):
	#changed image to STRING PUT BACK
        self.pub_ID_image = rospy.Publisher("/above18", Bool, queue_size=1)
        self.pub_to_kinect = rospy.Publisher("/tilt_angle",Float64, queue_size=1)

        # publishers for visualisation in demo
        self.pub_white_mask = rospy.Publisher("/white_mask", Image, queue_size=1)
        self.pub_edges = rospy.Publisher("/edges", Image, queue_size=1)
        self.pub_cropped_ID = rospy.Publisher("/cropped_ID", Image, queue_size=1)
        self.pub_red_mask = rospy.Publisher("/red_mask", Image, queue_size=1)
        self.cv_bridge = CvBridge()

        self.lower_threshold, self.upper_threshold = rospy.get_param("~lower_threshold", 50), rospy.get_param("~upper_threshold", 200)

        # Note: I recommend creating the subscriber always last, so that all other variables already exist
        # For images, make sure to set a large buff_size to avoid lags
        self.sub_customer_welcomed=rospy.Subscriber("/customerwelcomed",String,self.callback_tilt,queue_size=1)

        print('Init done', self.lower_threshold, self.upper_threshold)

    def callback_tilt(self, msg):
        print("customer welcomed moving camera")
        self.pub_to_kinect.publish(-30)
        time.sleep(3)
        self.sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.callback, queue_size=1, buff_size=2**24)

    def callback(self, msg):
        # First convert the ROS message to an OpenCV compatible image type
        img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        #print("image recieved")
        # white mask for ID detection
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) #change colortype for openCV
        lower_white = np.array([0, 0, 0], dtype=np.uint8)
    	upper_white = np.array([0,0,255], dtype=np.uint8)
	    # Threshold the HSV image to get only blue colors
        mask = cv2.inRange(hsv, lower_white, upper_white)

    	# Bitwise-AND mask and original image
    	res = cv2.bitwise_and(img,img, mask= mask)
        white_mask_IMG = self.cv_bridge.cv2_to_imgmsg(res)
        self.pub_white_mask.publish(white_mask_IMG)

        # You can now use any OpenCV operation you find to extract "meaning" from the image
        # Here, let's extract edges from the image using the Canny Edge detector
        edges = cv2.Canny(res, self.lower_threshold, self.upper_threshold)
        edge_IMG = self.cv_bridge.cv2_to_imgmsg(edges)
        self.pub_edges.publish(edge_IMG)

        #find the contours in the edged image, keeping only the largest ones and initialize the screen contour
        cnts=cv2.findContours(edges.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        cnts=imutils.grab_contours(cnts)
        cnts=sorted(cnts,key = cv2.contourArea, reverse = True)[:5]
        screenCnt=[]
        out= np.zeros_like(img)
        #loop over the contours
        for c in cnts:
            size= cv2.contourArea(c)
            peri= cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c,0.02 * peri, True)
            #print("size= {}".format(size))
            #if our approximated contour has four points, then we can assume that we have found our ID
            if len(approx) == 4 and size > 10000:
                #self.sub.unregister()
                screenCnt = approx
                print("found card")
                #print("screenCnt= {}".format(screenCnt))
                print("size= {}".format(size))
                # x,y,w,h = cv2.boundingRect(c)
                # out=img[y:y+h, x:x+w]
                # mask=np.zeros_like(img)
                # cv2.drawContours(mask, [screenCnt],-1,(0,255,0),2)
                # out= np.zeros_like(img)
                # out[mask==(0,255,0)]= img[mask==(0,255,0)]
                # cv2.drawContours(mask, [screenCnt],-1,(0,255,0),2)
                # out= np.zeros_like(img)
                # out[mask==255]= img[mask==255]
                # (y,x) = np.where(mask == 255)
                # (topy,topx) = (np.min(y), np.min(x))
                # (bottomy, bottomx) = (np.max(y),np.max(x))
                # out= img[topy:bottomy+1, topx:bottomx+1]
                out = four_point_transform(img, screenCnt.reshape(4, 2))
                # cv2.drawContours(img, [screenCnt], -1, (0,255,0),2)
		        # Finally, publish the edge image
                # Now, let's convert the OpenCV image back to a ROS message
                ID_img = self.cv_bridge.cv2_to_imgmsg(out)
                self.pub_cropped_ID.publish(ID_img)
                # I recommend setting the timestamp consistent with that of the original image
                # This is useful if you want to synchronise images later on

                ID_img.header.stamp = msg.header.stamp
                #Publishing string MUST CHANGE
                try:
                    self.pub_ID_image.publish(True)
                except rospy.ROSException as e:
                    if str(e) == "publish() to a closed topic":
                        print("See ya")
                    else:
                        raise e
                print('Published image', msg.header.stamp.to_sec(), end='\r')





if __name__ == "__main__":
    try:
        rospy.init_node("ID_detector")
        edge_detector = IDDetector()
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
