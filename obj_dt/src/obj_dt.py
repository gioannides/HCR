#!/usr/bin/env python3
import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import UInt8
import os
print(os.path.dirname(os.path.abspath(__file__)))
print("Environment Ready")

def filtering(depth_frame):

    decimation = rs.decimation_filter() #Decimation reduces the number of samples so it reduces the amount of data that is metered and reduces the processing time
                                        #Need a fast enough implementation because the robot will interact with humans in real-time
    decimation.set_option(rs.option.filter_magnitude, 4)
    spatial = rs.spatial_filter() #Spatial Filtering. Spatial filtering is an image processing technique for changing the intensities 
                                  # of a pixel according to the intensities of the neighboring pixels.
    spatial.set_option(rs.option.filter_magnitude, 5)
    spatial.set_option(rs.option.filter_smooth_alpha, 1)
    spatial.set_option(rs.option.filter_smooth_delta, 50)
    spatial.set_option(rs.option.holes_fill, 3)
    hole_filling = rs.hole_filling_filter() #Fill image regions with holes
    temporal = rs.temporal_filter()

    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)

    frame = depth_frame
    frame = decimation.process(frame)
    frame = depth_to_disparity.process(frame)
    frame = spatial.process(frame)
    frame = temporal.process(frame)
    frame = disparity_to_depth.process(frame)  #Disparity filter can sufficiently reduce the network without destroying the multi-scale nature of the network.
    depth_frame = hole_filling.process(frame)

    return depth_frame

def colorSegementation(color, xmin, xmax, ymin, ymax):

    # Estrella red color
    hsv_frame = cv2.cvtColor(color, cv2.COLOR_RGB2HSV) #  HSV separates luma, or the image intensity, from chroma or the color information
                                                       #  separate color components from intensity for robustness to lighting changes, or removing shadows.
    mask1 = cv2.inRange(hsv_frame, (0,50,20), (5,255,255))
    mask2 = cv2.inRange(hsv_frame, (175,50,20), (180,255,255))

    ## Merge the mask and crop the red regions
    masked = cv2.bitwise_or(mask1, mask2)
    red = cv2.bitwise_and(color, color, mask=masked)

    indices = np.where(red != [0])

    avg_x_ = None
    avg_y_ = None
    counter = 0
    if len(indices[0]) and len(indices[1]):
        avg_x_ = 0
        avg_y_ = 0
        counter = 0

        for y in indices[0]:
            if (y >= int(ymin) and y <= int(ymax)):

                avg_y_ += y
                counter += 1
        try:
            avg_y_ /= counter
        except: 
            pass
        counter = 0
        for x in indices[1]:
            if (x >= int(xmin) and x <= int(xmax)):

                avg_x_ += x
                counter += 1
        try:
            avg_x_ /= counter
        except:
            pass
        #print("c:",counter)
    
    if (avg_x_ >= int(xmin) and avg_x_ <= int(xmax)) and (avg_y_ >= int(ymin) and avg_y_ <= int(ymax)) and counter >= 3000: #counter is a hyperparameter
        
        return "Estrella"
    else:
        return "Unknown"


# Setup

class obj_dt(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):
    rospy.init_node("obj_det")
    self.pub_position = rospy.Publisher('/beer_position', Pose, queue_size=1)
    rospy.Subscriber("/arm_in_position", UInt8, self.callback, queue_size=1)
    print("george init")

  def callback(self, msg):
    print("george thing now running")
    pose_msg = Pose()
    pose_msg.orientation.x = 0
    pose_msg.orientation.y = 0
    pose_msg.orientation.z = 0
    pose_msg.orientation.w = 0

    # Setup:
    pipe = rs.pipeline()

    config = rs.config()
    # config.enable_stream(rs.stream.color, 424, 240, rs.format.rgb8, 30)
    # config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30)
    # config.enable_stream(rs.stream.infrared, 1)
    # config.enable_stream(rs.stream.infrared, 2)
    profile = pipe.start()


    # Skip 15 first frames to give the Auto-Exposure time to adjust
    for x in range(15):
      pipe.wait_for_frames()


  
    # Store next frameset for later processing:
    frameset = pipe.wait_for_frames()
    color_frame = frameset.get_color_frame()
    depth_frame = frameset.get_depth_frame()

    depth_frame = filtering(depth_frame)

    # Cleanup:
    pipe.stop()
    print("Frames Captured")

    color = np.asanyarray(color_frame.get_data())
    #plt.rcParams["axes.grid"] = False
    #plt.rcParams['figure.figsize'] = [12, 6]
    #plt.imshow(color)
    #plt.show()
    colorizer = rs.colorizer()
    colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
    #plt.imshow(colorized_depth)

    # Create alignment primitive with color as its target stream:
    align = rs.align(rs.stream.color)
    frameset = align.process(frameset)

    # Update color and depth frames:
    aligned_depth_frame = frameset.get_depth_frame()
    colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

    # Show the two frames together:
    #images = np.hstack((color, colorized_depth))
    #plt.imshow(images)

    # Standard OpenCV boilerplate for running the net:
    height, width = color.shape[:2] #240, 424

    expected = 300
    aspect = width / height
    resized_image = cv2.resize(color, (int(round(expected * aspect)), expected))
    crop_start = int(round(expected * (aspect - 1) / 2))
    crop_img = resized_image[0:expected, crop_start:crop_start+expected]

    arg1 = "MobileNetSSD_deploy.prototxt.txt"
    arg2 = "MobileNetSSD_deploy.caffemodel"
    net = cv2.dnn.readNetFromCaffe(arg1, arg2)
    inScaleFactor = 0.007843
    meanVal       = 127.53
    classNames = ("background", "aeroplane", "bicycle", "bird", "boat",
                  "bottle", "bus", "car", "cat", "chair",
                  "cow", "diningtable", "dog", "horse",
                  "motorbike", "person", "pottedplant",
                  "sheep", "sofa", "train", "tvmonitor")

    blob = cv2.dnn.blobFromImage(crop_img, inScaleFactor, (expected, expected), meanVal, False)
    net.setInput(blob, "data")
    detections = net.forward("detection_out")

    results = []
    for i in np.arange(0, detections.shape[2]):
        idx = int(detections[0, 0, i, 1])
        #print(classNames[idx])
        if classNames[idx] == "bottle":
            #continue

            label = detections[0,0,i,1]
            conf  = detections[0,0,i,2]
            xmin  = detections[0,0,i,3]
            ymin  = detections[0,0,i,4]
            xmax  = detections[0,0,i,5]
            ymax  = detections[0,0,i,6]

            className = classNames[int(label)]

            cv2.rectangle(crop_img, (int(xmin * expected), int(ymin * expected)), 
                    (int(xmax * expected), int(ymax * expected)), (255, 255, 255), 2)
            cv2.putText(crop_img, className, 
                    (int(xmin * expected), int(ymin * expected) - 5),
                    cv2.FONT_HERSHEY_COMPLEX, 0.5, (255,255,255))

            # plt.imshow(crop_img)
            # plt.show()

            scale = height / expected
            xmin_depth = int((xmin * expected + crop_start) * scale)
            ymin_depth = int((ymin * expected) * scale)
            xmax_depth = int((xmax * expected + crop_start) * scale)
            ymax_depth = int((ymax * expected) * scale)
            xmin_depth,ymin_depth,xmax_depth,ymax_depth
            cv2.rectangle(colorized_depth, (xmin_depth, ymin_depth), 
                        (xmax_depth, ymax_depth), (255, 255, 255), 2)
            # plt.imshow(colorized_depth)
            # plt.show()

            x_depth_center = 0.5 * (xmax_depth + xmin_depth)
            y_depth_center = 0.5 * (ymax_depth + ymin_depth)

            depth = np.asanyarray(aligned_depth_frame.get_data())
            # Crop depth data:
            depth = depth[xmin_depth:xmax_depth,ymin_depth:ymax_depth].astype(float)

            # Get data scale from the device and convert to meters
            #depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
            #depth = depth * depth_scale
            #dist,_,_,_ = cv2.mean(depth)
            dist = aligned_depth_frame.get_distance(int(x_depth_center), int(y_depth_center))

            #avg_x = 0.5 * (xmin * (width/expected) + xmax * (width/expected))
            #avg_y = 0.5 * (ymin * (height/expected) + ymax * (height/expected))
            #print(avg_x, avg_y)
            depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics
            depth = aligned_depth_frame.get_distance(int(x_depth_center), int(y_depth_center))
            realx, realy, realz = rs.rs2_deproject_pixel_to_point(depth_intrin, [int(x_depth_center),int(y_depth_center)],depth)

            objectType = colorSegementation(crop_img,int(xmin * expected), int(xmax * expected), int(ymin * expected), int(ymax * expected))
            results.append((realx, realy, realz, objectType, className))

    for item in results:
        print("D a {0} of type {4} at (x, y, z) : {1:.3}, {2:.3}, {3:.3}.".format(item[4], item[0], item[1], item[2], item[3]))


      #The detections are in the list 'results' in the form of '(x, y, z, drinkType, className)

    closest_item = None

    for item in results:
        if (abs(item[0]) != 0.0 and abs(item[1]) != 0.0 and abs(item[2]) != 0.0):
            if item[4] == 'bottle' and abs(item[0]) <= 0.05:
                if closest_item == None or closest_item[2] > item[2]:
                    closest_item = item

    if closest_item != None:
        pose_msg.position.x = closest_item[0]
        pose_msg.position.y = closest_item[1]
        pose_msg.position.z = closest_item[2]

        print("Detected a {0} of type {4} at (x, y, z) : {1:.3}, {2:.3}, {3:.3}.".format(closest_item[4], closest_item[0], closest_item[1], closest_item[2], closest_item[3]))

        self.pub_position.publish(pose_msg)

        

    # for item in results:
    #     if item[4] == 'bottle':
    #       pose_msg.position.x = item[0]
    #       pose_msg.position.y = item[1]
    #       pose_msg.position.z = item[2]
    #       self.pub_position.publish(pose_msg)

def main():
    example = obj_dt()
    # example.callback(1)
    rospy.spin()

if __name__ == '__main__':
  main()