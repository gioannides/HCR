#!/usr/bin/env python
import cv2                                # state of the art computer vision algorithms library
import numpy as np                        # fundamental package for scientific computing
import matplotlib.pyplot as plt           # 2D plotting library producing publication quality figures
import pyrealsense2 as rs                 # Intel RealSense cross-platform open-source API
import rospy
from geometry_msgs.msg import Pose
print("Environment Ready")


# Setup
rospy.init_node("obj_det")
my_pub = rospy.Publisher('/beer_position',Pose,queue_size=10)
pose_msg = Pose()
pose_msg.orientation.x = 0
pose_msg.orientation.y = 0
pose_msg.orientation.z = 0
pose_msg.orientation.w = 0

pipe = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 424, 240, rs.format.rgb8, 30)
config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30)
# without config it defaults to 640, 480
profile = pipe.start()


# Skip 15 first frames to give the Auto-Exposure time to adjust
for x in range(15):
  pipe.wait_for_frames()
  
# Store next frameset for later processing:
frameset = pipe.wait_for_frames()
color_frame = frameset.get_color_frame()
depth_frame = frameset.get_depth_frame()

#depth_frame = frameset.get_depth_frame()
depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()


decimation = rs.decimation_filter()
decimation.set_option(rs.option.filter_magnitude, 4)
spatial = rs.spatial_filter()
spatial.set_option(rs.option.filter_magnitude, 5)
spatial.set_option(rs.option.filter_smooth_alpha, 1)
spatial.set_option(rs.option.filter_smooth_delta, 50)
spatial.set_option(rs.option.holes_fill, 3)
hole_filling = rs.hole_filling_filter()
temporal = rs.temporal_filter()

depth_to_disparity = rs.disparity_transform(True)
disparity_to_depth = rs.disparity_transform(False)

frame = depth_frame
frame = decimation.process(frame)
frame = depth_to_disparity.process(frame)
frame = spatial.process(frame)
frame = temporal.process(frame)
frame = disparity_to_depth.process(frame)
depth_frame = hole_filling.process(frame)

# Cleanup:
pipe.stop()
print("Frames Captured")

color = np.asanyarray(color_frame.get_data())
plt.rcParams["axes.grid"] = False
plt.rcParams['figure.figsize'] = [12, 6]
plt.imshow(color)

colorizer = rs.colorizer()
colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
plt.imshow(colorized_depth)

# Create alignment primitive with color as its target stream:
align = rs.align(rs.stream.color)
frameset = align.process(frameset)

# Update color and depth frames:
aligned_depth_frame = frameset.get_depth_frame()
colorized_depth = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())

# Show the two frames together:
images = np.hstack((color, colorized_depth))
plt.imshow(images)
plt.show()

# Standard OpenCV boilerplate for running the net:
height, width = color.shape[:2]
expected = 300

aspect = float(width) / float(height)
resized_image = cv2.resize(color, (int(round(expected * aspect)), expected))

#!!!! This way of caluculating crop_start doesn't seem right
crop_start = int(round(expected * (aspect - 1) / 2))
crop_img = resized_image[0:expected, crop_start:crop_start+expected]
print("width = {0}, height = {1}, aspect = {2}, crop_start = {3}".format(width, height, aspect, crop_start))

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



label = detections[0,0,0,1]
conf  = detections[0,0,0,2]
xmin  = detections[0,0,0,3]
ymin  = detections[0,0,0,4]
xmax  = detections[0,0,0,5]
ymax  = detections[0,0,0,6]


className = classNames[int(label)]

cv2.rectangle(crop_img, (int(xmin * expected), int(ymin * expected)), 
             (int(xmax * expected), int(ymax * expected)), (255, 255, 255), 2)
cv2.putText(crop_img, className, 
            (int(xmin * expected), int(ymin * expected) - 5),
            cv2.FONT_HERSHEY_COMPLEX, 0.5, (255,255,255))

plt.imshow(crop_img)
plt.show()


scale = float(height) / float(expected)
xmin_depth = int((xmin * expected + crop_start) * scale)
ymin_depth = int((ymin * expected) * scale)
xmax_depth = int((xmax * expected + crop_start) * scale)
ymax_depth = int((ymax * expected) * scale)
x_centre = int((xmax_depth - xmin_depth)/2) + xmin_depth
y_centre = int((ymax_depth - ymin_depth)/2) + ymin_depth
print(x_centre)
print(y_centre)
#xmin_depth,ymin_depth,xmax_depth,ymax_depth
cv2.rectangle(colorized_depth, (xmin_depth, ymin_depth), 
             (xmax_depth, ymax_depth), (255, 255, 255), 2)

cv2.rectangle(colorized_depth, (x_centre-2, y_centre-2), 
             (x_centre+2, y_centre+2), (255, 255, 255), 2)
plt.imshow(colorized_depth)
plt.show()

# depth = np.asanyarray(aligned_depth_frame.get_data())
# Crop depth data:
# depth = depth[xmin_depth:xmax_depth,ymin_depth:ymax_depth].astype(float)

# Get data scale from the device and convert to meters


# depth = depth * depth_scale
# dist,_,_,_ = cv2.mean(depth)

#newX = xmin * (1280/expected)
#newY = 



depth = np.asanyarray(aligned_depth_frame.get_data())

print(len(depth))
depth = depth[x_centre-2:x_centre+2,y_centre-2:y_centre+2].astype(float)
print(len(depth))

depth = depth * depth_scale
dist,_,_,_ = cv2.mean(depth)

avg_x = 0.5 * (xmin * (width) + xmax * (height))
avg_y = 0.5 * (ymin * (width) + ymax * (height))
avg_z = dist


intr = profile.get_stream(rs.stream.depth) # Fetch stream profile for depth stream
intr = intr.as_video_stream_profile().get_intrinsics() 

realx, realy, realz = rs.rs2_deproject_pixel_to_point(intr, [int(avg_x),int(avg_y)],float(avg_z))
print("Pixel coord: x = {0}, y = {1}, z = {2}".format(avg_x,avg_y,avg_z))

display = (realx, realy, realz)
print("Detected a {0} at (x, y, z) : ({1:.3}, {2:.3}, {3:.3}).".format(className, display[0], display[1], display[2]))

pose_msg.position.x = display[0]
pose_msg.position.y = display[1]
pose_msg.position.z = display[2]

rate = rospy.Rate(1)
while not rospy.is_shutdown():
    my_pub.publish(pose_msg)
    rate.sleep()
