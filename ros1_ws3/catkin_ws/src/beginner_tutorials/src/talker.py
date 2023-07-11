#!/usr/bin/env python
# Software License Agreement (BSD License)
#

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np


FPS=1.0
def image_publisher():
	
	pub = rospy.Publisher('image_topic', Image, queue_size=None)
	rospy.init_node('image_publisher', anonymous=True)
    
	cap=cv2.VideoCapture('/dev/video0')
	#cap=cv2.VideoCapture('/home/jetson/Videos/Webcam/test.mp4')
	bridge=CvBridge()
	rate = rospy.Rate(FPS) # 10hz
	while not rospy.is_shutdown():
		ret, frame= cap.read()
		if ret== True:
			resized_frame=cv2.resize(frame,(200, 200))
			ros_image= bridge.cv2_to_imgmsg(resized_frame, "bgr8")
			pub.publish(ros_image)
			rospy.loginfo("Image is being published to the topic /image_raw ...")
	rate.sleep()
	rospy.loginfo("I")
	cap.release()
	cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        image_publisher()
    except rospy.ROSInterruptException:
        pass

