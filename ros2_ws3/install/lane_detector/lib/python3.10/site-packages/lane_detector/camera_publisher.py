#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String
import numpy as np

class ImagePublisher(Node):
    def __init__(self):
        super().__init__("publisher")
        self.publisher_ = self.create_publisher(Image, 'Image', 10)
        self.timer = self.create_timer(0.05,self.publish_image)
        # self.capture=cv2.imread('/home/pradeshi/Downloads/road1.jpg')
        self.capture=cv2.VideoCapture('/home/pradeshi/Downloads/track.mp4')
        self.cv_bridge=CvBridge()

    def publish_image(self):
        ret, frame=self.capture.read()
        if ret == True:
            self.publisher_.publish(self.cv_bridge.cv2_to_imgmsg(frame, 'bgr8')) 
            self.get_logger().info("got frames")
        # self.publisher_.publish(self.cv_bridge.cv2_to_imgmsg(np.array(self.capture),'bgr8'))
        self.get_logger().info("PUblishing video frame")

def main(args=None):
    rclpy.init(args=args)
    image_node = ImagePublisher()       
    rclpy.spin(image_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()