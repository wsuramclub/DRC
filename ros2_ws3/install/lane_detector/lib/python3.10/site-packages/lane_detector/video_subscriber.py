#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from . import lane_detection 
from . import constants as const
from . import visualization
from . import transform_points

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.sub_= self.create_subscription(
            Image, 'Image', self.listener_callback,10)
        self.cv_bridge=CvBridge()

    def object_detect(self, image):
        visual = visualization.Visualization()
        ld = lane_detection.Lane_Detection()
        tp = transform_points.Transform_Points()
        edge_image = self.edge_detection(image)            # Detect Edges in image

        ld.get_initial_lane_points(edge_image)      # Apply Lane initialization on first image
        ld.solve_lane()                             # Solve lane model parameters from lane points
        ld.lane_sanity_checks(edge_image)           # Apply corrections to lane model
        tp.transform_lane_to_poly(ld)  

        visual.clear_imgs()                                         # Clear Images to show every iteration
        visual.draw_lane_lines(image, ld)                             # Draw lane model lines on image
        edge_image = cv2.cvtColor(edge_image, cv2.COLOR_GRAY2RGB)   # Generate Edge Image
        visual.draw_lane_points(edge_image, ld)                     # Draw lane edge points on image
        visual.append_img(image)                                      # Append img for drawing
        visual.append_img(edge_image)                               # Append img for drawing
        visual.show_imgs() 
        
    def edge_detection(self,img):
        # gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY).astype(np.float)       # convert to gray 
        # blurredGray = cv2.blur(gray, (3, 3))                                # blur img for robustness
        # cv2.imshow('gray',blurredGray)
        # blurredSobelImg = cv2.Sobel(blurredGray, cv2.CV_8U, 1, 0, ksize=1)  # calculate sobel gradient
        # ret, threshSobel = cv2.threshold(                                   # soebel img to binary by threshold
        #     blurredSobelImg, 7, 255, cv2.THRESH_BINARY)
        # kernel = np.ones((3, 3), np.uint8)                                  # erode image to remove noise
        # erodedSobel = cv2.erode(threshSobel, kernel, iterations=1)          # erode image to remove noise
        # cv2.imshow('sobel',erodedSobel)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        #cv2.imshow("hsv", hsv)
        lower_yellow = np.array([15, 40, 40])
        upper_yellow = np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        #cv2.imshow("yellow mask", mask)

        # detect edges
        edges = cv2.Canny(mask, 200, 400)
        return edges
    
    def listener_callback(self,data):
        # self.get_logger().info('receiving video frame')
        image=self.cv_bridge.imgmsg_to_cv2(data, 'bgr8')
        # cv2.imshow('video',image)
        # cv2.waitKey(1)
        # self.object_detect(image)
        self.object_detect(image)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()