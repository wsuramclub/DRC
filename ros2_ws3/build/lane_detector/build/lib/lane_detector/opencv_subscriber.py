#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.sub_= self.create_subscription(
            Image, 'Image', self.listener_callback,10)
        self.cv_bridge=CvBridge()

    def object_detect(self, image):
        edges = self.detect_edges(image)
        cropped_edges = self.region_of_interest(edges)
        line_segments = self.detect_line_segments(cropped_edges)
        lane_lines = self.average_slope_intercept(image, line_segments)
        lane_lines_image = self.display_lines(image, lane_lines)
        cv2.imshow("image", lane_lines_image)
        cv2.waitKey(1)

    def detect_edges(self,frame):
        # filter for blue lane lines
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #cv2.imshow("hsv", hsv)
        lower_yellow = np.array([20, 40, 40])
        upper_yellow = np.array([35, 255, 255])
        lower_blue = np.array([60, 40, 40])
        upper_blue = np.array([150, 255, 255])
        
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        blue_mask = cv2.inRange(hsv, lower_blue, upper_blue)
        #cv2.imshow("yellow mask", mask)
        mask = cv2.bitwise_or(yellow_mask, blue_mask)
        mask= cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel=np.ones((8,8),dtype=np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel=np.ones((20,20),dtype=np.uint8))

        # improve mask by drawing the convexhull 
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            hull = cv2.convexHull(cnt)
            cv2.drawContours(mask,[hull],0,(255), -1)
        # erode mask a bit to migitate mask bleed of convexhull
        mask = cv2.morphologyEx(mask, cv2.MORPH_ERODE, kernel=np.ones((5,5),dtype=np.uint8))
        masked = cv2.bitwise_and(hsv, hsv, mask = mask)
        # detect edges
        edges = cv2.Canny(masked, 200, 400)
        #cv2.imshow('edges',edges)

        return edges

    def region_of_interest(self,edges):
        height, width = edges.shape
        self.get_logger().info('height:{}, width:{}'.format(height,width))
        mask = np.zeros_like(edges)

        # only focus bottom half of the screen
        polygon = np.array([[
            (0, height),
            (0, height*1/2 ),
            (width, height*1/2),
            (width, height),
        ]], np.int32)

        cv2.fillPoly(mask, polygon, 255)
        cropped_edges = cv2.bitwise_and(edges, mask)
        #cv2.imshow('cropped',cropped_edges)
        return cropped_edges


    def detect_line_segments(self,cropped_edges):
        # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
        rho = 1  # distance precision in pixel, i.e. 1 pixel
        angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
        min_threshold = 20 # minimal of votes
        line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, 
                                        np.array([]), minLineLength=12, maxLineGap=2)

        return line_segments

    def average_slope_intercept(self,frame, line_segments):
        """
        This function combines line segments into one or two lane lines
        If all line slopes are < 0: then we only have detected left lane
        If all line slopes are > 0: then we only have detected right lane
        """
        lane_lines = []
        if line_segments is None:
            # logging.info('No line_segment segments detected')
            return lane_lines

        height, width, _ = frame.shape
        left_fit = []
        right_fit = []

        boundary = 1/3
        left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
        right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen

        for line_segment in line_segments:
            for x1, y1, x2, y2 in line_segment:
                if x1 == x2:
                    # logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                    self.get_logger().info('skipping vertical line segment (slope=inf): %s' % line_segment)

                    continue
                fit = np.polyfit((x1, x2), (y1, y2), 1)
                slope = fit[0]
                intercept = fit[1]
                if slope < 0:
                    if x1 < left_region_boundary and x2 < left_region_boundary:
                        left_fit.append((slope, intercept))
                else:
                    if x1 > right_region_boundary and x2 > right_region_boundary:
                        right_fit.append((slope, intercept))

        left_fit_average = np.average(left_fit, axis=0)
        if len(left_fit) > 0:
            lane_lines.append(self.make_points(frame, left_fit_average))

        right_fit_average = np.average(right_fit, axis=0)
        if len(right_fit) > 0:
            lane_lines.append(self.make_points(frame, right_fit_average))

        # logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]
        self.get_logger().info('lane lines: %s' % lane_lines)
        return lane_lines

    def make_points(self,frame, line):
        height, width, _ = frame.shape
        slope, intercept = line
        y1 = height  # bottom of the frame
        y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

        # bound the coordinates within the frame
        try:
            x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        except:
            x1=0
        try:
            x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        except:
            x2=0
        return [[x1, y1, x2, y2]]

    def display_lines(self,frame, lines, line_color=(0, 255, 0), line_width=2):
        line_image = np.zeros_like(frame)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
        line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
        return line_image

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