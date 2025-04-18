#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import Float64, String

class BorderDetector:
    def __init__(self,simulation):
        self.bridge = CvBridge()
        self.simulation = simulation
        # self.image_sub = rospy.Subscriber("/sky_vision/front_cam/img_raw", Image, self.image_callback)
        self.corners_pub = rospy.Publisher("/border_corners", Polygon, queue_size=1)
        self.area_pub = rospy.Publisher("/border_area", Float64, queue_size=1)
        self.color_pub = rospy.Publisher("/border_color", String, queue_size=1)
        self.latest_image = None
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def timer_callback(self, event):
        if self.latest_image is not None:
            border_corners, final_image, area = self.detect_border(self.latest_image, 'red')
            if border_corners is not None:
                self.publish_detection(border_corners, final_image, area, 'red')
                #cv2.imshow("Border Detection", final_image)
                #cv2.waitKey(1)

    def detect_border(self, image, color):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        if not self.simulation:

            # Add your own calibration here using calibrator.py
            lower_bound = np.array([149, 101, 79])
            upper_bound = np.array([255, 255, 255])
            mask = cv2.inRange(hsv, lower_bound, upper_bound)        # Apply Gaussian blur to reduce noise
            blurred_mask = cv2.GaussianBlur(mask, (15, 15), 0)

            # Use morphological operations to clean up the mask
            kernel = np.ones((5, 5), np.uint8)
            morphed_mask = cv2.morphologyEx(blurred_mask, cv2.MORPH_CLOSE, kernel)
            morphed_mask = cv2.morphologyEx(morphed_mask, cv2.MORPH_OPEN, kernel)

            # Additional noise reduction
            morphed_mask = cv2.erode(morphed_mask, kernel, iterations=2)
            morphed_mask = cv2.dilate(morphed_mask, kernel, iterations=2)

            mask = morphed_mask

        else:
            # Define the red color range in HSV
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv, lower_red, upper_red)

            lower_red = np.array([160, 100, 100])
            upper_red = np.array([179, 255, 255])
            mask2 = cv2.inRange(hsv, lower_red, upper_red)

            mask = mask1 + mask2

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # rospy.loginfo(f"Found {len(contours)} {color} contours")

        min_area = 6000
        max_area_contour = None

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = float(w) / h
            area = cv2.contourArea(contour)
            rospy.loginfo(f"Border: {color.capitalize()}  contour area: {area}, aspect ratio: {aspect_ratio}")

            if 0.3 < aspect_ratio < 1.0 and area > min_area:
                max_area = area
                max_area_contour = contour

        if max_area_contour is not None:
            epsilon = 0.02 * cv2.arcLength(max_area_contour, True)
            approx = cv2.approxPolyDP(max_area_contour, epsilon, True)

            if len(approx) >= 4:
                rospy.loginfo(f"Detected {color} quadrilateral contour")
                corners = approx.reshape(-1, 2)

               
                sorted_corners = sorted(corners, key=lambda x: x[1])

                top_corners = sorted_corners[:2]
                bottom_corners = sorted_corners[2:]

                top_sorted = sorted(top_corners, key=lambda x: x[0])
                bottom_sorted = sorted(bottom_corners, key=lambda x: x[0])

                border_corners = [
                    top_sorted[0],
                    top_sorted[-1],
                    bottom_sorted[0],
                    bottom_sorted[-1]
                ]

                for i, corner in enumerate(border_corners):
                    cv2.circle(image, tuple(corner), 10, (255, 0, 0), -1)
                    cv2.putText(image, f'Corner {i+1}', tuple(corner), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                return border_corners, image, max_area

        return None, image, 0

    def publish_detection(self, border_corners, final_image, area, color):
        self.color_pub.publish(color)
        rospy.loginfo(f"Border color: {color}")
        polygon_msg = Polygon()
        for corner in border_corners:
            point32 = Point32()
            point32.x = corner[0]
            point32.y = corner[1]
            point32.z = 0.0
            polygon_msg.points.append(point32)
        self.corners_pub.publish(polygon_msg)
        rospy.loginfo(f"Published corners: {border_corners}")

        area_msg = Float64()
        area_msg.data = area
        self.area_pub.publish(area_msg)
        rospy.loginfo(f"Published area: {area}")

        #cv2.imshow(f"{color.capitalize()} Border Detection", final_image)
        #cv2.waitKey(1)

    def draw_border(self, image, corners):
        for i in range(len(corners)):
            cv2.circle(image, tuple(corners[i]), 5, (0, 255, 0), -1)

        for i in range(len(corners)):
            cv2.line(image, tuple(corners[i]), tuple(corners[(i + 1) % len(corners)]), (255, 0, 0), 2)

        return image

if __name__ == '__main__':
    rospy.init_node('border_detector', anonymous=True)
    
    simulation = rospy.get_param('~simulation', True)
    detector = BorderDetector(simulation)

    rate = rospy.Rate(10)  # Adjust the rate as needed
    subscribed = False

    while not rospy.is_shutdown():
        if not subscribed:
            topics = rospy.get_published_topics()
            topic_list = [topic for topic, _ in topics]
            if "/sky_vision/front_cam/img_raw" in topic_list:
                rospy.Subscriber("/sky_vision/front_cam/img_raw", Image, detector.image_callback)
                subscribed = True
                print("Subscribed to /sky_vision/front_cam/img_raw")
        rate.sleep()
    rospy.spin()
