#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from geometry_msgs.msg import Polygon, Point32
from std_msgs.msg import Float64, String

class GateDetector:
    def __init__(self):
        self.bridge         = CvBridge()
        self.image_sub      = rospy.Subscriber("/sky_vision/front_cam/img_raw", Image, self.image_callback)
        self.corners_pub    = rospy.Publisher("/gate_corners", Polygon, queue_size=10)
        self.area_pub       = rospy.Publisher("/gate_area", Float64, queue_size=10)
        self.color_pub      = rospy.Publisher("/gate_color", String, queue_size=10)
        self.latest_image   = None
        self.timer          = rospy.Timer(rospy.Duration(0.5), self.timer_callback)

    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def timer_callback(self, event):
        if self.latest_image is not None:
            gate_corners, final_image, area, gate_color = self.detect_gate(self.latest_image)
            if gate_corners is not None:
                
                self.color_pub.publish(gate_color)
                rospy.loginfo(f"Gate color: {gate_color}")

                # Publish the corners
                polygon_msg = Polygon()
                for corner in gate_corners:
                    point32 = Point32()
                    point32.x = corner[0]
                    point32.y = corner[1]
                    point32.z = 0.0  # Assuming the corners are in 2D plane
                    polygon_msg.points.append(point32)
                self.corners_pub.publish(polygon_msg)
                rospy.loginfo(f"Published corners: {gate_corners}")

                # Publish the area
                area_msg = Float64()
                area_msg.data = area
                self.area_pub.publish(area_msg)
                rospy.loginfo(f"Published area: {area}")

                final_image = self.draw_gate(final_image, gate_corners)
                cv2.imshow("Gate Detection", final_image)
                cv2.waitKey(1)
            else:
                rospy.loginfo("No gate detected")

    def detect_gate(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the red color range in HSV
        lower_red   = np.array([0, 100, 100])
        upper_red   = np.array([10, 255, 255])
        mask1       = cv2.inRange(hsv, lower_red, upper_red)

        lower_red   = np.array([160, 100, 100])
        upper_red   = np.array([179, 255, 255])
        mask2       = cv2.inRange(hsv, lower_red, upper_red)
        red_mask    = mask1 + mask2

        # Define the yellow color range in HSV
        lower_yellow    = np.array([20, 100, 100])
        upper_yellow    = np.array([30, 255, 255])
        yellow_mask     = cv2.inRange(hsv, lower_yellow, upper_yellow)

        mask = red_mask + yellow_mask

        cv2.imshow("Mask", mask)
        cv2.waitKey(1)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        rospy.loginfo(f"Found {len(contours)} contours")

        max_area = 0
        max_area_contour = None
        detected_color = None

        # Loop over the contours to find the gate
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = float(w) / h
            area = cv2.contourArea(contour)
            rospy.loginfo(f"Contour area: {area}, aspect ratio: {aspect_ratio}")

            if 0.9 < aspect_ratio < 1.5 and 1000 < area and area > max_area:
                max_area = area
                max_area_contour = contour

                # Create a mask for the current contour
                contour_mask = np.zeros_like(mask)
                cv2.drawContours(contour_mask, [contour], -1, 255, -1)

                # Calculate the percentage of red and yellow pixels in the contour
                red_pixels      = cv2.bitwise_and(red_mask, red_mask, mask=contour_mask)
                yellow_pixels   = cv2.bitwise_and(yellow_mask, yellow_mask, mask=contour_mask)

                red_percentage      = cv2.countNonZero(red_pixels) / cv2.countNonZero(contour_mask)
                yellow_percentage   = cv2.countNonZero(yellow_pixels) / cv2.countNonZero(contour_mask)

                if red_percentage > yellow_percentage: detected_color = "red"
                else:                                  detected_color = "yellow"

                rospy.loginfo("Detected rectangular contour")

        if max_area_contour is not None:
            epsilon = 0.02 * cv2.arcLength(max_area_contour, True)
            approx = cv2.approxPolyDP(max_area_contour, epsilon, True)

            if len(approx) >= 4:
                rospy.loginfo("Detected quadrilateral contour")
                corners = approx.reshape(-1, 2)  # Extract all corners

                # Sort corners based on their coordinates
                sorted_corners = sorted(corners, key=lambda x: x[1])  # Sort by y-coordinate

                # Determine top and bottom corners
                top_corners = sorted_corners[:2]
                bottom_corners = sorted_corners[2:]

                # Sort top corners by x-coordinate
                top_sorted = sorted(top_corners, key=lambda x: x[0])
                bottom_sorted = sorted(bottom_corners, key=lambda x: x[0])

                # Arrange extreme points into a list
                gate_corners = [
                    top_sorted[0],  # Top-left
                    top_sorted[-1],  # Top-right
                    bottom_sorted[0],  # Bottom-left
                    bottom_sorted[-1]  # Bottom-right
                ]

                return gate_corners, image, max_area, detected_color

        return None, image, 0, None

    def draw_gate(self, image, corners):

        for i in range(len(corners)):
            cv2.circle(image, tuple(corners[i]), 5, (0, 255, 0), -1)

        for i in range(len(corners)):
            cv2.line(image, tuple(corners[i]), tuple(corners[(i+1) % len(corners)]), (255, 0, 0), 2)
        
        return image


if __name__ == '__main__':
    rospy.init_node('gate_detector', anonymous=True)
    detector = GateDetector()
    rospy.spin()
