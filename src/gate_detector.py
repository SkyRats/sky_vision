#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

class GateDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/sky_vision/front_cam/img_raw", Image, self.image_callback)
        self.position_pub = rospy.Publisher("/gate_position", Point, queue_size=10)

        self.camera_matrix = np.array([[536.60468864, 0.0, 336.71838244],
                                       [0.0, 478.13866264, 353.24213721],
                                       [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
        
    def image_callback(self, msg):

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        rospy.loginfo("Received image")
        
        position = self.detect_gate(cv_image)
        if position is not None:
            point_msg = Point()
            point_msg.x, point_msg.y, point_msg.z = position
            self.position_pub.publish(point_msg)
            rospy.loginfo(f"Published position: {position}")
        else:
            rospy.loginfo("No gate detected")

    def detect_gate(self, image):

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define the red color range in HSV
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)
        
        lower_red = np.array([160, 100, 100])
        upper_red = np.array([179, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask1 + mask2

        cv2.imshow("Mask", mask)
        cv2.waitKey(1)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        rospy.loginfo(f"Found {len(contours)} contours")
        
        # Loop over the contours to find the gate
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            aspect_ratio = float(w) / h
            area = cv2.contourArea(contour)
            rospy.loginfo(f"Contour area: {area}, aspect ratio: {aspect_ratio}")

            if 0.9 < aspect_ratio < 1.5 and 1000 < area < 10000:
                rospy.loginfo("Detected rectangular contour")
                

                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])

                    z = 1.0  # Assume a certain distance, need calibration
                    x = (cX - self.camera_matrix[0, 2]) * z / self.camera_matrix[0, 0]
                    y = (cY - self.camera_matrix[1, 2]) * z / self.camera_matrix[1, 0]
                    rospy.loginfo(f"Detected gate at (x, y, z): ({x}, {y}, {z})")
                    return (x, y, z)
        return None

if __name__ == '__main__':
    rospy.init_node('gate_detector', anonymous=True)
    detector = GateDetector()
    rospy.spin()
