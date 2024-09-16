#!/usr/bin/python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
import math

class PackageDetector:
    
    def __init__(self, debug=True, lower_mask = np.array([125,45,145]), upper_mask = np.array([160,145,255])) -> None:
        self.lower_mask = lower_mask
        self.upper_mask = upper_mask
        self.debug = True
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher('/sky_vision/down_cam/package_dist_center', Int16MultiArray, queue_size=1)
        self.center = Int16MultiArray()
        if debug: rospy.loginfo("[PACKAGE] Package detector initiated")

    def filter_and_mask(self, frame):
        
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_mask, self.upper_mask)
        mask = cv2.erode(mask, self.kernel, iterations=1)
        mask = cv2.dilate(mask, self.kernel, iterations=1)
        
        return mask
        
    def is_valid_cnt(self, cnt, min_area, ratio_bound = 0.3):
        if cv2.contourArea(cnt) < min_area:
            return False

        epsilon = 0.002 * cv2.arcLength(cnt, True)
        cnt = cv2.approxPolyDP(cnt, epsilon, True)
        
        _, _, w, h = cv2.boundingRect(cnt)
        ratio = w/h

        if ratio > 1 - ratio_bound and ratio < 1 + ratio_bound:
            return True
    
        return False


    def process_frame(self, frame, min_area = 400):

        mask = self.filter_and_mask(frame)
        package = None
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # cv2.imshow("mask", mask)

        if cnts is None:
            return None
        
        draw = cv2.drawContours(frame, cnts, -1, (0, 255, 0), 2)
        # cv2.imshow("draw", frame)
        
        valid_cnts = [cnt for cnt in cnts if self.is_valid_cnt(cnt, min_area=min_area)]
        
        if len(valid_cnts) < 1:
            return None    
        
        # cv2.drawContours(frame, valid_cnts, -1, (0, 255, 0), 2)

        package = max(valid_cnts, key=lambda x: cv2.contourArea(x))
        
        # cv2.drawContours(frame, [package], 0, (255, 0, 0), 2)
        # cv2.imshow("frame", frame)
        # cv2.waitKey(1)

        return package

    def publish(self, center : Int16MultiArray):
        self.publisher.publish(center)

    def callback(self, msg):        
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        package = self.process_frame(frame)
        if package is None:
            return
        
        M = cv2.moments(package)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

        h, w = frame.shape[:2]

        if self.debug: rospy.loginfo(f"[PACKAGE] found package at x:{cX - w//2} y:{cY - h//2}")
        self.center.data = [cX - w/2, cY - h/2]
       
        # cv2.circle(frame, (w//2, h//2), 2, (0, 255, 0), 2)
        # cv2.imshow("frame", frame)
        # cv2.waitKey(1)
       
        self.publish(self.center)

# cap = cv2.VideoCapture(0)
# package = PackageDetector()

# while(True):
#     try:
#         _, frame = cap.read()
#         cv2.imshow("fra", frame)
#         cv2.waitKey(1)
#         package.process_frame(frame)
#     except KeyboardInterrupt:
#         cap.release()
#         break
    
if __name__ == "__main__":
    rospy.init_node("package_detection")
    package = PackageDetector()

    rospy.Subscriber(f"/sky_vision/down_cam/img_raw", Image, package.callback)
    rospy.spin()