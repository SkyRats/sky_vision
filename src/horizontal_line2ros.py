#!/usr/bin/env python3

import cv2
import numpy as np
import math
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

#taken from https://stackoverflow.com/questions/43664328/remove-similar-lines-provided-by-hough-transform

def calculate_length(line):
    rho, theta = line
    return abs(rho)  # Length could be represented by the absolute value of rho

def unify_lines(lines, rho_threshold=120, theta_threshold=np.pi/180*6):
    grouped_lines = []

    # Step 1: Group similar lines
    for line in lines:
        for rho, theta in line:
            matched_group = None
            for group in grouped_lines:
                for u_rho, u_theta in group:
                    if (abs(u_rho - rho) < rho_threshold and abs(u_theta - theta) < theta_threshold) or \
                   (abs(u_rho + rho) < rho_threshold and abs(u_theta - (theta + np.pi)) % (2 * np.pi) < theta_threshold):
                        matched_group = group
                        break
                if matched_group:
                    break
            if matched_group:
                matched_group.append((rho, theta))
            else:
                grouped_lines.append([(rho, theta)])

    # Step 2: Compute weighted average for each group
    unified_lines = []
    for group in grouped_lines:
        total_weight = sum(calculate_length(line) for line in group)
        average_rho = sum(rho * calculate_length((rho, theta)) for rho, theta in group) / total_weight
        average_theta = sum(theta * calculate_length((rho, theta)) for rho, theta in group) / total_weight
        unified_lines.append((average_rho, average_theta))

    return np.array([[line] for line in unified_lines], dtype=np.float32)

class lineFollower:
    def __init__(self) -> None:
        
        self.publisher = rospy.Publisher("/sky_vision/down_cam/horizontal_line/angle", Bool, queue_size = 10)
        self.cvBridge = CvBridge()
        #self.angle = Float32()
        self.has_horizontal_line = Bool(False)
        self.lower_mask = np.array([80, 50, 50])
        self.upper_mask = np.array([105, 255, 255])
        self.kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (4, 15))
        self.rho_delta = 100
        self.theta_delta = np.pi/8

    def image_callback(self, msg):
        frame = self.cvBridge.imgmsg_to_cv2(msg, "bgr8")
        height, width, channels = frame.shape
        desired_height = 0.1 * height

        height_start = height // 2 - desired_height // 2
        height_end = height_start + desired_height

        cropped_frame = frame[height_start:height_end, :]

        lines = self.find_lines(cropped_frame)
        if lines is not None:
            for line in lines:
                #print(line)
                theta = line[0][1]
                #print(theta)
                if (theta > (np.pi/2)*0.9) and (theta < (np.pi/2)*1.1):
                    rospy.loginfo("linha horizontal mano brother") 
                    rospy.loginfo(theta)
                    self.has_horizontal_line.data = True
                    break

        self.publisher.publish(self.has_horizontal_line)    

    def filter(self, frame):

        mask = cv2.inRange(frame, self.lower_mask, self.upper_mask)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        mask = cv2.bitwise_not(mask, mask)

        return cv2.bitwise_and(frame, frame, mask=mask)
        
    def merge_lines(self, lines : np.ndarray):

        return unify_lines(lines)

    def find_lines(self, frame):
        
        filtered_image = self.filter(frame)
        #cv2.imshow("Filtered", filtered_image)
        gray = cv2.cvtColor(filtered_image, cv2.COLOR_BGR2GRAY)
        canny = cv2.Canny(gray, 50, 200, None, 3)

        cdst = cv2.cvtColor(canny, cv2.COLOR_GRAY2BGR)

        lines = None
        lines = cv2.HoughLines(canny, 1, np.pi / 180, 150, None, 0, 0)

        if lines is not None:
            if len(lines) > 1: lines = self.merge_lines(lines)

            for i in range(0, len(lines)):
                rho = lines[i][0][0]
                theta = lines[i][0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000*(-b)), int(y0 + 1000*(a)))
                pt2 = (int(x0 - 1000*(-b)), int(y0 - 1000*(a)))
                cv2.line(cdst, pt1, pt2, (0,0,255), 3, cv2.LINE_AA)

        #cv2.imshow("LINE BRUH", cdst)
        #cv2.waitKey(1)
          
        return lines

if __name__ == '__main__':
    rospy.init_node('horizontal_line_detector', anonymous=True)
    
    simulation = rospy.get_param('~simulation', False)
    detector = lineFollower()

    rate = rospy.Rate(10)  # Adjust the rate as needed
    subscribed = False

    while not rospy.is_shutdown():
        if not subscribed:
            topics = rospy.get_published_topics()
            topic_list = [topic for topic, _ in topics]
            if "/sky_vision/color_filter/img_result" in topic_list:
                rospy.Subscriber("/sky_vision/color_filter_img_result", Image, detector.image_callback)
                subscribed = True
                print("Subscribed to /sky_vision/down_cam/img_raw")
        rate.sleep()
    rospy.spin()
