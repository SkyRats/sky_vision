#!/usr/bin/python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray
import numpy as np

class crossDetector():

    def __init__(self):
        self.publisher = rospy.Publisher("/sky_vision/cross_center", Int16MultiArray, queue_size=1)
        self.image_publisher = rospy.Publisher("/sky_vision/cross", Image, queue_size=1)
        print("Created publishers")
        self.bridge = CvBridge()
        self.point = Int16MultiArray()

        self.kernel = np.ones((5, 5), np.uint8)
        self.min_area = 6000

        self.lower_red1 = np.array([172, 140, 122])
        self.upper_red1 = np.array([205, 230, 255])

    def process_frame(self, image: np.ndarray):
        og = image

        hsv_frame = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create masks for red color
        image = cv2.inRange(hsv_frame, self.lower_red1, self.upper_red1)
        
        # Dynamic threshold
        threshold = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 15, 7)

        contours, _ = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            if cv2.contourArea(contour) > self.min_area:

                epsilon = 0.005 * cv2.arcLength(contour, True)
                contour = cv2.approxPolyDP(contour, epsilon, True)
                
                hull = cv2.convexHull(contour, returnPoints=False)
                defects = cv2.convexityDefects(contour, hull)
                
                # Check if defects exist and are four
                if defects is not None and len(defects) == 4:
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cX = int(M["m10"] / M["m00"])
                        cY = int(M["m01"] / M["m00"])
                        self.point.data = (cX, cY)
                        self.publisher.publish(self.point)

                    cv2.circle(image, (cX, cY), 7, (255, 0, 0), -1)
                    cv2.putText(image, "center", (cX - 20, cY - 20), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    
                    cv2.drawContours(image, [contour], -1, (0, 255, 0), 3)
                    
                    for i in range(defects.shape[0]):
                        s, e, f, d = defects[i, 0]
                        start = tuple(contour[s][0])
                        end = tuple(contour[e][0])
                        far = tuple(contour[f][0])
                        cv2.circle(image, far, 10, [0, 0, 255], -1)
                else:
                    cv2.drawContours(image, [contour], -1, (0, 0, 255), 3)

        # Convert the image with overlay to ROS message and publish
        image = self.bridge.cv2_to_imgmsg(og, "bgr8")
        self.image_publisher.publish(image)

    def callback(self, message):
        frame = self.bridge.imgmsg_to_cv2(message, "bgr8")

        # Only process the frame if it is not empty
        if frame is not None and frame.size > 0:
            self.process_frame(frame)

def main():
    rospy.init_node("cross_detector")
    detector = crossDetector()

    rate = rospy.Rate(10)
    subscribed = False

    while not rospy.is_shutdown():
        if not subscribed:
            topics = rospy.get_published_topics()
            topic_list = [topic for topic, _ in topics]
            if "/sky_vision/down_cam/img_raw" in topic_list:
                rospy.Subscriber("/sky_vision/down_cam/img_raw", Image, detector.callback)
                subscribed = True
                print("Subscribed to /sky_vision/down_cam/img_raw")
            elif "/sky_vision/generic_cam/img_raw" in topic_list:
                rospy.Subscriber("/sky_vision/generic_cam/img_raw", Image, detector.callback)
                subscribed = True
                print("Subscribed to /sky_vision/generic_cam/img_raw")
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    main()
