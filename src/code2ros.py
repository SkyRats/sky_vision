#!/usr/bin/python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import pyzbar.pyzbar as pyzbar
import numpy as np

class codeDetector:
    def __init__(self):
        self.publisher = rospy.Publisher("/sky_vision/down_cam/code/read", String, queue_size=10)
        print("Created down cam publisher")
        self.publisher = rospy.Publisher("/sky_vision/front_cam/code/read", String, queue_size=10)
        print("Created front cam publisher")
        self.bridge = CvBridge()
        self.code = String()

    def processFrame(self, image: np.ndarray):
        threshold = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
        for code in pyzbar.decode(threshold):
            code_read = code.data.decode('utf-8')
            self.code.data = code_read
            self.publisher.publish(self.code)
            print("Found QR Code: %s" % code_read)

    def callback(self, message):
        frame = cv2.cvtColor(self.bridge.imgmsg_to_cv2(message, "bgr8"), cv2.COLOR_BGR2GRAY)
        self.processFrame(frame)

def main():
    rospy.init_node("code_detector")
    detector = codeDetector()

    rate = rospy.Rate(10)  # Adjust the rate as needed
    subscribed = False

    while not rospy.is_shutdown():
        if not subscribed:
            topics = rospy.get_published_topics()
            topic_list = [topic for topic, _ in topics]
            if "/sky_vision/down_cam/img_raw" in topic_list:
                rospy.Subscriber("/sky_vision/down_cam/img_raw", Image, detector.callback)
                subscribed = True
                print("Subscribed to /sky_vision/down_cam/img_raw")
            if "/sky_vision/front_cam/img_raw" in topic_list:
                rospy.Subscriber("/sky_vision/front_cam/img_raw", Image, detector.callback)
                subscribed = True
                print("Subscribed to /sky_vision/front_cam/img_raw")
            elif "/sky_vision/generic_cam/img_raw" in topic_list:
                rospy.Subscriber("/sky_vision/generic_cam/img_raw", Image, detector.callback)
                subscribed = True
                print("Subscribed to /sky_vision/generic_cam/img_raw")
        rate.sleep()

if __name__ == '__main__':
    main()
