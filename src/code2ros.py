#!/usr/bin/python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int16MultiArray
import pyzbar.pyzbar as pyzbar
import numpy as np

class codeDetector:
    def __init__(self, camera_name):
        self.camera_name = camera_name
        self.publisher_read = rospy.Publisher(f"/sky_vision/{camera_name}/code/read", String, queue_size=1)
        
        if self.camera_name == "down_cam":
            self.publisher_center = rospy.Publisher("/sky_vision/code/center", Int16MultiArray, queue_size=1)
            self.publisher_down_cam_read = rospy.Publisher("/sky_vision/code/read", String, queue_size=1)
        
        print(f"Created publisher for {camera_name}")
        self.bridge = CvBridge()
        self.code = String()
        self.center = Int16MultiArray()

    def find_center(self, rect):
        x, y, width, height = rect[0], rect[1], rect[2], rect[3]
        return int(x + width / 2), int(y + height / 2)

    def process_frame(self, image: np.ndarray):
        threshold = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)

        height, width = image.shape
        image_center = (width // 2, height // 2)

        closest_code = None
        min_distance = float('inf')

        for code in pyzbar.decode(threshold):
            code_center = self.find_center(code.rect)
            distance = np.linalg.norm(np.array(image_center) - np.array(code_center))

            if distance < min_distance:
                min_distance = distance
                closest_code = code

        if closest_code:
            code_read = closest_code.data.decode('utf-8')
            self.code.data = code_read
            
            if self.camera_name == "down_cam":
                self.center.data = self.find_center(closest_code.rect)
                self.publisher_center.publish(self.center)
                self.publisher_down_cam_read.publish(self.code)
                print(f"Published to /sky_vision/code/read: {code_read}")

            self.publisher_read.publish(self.code)
            print(f"Found centralized QR Code on {self.camera_name}: {code_read}")

    def callback(self, message):
        frame = cv2.cvtColor(self.bridge.imgmsg_to_cv2(message, "bgr8"), cv2.COLOR_BGR2GRAY)
        self.process_frame(frame)

def main():
    rospy.init_node("code_detector")

    down_cam_detector = codeDetector("down_cam")
    front_cam_detector = codeDetector("front_cam")

    rate = rospy.Rate(10)  # Adjust the rate as needed
    subscribed = {"down_cam": False, "front_cam": False}

    while not rospy.is_shutdown():
        topics = rospy.get_published_topics()
        topic_list = [topic for topic, _ in topics]

        if not subscribed["down_cam"] and "/sky_vision/down_cam/img_raw" in topic_list:
            rospy.Subscriber("/sky_vision/down_cam/img_raw", Image, down_cam_detector.callback)
            subscribed["down_cam"] = True
            print("Subscribed to /sky_vision/down_cam/img_raw")

        if not subscribed["front_cam"] and "/sky_vision/front_cam/img_raw" in topic_list:
            rospy.Subscriber("/sky_vision/front_cam/img_raw", Image, front_cam_detector.callback)
            subscribed["front_cam"] = True
            print("Subscribed to /sky_vision/front_cam/img_raw")

        rate.sleep()

if __name__ == '__main__':
    main()
