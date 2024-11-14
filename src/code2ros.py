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
        self.publisher_img = rospy.Publisher(f"/sky_vision/{camera_name}/img_res", Image, queue_size=1)

        
        if self.camera_name == "down_cam":
            self.publisher_center = rospy.Publisher("/sky_vision/code/center", Int16MultiArray, queue_size=1)
            self.publisher_down_cam_read = rospy.Publisher("/sky_vision/code/read", String, queue_size=1)
            self.publisher_down_cam_bd = rospy.Publisher("/sky_vision/code/bounding_box", Int16MultiArray, queue_size=1)
        
        print(f"Created publisher for {camera_name}")
        self.bridge = CvBridge()
        self.code = String()
        self.center = Int16MultiArray()

    def decrease_brightness(self, img, value=30):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        lim = 255 - value
        v[v > lim] = 255
        v[v <= lim] -= value

        final_hsv = cv2.merge((h, s, v))
        res = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
        return res


    def find_center(self, rect):
        x, y, width, height = rect[0], rect[1], rect[2], rect[3]
        return int(x + width / 2), int(y + height / 2)

    def process_frame(self, image: np.ndarray):
        gray = cv2.cvtColor(self.decrease_brightness(image, 75), cv2.COLOR_BGR2GRAY)
        #threshold = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)  
        _, threshold = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)

        self.publisher_img.publish(self.bridge.cv2_to_imgmsg(threshold, "mono8"))


        height, width = gray.shape
        image_center = (width // 2, height // 2)

        if self.camera_name == "down_cam":
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
                
                self.center.data = self.find_center(closest_code.rect)
                self.publisher_center.publish(self.center)
                self.publisher_down_cam_read.publish(self.code)
                print(f"Published to /sky_vision/code/read: {code_read}")

                self.publisher_read.publish(self.code)
                print(f"Found centralized QR Code on {self.camera_name}: {code_read}")

                bounding_box = Int16MultiArray() 
                bounding_box.data = [closest_code.rect[0], closest_code.rect[1], 
                                closest_code.rect[0] + closest_code.rect[2], closest_code.rect[1],
                                closest_code.rect[0] + closest_code.rect[2], closest_code.rect[1] + closest_code.rect[3], 
                                closest_code.rect[0], closest_code.rect[1] + closest_code.rect[3]]
                self.publisher_down_cam_bd.publish(bounding_box)
                print(f"bounding box: {bounding_box}")

        elif self.camera_name == "front_cam":
            for code in pyzbar.decode(threshold):
                code_read = code.data.decode('utf-8')
                self.code.data = code_read
                self.publisher_read.publish(self.code)
                print(f"Found QR Code on {self.camera_name}: {code_read}")

    def callback(self, message):
        img = self.bridge.imgmsg_to_cv2(message, "bgr8")
        self.process_frame(img)

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
