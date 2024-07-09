#!/usr/bin/python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import pyzbar.pyzbar as pyzbar

#basically a wrapper that decodes a qrcode or barcode and publishes the results
class codeDetector:
    def __init__(self):
        self.publisher = rospy.Publisher("/sky_vision/code/read", String, queue_size=10)
        self.bridge = CvBridge()
        self.code = String()

    def processFrame(self, image : cv2.Mat):

        #this first step is quite important! We are using an adaptive threshold to dinamically binarize the image
        threshold = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,11,2)
        for code in pyzbar.decode(threshold):
            code_read = code.data.decode('utf-8')
            self.code.data = code_read
            self.publisher.publish(self.code.data)

    def callback(self, message):
        frame = cv2.cvtColor(self.bridge.imgmsg_to_cv2(message, "bgr8"),cv2.COLOR_BGR2GRAY)
        self.processFrame(frame)

def main():
    rospy.init_node("code_detector")

    detector = codeDetector()
    rospy.Subscriber("/sky_vision/camera_0/image_raw", Image, detector.callback)

    rospy.spin()

if __name__ == '__main__':
    main()