#!/usr/bin/ python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class VideoCapture:
    def __init__(self):
        self.cap = cv2.VideoCapture(0)
        self.im_pub = rospy.Publisher('/sky_vision/down_cam/img_raw', Image, queue_size=10)
        self.bridge = CvBridge()
    
    def capture(self):
        try:
            _, frame = self.cap.read()
            self.im_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except CvBridgeError as e:
            print(e)


rospy.init_node('sky_vision_camera', anonymous=False)

vc = VideoCapture()

while not rospy.is_shutdown():
    vc.capture()
    
vc.cap.release()
cv2.destroyAllWindows()