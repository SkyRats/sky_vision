#!/usr/bin/python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class VideoCapture:
    def __init__(self, simulation):
        self.simulation = simulation
        self.bridge = CvBridge()
        
        if self.simulation:
            self.front_cam_sub = None
            self.down_cam_sub = None
            if self.topic_exists('/webcam_front/image_raw_front'):
                self.front_cam_sub = rospy.Subscriber('/webcam_front/image_raw_front', Image, self.front_cam_callback)
            if self.topic_exists('/webcam_down/image_raw_down'):
                self.down_cam_sub = rospy.Subscriber('/webcam_down/image_raw_down', Image, self.down_cam_callback)
            self.front_cam_pub = rospy.Publisher('/sky_vision/front_cam/img_raw', Image, queue_size=10)
            self.down_cam_pub = rospy.Publisher('/sky_vision/down_cam/img_raw', Image, queue_size=10)
        else:
            self.cap_front = None
            self.cap_down = None
            if self.camera_exists(1):
                self.cap_front = cv2.VideoCapture(1)
            if self.camera_exists(0):
                self.cap_down = cv2.VideoCapture(0)
            self.front_cam_pub = rospy.Publisher('/sky_vision/front_cam/img_raw', Image, queue_size=10)
            self.down_cam_pub = rospy.Publisher('/sky_vision/down_cam/img_raw', Image, queue_size=10)
    
    def topic_exists(self, topic_name):
        topics = rospy.get_published_topics()
        for topic, _ in topics:
            if topic == topic_name:
                return True
        return False
    
    def camera_exists(self, index):
        cap = cv2.VideoCapture(index)
        if cap.isOpened():
            cap.release()
            return True
        return False

    def front_cam_callback(self, msg):
        try:
            self.front_cam_pub.publish(msg)
        except CvBridgeError as e:
            rospy.logerr(f"Error in front cam callback: {e}")

    def down_cam_callback(self, msg):
        try:
            self.down_cam_pub.publish(msg)
        except CvBridgeError as e:
            rospy.logerr(f"Error in down cam callback: {e}")

    def capture(self):
        try:
            if self.cap_front:
                ret, frame = self.cap_front.read()
                if ret:
                    self.front_cam_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
            if self.cap_down:
                ret, frame = self.cap_down.read()
                if ret:
                    self.down_cam_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr(f"Error in capture: {e}")

if __name__ == '__main__':
    rospy.init_node('sky_vision_camera_node', anonymous=False)
    simulation = rospy.get_param('~simulation', True)
    vc = VideoCapture(simulation)

    rate = rospy.Rate(10)  # Adjust the rate as needed

    while not rospy.is_shutdown():
        if not simulation:
            vc.capture()
        rate.sleep()
    
    if not simulation:
        if vc.cap_front:
            vc.cap_front.release()
        if vc.cap_down:
            vc.cap_down.release()
        cv2.destroyAllWindows()
