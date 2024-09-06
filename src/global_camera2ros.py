#!/usr/bin/python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class VideoCapture:
    def __init__(self, simulation, index_front, index_down):
        self.simulation = simulation
        self.bridge = CvBridge()

        if self.simulation:
            self.front_cam_sub = None
            self.down_cam_sub = None
            if self.topic_exists('/webcam_front/image_raw_front'):
                self.front_cam_sub = rospy.Subscriber('/webcam_front/image_raw_front', Image, self.front_cam_callback)
            if self.topic_exists('/webcam_down/image_raw_down'):
                self.down_cam_sub = rospy.Subscriber('/webcam_down/image_raw_down', Image, self.down_cam_callback)
            self.front_cam_pub = rospy.Publisher('/sky_vision/front_cam/img_raw', Image, queue_size=1)
            self.down_cam_pub = rospy.Publisher('/sky_vision/down_cam/img_raw', Image, queue_size=1)
        else:
            self.cameras = []
            if index_down == -1 and index_front == -1:
                if self.camera_exists(0):
                    self.cameras.append(Camera(0, 'generic_cam'))
            else:
                if index_front != -1 and self.camera_exists(index_front):
                    self.cameras.append(Camera(index_front, 'front_cam'))
                if index_down != -1 and self.camera_exists(index_down):
                    self.cameras.append(Camera(index_down, 'down_cam'))

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
        for camera in self.cameras:
            camera.process_frame()

class Camera:
    def __init__(self, index, cam_name):
        self.cam_name = cam_name
        self.capture = cv2.VideoCapture(index)
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher(f'/sky_vision/{cam_name}/img_raw', Image, queue_size=1)

        rospy.loginfo(f"Trying to open {cam_name}.")
        while not self.capture.isOpened():
            rospy.logwarn(f"{cam_name}: Wasn't able to open the camera. Trying again...")
            rospy.sleep(1)

        rospy.loginfo(f"{cam_name} open!")
        camera_info = f"{cam_name} INFO:\nWIDTH: {self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)}\nHEIGHT: {self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)}\nFPS: {self.capture.get(cv2.CAP_PROP_FPS)}\nFORMAT: {self.capture.get(cv2.CAP_PROP_FORMAT)}\nPUBLISHING AS: bgr8"
        rospy.loginfo(camera_info)

    def process_frame(self):
        try:
            check, cv_image = self.capture.read()
            if check:
                resized_image = cv2.resize(cv_image, (640,480))
                self.publisher.publish(self.bridge.cv2_to_imgmsg(resized_image, "bgr8"))
            else:
                rospy.logerr(f"Couldn't read the frame from {self.cam_name}.")
        except CvBridgeError as e:
            rospy.logerr(f"Error in process_frame: {e}")

    def __del__(self):
        self.capture.release()

if __name__ == '__main__':
    rospy.init_node('sky_vision_camera_node', anonymous=False)
    simulation = rospy.get_param('~simulation', True)
    index_front = rospy.get_param('~index_front', 1)
    index_down = rospy.get_param('~index_down', 0)
    vc = VideoCapture(simulation, index_front, index_down)

    rate = rospy.Rate(10)  # Adjust the rate as needed

    while not rospy.is_shutdown():
        if not simulation:
            vc.capture()
        rate.sleep()

    if not simulation:
        for camera in vc.cameras:
            del camera
        cv2.destroyAllWindows()
