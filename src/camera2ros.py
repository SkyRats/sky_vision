#!/usr/bin/python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Camera:
    def __init__(self, index):

        cameraName = f"camera_{index}"
        topicName = f"sky_vision/{cameraName}/image_raw" 
        self.capture = cv2.VideoCapture(index) #openCV camera object. Default camera index is 0
        self.bridge = CvBridge() #bridge type that converts between openCV and ROS images
        self.publisher = rospy.Publisher(topicName, Image, queue_size=1)

        rospy.loginfo(f"Trying to open {cameraName}.")
        while(not self.capture.isOpened()):
            rospy.logwarn(f"{cameraName}: Wasn't able to open the camera. Trying again...")
            rospy.sleep(1)

        rospy.loginfo(f"{cameraName} open!")
        cameraInfo = f"{cameraName} INFO:\nWIDTH: {self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)}\nHEIGHT: {self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)}\nFPS: {self.capture.get(cv2.CAP_PROP_FPS)}\nFORMAT: {self.capture.get(cv2.CAP_PROP_FORMAT)}\nPUBLISHING AS: bgr8"
        rospy.loginfo(cameraInfo)

    def processFrame(self):

        try:
            check, cvImage = self.capture.read() #check returns if the function was successful or not.
            self.publisher.publish(self.bridge.cv2_to_imgmsg(cvImage, "bgr8"))
        
        except CvBridgeError as e:
            if(not check):
                rospy.logerr("Couldn't read the frame.")
            rospy.logerr(e)

    def __del__(self):
        self.capture.release()

def main():
    rospy.init_node(f"camera")
    index = rospy.get_param(f"~camera/index", 0) #default openCV camera index, could also use -1
    cam = Camera(index)

    while not rospy.is_shutdown():
        cam.processFrame()

if __name__ == '__main__':
    main()
