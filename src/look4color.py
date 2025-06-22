#!/usr/bin/python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

#finds out if a percentage of the camera is of a certain color
class colorDetector:
    def __init__(self, color, index, percentage, show):
        self.publisher = rospy.Publisher("/sky_vision/utils/look4color", String, queue_size=1)
        self.bridge = CvBridge()
        self.show = show
        self.minimalPercentage = percentage
        self.color = color.lower()
        self.cameraIndex = index

        rospy.loginfo(f"LOOK4COLOR: Looking for a majority of the color {color} in camera_{index}. Try to make the screen {self.minimalPercentage * 100} percent of the color.")
        
        if self.color == "red":
            self.lower = (128, 104, 100)
            self.upper = (255, 255, 255)
        elif self.color == "green":
            self.lower = (36,25,25)
            self.upper = (86, 255,255)
        elif self.color == "blue":
            self.lower = (90, 50, 20)
            self.upper = (130, 255, 255)

    def processFrame(self, image):
        
        mask = cv2.inRange(image, self.lower, self.upper)
        count = cv2.countNonZero(mask)
        
        if count/(image.size/3) > self.minimalPercentage:
            rospy.loginfo(f"Majority of {self.color} found in camera_{self.cameraIndex}!")

        if self.show:
            cv2.imshow("mask", mask)
            cv2.waitKey(1)

    def callback(self, message):
        frame = cv2.cvtColor(self.bridge.imgmsg_to_cv2(message, "bgr8"), cv2.COLOR_BGR2HSV)
        
        self.processFrame(frame)

def main():
    rospy.init_node("look4color")

    color = rospy.get_param("look4color/color", "blue")
    index = rospy.get_param("look4color/index", 0)
    percentage = rospy.get_param("look4color/minimalPercentage", 0.7)
    show = rospy.get_param("look4color/show", True)

    look = colorDetector(color, index, percentage, show)
    rospy.Subscriber(f"/sky_vision/camera_{index}/image_raw", Image, look.callback)

    rospy.spin()

if __name__ == '__main__':
    main()
