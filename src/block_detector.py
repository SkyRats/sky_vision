import rospy
import cv2 as cv
import numpy as np
import time
import dronekit

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from mavros_msgs.msg import PositionTarget
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32

class blockMarker():
    def __init__(self):

        self.bridge = CvBridge()
        rospy.init_node('blockMarker', anonymous=False)
        
        self.image_sub = rospy.Subscriber('/webcam/image_raw', Image, self.callback)
        self.centers_pub = rospy.Publisher('centers', Point , queue_size=1)
        self.cv_image = None
        self.centers = None
        

    def findMask(self):
        hsv = cv.cvtColor(self.cv_image, cv.COLOR_BGR2HSV)
        lower = np.array([110,50,50])
        upper = np.array([130, 255, 255])
        mask = cv.inRange(hsv, lower, upper)
        return mask

    def mapCircles(self):
        mask = self.findMask()
        contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        centers = []
        for i in contours:
            curve = cv.approxPolyDP(i, 0.01*cv.arcLength(i, True), True)
            M = cv.moments(curve)
            cX = int(M['m10']/M['m00'])
            cY = int(M["m01"]/M["m00"])
            centers.append([cX, cY])
        
        self.centers_pub.publish(Point(centers[0][0], centers[0][1], 0))
        
    
    def callback(self, data): #Converts the imagemsg to a cv2 image
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            width = int(self.cv_image.shape[1] * 0.5)
            height = int(self.cv_image.shape[0]) 
            cv.circle(self.cv_image, (width, height), width//4, (0, 0, 0), -1)
            self.centers = self.mapCircles()
            
        except CvBridgeError as e:
            print(e)
        cv.imshow("Image window", self.cv_image)
        cv.waitKey(1) & 0xFF

if __name__ == '__main__':
    bd = blockMarker()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv.destroyAllWindows()