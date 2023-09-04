#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String

from cam_config import CAMERA_MATRIX, DISTORTION_MATRIX, CAMERA_FOV, CAMERA_RES

from sky_detectors import ArucoDetector

CAMERA_INFO = [CAMERA_MATRIX, DISTORTION_MATRIX, CAMERA_RES, CAMERA_FOV]

ARUCO_TYPE = 5 # (5X5, 4X4, etc...)

TARGET_SIZE = 50 # 50x50cm square


class ArucoROS:
    def __init__(self):

        # Create the aruco detector object
        self.detector = ArucoDetector(ARUCO_TYPE, TARGET_SIZE, CAMERA_INFO)

        # State of the detection
        self.type = ""

        # ROS node
        rospy.init_node('sky_vision_aruco', anonymous=False)

        # Bridge ros-opencv
        self.bridge_object = CvBridge()

        # Post detection image publisher
        self.newimg_pub = rospy.Publisher('sky/vision/down_cam/aruco/image', Image, queue_size=10)
        self.cam = Image()

        try:
            print("Creating subscribers...")
            rospy.Subscriber('/sky/vision/down_cam/img_raw', Image, self.camera_callback)
            rospy.Subscriber('/sky/vision/down_cam/detection/type', String, self.type_callback)
            print("Subscribers up!")
        except:
            print('Error trying to create subscribers!')

        self.frame = None
    
    #-- Get new frame
    def camera_callback(self, message):

        # print("camera")

        # Bridge de ROS para CV
        cam = self.bridge_object.imgmsg_to_cv2(message, "bgr8")
        self.frame = cam

        if self.type == "aruco":
            print("aruco")
            # Look for the closest target in the frame
            closest_target = self.detector.find_closest_aruco(self.frame)

            if closest_target is not None:
            
                x, y, z, x_ang, y_ang, payload, draw_img = closest_target

                print(f'MARKER POSITION: x = {x} | y = {y} | z = {z} | x_ang = {round(x_ang, 2)} | y_ang = {round(y_ang, 2)} | ID = {payload}')
                
                # Publish image with target identified
                ros_img = self.bridge_object.cv2_to_imgmsg(draw_img, 'bgr8')
                self.newimg_pub.publish(ros_img)
    
    def type_callback(self, message):

        print("type")

        # Get current state
        self.type = message.data


# Init the aruco detector package
package = ArucoROS()

rospy.spin()
