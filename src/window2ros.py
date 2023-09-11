#!/usr/bin/python3
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

from sky_detectors import WindowDetector

from cam_config import CAMERA_MATRIX, DISTORTION_MATRIX, CAMERA_FOV, CAMERA_RES


class WindowROS:
    def __init__(self):

        # Red Mask ranges
        lower_mask = np.array([ 0, 238, 147])
        upper_mask = np.array([ 47, 255, 223])

        # Create the block detector object
        self.detector = WindowDetector(CAMERA_RES, lower_mask, upper_mask)

        # State of the detection
        self.type = ""

        # ROS node
        rospy.init_node('sky_vision_window', anonymous=False)

        # Bridge ros-opencv
        self.bridge_object = CvBridge()

        # Post detection image publisher
        self.newimg_pub = rospy.Publisher('/sky_vision/front_cam/img_result', Image, queue_size=10)
        self.cam = Image()

        # Post detection pose info publisher
        self.pose_pub = rospy.Publisher('/sky_vision/front_cam/window/pose', Point, queue_size=1)
        self.pose = Point()

        try:
            print("\nCreating window subscribers...")
            rospy.Subscriber('/sky_vision/front_cam/img_raw', Image, self.camera_callback)
            rospy.Subscriber('/sky_vision/front_cam/type', String, self.type_callback)
            print("Window Subscribers up!")
        except:
            print('Error trying to create subscribers!')

        self.frame = None

        rospy.spin()


    def type_callback(self, message):

        # Get current state
        self.type = message.data


    #-- Get new frame
    def camera_callback(self, message):

       
        if self.type == "window":
            # Bridge de ROS para CV
            cam = self.bridge_object.imgmsg_to_cv2(message, "bgr8")
            self.frame = cam

            dy, dz, draw_img = self.detector.getErrors(self.frame)
            print(f"DY: {dy}")

            if dy or dz:
                # Publish image with window identified
                ros_img = self.bridge_object.cv2_to_imgmsg(draw_img, 'bgr8')
                self.newimg_pub.publish(ros_img)

                # Set errors
                self.pose.x = 0
                self.pose.y = dy
                self.pose.z = dz
                
                # Publish target pose info
                self.pose_pub.publish(self.pose)


package = WindowROS()
