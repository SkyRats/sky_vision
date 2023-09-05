#!/usr/bin/env python
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

from sky_detectors import BlockDetector

from cam_config import CAMERA_MATRIX, DISTORTION_MATRIX, CAMERA_FOV, CAMERA_RES


class BlockROS:
    def __init__(self):
        #Sim ranges
        #lower_mask = np.array([110, 50, 50]) 
        #upper_mask = np.array([130, 255, 255])

        #Drone ranges
        lower_mask = np.array([[90,50,50]])
        upper_mask = np.array([105, 255, 255])

        # Create the block detector object
        self.detector = BlockDetector(CAMERA_RES, lower_mask, upper_mask)

        # State of the detection
        self.type = ""

        # ROS node
        rospy.init_node('sky_vision_block', anonymous=False)

        # Bridge ros-opencv
        self.bridge_object = CvBridge()

        # Post detection image publisher
        self.newimg_pub = rospy.Publisher('/sky_vision/down_cam/block/image', Image, queue_size=10)
        self.cam = Image()

        # Post detection pose info publisher
        self.pose_pub = rospy.Publisher('/sky_vision/down_cam/block/pose', Point, queue_size=1)
        self.pose = Point()

        try:
            print("\nCreating block subscribers...")
            rospy.Subscriber('/sky_vision/down_cam/img_raw', Image, self.camera_callback)
            rospy.Subscriber('/sky_vision/down_cam/type', String, self.type_callback)
            print("Block Subscribers up!")
        except:
            print('Error trying to create subscribers!')

        self.frame = None

        rospy.spin()


    def type_callback(self, message):

        # Get current state
        self.type = message.data


    #-- Get new frame
    def camera_callback(self, message):
       
        if self.type == "block":
            #print("UIUIUIUI")
            # Bridge de ROS para CV
            cam = self.bridge_object.imgmsg_to_cv2(message, "bgr8")
            self.frame = cam

            target = self.detector.mapCircles(self.frame)

            if target is not None:
                #print("AAAAAAAAAAAAAAAAAAAAAAAAA")
                (cx, cy), (w, h), angle = target
                draw_img = cv2.circle(self.frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                # Publish image with target identified
                ros_img = self.bridge_object.cv2_to_imgmsg(draw_img, 'bgr8')
                self.newimg_pub.publish(ros_img)

                self.pose.x = cx
                self.pose.y = cy
                self.pose.z = 0

                # Publish target pose info
                self.pose_pub.publish(self.pose)


package = BlockROS()