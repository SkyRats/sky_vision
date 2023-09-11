#!/usr/bin/python3
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

from sky_detectors import LineDetector

from cam_config import CAMERA_MATRIX, DISTORTION_MATRIX, CAMERA_FOV, CAMERA_RES


class LineROS:
    def __init__(self):

        # Blue Mask ranges for simulation
        #lower_mask = np.array([ 69, 69, 37])
        #upper_mask = np.array([ 157, 255, 255])

        #Blue Mask ranges for drone
        lower_mask = np.array([69, 69, 37])
        upper_mask = np.array([157, 255, 255])

        # Create the block detector object
        self.detector = LineDetector(CAMERA_RES, lower_mask, upper_mask)

        # State of the detection
        self.type = ""

        # ROS node
        rospy.init_node('sky_vision_line', anonymous=False)

        # Bridge ros-opencv
        self.bridge_object = CvBridge()

        # Post detection image publisher
        self.newimg_pub = rospy.Publisher('/sky_vision/down_cam/img_result', Image, queue_size=10)
        self.cam = Image()

        # Post detection pose info publisher
        self.pose_pub = rospy.Publisher('/sky_vision/down_cam/line/pose', Point, queue_size=10)
        self.pose = Point()

        try:
            print("\nCreating line subscribers...")
            rospy.Subscriber('/sky_vision/down_cam/img_raw', Image, self.camera_callback)
            rospy.Subscriber('/sky_vision/down_cam/type', String, self.type_callback)
            print("Line Subscribers up!")
        except:
            print('Error trying to create subscribers!')

        self.frame = None

        rospy.spin()


    def type_callback(self, message):

        # Get current state
        self.type = message.data


    #-- Get new frame
    def camera_callback(self, message):
       
        if self.type == "line":
            # Bridge de ROS para CV
            cam = self.bridge_object.imgmsg_to_cv2(message, "bgr8")
            self.frame = cam

            error, angle, draw_img = self.detector.getErrorAndAngle(self.frame)

            if angle and error:

                print(f"LINE DETECTED: angle = {angle} | y_error = {error}")

                # Publish image with line identified
                ros_img = self.bridge_object.cv2_to_imgmsg(draw_img, '8UC3')
                self.newimg_pub.publish(ros_img)

                # Calculate error correction
                self.pose.x = error
                self.pose.y = angle
                self.pose.z = 1
                
                # Publish target pose info
                self.pose_pub.publish(self.pose)
            else:
                #print("NO LINE DETECTED")
                self.pose.x = 0
                self.pose.y = 0
                self.pose.z = 0

                self.pose_pub.publish(self.pose)


package = LineROS()
