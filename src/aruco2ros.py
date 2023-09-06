#!/usr/bin/env python

import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point, Vector3

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
        self.newimg_pub = rospy.Publisher('/sky_vision/down_cam/img_result', Image, queue_size=10)
        self.cam = Image()

        # Post detection pose info publisher
        self.pose_pub = rospy.Publisher('/sky_vision/down_cam/aruco/pose', Point, queue_size=1)
        self.pose = Point()

        self.angle_pub = rospy.Publisher('/sky_vision/down_cam/aruco/angle', Vector3, queue_size=1)
        self.angle = Vector3()

        try:
            print("\nCreating aruco subscribers...")
            rospy.Subscriber('/sky_vision/down_cam/img_raw', Image, self.camera_callback)
            rospy.Subscriber('/sky_vision/down_cam/type', String, self.type_callback)
            print("Aruco Subscribers up!")
        except:
            print('Error trying to create subscribers!')

        self.frame = None

        rospy.spin()


    def type_callback(self, message):

        # Get current state
        self.type = message.data
    

    #-- Get new frame
    def camera_callback(self, message):

        if self.type == "aruco":

            # Bridge de ROS para CV
            cam = self.bridge_object.imgmsg_to_cv2(message, "bgr8")
            self.frame = cam

            # Look for the closest target in the frame
            closest_target = self.detector.find_closest_aruco(self.frame)

            if closest_target is not None and len(closest_target) > 0:
                
                # Unpack results
                x, y, z, x_ang, y_ang, payload, draw_img = closest_target

                print(f'MARKER POSITION: x = {x} | y = {y} | z = {z} | x_ang = {round(x_ang, 2)} | y_ang = {round(y_ang, 2)} | ID = {payload}')
                
                # Publish image with target identified
                ros_img = self.bridge_object.cv2_to_imgmsg(draw_img, 'bgr8')
                self.newimg_pub.publish(ros_img)

                self.pose.x = x
                self.pose.y = y
                self.pose.z = z

                self.angle.x = x_ang
                self.angle.y = y_ang
                self.angle.z = 0

                # Publish aruco pose info
                self.pose_pub.publish(self.pose)
                self.angle_pub.publish(self.angle)


# Init the aruco detector package
package = ArucoROS()
