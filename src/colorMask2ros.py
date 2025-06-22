#!/usr/bin/python3

import rospy
import cv2
import numpy as np
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge



class ColorFilterRos:
  def __init__(self) -> None:
    rospy.init_node("color_filter", anonymous=False)

    rospy.Subscriber("/sky_vision/color_filter/lower", Int16MultiArray, self.lower_callback)
    rospy.Subscriber("/sky_vision/color_filter/upper", Int16MultiArray, self.upper_callback)
    rospy.Subscriber("/sky_vision/color_filter/camera_topic", String, self.camera_topic_callback)
    self._camera_topic = "/sky_vision/down_cam/img_raw"
    self._lower = np.array([147, 38, 0])
    self._upper = np.array([180, 255, 255])
    self.bridge_object = CvBridge()

    rospy.Subscriber(self._camera_topic, Image, self.camera_callback)
    self._camera_pub = rospy.Publisher("/sky_vision/color_filter/img_result", Image, queue_size=10)
    self._mask_pub = rospy.Publisher("/sky_vision/color_filter/mask", Image, queue_size=10)

    rospy.spin()
  
  def camera_callback(self, message) -> None:
    # Convert ROS Image to OpenCV Image
    cv_image = self.bridge_object.imgmsg_to_cv2(message, desired_encoding="bgr8")

    # Convert the image to HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Apply the mask
    mask = cv2.inRange(hsv, self._lower, self._upper)

    # Apply the mask to the original image
    result = cv2.bitwise_and(cv_image, cv_image, mask=mask)

    # Convert the image back to ROS format
    result = self.bridge_object.cv2_to_imgmsg(result, encoding="bgr8")
    mask = self.bridge_object.cv2_to_imgmsg(mask, encoding="mono8")

    # Publish the image
    self._camera_pub.publish(result)
    self._mask_pub.publish(mask)

  def camera_topic_callback(self, message) -> None:
    self._camera_topic = message.data

  def lower_callback(self, message) -> None:
    self._lower = np.array(message.data)
  
  def upper_callback(self, message) -> None:
    self._upper = np.array(message.data)

package = ColorFilterRos()
