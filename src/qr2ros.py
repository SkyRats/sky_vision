#!/usr/bin/python3

import rospy
import cv2
from random import uniform
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Char, Int16MultiArray, String
import pyzbar.pyzbar as pyzbar
import re
import numpy as np

class QRcodeDetector():
    def __init__(self) -> None:
        
        # ROS node
        rospy.init_node('sky_vision_qrcode', anonymous=False)
        
        # Post detection image publisher
        self.newqrcode_pub = rospy.Publisher('/sky_vision/down_cam/qrcode_read', Char, queue_size=10)
        self.code = Char()
        
        self.qrcodecenter_pub = rospy.Publisher('/sky_vision/down_cam/qrcode_center', Int16MultiArray, queue_size=10)
        self.center = Int16MultiArray()
        
        # bd -> bounding box
        self.qrcodebd_pub = rospy.Publisher('/sky_vision/down_cam/qrcode_bd', Int16MultiArray, queue_size = 10)
        self.qrcode_bd = Int16MultiArray()
        # State of the detection
        self.type = "qrcode"       

        # Bridge ros-opencv
        self.bridge_object = CvBridge()
        
        # Pattern to match
        self.pattern = "ABCDE"
                
        try:
            print("\nCreating pad subscribers...")
            #rospy.Subscriber('/webcam/image_raw', Image, self.camera_callback)
            rospy.Subscriber('/sky_vision/down_cam/img_raw', Image, self.camera_callback)
            rospy.Subscriber('/sky_vision/down_cam/type', String, self.type_callback)
            print("Pad subscribers up!")
        except:
            print('Error trying to create subscribers!')
            
        rospy.spin()
    
    def adjust_brightness_contrast(self, frame, brightness=0, contrast=0):
        # Adjust brightness and contrast using numpy operations
        frame = np.int16(frame)
        frame = frame * (contrast / 127 + 1) - contrast + brightness
        frame = np.clip(frame, 0, 255)
        frame = np.uint8(frame)
        return frame
    
    def type_callback(self, message):

        # Get current state
        self.type = message.data
        
    def camera_callback(self, message):
        
        if self.type != "qrcode":
            print("---ended callback (barcode detection not desired)---")
            return
        
        # Bridge de ROS para CV
        frame = self.bridge_object.imgmsg_to_cv2(message, "bgr8")
        frame = self.adjust_brightness_contrast(frame, brightness=30, contrast=70)

        # Find barcodes and Qr
        for qrcode in pyzbar.decode(frame):
            
            # Convert the data from bytes to string
            qrcode_read = qrcode.data.decode('utf-8')
            if qrcode_read in self.pattern:
                self.code.data = qrcode_read[0]
                
                bounding_box = qrcode.rect
                self.qrcode_bd.data = bounding_box
                self.center.data = (int(bounding_box[0] + bounding_box[2]/2),
                                    int(bounding_box[1] + bounding_box[3]/2))
                
                print(self.code, self.center.data)
                
                self.newqrcode_pub.publish(self.code)
                self.qrcodecenter_pub.publish(self.center)
                self.qrcodebd_pub.publish(self.qrcode_bd)
                
            
package = QRcodeDetector()