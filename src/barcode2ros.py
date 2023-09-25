#!/usr/bin/python3

import rospy
import cv2
from random import uniform
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
import pyzbar.pyzbar as pyzbar
import re
import numpy as np

class QRcodeDetector():
    def __init__(self) -> None:
        
        # ROS node
        rospy.init_node('sky_vision_barcode', anonymous=False)
        
        # Post detection image publisher
        self.newbarcode_pub = rospy.Publisher('/sky_vision/down_cam/barcode_read', String, queue_size=10)
        self.code = String()
        
        # State of the detection
        self.type = "barcode"       

        # Bridge ros-opencv
        self.bridge_object = CvBridge()
        
        # Pattern to match
        self.pattern = r'[A-Da-d][0-4]'
                
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
    
    def add_to_list_if_matches_pattern(self, mydata, data_list):
        pattern = r'[A-Da-d][0-4]'
        if re.match(pattern, mydata):
            if mydata not in data_list:
                data_list.append(mydata)        
        return data_list

    def print_if_matches_pattern(self, mydata):
        pattern = r'[A-Da-d][0-4]'
        if re.match(pattern, mydata):
            print ("Impressao com filtro:", mydata)
        else:
            print ("Sem filtro:", mydata)
    
    def type_callback(self, message):

        # Get current state
        self.type = message.data
        
    def camera_callback(self, message):
        
        if self.type != "barcode":
            print("---ended callback (barcode detection not desired)---")
            return
        
        # Bridge de ROS para CV
        frame = self.bridge_object.imgmsg_to_cv2(message, "bgr8")
        frame = self.adjust_brightness_contrast(frame, brightness=30, contrast=70)

        # Find barcodes and Qr
        for barcode in pyzbar.decode(frame):
            
            # Convert the data from bytes to string
            barcode_read = barcode.data.decode('utf-8')
            
            self.print_if_matches_pattern(barcode_read) #if for pattern?
            
            if re.match(self.pattern, barcode_read):
                self.code.data = barcode_read
                self.newbarcode_pub.publish(self.code)
            
package = QRcodeDetector()
            
            

