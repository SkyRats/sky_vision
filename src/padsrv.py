#!/usr/bin/env python

import rospy

import cv2
import torch
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int16MultiArray
from pad.srv import A,B

class PadService():

    def __init__(self):
        print("initiating detector...")
        os.environ["CUDA_VISIBLE_DEVICES"]=""
        self.model = torch.hub.load('/home/gui/yolov5', 'custom', path='./cbr_best.pt', force_reload=True, source='local')
        self.processed_image = None
        
        
        rospy.init_node('pad_service', anonymous=True)
        rospy.Subscriber("/sky_vision/down_cam/image_raw", Image, self.camera_callback)
        
        self.bb_pub = rospy.Publisher('/sky_vision/down_cam/bb', Int16MultiArray, queue_size=10)
        self.bridge = CvBridge()
        self.cam = Image()
        self.bb = Int16MultiArray()
        self.service = rospy.Service('pad_service', A, self.handle_pad_service)

    def handle_pad_service(self, req):
        
        results = self.model(self.processed_image)
        objects = results.xyxy[0]
        print("SEEN OBJECTS:: ", len(objects))
        for object in objects:
            print(object)
            if object[5] != 100:
                box = (object[0], object[1],
                       object[2] - object[0], 
                       object[3] - object[1])
                self.bb.data = box
                self.bb_pub.publish(self.bb)
                return B("Pad Found!")
        return B("Pad Not Found!")
    
    def camera_callback(self, data):
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.processed_image = self.image

        
    

if __name__ == '__main__':
    service = PadService()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        
        