#!/usr/bin/python3

import rospy
import os
import cv2
import torch
import time
from random import uniform
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int16MultiArray

from cam_config import CAMERA_MATRIX, DISTORTION_MATRIX, CAMERA_FOV, CAMERA_RES

CAMERA_INFO = [CAMERA_MATRIX, DISTORTION_MATRIX, CAMERA_RES, CAMERA_FOV]
 
TARGET_SIZE = 30 # 1mx1m square

class PadDetector:

    def __init__(self) -> None:
        print("initiating detector...")
        os.environ["CUDA_VISIBLE_DEVICES"]=""
        self.model = torch.hub.load('/home/gui/yolov5', 'custom', path='./best.pt', force_reload=True, source='local')
        self.tracker = cv2.TrackerKCF_create()    
        self.tracking = False  
        self.delta = 3
        self.processed_image = None
        

        #max time given to tracker since last correct tracking
        self.start = 0
        #time elapsed since tracker stopped tracking
        
    def detect(self, image):
        
        #simulating processing time
        time.sleep(uniform(0.7, 0.9))
        
        tracked_object = None
        
        results = self.model(image)
        
        objects = results.xyxy[0]
        print("SEEN OBJECTS:: ", len(objects))
        for object in objects:
            print(object)
            if object[5] == 0:
                
                tracked_object = object
                box = (object[0], object[1],
                       object[2] - object[0], 
                       object[3] - object[1])
                        
        if tracked_object != None:
            print("TRACKING")
            self.tracking = True
            self.start = time.time()
            self.tracker.init(image, box)
            
        else:
            self.tracking = False

    def track(self, image) -> bool:
        
        if self.tracking:
            (success, box) = self.tracker.update(image)
            
            if success:
                self.start = time.time()
                
                (x, y, w, h) = [int(v) for v in box]
                cv2.rectangle(image, (x, y), (x + w, y + h),
                    (0, 255, 0), 2)
                
                self.processed_image = image
                return (x, y, w, h)
            
            else:  
                self.processed_image = None
                print("LOST OBJECT. TRYING AGAIN...")
                if (time.time() - self.start) > self.delta:
                    print("STOPPED TRYING")
                    self.tracking = False
                return None
                
        else:
            print("NOT TRACKING")

class PadRos:
    def __init__(self):

        # ROS node
        rospy.init_node('sky_vision_pad', anonymous=False)

        #model and tracker
        self.detector = PadDetector()
        
        # State of the detection
        self.type = "pad"       

        # Bridge ros-opencv
        self.bridge_object = CvBridge()

        # Post detection image publisher
        self.newimg_pub = rospy.Publisher('/sky_vision/down_cam/img_result', Image, queue_size=10)
        self.cam = Image()

        # Post detection pose info publisher
        self.bb_pub = rospy.Publisher('/sky_vision/down_cam/pad/bounding_box', Int16MultiArray, queue_size=1)
        self.bb = Int16MultiArray()
        
        try:
            print("\nCreating pad subscribers...")
            rospy.Subscriber('/webcam/image_raw', Image, self.camera_callback)
            #rospy.Subscriber('/sky_vision/down_cam/img_raw', Image, self.camera_callback)
            rospy.Subscriber('/sky_vision/down_cam/type', String, self.type_callback)
            print("Pad subscribers up!")
        except:
            print('Error trying to create subscribers!')

        self.frame = None

        rospy.spin()

    def type_callback(self, message):

        # Get current state
        self.type = message.data
    
    def camera_callback(self, message):
        print("---camera callback---")
        
        if self.type != "pad":
            print("---ended callback (pad detection not desired)---")
            return
    
        # Bridge de ROS para CV
        cam = self.bridge_object.imgmsg_to_cv2(message, "bgr8")
        self.frame = cam

        #process image
        self.processing = True
        
        if self.detector.tracking == False:
            print("TRYING TO DETECT...")
            self.detector.detect(self.frame)
        
        else:
            print("TRYING TO TRACK...")
            #get bounding box from tracker
            bounding_box = self.detector.track(self.frame)
            image = self.detector.processed_image
            if bounding_box != None:
                ros_image = self.bridge_object.cv2_to_imgmsg(image, 'bgr8')
                
                self.bb.data = bounding_box
                
                self.newimg_pub.publish(ros_image)
                self.bb_pub.publish(self.bb)
        
        self.processing = False
        
        print("---ended callback---\n")

# Init the pad detector package
package = PadRos()
 