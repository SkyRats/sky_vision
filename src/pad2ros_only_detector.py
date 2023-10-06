#!/usr/bin/python3

import rospy
import os
import cv2
import torch
import time
from random import uniform
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray

class Pad:
    def __init__(self) -> None:
        # ROS node
        rospy.init_node('sky_vision_pad', anonymous=False)
        os.environ["CUDA_VISIBLE_DEVICES"]=""
        self.model = torch.hub.load('/home/gui/yolov5', 'custom', path='./cbr_best.pt', force_reload=True, source='local')
        
        # Bridge ros-opencv
        self.bridge_object = CvBridge()

        # Post detection image publisher
        self.newimg_pub = rospy.Publisher('/sky_vision/down_cam/img_result', Image, queue_size=10)
        self.cam = Image()

        # Post detection pose info publisher
        self.pad_pub = rospy.Publisher('/sky_vision/down_cam/pad/bounding_box', Int16MultiArray, queue_size=1)
        self.pad_point = Int16MultiArray()
        
        try:
            print("\nCreating pad subscribers...")
            rospy.Subscriber('/webcam/image_raw', Image, self.camera_callback)
            #rospy.Subscriber('/sky_vision/down_cam/img_raw', Image, self.camera_callback)
            print("Pad subscribers up!")
        except:
            print('Error trying to create subscribers!')

        self.frame = None
    
    def camera_callback(self, message):
        
        # Bridge de ROS para CV
        cam = self.bridge_object.imgmsg_to_cv2(message, "bgr8")
        self.frame = cv2.cvtColor(cam, cv2.COLOR_BGR2RGB)
    
    def detect(self):
        
        try:
            results = self.model(self.frame)
            #print(results)
            
        except:
            print("Frame as", self.frame)
            return
        
        objects = results.xyxy[0]
        detected_object = None
        print("SEEN OBJECTS: ", len(objects))
        for object in objects:
            print(object)
            if object[5] != 10:
                
                print("Object detected!")
                detected_object = object
            
            try:
                if(detected_object != None):
                    point = (int((detected_object[0] + detected_object[2])/2),
                            int((detected_object[1] + detected_object[3])/2),
                            self.frame.shape[0], self.frame.shape[1])
                    print("Publishing:")
                    print(point)
                    
                    ros_image = self.bridge_object.cv2_to_imgmsg(self.frame, 'bgr8')
                    self.pad_point.data = point
                    
                    self.newimg_pub.publish(ros_image)
                    self.pad_pub.publish(self.pad_point)
            except:
                "No object to publish."
                
        #results.show()          
       
        
# Init the pad detector package
package = Pad()

while(not rospy.is_shutdown()):
    try:
        package.detect()
        
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()