#!/usr/bin/env python
import os
import rospy
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from sky_vision.srv import YOLOservice

# This assumes you have already made a detection inside de ZebraDetector folder, and its ncnn model is there

# Expand the '~' to the full home directory path
model_path = os.path.expanduser('~/sky_ws/src/sky_vision/src/ZebraDetector/best_ncnn_model')

# YOLO model initialization
model = YOLO(model_path)

# Convert ROS Image message to OpenCV format
bridge = CvBridge()

def handle_yolo_detection(req):
    try:
        # Convert the ROS Image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(req.image, "bgr8")

        # Perform YOLO detection
        results = model(cv_image, conf=req.confidence, save=True)

        # You can process 'results' and return relevant information in the response
        rospy.loginfo(f"YOLO detection completed. {len(results)} objects detected.")

        return True  # Replace this with the actual response message containing the results

    except Exception as e:
        rospy.logerr(f"YOLO detection failed: {e}")
        return False  # Update the response with error details if necessary

def yolo_service():
    rospy.init_node('yolo_detection_service')

    # Define the service, e.g., 'run_yolo' with Image as input and output
    service = rospy.Service('run_yolo', YOLOservice, handle_yolo_detection)
    rospy.loginfo("YOLO Detection Service Ready.")
    
    rospy.spin()

if __name__ == '__main__':
    try:
        yolo_service()
    except rospy.ROSInterruptException:
        pass
