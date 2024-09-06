#!/usr/bin/env python
import os
import rospy
from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from sky_vision.srv import YOLOservice, YOLOserviceResponse  # Import the service message

# Expand the '~' to the full home directory path for the model
model_path = os.path.expanduser('~/sky_ws/src/sky_vision/src/ZebraDetector/best_ncnn_model')

# YOLO model initialization
model = YOLO(model_path)

# Convert ROS Image message to OpenCV format
bridge = CvBridge()

def crop_and_resize(image, target_size=(640, 640)):
    """Crops the image to a square from the center and resizes it to target_size."""
    h, w, _ = image.shape

    # Determine the center and the size of the square crop
    if h > w:
        # If height is greater than width, crop the height
        top = (h - w) // 2
        cropped_image = image[top:top + w, :]
    else:
        # If width is greater than height, crop the width
        left = (w - h) // 2
        cropped_image = image[:, left:left + h]

    # Resize to target size (640x640)
    resized_image = cv2.resize(cropped_image, target_size)

    return resized_image

def handle_yolo_detection(req):
    try:
        # Convert the ROS Image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(req.image, "bgr8")
        
        # Get original image dimensions
        original_height, original_width, _ = cv_image.shape
        
        # Crop the image to a square and resize to 640x640
        resized_image = crop_and_resize(cv_image, target_size=(640, 640))

        # Perform YOLO detection
        results = model(resized_image)

        # Initialize response data
        num_objects = 0
        confidences = []
        bbox_centers_x = []
        bbox_centers_y = []

        if len(results) > 0:
            result = results[0]  # Only take the first result (since we process one image)

            if result.boxes:
                num_objects = len(result.boxes)  # Number of detected objects
                confidences = [float(box.conf) for box in result.boxes]  # Extract confidence scores

                for box in result.boxes:
                    # Extract normalized center coordinates and dimensions
                    x_center_normalized, y_center_normalized, width_normalized, height_normalized = box.xywhn[0].tolist()
                    
                    # Convert normalized coordinates to original image coordinates
                    x_center_pixel = x_center_normalized * original_width
                    y_center_pixel = y_center_normalized * original_height
                    
                    bbox_centers_x.append(x_center_pixel)
                    bbox_centers_y.append(y_center_pixel)

                    # Draw a red point at the center of each bounding box on the original image
                    cv2.circle(cv_image, (int(x_center_pixel), int(y_center_pixel)), 5, (0, 0, 255), -1)

                # Save the image with annotations
                save_path = os.path.expanduser('~/sky_ws/src/sky_vision/annotated_image.jpg')
                cv2.imwrite(save_path, cv_image)
                rospy.loginfo(f"Annotated image saved to {save_path}")

                rospy.loginfo(f"YOLO detection completed. {num_objects} objects detected.")
                
                # Return the response with the number of detected objects and their confidences
                return YOLOserviceResponse(
                    success=True,
                    num_objects=num_objects,
                    confidences=confidences,
                    bbox_centers_x=bbox_centers_x,
                    bbox_centers_y=bbox_centers_y
                )
            else:
                rospy.logerr("No bounding boxes found in YOLO detection results.")
                return YOLOserviceResponse(
                    success=False,
                    num_objects=0,
                    confidences=[],
                    bbox_centers_x=[],
                    bbox_centers_y=[]
                )
        else:
            rospy.logerr("No results returned from YOLO detection.")
            return YOLOserviceResponse(
                success=False,
                num_objects=0,
                confidences=[],
                bbox_centers_x=[],
                bbox_centers_y=[]
            )

    except Exception as e:
        rospy.logerr(f"YOLO detection failed: {e}")
        return YOLOserviceResponse(
            success=False,
            num_objects=0,
            confidences=[],
            bbox_centers_x=[],
            bbox_centers_y=[]
        )

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
