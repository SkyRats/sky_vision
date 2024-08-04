#!/usr/bin/python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray

class HDetector:
    def __init__(self) -> None:
        self.publisher = rospy.Publisher("/sky_vision/h_center", Int16MultiArray, queue_size=1)
        self.image_publisher = rospy.Publisher("/sky_vision/image_with_overlay", Image, queue_size=1)
        self.mask_publisher = rospy.Publisher("/sky_vision/blue_mask", Image, queue_size=1)
        print("Created publishers")
        self.bridge = CvBridge()
        self.point = Int16MultiArray()

    def process_frame(self, image: np.ndarray):
        altura, largura, _ = image.shape
        centro_imagem = (largura // 2, altura // 2)
        
        # Convert image to HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Define blue color range
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])
        
        # Create a mask for blue color
        blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)
        
        # Find contours
        contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Calculate the centroid of the largest contour
            moments = cv2.moments(largest_contour)
            if moments['m00'] != 0:
                cx = int(moments['m10'] / moments['m00'])
                cy = int(moments['m01'] / moments['m00'])
                print(f"Centro do H detectado em: ({cx}, {cy})")
                
                # Draw a circle at the centroid
                cv2.circle(image, (cx, cy), 10, (0, 255, 0), -1)  # Green circle with radius 10
                # Publish the center coordinates
                self.point.data = (cx, cy)
                self.publisher.publish(self.point)
            else:
                cx, cy = -1, -1
        else:
            cx, cy = -1, -1
        
        # Convert the image with overlay to ROS message and publish
        image_with_overlay = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.image_publisher.publish(image_with_overlay)
        
        # Convert the mask to ROS message and publish
        blue_mask_color = cv2.cvtColor(blue_mask, cv2.COLOR_GRAY2BGR)  # Convert mask to 3-channel image for visualization
        mask_image = self.bridge.cv2_to_imgmsg(blue_mask_color, "bgr8")
        self.mask_publisher.publish(mask_image)

    def callback(self, message) -> None:
        self.process_frame(self.bridge.imgmsg_to_cv2(message, "bgr8"))

def main():
    rospy.init_node("h_detector")
    detector = HDetector()
    rospy.Subscriber("/sky_vision/down_cam/img_raw", Image, detector.callback)
    rospy.spin()

if __name__ == '__main__':
    main()
