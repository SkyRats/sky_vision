import cv2
import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision


MODEL_PATH = "gesture_recognizer.task"

import cv2
import numpy as np
import math
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int16MultiArray

class GestureDetector:
    def __init__(self) -> None:
        BaseOptions = mp.tasks.BaseOptions
        GestureRecognizer = mp.tasks.vision.GestureRecognizer
        GestureRecognizerOptions = mp.tasks.vision.GestureRecognizerOptions
        GestureRecognizerResult = mp.tasks.vision.GestureRecognizerResult
        VisionRunningMode = mp.tasks.vision.RunningMode
        
        options = GestureRecognizerOptions(
            base_options=BaseOptions(model_asset_path=MODEL_PATH),
            running_mode=VisionRunningMode.LIVE_STREAM,
            result_callback=self.gesture_callback)
        
        self.recognizer = GestureRecognizer.create_from_options(options)
        self.timestamp = 0

        self.cvBridge = CvBridge()
        self.gesture_msg = String()
        self.hand_position_msg = Int16MultiArray

        
        self.publisher_gesture = rospy.Publisher("/sky_vision/front_cam/gesture", String, queue_size=10)
        self.publisher_hand_ = rospy.Publisher("sky_vision/front_cam/hand_position", Int16MultiArray, queue_size=10)
        print("publishers created!")

    def gesture_callback(self, result, output_image: mp.Image, timestamp_ms: int):

        # Create a gesture recognizer instance with the live stream mode:
        # print('gesture recognition result: {}'.format(result))
    
        for i, gesture in enumerate(result.gestures):
            # Get the top gesture from the recognition result
            print("Top Gesture Result: ", gesture[0].category_name)
            self.gesture_msg.data = gesture[0].category_name
            self.publisher_gesture.publish(self.gesture_msg)

    def image_callback(self, msg):

        frame = self.cvBridge.imgmsg_to_cv2(msg, "bgr8")
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame)

        self.timestamp += 1
        self.recognizer.recognize_async(mp_image, self.timestamp)
            
if __name__ == '__main__':
    rospy.init_node('gesture_detector', anonymous=True)
    
    detector = GestureDetector()

    rate = rospy.Rate(10)  # Adjust the rate as needed
    subscribed = False

    while not rospy.is_shutdown():
        if not subscribed:
            topics = rospy.get_published_topics()
            topic_list = [topic for topic, _ in topics]
            if "/sky_vision/camera_0/image_raw" in topic_list:
                rospy.Subscriber("/sky_vision/camera_0/image_raw", Image, detector.image_callback)
                subscribed = True
                print("Subscribed to /sky_vision/camera_0/image_raw")
        rate.sleep()
    rospy.spin()

