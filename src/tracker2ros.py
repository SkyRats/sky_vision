import cv2
import rospy
import time
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray


class Tracker():
    def __init__(self):

        self.tracker = cv2.TrackerKCF_create()
        self.tracking = False
        self.processed_image = None
        self.bridge = CvBridge()
        self.delta = 1
        # max time given to tracker since last correct tracking
        self.start = 0
        # time elapsed since tracker stopped tracking

        self.bb = Int16MultiArray()
        rospy.Subscriber("/sky_vision/down_cam/image_raw", Image, self.camera_callback)
        rospy.Subscriber("/sky_vision/down_cam/bb", Int16MultiArray, self.bb_callback)
        coord_pub = rospy.Publisher('/sky_vision/down_cam/pad/bounding_box', Int16MultiArray, queue_size=10)

    def bb_callback(self, data):
        self.bb = data
        self.tracking = True
        self.start = time.time()
        self.tracker.init(self.image, self.bb)
    
    def camera_callback(self, data):
        
        self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    
    def track(self, image) -> bool:
        
        if self.tracking:
            (success, box) = self.tracker.update(image)
            
            if success:
                self.start = time.time()
                
                (x, y, w, h) = [int(v) for v in box]
                
                
                self.bb.data = (x, y, w, h)
                self.coord_pub.publish(self.bb)
            
            else:  
                self.processed_image = None
                print("LOST OBJECT. TRYING AGAIN...")
                if (time.time() - self.start) > self.delta:
                    print("STOPPED TRYING")
                    self.tracking = False
                return None
                
        else:
            print("NOT TRACKING")

