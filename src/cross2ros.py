#!/usr/bin/python3

import rospy

import cv2

from cv_bridge import CvBridge

from sensor_msgs.msg import Image

from std_msgs.msg import String, Int16MultiArray

import pyzbar.pyzbar as pyzbar

import numpy as np



class codeDetector:

    def __init__(self):

        print("Created publisher")

        self.bridge = CvBridge()

        self.kernel = np.ones((5, 5), np.uint8)
        self.min_area = 6000

    def process_frame(self, image: np.ndarray):
        
        # PERDAO PELO CÓDIGO NOJENTO PRECISO DORMIR
        og = image

        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        #escolha entre threshold dinamico ou n
        #image = cv2.medianBlur(image, 3)
        #_, threshold = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY_INV)
        
        threshold = cv2.adaptiveThreshold(image, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 15, 7)

        contours, _ = cv2.findContours(threshold, cv2.RETR_TREE ,cv2.CHAIN_APPROX_SIMPLE)

        #cv2.drawContours(og, contours, -1, (0, 0 ,255), 3)
        for contour in contours:

            if cv2.contourArea(contour) > self.min_area:
                
                epsilon = 0.005*cv2.arcLength(contour,True)
                contour = cv2.approxPolyDP(contour, epsilon, True)
                
                hull = cv2.convexHull(contour, returnPoints=False)
                defects = cv2.convexityDefects(contour, hull)
                
                #AQUI A "LEN" MUDARIA DEPENDENDO DO H OU DO +
                if type(defects) != type(None) and len(defects) == 4:


                    #AQUI VIRIA UM FILTRO DE COR INSANO
                                            
                    for i in range(defects.shape[0]):
                        s,e,f,d = defects[i,0]
                        start = tuple(contour[s][0])
                        end = tuple(contour[e][0])
                        far = tuple(contour[f][0])
                        #cv2.line(og,start,end,[0,255,0],2)
                        cv2.circle(og,far,10,[0,0,255],-1)    

                # print(defects)
                if type(defects) != type(None) and len(defects) == 4:
                    cv2.drawContours(og, [contour], 0, (0,255,0), 3)    
                else:
                    cv2.drawContours(og, [contour], 0, (0,0,255), 3)

        cv2.imshow("AAA", og)
        cv2.imshow("TWA", threshold2)
        cv2.waitKey(1)


    def callback(self, message):

        frame = self.bridge.imgmsg_to_cv2(message, "bgr8")

        self.process_frame(frame)



def main():

    rospy.init_node("code_detector")

    detector = codeDetector()

    rospy.Subscriber("/sky_vision/camera_0/image_raw", Image,detector.callback)

    rospy.spin()


if __name__ == '__main__':

    main()
