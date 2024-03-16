#!/usr/bin/python3

import rospy
import os
import cv2
import torch
import time
from random import uniform
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray, String


class Pad:
    def __init__(self) -> None:
        # ROS node
        rospy.init_node('sky_vision_pad', anonymous=False)
        os.environ["CUDA_VISIBLE_DEVICES"]=""
        self.model = torch.hub.load('/home/software/yolov5', 'custom', path='./cbr_best.pt', force_reload=True, source='local')
        
        # Bridge ros-opencv
        self.bridge_object = CvBridge()
        self.type = None
        
        # Post detection image publisher
        self.newimg_pub = rospy.Publisher('/sky_vision/down_cam/img_result', Image, queue_size=10)
        self.cam = Image()

        # Post detection pose info publisher
        self.pad_pub = rospy.Publisher('/sky_vision/down_cam/pad/bounding_box', Int16MultiArray, queue_size=1)
        self.pad_point = Int16MultiArray()
        
        try:
            print("\nCreating pad subscribers...")
            #rospy.Subscriber('/webcam/image_raw', Image, self.camera_callback)
            rospy.Subscriber('/sky_vision/down_cam/type', String, self.type_callback)
            rospy.Subscriber('/sky_vision/down_cam/img_raw', Image, self.camera_callback)
            print("Pad subscribers up!")
        except:
            print('Error trying to create subscribers!')

        self.frame = None
    
    def type_callback(self, message):
            
            print("callback")
            # Get current state
            print(message)
            self.type = message.data

    def camera_callback(self, message):
        
        # Bridge de ROS para CV
        cam = self.bridge_object.imgmsg_to_cv2(message, "bgr8")
        self.frame = cv2.cvtColor(cam, cv2.COLOR_BGR2RGB)
        imagem = self.frame

        # Calcula o tamanho do quadrado central com base na porcentagem
        tamanho_quadrado = int(min(altura, largura) * porcentagem_cinza / 100)

        # Calcula as coordenadas para o quadrado central
        inicio_x = (largura - tamanho_quadrado) // 2
        inicio_y = (altura - tamanho_quadrado) // 2
        fim_x = inicio_x + tamanho_quadrado
        fim_y = inicio_y + tamanho_quadrado

        # Cria uma cópia da imagem original
        imagem_editada = imagem.copy()

        # Define a cor cinza (neste caso, [128, 128, 128] representa cinza)
        cor_cinza = [128, 128, 128]

        # Preenche as bordas da imagem com a cor cinza, exceto o quadrado central
        imagem_editada[:inicio_y, :] = cor_cinza  # Parte superior
        imagem_editada[fim_y:, :] = cor_cinza      # Parte inferior
        imagem_editada[:, :inicio_x] = cor_cinza  # Parte esquerda
        imagem_editada[:, fim_x:] = cor_cinza      # Parte direita

        self.frame = imagem

    
    def detect(self):
        #print(self.type)
        if self.type == "pad":
            
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
                        self.type = None
                except:
                    print("No object to publish.")
                
        #results.show()          
       
        
# Init the pad detector package
package = Pad()

while(not rospy.is_shutdown()):
    try:
        package.detect()
        
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()
