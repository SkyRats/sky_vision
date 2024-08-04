#!/usr/bin/python3

import rospy
import cv2
import easyocr
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int16MultiArray

class hDetector:
    def __init__(self) -> None:
        self.publisher = rospy.Publisher("/sky_vision/h_center", Int16MultiArray, queue_size=10)
        print("Created publisher")
        self.bridge = CvBridge()
        self.point = Int16MultiArray()

    def calcular_distancia(self, ponto1, ponto2):
        return np.sqrt((ponto1[0] - ponto2[0])**2 + (ponto1[1] - ponto2[1])**2)
    
    def centro(self, bbox):
        # bbox é a lista de coordenadas [x_min, y_min, x_max, y_max]
        x_min, y_min = bbox[0]
        x_max, y_max = bbox[2]
        cx = (x_min + x_max) / 2
        cy = (y_min + y_max) / 2
        return int(cx), int(cy)

    def process_frame(self, image: np.ndarray):
        reader = easyocr.Reader(['en'], gpu=True)
        altura, largura, _ = image.shape
        centro_imagem = (largura // 2, altura // 2)
        text = reader.readtext(image, canvas_size=640, batch_size=1)
        filtro = [result for result in text if result[1] == 'H']
        
        if filtro:
            # Encontrar a caixa delimitadora do 'H' mais próximo do centro
            h_mais_proximo = min(filtro, key=lambda result: self.calcular_distancia(self.centro(result[0]), centro_imagem))
            bbox, text, score = h_mais_proximo
            
            # Calcular o centro da caixa delimitadora
            px, py = self.centro(bbox)
            print(f"Centro do H detectado em: ({px}, {py})")
        else:
            # Se nenhum 'H' for detectado, definir valores padrão
            px, py = -1, -1
        
        self.point.data = (px, py)
        self.publisher.publish(self.point)

    def callback(self, message) -> None:
        self.process_frame(self.bridge.imgmsg_to_cv2(message, "bgr8"))

def main():
    rospy.init_node("h_detector")
    detector = hDetector()
    rospy.Subscriber("/sky_vision/0/img_raw", Image, detector.callback)
    rospy.spin()

if __name__ == '__main__':
    main()
