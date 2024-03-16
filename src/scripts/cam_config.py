import numpy as np

CAMERA_MATRIX = np.array([[906, 0., 317], 
                 [0., 909, 240.3654111 ], 
                 [0., 0., 1.]], dtype = np.float32)

DISTORTION_MATRIX = np.array([-0.42, 0.2, 0, 0, 0.4])
# DISTORTION_MATRIX = np.zeros((5,1))

CAMERA_FOV = (0.7, 0.56)

CAMERA_RES = (640, 480)