import numpy as np

CAMERA_MATRIX = np.array([[644.60188395, 0., 310.97994546], 
                 [0., 602.01153122, 261.3654111 ], 
                 [0., 0., 1.]], dtype = np.float32)

# distortion_matrix = np.array([-2.710e-01, 1.208e+00, -2.564e-03, -1.967e-03, -2.322e+00])
DISTORTION_MATRIX = np.zeros((5,1))

CAMERA_FOV = (1.2, 1.1)

CAMERA_RES = (640, 480)