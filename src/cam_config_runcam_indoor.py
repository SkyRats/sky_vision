import numpy as np

CAMERA_MATRIX = np.array([[629.60088304,  0.0,         649.96450783],
                          [0.0,           628.99975883, 323.37037351],
                          [0.0,           0.0,           1.0]], dtype=np.float32)

DISTORTION_MATRIX = np.array([-0.08654266,  0.00064634, -0.01367921,  0.00537603,  0.00417901])

CAMERA_FOV = (1.58717, 1.03966)

CAMERA_RES = (1280, 720)
