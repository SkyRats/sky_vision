import numpy as np

CAMERA_MATRIX = np.array([[791.94357631   0.         549.31835049]
                          [  0.         796.5126599  374.50779314]
                          [  0.           0.           1.        ]], dtype=np.float32)

DISTORTION_MATRIX = np.array([[-0.34482158  0.06084902 -0.00248649  0.0104638   0.05379377]])

CAMERA_FOV = (1.58717, 1.03966)

CAMERA_RES = (1280, 720)
