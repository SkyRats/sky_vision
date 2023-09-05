#!/usr/bin/env python

import cv2
from cv2 import aruco
import numpy as np


class ColorMask:
   def __init__(self):
      pass


class ArucoDetector:
   def __init__(self, size, marker_size, camera_info) -> None:

      self.marker_size = marker_size

      if size == 4:
         lib = cv2.aruco.DICT_4X4_1000
      elif size == 5:
         lib = cv2.aruco.DICT_5X5_1000
      elif size == 6:
         lib = cv2.aruco.DICT_6X6_1000
      else:
         lib = cv2.aruco.DICT_7X7_1000

      dictionary = aruco.getPredefinedDictionary(lib)
      parameters =  aruco.DetectorParameters()
      self.detector = aruco.ArucoDetector(dictionary, parameters)

      self.camera_matrix = camera_info[0]
      self.dist_coeff = camera_info[1]
      
      self.np_camera_matrix = np.array(self.camera_matrix)
      self.np_dist_coeff = np.array(self.dist_coeff)

      self.horizontal_res = camera_info[2][0]
      self.vertical_res = camera_info[2][1]

      self.horizontal_fov = camera_info[3][0]
      self.vertical_fov = camera_info[3][1]

   def pose_estimation(self, corners, marker_size, mtx, distortion):
      '''
      This will estimate the rvec and tvec for each of the marker corners detected by:
      corners, ids, rejectedImgPoints = detector.detectMarkers(image)
      corners - is an array of detected corners for each detected marker in the image
      marker_size - is the size of the detected markers
      mtx - is the camera matrix
      distortion - is the camera distortion matrix
      RETURN list of rvecs, tvecs, and trash (so that it corresponds to the old estimatePoseSingleMarkers())
      '''
      marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, marker_size / 2, 0],
                              [marker_size / 2, -marker_size / 2, 0],
                              [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)

      nada, rvec, tvec = cv2.solvePnP(marker_points, corners, mtx, distortion, False, cv2.SOLVEPNP_IPPE_SQUARE)
      return rvec, tvec
   
   def draw_marker(self, frame, points):
      topLeft, topRight, bottomRight, bottomLeft = points

      # Marker corners
      tR = (int(topRight[0]), int(topRight[1]))
      bR = (int(bottomRight[0]), int(bottomRight[1]))
      bL = (int(bottomLeft[0]), int(bottomLeft[1]))
      tL = (int(topLeft[0]), int(topLeft[1]))

      points = np.array([tR, bR, bL, tL])

      # Find the Marker center
      cX = int((tR[0] + bL[0]) / 2.0)
      cY = int((tR[1] + bL[1]) / 2.0)

      # Draw rectangle and circle
      rect = cv2.drawContours(frame, [points], 0, (0, 0, 255), 2)
      final = cv2.circle(rect, (cX, cY), radius=4, color=(0, 0, 255), thickness=-1)

      return final
   
   def find_closest_aruco(self, frame):

      # Marker detection
      markerCorners, markerIds, rejected = self.detector.detectMarkers(frame)

      i = 0
      if len(markerCorners) > 0: # if detect any Arucos

         closest_target = []
         closest_dist = 100000 # 1000 m (arbitrary large value)

         for corners in markerCorners: # For each Aruco

               marker_points = corners[0] # Vector with 4 points (x, y) for the corners

               # Check for false positives
               if cv2.arcLength(np.array([marker_points]), True) > 180:
                  print(cv2.arcLength(np.array([marker_points]), True))

                  # Draw points in image
                  final_image = self.draw_marker(frame, marker_points)

                  # Pose estimation
                  pose = self.pose_estimation(marker_points, self.marker_size, self.np_camera_matrix, self.np_dist_coeff)

                  rvec, tvec = pose

                  # 3D pose estimation vector
                  x = round(tvec[0][0], 2)
                  y = round(tvec[1][0], 2)
                  z = round(tvec[2][0], 2)

                  x_sum = marker_points[0][0] + marker_points[1][0] + marker_points[2][0] + marker_points[3][0]
                  y_sum = marker_points[0][1] + marker_points[1][1] + marker_points[2][1] + marker_points[3][1]

                  x_avg = x_sum / 4
                  y_avg = y_sum / 4

                  x_ang = (x_avg - self.horizontal_res*0.5)*self.horizontal_fov/self.horizontal_res
                  y_ang = (y_avg - self.vertical_res*0.5)*self.vertical_fov/self.vertical_res

                  payload = markerIds[i][0]
                  i += 1
                  
                  # Check for the closest target
                  if z < closest_dist:
                     closest_dist = z
                     closest_target = [x, y, z, x_ang, y_ang, payload, final_image]
         
         return closest_target
      return None
    

class BlockDetector:
   def __init__(self, cam_shape, lower, upper):
      self.cam_shape = cam_shape
      self.lower = lower
      self.upper = upper
      
   def findMask(self, frame):
      hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
      mask = cv2.inRange(hsv, self.lower, self.upper)
      return mask

   def mapCircles(self, frame):
      mask = self.findMask(frame)
      contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      centers = []
      for i in contours:
         curve = cv2.approxPolyDP(i, 0.01*cv2.arcLength(i, True), True)
         M = cv2.moments(curve)
         cX = int(M['m10']/M['m00'])
         cY = int(M["m01"]/M["m00"])
         centers.append([cX, cY])
         
      return centers
   
   def find_closest_circle(self, frame):
      centers = self.mapCircles(frame)
      max_dist = 0
      closest_target = None
      for center in centers:
         dist = center[0]**2 + center[1]**2
         if dist > max_dist:
            closest_target = center
            max_dist = dist
      return closest_target

class LineDetector:
   def __init__(self, cam_shape, lower, upper):
      self.cam_shape = cam_shape
      self.lower_mask = lower
      self.upper_mask = upper
      self.minArea = 5000
      
      
   def findMask(self, frame):
      hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
      mask = cv2.inRange(hsv, self.lower_mask, self.upper_mask)
      kernel = np.ones((3, 3), np.uint8)
      mask = cv2.erode(mask, kernel, iterations=5)
      mask = cv2.dilate(mask, kernel, iterations=9)
      return mask
   
   def zoom(self, cv_image, scale):
        if cv_image is None: return None
        width, height = self.cam_shape[0], self.cam_shape[1]

        # Crop the image
        centerX, centerY = int(height / 2), int(width / 2)
        radiusX, radiusY = int(scale * height / 100), int(scale * width / 100)

        minX, maxX = centerX - radiusX, centerX + radiusX
        minY, maxY = centerY - radiusY, centerY + radiusY

        cv_image = cv_image[minX:maxX, minY:maxY]
        cv_image = cv2.resize(cv_image, (width, height))

        return cv_image

   def getErrorAndAngle(self, frame):
      # Apply zoom
      frame = self.zoom(frame, scale=20)

      # Get mask
      mask = self.findMask(frame)
      
      # Find contours in mask
      contours_blk, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      contours_blk = list(contours_blk)
      normal_error, angle = None, None
     # If there is a line
      if len(contours_blk) > 0:

         # starts looking from left to right
         contours_blk.sort(key=cv2.minAreaRect)

         # if the area of the contour is greater than minArea
         if cv2.contourArea(contours_blk[0]) > self.minArea:
             
             blackbox = cv2.minAreaRect(contours_blk[0])
             (x_min, y_min), (w_min, h_min), angle = blackbox
             cv2.rectangle(frame, (int(x_min), int(y_min)), (int(x_min+w_min), int(y_min+h_min)), (0, 0, 255), 3)

             # fix angles
             if angle < -45:
                 angle = 90 + angle
             if w_min < h_min and angle > 0:
                 angle = (90 - angle) * -1
             if w_min > h_min and angle < 0:
                 angle = 90 + angle

             # Rotate image
             angle += 90

             # Optmize angle 
             if angle > 90:
                 angle = angle - 180

             # Get distance from center of image
             setpoint = self.cam_shape[1] / 2
             error = int(x_min - setpoint)

             ## Error correction (y axis)
             normal_error = float(error) / setpoint


      
      return normal_error, angle, frame

class WindowDetector:
   def __init__(self, cam_shape, lower, upper):
      self.cam_shape = cam_shape
      self.lower_mask = lower
      self.upper_mask = upper
      self.dy = None
      self.dz = None
      self.maxd = 5
      
      
   def findMask(self, frame):
      hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
      mask = cv2.inRange(hsv, self.lower_mask, self.upper_mask)
      kernel = np.ones((3, 3), np.uint8)
      mask = cv2.erode(mask, kernel, iterations=5)
      mask = cv2.dilate(mask, kernel, iterations=9)
      return mask
   
   def get_bigger_square(self, squares):
   # Recieves a list of squares and returns the bigger one
      bigger_square = None
      bigger_square_area = 0

      for square in squares:
            area = cv2.contourArea(square)
            if area > bigger_square_area:
               bigger_square = square
               bigger_square_area = area

      return bigger_square

   def getErrors(self, frame):
      # Get mask
      mask = self.findMask(frame)
      
      # Find contours in mask
      contours_blk, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      contours_blk = list(contours_blk)

     # If there is a contour
      if len(contours_blk) > 0:

         # Get the bigger one
         contour = self.get_bigger_square(contours_blk)

         # If the area is bigger than the min area
         if cv2.contourArea(contour) > self.minWindowArea:
               # Get the center of the image
               width, height = self.cam_shape[0] , self.cam_shape[1]
               centerX, centerY = int(width / 2), int(height / 2)

               # Get the center of the contour
               blackbox = cv2.minAreaRect(contour)
               (x_min, y_min), (w_min, h_min), angle = blackbox
               
               # Draw on image
               box = cv2.boxPoints(blackbox)
               box = np.int0(box)
               cv2.drawContours(frame, [box], 0, (0, 0, 255), 10)
               cv2.circle(frame, (int(x_min), int(y_min)), 5, (255, 0, 0), 10)
               cv2.circle(frame, (centerX, centerY), 5, (0, 255, 0), 10)

               # Error calculation
               self.dy = x_min - centerX
               self.dz = y_min - centerY

      
      return self.dy, self.dz, frame
   


