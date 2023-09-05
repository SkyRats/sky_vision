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

