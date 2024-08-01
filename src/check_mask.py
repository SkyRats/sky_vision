import cv2
import numpy as np

# Open the existing video file
input_video_path = 'output.mp4'  # Change this to your input video file path
output_video_path = 'output_with_red_mask.mp4'  # Output video file path

cap = cv2.VideoCapture(input_video_path)

# Check if the video opened successfully
if not cap.isOpened():
    print("Error: Could not open video file.")
    exit()

# Get video properties
fps = int(cap.get(cv2.CAP_PROP_FPS))
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Define the codec and create a VideoWriter object
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for MP4 format
out = cv2.VideoWriter(output_video_path, fourcc, fps, (frame_width, frame_height))

# Minimum area for gate detection
min_area = 10000  # Change this value based on your requirements
min_distance_between_corners = 100  # Minimum distance between corners

while True:
    # Read frame-by-frame
    ret, frame = cap.read()
    
    if not ret:
        break
    
    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define the red color range in HSV
    # lower_red1 = np.array([0, 50, 50])
    # upper_red1 = np.array([10, 255, 255])
    lower_red1 = np.array([149, 115, 50])
    upper_red1 = np.array([255, 255, 255])
    
    # Create masks for red color
    mask1 = cv2.inRange(hsv_frame, lower_red1, upper_red1)
    # mask2 = cv2.inRange(hsv_frame, lower_red2, upper_red2)
    red_mask = mask1
    
    # Apply Gaussian blur to reduce noise
    blurred_mask = cv2.GaussianBlur(red_mask, (15, 15), 0)

    # Use morphological operations to clean up the mask
    kernel = np.ones((5, 5), np.uint8)
    morphed_mask = cv2.morphologyEx(blurred_mask, cv2.MORPH_CLOSE, kernel)
    morphed_mask = cv2.morphologyEx(morphed_mask, cv2.MORPH_OPEN, kernel)

    # Additional noise reduction
    morphed_mask = cv2.erode(morphed_mask, kernel, iterations=2)
    morphed_mask = cv2.dilate(morphed_mask, kernel, iterations=2)

    mask = morphed_mask

    # Find contours
    contours, _ = cv2.findContours(morphed_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    max_area = 0
    max_area_contour = None
    
    # Loop over the contours to find the gate
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = float(w) / h
        area = cv2.contourArea(contour)

        if 0.9 < aspect_ratio < 1.5 and area > min_area:
            if area > max_area:
                max_area = area
                max_area_contour = contour

    if max_area_contour is not None:
        epsilon = 0.02 * cv2.arcLength(max_area_contour, True)
        approx = cv2.approxPolyDP(max_area_contour, epsilon, True)

        if len(approx) >= 4:
            corners = approx.reshape(-1, 2)  # Extract all corners

            # Filter out corners that are too close to each other
            filtered_corners = []
            for corner in corners:
                if all(np.linalg.norm(corner - other_corner) > min_distance_between_corners for other_corner in filtered_corners):
                    filtered_corners.append(corner)

            if len(filtered_corners) >= 4:
                # Sort corners based on their coordinates
                sorted_corners = sorted(filtered_corners, key=lambda x: x[1])  # Sort by y-coordinate

                # Determine top and bottom corners
                top_corners = sorted_corners[:2]
                bottom_corners = sorted_corners[2:]

                # Sort top corners by x-coordinate
                top_sorted = sorted(top_corners, key=lambda x: x[0])
                bottom_sorted = sorted(bottom_corners, key=lambda x: x[0])

                # Arrange extreme points into a list
                gate_corners = [
                    top_sorted[0],  # Top-left
                    top_sorted[-1],  # Top-right
                    bottom_sorted[0],  # Bottom-left
                    bottom_sorted[-1]  # Bottom-right
                ]

                # Draw bounding box around the detected gate
                x, y, w, h = cv2.boundingRect(max_area_contour)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Green bounding box

                # Draw corners
                for i, corner in enumerate(gate_corners):
                    cv2.circle(frame, tuple(corner), 10, (255, 0, 0), -1)  # Blue circles for corners
                    cv2.putText(frame, f'Corner {i+1}', tuple(corner), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

    # Write the frame with annotations to the output video file
    out.write(frame)
    
    # Display the resulting frame and mask
    cv2.imshow('Red Masked Video', frame)
    cv2.imshow('Mask', mask)
    
    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release everything when done
cap.release()
out.release()
cv2.destroyAllWindows()
