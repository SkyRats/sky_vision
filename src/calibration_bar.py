import cv2
import numpy as np

def do_nothing(x):
    pass

# Create slider window
cv2.namedWindow("Slider")
cv2.resizeWindow("Slider", 640, 480)
cv2.createTrackbar("Hue Min", "Slider", 0, 255, do_nothing)
cv2.createTrackbar("Hue Max", "Slider", 255, 255, do_nothing)
cv2.createTrackbar("Saturation Min", "Slider", 50, 255, do_nothing)
cv2.createTrackbar("Saturation Max", "Slider", 255, 255, do_nothing)
cv2.createTrackbar("Value Min", "Slider", 50, 255, do_nothing)
cv2.createTrackbar("Value Max", "Slider", 255, 255, do_nothing)

# Initialize video capture
input_video_path = 'output.mp4'  # Change this to your input video file path
cap = cv2.VideoCapture(input_video_path)

# Check if the video opened successfully
if not cap.isOpened():
    print("Error: Could not open video file.")
    exit()

while True:
    # Read frame-by-frame
    ret, frame = cap.read()
    
    if not ret:
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Restart video if it ends
        continue

    # Get the current positions of the trackbars
    hue_min = cv2.getTrackbarPos("Hue Min", "Slider")
    hue_max = cv2.getTrackbarPos("Hue Max", "Slider")
    sat_min = cv2.getTrackbarPos("Saturation Min", "Slider")
    sat_max = cv2.getTrackbarPos("Saturation Max", "Slider")
    val_min = cv2.getTrackbarPos("Value Min", "Slider")
    val_max = cv2.getTrackbarPos("Value Max", "Slider")

    # Set bounds
    lower_bound = np.array([hue_min, sat_min, val_min])
    upper_bound = np.array([hue_max, sat_max, val_max])

    # Convert the frame to HSV color space
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create mask for red color using calibrated values
    mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

    # Apply Gaussian blur to reduce noise
    blurred_mask = cv2.GaussianBlur(mask, (15, 15), 0)

    # Use morphological operations to clean up the mask
    kernel = np.ones((5, 5), np.uint8)
    morphed_mask = cv2.morphologyEx(blurred_mask, cv2.MORPH_CLOSE, kernel)
    morphed_mask = cv2.morphologyEx(morphed_mask, cv2.MORPH_OPEN, kernel)

    # Additional noise reduction
    morphed_mask = cv2.erode(morphed_mask, kernel, iterations=2)
    morphed_mask = cv2.dilate(morphed_mask, kernel, iterations=2)

    resulting_img = cv2.bitwise_and(frame, frame, mask=morphed_mask)

    stacked_imgs = np.hstack([frame, resulting_img])

    # Display the resulting frame and mask
    cv2.imshow('Image', stacked_imgs)
    
    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release everything when done
cap.release()
cv2.destroyAllWindows()
