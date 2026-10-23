import cv2 as cv
import numpy as np

# Load OpenCV 5.0 Dictionary
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_50)

# Generate marker (ID: 0) at 600x600 pixels
marker_size = 600
marker = cv.aruco.generateImageMarker(dictionary, 0, marker_size)

# Add the perfect white padding (Quiet Zone) required by OpenCV 5
margin = 150
canvas = np.full((marker_size + 2 * margin, marker_size + 2 * margin), 255, dtype=np.uint8)
canvas[margin:margin + marker_size, margin:margin + marker_size] = marker

# Save the master file
cv.imwrite("clean_5x5_marker.png", canvas)
print("SUCCESS: 'clean_5x5_marker.png' saved. Open this image on your screen.")
