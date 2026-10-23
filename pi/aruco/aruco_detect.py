import cv2 as cv
import numpy as np

# 1. Dictionary Setup
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_50)
parameters = cv.aruco.DetectorParameters()

# OpenCV 5.0 OPTIMIZATIONS FOR ROBOTICS:
# Broaden the grid analysis window to look at more pixel blocks per cell
parameters.adaptiveThreshWinSizeMin = 3
parameters.adaptiveThreshWinSizeMax = 63
parameters.adaptiveThreshWinSizeStep = 5

# Allow OpenCV 5.0 to perform heavier bit error correction (critical for screens)
parameters.errorCorrectionRate = 0.8  

detector = cv.aruco.ArucoDetector(dictionary, parameters)

# 2. Load the Frame
frame = cv.imread("clean_5x5_marker.png")
if frame is None:
    print("Error: clean_5x5_marker.png not found.")
    exit()

gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)

# 3. Comprehensive Dual-Path Scan (Handles standard and inverted displays)
corners, ids, rejected = detector.detectMarkers(gray)

if ids is None:
    print("Standard color map failed to decode bits. Scanning inverted track...")
    gray_inverted = cv.bitwise_not(gray)
    corners, ids, rejected = detector.detectMarkers(gray_inverted)

# 4. Results
print("\n--- OpenCV 5.0 Diagnostic Report ---")
print("OpenCV Version:", cv.__version__)
print("Rejected candidates remaining:", len(rejected))

if ids is not None:
    print("STATUS: SUCCESS! Marker identified.")
    print("Detected IDs:", ids.flatten())
    
    # Visual validation check
    cv.aruco.drawDetectedMarkers(frame, corners, ids)
    cv.imwrite("diagnostic_success.jpg", frame)
    print("Saved 'diagnostic_success.jpg' showing tracking confirmation.")
else:
    print("STATUS: FAILED. The grid squares are clear but their sequence is invalid.")

