import cv2 as cv
from picamera2 import Picamera2
import time
import numpy as np

# =========================================================================
# 1. CAMERA POSE & TARGET CONFIGURATION
# =========================================================================
# The exact size of the printed marker's outer black edge in meters
MARKER_LENGTH_METERS = 0.15  # Adjust based on your physical print size (e.g., 15cm)

# Calibrated camera intrinsic parameters for 1280x720 Pi Camera stream
# (Highly recommended to calibrate your specific lens using a chessboard grid later)
camera_matrix = np.array([
    [1000.0, 0.0, 640.0],
    [0.0, 1000.0, 360.0],
    [0.0, 0.0, 1.0]
], dtype=np.float32)

dist_coeffs = np.zeros((5, 1), dtype=np.float32) 

# =========================================================================
# 2. OPTIMIZED OPENCV 5.x ARUCO DETECTOR
# =========================================================================
dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_50)
parameters = cv.aruco.DetectorParameters()

# Apply the high-tolerance screen & real-world filters that gave SUCCESS
parameters.adaptiveThreshWinSizeMin = 3
parameters.adaptiveThreshWinSizeMax = 63
parameters.adaptiveThreshWinSizeStep = 5
parameters.errorCorrectionRate = 0.8  

detector = cv.aruco.ArucoDetector(dictionary, parameters)

# Local 3D coordinates defining the marker geometry
obj_points = np.array([
    [-MARKER_LENGTH_METERS / 2.0,  MARKER_LENGTH_METERS / 2.0, 0.0],
    [ MARKER_LENGTH_METERS / 2.0,  MARKER_LENGTH_METERS / 2.0, 0.0],
    [ MARKER_LENGTH_METERS / 2.0, -MARKER_LENGTH_METERS / 2.0, 0.0],
    [-MARKER_LENGTH_METERS / 2.0, -MARKER_LENGTH_METERS / 2.0, 0.0]
], dtype=np.float32)

# =========================================================================
# 3. STREAM INGESTION LOOP
# =========================================================================
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"size": (1280, 720), "format": "RGB888"}
)
picam2.configure(config)
picam2.start()

print("\n[ROBOT SYSTEM] Live navigation tracking active. Press 'q' to stop.")
time.sleep(2.0)

try:
    while True:
        # Pull live frame matrix from the CSI sensor
        frame = picam2.capture_array("main")
        frame_bgr = cv.cvtColor(frame, cv.COLOR_RGB2BGR)
        
        gray = cv.cvtColor(frame_bgr, cv.COLOR_BGR2GRAY)
        
        # Dual-path scanning (handles normal paper and inverted screens seamlessly)
        corners, ids, rejected = detector.detectMarkers(gray)
        if ids is None:
            gray_inverted = cv.bitwise_not(gray)
            corners, ids, rejected = detector.detectMarkers(gray_inverted)

        # Calculate 3D Positioning data if a marker is locked
        if ids is not None:
            cv.aruco.drawDetectedMarkers(frame_bgr, corners, ids)
            
            for index, marker_id in enumerate(ids.flatten()):
                # Run Perspective-n-Point to get Rotation (rvec) and Translation (tvec) vectors
                success, rvec, tvec = cv.solvePnP(
                    obj_points, corners[index], camera_matrix, dist_coeffs, flags=cv.SOLVEPNP_ITERATIVE
                )
                
                if success:
                    # Extract distance coordinate offsets relative to the robot's camera face
                    x_offset = tvec[0][0]
                    y_offset = tvec[1][0]
                    z_forward = tvec[2][0]  # Direct distance away from the robot in meters
                    
                    # Print real-time spatial positioning to your robot control console
                    print(f"Tracking ID: {marker_id} | Spatial Pos: X={x_offset:.2f}m, Y={y_offset:.2f}m, Z={z_forward:.2f}m")
                    
                    # Project a 3D coordinate frame axis onto the marker to visualize spatial orientation
                    cv.drawFrameAxes(frame_bgr, camera_matrix, dist_coeffs, rvec, tvec, MARKER_LENGTH_METERS * 0.6)
                    
                    # Draw a nice clean UI label near the tracking bounding box
                    cx = int(np.mean(corners[index][:, 0]))
                    cy = int(np.mean(corners[index][:, 1]))
                    cv.putText(frame_bgr, f"Dist: {z_forward:.2f}m", (cx - 40, cy - 20), 
                               cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # Render the tracking viewport stream window
        cv.imshow("Robot Mapping Feed", frame_bgr)
        
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("\nRobot process interrupted manually.")

finally:
    cv.destroyAllWindows()
    picam2.stop()
    print("[SYSTEM] Navigation pipeline safely offline.")
