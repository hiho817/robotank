import pupil_apriltags as apriltag
import cv2
import numpy as np

# Camera parameters
fx = 1.29303495e+03
fy = 1.29890444e+03
cx = 9.25924670e+02
cy = 5.45383028e+02

# Camera matrix and distortion coefficients
camera_matrix = np.array([[fx, 0, cx],
                          [0, fy, cy],
                          [0, 0, 1]])
dist_coeffs = np.array([ 8.53173082e-02, -3.02010441e-01,  4.65198399e-03, -1.23371448e-04, 3.29937136e-01])

# Define the size of the AprilTag (in meters)
tag_size = 0.057  # 57 mm

# Create the AprilTag detector
detector = apriltag.Detector()

# Start video capture
cap = cv2.VideoCapture(0)

# Define the 3D coordinates of the tag corners in the tag's coordinate system
tag_corners_3d = np.array([
    [-tag_size / 2, -tag_size / 2, 0],
    [ tag_size / 2, -tag_size / 2, 0],
    [ tag_size / 2,  tag_size / 2, 0],
    [-tag_size / 2,  tag_size / 2, 0]
])

        
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect AprilTags
    detections = detector.detect(gray)

    for detection in detections:
        # Get the 2D image points of the tag corners
        image_points = detection.corners

        # Draw the detected tag boundary
        pts = image_points.reshape(-1, 1, 2).astype(int)
        cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

        # Estimate pose using solvePnP
        success, rvec, tvec = cv2.solvePnP(
            tag_corners_3d,
            image_points,
            camera_matrix,
            dist_coeffs
        )
        
        if success:
            # Convert rotation vector to rotation matrix
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            
            # Draw the pose axes on the image
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, tag_size)
            
            # Optionally, display the translation and rotation vectors
            print(f"Tag ID: {detection.tag_id}, tvec: {tvec.ravel()}, rvec: {rvec.ravel()}")
            print(f"Rotation Matrix:\n{rotation_matrix}")

    # Display the frame
    cv2.imshow('AprilTag Detection', frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()
