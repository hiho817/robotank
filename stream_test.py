from estimator import AprilTagPoseEstimator
import numpy as np
import cv2

# Camera parameters and tag size
fx = 1.29303495e+03
fy = 1.29890444e+03
cx = 9.25924670e+02
cy = 5.45383028e+02
dist_coeffs = np.array([8.53173082e-02, -3.02010441e-01, 4.65198399e-03, -1.23371448e-04, 3.29937136e-01])
tag_size = 0.057

estimator = AprilTagPoseEstimator(fx, fy, cx, cy, dist_coeffs, tag_size)

# Start video capture
cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    estimator.frame = frame
    estimator.image_init()
    estimator.detect_tags()
    estimator.estimate_poses()
    relative_pose = estimator.compute_relative_pose(1, 2)

    if relative_pose is not None:
        estimator.compute_advances_error(relative_pose)
        estimator.draw_detections()
        estimator.draw_errors_on_image()

    estimator.show_image()

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()