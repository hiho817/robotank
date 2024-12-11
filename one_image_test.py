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
estimator.frame = cv2.imread('test_1.jpg')
estimator.detect_tags()
estimator.estimate_poses()
relative_pose = estimator.compute_relative_pose(1, 2)

if relative_pose is not None:
    estimator.compute_push_error(relative_pose)
    estimator.compute_align_error(relative_pose)
    print(f"Distance error: {estimator.push_distance_err:.2f} m")
    print(f"Push angle error: {np.degrees(estimator.push_angle_err):.2f} degrees")
    print(f"Align angle error: {np.degrees(estimator.align_angle_err):.2f} degrees")    

# if relative_pose is not None:
#     print(f"Transformation from ID1 to ID2:\n{relative_pose}")

estimator.draw_detections()
estimator.draw_errors_on_image()
estimator.show_image()
cv2.waitKey(0)
