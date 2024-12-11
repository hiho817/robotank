import pupil_apriltags as apriltag
import cv2
import numpy as np

class AprilTagPoseEstimator:
    def __init__(self, fx, fy, cx, cy, dist_coeffs, tag_size):
        # Camera parameters
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.dist_coeffs = dist_coeffs
        self.tag_size = tag_size

        # Camera matrix
        self.camera_matrix = np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])

        # 3D tag corners
        self.tag_corners_3d = np.array([
            [-tag_size / 2, -tag_size / 2, 0],
            [ tag_size / 2, -tag_size / 2, 0],
            [ tag_size / 2,  tag_size / 2, 0],
            [-tag_size / 2,  tag_size / 2, 0]
        ])

        # AprilTag Detector
        self.detector = apriltag.Detector()

        # Variables to hold detections, transformations, and image
        self.frame = None
        self.detections = []
        self.transforms = {}
        self.align_angle_err = None
        self.push_angle_err = None
        self.push_distance_err = None
        self.detected = False

    def image_init(self):
        #set error to None
        self.push_angle_err = None
        self.push_distance_err = None
        self.align_angle_err = None
        self.transforms = {}


    def detect_tags(self):
        if self.frame is None:
            raise ValueError("No image loaded. Call load_image() first.")

        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        self.detections = self.detector.detect(gray)

    def estimate_poses(self):
        # Estimate poses for all detected tags
        for detection in self.detections:
            image_points = detection.corners
            success, rvec, tvec = cv2.solvePnP(
                self.tag_corners_3d,
                image_points,
                self.camera_matrix,
                self.dist_coeffs
            )
            if success:
                R, _ = cv2.Rodrigues(rvec)
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = tvec.reshape(3,)
                self.transforms[detection.tag_id] = T

    def compute_relative_pose(self, id1, id2):
        # Check if both IDs exist in transforms dictionary
        if id1 not in self.transforms:
            # print(f"Tag with ID {id1} not found.")
            return None

        if id2 not in self.transforms:
            # print(f"Tag with ID {id2} not found.")
            return None

        T_id1_to_id2 = np.linalg.inv(self.transforms[id1]) @ self.transforms[id2]
        return T_id1_to_id2
    
    def compute_push_error(self, transpose_matrix):

        transpose_matrix = transpose_matrix @ np.array([[1, 0, 0, 0], 
                                                        [0, 1, 0, -0.2], 
                                                        [0, 0, 1, 0], 
                                                        [0, 0, 0, 1]])
        # Extract translation vector
        translation_xy = transpose_matrix[:2, 3]

        # translation[0] -> x component, translation[1] -> y component
        x, y = translation_xy[0], translation_xy[1]

        # Compute position error (magnitude of translation vector)
        self.push_distance_err = np.linalg.norm(translation_xy)

        # Compute angle error (angle of translation vector)
        self.push_angle_err = np.arctan2(x, y)

        return self.push_distance_err, self.push_angle_err
    
    def compute_align_error(self, transpose_matrix):
        rotation_xy = transpose_matrix[:2, :2]

        # Compute angle error (angle of rotation matrix)
        self.align_angle_err = np.arctan2(rotation_xy[1, 0], rotation_xy[0, 0])

        return self.align_angle_err

    def draw_detections(self):
        # Draw polygons and axes on the image for each detection
        for detection in self.detections:
            image_points = detection.corners
            pts = image_points.reshape(-1, 1, 2).astype(int)
            cv2.polylines(self.frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

            # Draw tag ID
            cv2.putText(self.frame, f"ID: {detection.tag_id}", 
                        (pts[0][0][0], pts[0][0][1] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # If pose was estimated, draw axes
            if detection.tag_id in self.transforms:
                # Recompute rvec/tvec from transform
                T = self.transforms[detection.tag_id]
                R = T[:3, :3]
                tvec = T[:3, 3].reshape(3,1)
                rvec, _ = cv2.Rodrigues(R)
                cv2.drawFrameAxes(self.frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.tag_size)

    def draw_errors_on_image(self):
        # Only draw if we have both angle_err and distance_err computed
        if self.push_angle_err is not None and self.push_distance_err is not None:
            text_angle = f"Angle Error (deg): {np.degrees(self.push_angle_err):.2f}"
            text_pos = f"Position Error (m): {self.push_distance_err:.2f}"
        
        if self.align_angle_err is not None:
            text_align = f"Align Error (deg): {np.degrees(self.align_angle_err):.2f}"

            # Put the text on the image (top-left corner)
            cv2.putText(self.frame, text_angle, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 
                        1, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(self.frame, text_pos, (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 
                        1, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(self.frame, text_align, (50, 150), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 0, 255), 2, cv2.LINE_AA)
            
    def show_image(self):
        cv2.imshow('AprilTag Detections', self.frame)


if __name__ == "__main__":
    pass
