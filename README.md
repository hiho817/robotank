# RoboTank Project
This project is designed to control a RoboTank using AprilTag detection and pose estimation. The project includes various scripts for calibration, testing, and communication with the EV3 robot.

# Usage
## Camera Calibration
1. Place chessboard images in the calibration_images directory.
2. Run the camera calibration script:
`python cam_calibration.py`
## AprilTag Detection and Pose Estimation
### Single Image Test
1. Place the test image in the project directory.
2. Run the single image test script:
`python one_image_test.py`
### Video Stream Test
1. Connect a webcam to your computer.
2. Run the video stream test script:
`python stream_test.py`

### EV3 communication
communicate with wifi, can test with wifitest/client.py file

### main function

... state machine