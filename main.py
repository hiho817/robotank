from transitions import Machine
import time
from estimator import AprilTagPoseEstimator
import numpy as np
import cv2
import logging

class Robotank:
    # Define all possible states
    states = ['IDLE', 'PUSH', 'AIMING', 'SHOOT']

    def __init__(self):
        # Initialize the state machine
        self.machine = Machine(model=self, states=Robotank.states, initial='IDLE')

        # Define transitions
        self.machine.add_transition(trigger='push', source='IDLE', dest='PUSH')
        self.machine.add_transition(trigger='get_destination', source='PUSH', dest='AIMING')
        self.machine.add_transition(trigger='lose_target', source='PUSH', dest='IDLE')
        self.machine.add_transition(trigger='lose_target', source='AIMING', dest='IDLE')
        self.machine.add_transition(trigger='aim_check', source='AIMING', dest='SHOOT')
        self.machine.add_transition(trigger='fire', source='SHOOT', dest='IDLE')

        # Optional: Add transitions that allow moving between specific states
        # self.machine.add_transition(trigger='reset', source='*', dest='IDLE')

    # Optional: Define methods that are called when entering a state
    def on_enter_IDLE(self):
        logging.info("Entered IDLE state.")

    def on_enter_PUSH(self):
        push_angle_pd.previous_error = 0
        push_speed_pd.previous_error = 0
        logging.info("Entered PUSH state.")

    def on_enter_AIMING(self):
        aim_angle_pd.previous_error = 0
        logging.info("Entered AIMING state.")

    def on_enter_SHOOT(self):
        logging.info("Entered SHOOT state.")

# Configure the logging system
logging.basicConfig(
    level=logging.DEBUG,  # Set the minimum logging level
    format='%(asctime)s - %(levelname)s - %(message)s',  # Define the log message format
    handlers=[
        logging.StreamHandler()  # Output logs to the console
    ]
)

# Camera parameters and tag size
fx = 1.29303495e+03
fy = 1.29890444e+03
cx = 9.25924670e+02
cy = 5.45383028e+02
dist_coeffs = np.array([8.53173082e-02, -3.02010441e-01, 4.65198399e-03, -1.23371448e-04, 3.29937136e-01])
tag_size = 0.057

estimator = AprilTagPoseEstimator(fx, fy, cx, cy, dist_coeffs, tag_size)

# motor parameters
motor_command = [0,0,0] # 0: left, 1:right, 2:shoot

# Start video capture
cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

def process_frame():
    ret, frame = cap.read()
    if not ret:
        return
    estimator.detected = False
    estimator.frame = frame
    estimator.image_init()
    estimator.detect_tags()
    estimator.estimate_poses()
    relative_pose = estimator.compute_relative_pose(1, 2)

    if relative_pose is not None:
        estimator.detected = True

    logging.debug(relative_pose)

    if estimator.detected == True:

        if robotank.state == 'PUSH':
            estimator.compute_push_error(relative_pose)
        elif robotank.state == 'AIMING':
            estimator.compute_align_error(relative_pose)

        estimator.draw_detections()
        estimator.draw_errors_on_image()
               
    cv2.putText(estimator.frame, f"Current state: {robotank.state}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX,
    1, (0, 0, 255), 2, cv2.LINE_AA)

    estimator.show_image()

def command_ev3():
    #TODO: commute with ev3 with wifi, msg l___r___s_ ex:l100r100s1
    pass

# PID Controller Class
class PDController:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.previous_error = 0
    
    def update(self, dt, error):
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        output = self.kp * error+ self.kd * derivative
        self.previous_error = error
        return output
    
push_angle_pd = PDController(kp=1.0, kd=0.1)
push_speed_pd = PDController(kp=1.0, kd=0.1)
aim_angle_pd = PDController(kp=1.0, kd=0.1)

# Example Usage
if __name__ == "__main__":

    prev_time = 0
    robotank = Robotank()
    logging.info(f"Initial State: {robotank.state}")
    while True:
        dt = time.time() - prev_time

        if robotank.state == 'IDLE' and estimator.detected == True:
            logging.info("Detected tag, transitioning to PUSH state.")
            robotank.push()

        elif robotank.state == 'PUSH':
            if estimator.detected == False:
                logging.info("Lose target, transitioning to IDLE state.")
                robotank.lose_target()
            else:
                if estimator.push_distance_err < 0.05:
                    logging.info("Get the destination.")
                    robotank.get_destination()
                else:
                    logging.debug("Getting to destination.")
                    push_speed = push_speed_pd.update(dt, estimator.push_distance_err)
                    push_angle = push_angle_pd.update(dt, estimator.push_angle_err)

        elif robotank.state == 'AIMING':
            if estimator.detected == False:
                logging.info("Lose target, transitioning to IDLE state.")
                robotank.lose_target()
            else:
                logging.debug(estimator.align_angle_err)
                if np.degrees(estimator.align_angle_err) < 1.0:
                    logging.info("Aiming check.")
                    robotank.aim_check()
                else:
                    logging.debug("Aiming.")
                    aim_angle = aim_angle_pd.update(dt, estimator.align_angle_err)

        elif robotank.state == 'SHOOT':
            robotank.fire()
        
        logging.debug(estimator.detected)
        process_frame()
        command_ev3()
        # Exit on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        prev_time = time.time()
