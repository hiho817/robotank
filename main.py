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
        logging.info("Entered PUSH state.")
        push_angle_pd.previous_error = 0
        push_speed_pd.previous_error = 0


    def on_enter_AIMING(self):
        logging.info("Entered AIMING state.")
        aim_angle_pd.previous_error = 0


    def on_enter_SHOOT(self):
        logging.info("Entered SHOOT state.")

    def on_exit_SHOOT(self):
        logging.info("WAIT FOR SHOOTING.")
        time.sleep(3)

# Configure the logging system
logging.basicConfig(
    level=logging.INFO,  # Set the minimum logging level
    format='%(asctime)s - %(levelname)s - %(message)s',  # Define the log message format
    handlers=[
        logging.StreamHandler()  # Output logs to the console
    ]
)

# Camera parameters and tag size
fx = 1.45135144e+03
fy = 1.45808064e+03
cx = 6.97316163e+02
cy = 3.87170119e+02
dist_coeffs = np.array([0.07852367,  0.90420852, -0.00676142,  0.01412085, -4.09736367])
tag_size = 0.057

estimator = AprilTagPoseEstimator(fx, fy, cx, cy, dist_coeffs, tag_size)

# motor parameters
motor_command = [0,0,0] # 0: left, 1:right, 2:shoot

# Start video capture
cap = cv2.VideoCapture(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

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

def init_wifi():
    import socket
    HOST = "192.168.20.220"
    PORT = 9999
    global s 
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((HOST, PORT))

def command_ev3(cmd):
    cmd = "l" + str(motor_command[0]) + "r" + str(motor_command[1]) + "s" + str(motor_command[2])
    s.send(cmd.encode())

def control_mixer(speed, angle):
    motor_max = 500
    motor_min = -motor_max
    if robotank.state == 'PUSH':
        left_speed = speed + angle
        right_speed = speed - angle
        shoot_command = 0
    elif robotank.state == 'AIMING':
        left_speed = -angle
        right_speed = angle
        shoot_command = 0

    motor_command = [int(left_speed), int(right_speed), shoot_command]
    for i in range(2):
        if motor_command[i] > motor_max:
            motor_command[i] = motor_max
        if motor_command[i] < motor_min:
            motor_command[i] = motor_min
    
    return motor_command

# PID Controller Class
class PDController:
    def __init__(self, kp, kd):
        self.kp = kp
        self.kd = kd
        self.previous_error = 0
    
    def update(self, dt, error):
        max_output = 500
        min_output = -500
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        output = self.kp * error+ self.kd * derivative
        self.previous_error = error
        if output < min_output: output = min_output
        if output > max_output: output = max_output
        return output
    
push_angle_pd = PDController(kp=50.0, kd=0.1)
push_speed_pd = PDController(kp=1000.0, kd=0.1)
aim_angle_pd = PDController(kp=100.0, kd=0.1)

# Example Usage
if __name__ == "__main__":

    init_wifi()
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
                    motor_command = control_mixer(push_speed, push_angle)

        elif robotank.state == 'AIMING':
            if estimator.detected == False:
                logging.info("Lose target, transitioning to IDLE state.")
                robotank.lose_target()
            else:
                logging.debug(f"Align angle error: {np.degrees(estimator.align_angle_err):.2f} degrees")
                if abs(np.degrees(estimator.align_angle_err)) < 3.0:
                    logging.info("Aiming check.")
                    robotank.aim_check()
                else:
                    logging.debug("Aiming.")
                    aim_angle = aim_angle_pd.update(dt, estimator.align_angle_err)
                    motor_command = control_mixer(0, aim_angle)

        elif robotank.state == 'SHOOT':
            motor_command = [0,0,1]
            robotank.fire()
            
        
        process_frame()
        command_ev3(motor_command)
        logging.debug("Motor command: "+ "l" + str(motor_command[0]) + "r" + str(motor_command[1]) + "s" + str(motor_command[2]))
        motor_command = [0,0,0]
        # Exit on pressing 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        prev_time = time.time()
