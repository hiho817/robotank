#AUTHOR: Sheng-Lun 
#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.media.ev3dev import Font
import time
import math 

import socket

target_host = "192.168.20.220"
target_port = "9999"
server = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
server.bind((target_host, target_port))

server.listen(5)

ev3 = EV3Brick()
# ev3.screen.print("[*] Listening on %s:%d " % (target_host, target_port))
left_wheel=Motor(Port.B)
right_wheel=Motor(Port.D)
shoot_motor=Motor(Port.A)

def move(left_speed,right_speed):
    left_wheel.run(-left_speed)
    right_wheel.run(-right_speed)

def shoot(command):
    if command==1:
        shoot_motor.run_angle(200, -360, then=Stop.HOLD, wait=True)

def parse_command(command):
    try:
        # 移除最前面的 'l'
        command = command[1:]
        
        # 以 'r' 分割字符串
        left_speed_str, rest = command.split('r', 1)
        left_speed = int(left_speed_str)

        # 以 's' 分割剩余字符串
        right_speed_str, stop_str = rest.split('s', 1)
        right_speed = int(right_speed_str)
        stop_command = int(stop_str)
        
        return left_speed, right_speed, stop_command
    except:
        print('Invalid command')
        return 0, 0, 0



while True:
    client, addr = server.accept()
    print('Connected by ', addr)
    

    while True:
        data = client.recv(1024)
        datastr=data.decode()
        ev3.screen.print('Client recv data :', datastr )
        left_speed, right_speed, stop_command = parse_command(datastr)
        move(left_speed,right_speed)
        shoot(stop_command)
server.close()

# server.connect((target_host,target_port))
# ev3 = EV3Brick()
# ev3.screen.print('Hello!')


