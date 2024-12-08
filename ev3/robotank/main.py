#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import bluetooth

# The EV3 should already have Bluetooth enabled and be discoverable.
# Make sure to set the EV3’s Bluetooth visibility and pairing in its menus.

server_sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
server_sock.bind(("", bluetooth.PORT_ANY))
server_sock.listen(1)

print("Waiting for a Bluetooth connection...")
client_sock, client_info = server_sock.accept()
print("Connected to:", client_info)

# Receive data (up to 1024 bytes)
data = client_sock.recv(1024)
print("Received message:", data.decode('utf-8'))

# Optionally, send a response back:
client_sock.send("Hello from EV3!")

client_sock.close()
server_sock.close()
print("Connection closed.")
