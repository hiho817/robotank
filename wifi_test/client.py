#AUTHOR: Sheng-Lun 
import socket

# HOST = "127.0.0.1"
HOST = "192.168.20.220"
PORT = 9999

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

while True:
    cmd = input("Please input msg:")
    s.send(cmd.encode()) 