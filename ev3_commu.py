import serial

# Replace 'COM3' with your EV3's COM port
ser = serial.Serial('COM3', 115200, timeout=1)
ser.write(b'Hello EV3!\n')
response = ser.readline().decode('utf-8').strip()
print(f"Received from EV3: {response}")
ser.close()
